#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <future>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <GLFW/glfw3.h>
#include <imgui.h>
#include <backends/imgui_impl_glfw.h>
#include <backends/imgui_impl_opengl3.h>

#include "al/al_ilqr_solver.hpp"
#include "autodrive/demo_scenario.hpp"
#include "autodrive/vehicle_bicycle_config.hpp"
#include "visualization/trajectory_writer.hpp"

namespace {

using my_al_ilqr::ALILQRSolver;
using my_al_ilqr::ALILQROuterIterationLog;
using my_al_ilqr::StaticObstacleScenarioConfig;
using my_al_ilqr::VehicleBicycleConfig;
using my_al_ilqr::Vector;

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;

struct PlanningOutput {
  bool success = false;
  std::string status;
  std::string terminal_state;
  bool converged = false;
  double best_violation = 0.0;
  double initial_violation = 0.0;
  double final_violation = 0.0;
  double initial_cost = 0.0;
  double final_cost = 0.0;
  double planning_time_ms = 0.0;
  int horizon = 0;
  int outer_iterations = 0;
  int total_inner_iterations = 0;
  std::filesystem::path trajectory_csv_path;
  VehicleBicycleConfig vehicle_config;
  StaticObstacleScenarioConfig scenario_config;
  std::vector<ALILQROuterIterationLog> outer_logs;
  std::vector<double> initial_x;
  std::vector<double> initial_y;
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> yaw;
  std::vector<double> speed;
  std::vector<double> accel;
  std::vector<double> kappa;
  std::vector<double> base_cost_history;
  std::vector<double> violation_history;
  std::vector<double> penalty_history;
};

struct ScenePreview {
  bool valid = false;
  std::string status;
  VehicleBicycleConfig vehicle_config;
  StaticObstacleScenarioConfig scenario_config;
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> yaw;
  std::vector<double> speed;
};

struct FrontendState {
  std::array<char, 512> config_path{};
  VehicleBicycleConfig vehicle_config;
  StaticObstacleScenarioConfig scenario_config;
  std::string status = "Load a config and press Plan.";
  std::string terminal_state;
  std::filesystem::path last_csv_path;
  double last_best_violation = 0.0;
  bool planning = false;
  bool has_result = false;
  bool preview_dirty = true;
  double scene_zoom = 1.0;
  double scene_pan_x = 0.0;
  double scene_pan_y = 0.0;
  bool show_reference_line = true;
  bool show_obstacle = true;
  bool show_initial_guess = true;
  bool show_optimized_trajectory = true;
  bool show_vehicle_outline = true;
  bool show_safety_circle = true;
  ScenePreview preview;
  PlanningOutput result;
};

struct Bounds2D {
  double min_x = 0.0;
  double max_x = 1.0;
  double min_y = 0.0;
  double max_y = 1.0;
};

bool SliderInputDouble(const char* label,
                       double* value,
                       double min_value,
                       double max_value,
                       const char* format = "%.3f") {
  bool changed = false;
  ImGui::PushID(label);
  const float full_width = ImGui::GetContentRegionAvail().x;
  const float label_width = 120.0f;
  const float input_width = 88.0f;
  const float spacing = ImGui::GetStyle().ItemSpacing.x;
  const float slider_width =
      std::max(80.0f, full_width - label_width - input_width - 2.0f * spacing);
  ImGui::AlignTextToFramePadding();
  ImGui::TextUnformatted(label);
  ImGui::SameLine(label_width);
  ImGui::SetNextItemWidth(slider_width);
  changed |= ImGui::SliderScalar("##slider", ImGuiDataType_Double, value, &min_value, &max_value, format);
  ImGui::SameLine(0.0f, spacing);
  ImGui::SetNextItemWidth(input_width);
  changed |= ImGui::InputScalar("##input", ImGuiDataType_Double, value, nullptr, nullptr, format);
  ImGui::PopID();
  return changed;
}

ImVec2 ToCanvas(const ImVec2& origin, const ImVec2& size, const Bounds2D& bounds, double x, double y) {
  const double span_x = std::max(1e-6, bounds.max_x - bounds.min_x);
  const double span_y = std::max(1e-6, bounds.max_y - bounds.min_y);
  const float px = origin.x + static_cast<float>((x - bounds.min_x) / span_x) * size.x;
  const float py = origin.y + size.y - static_cast<float>((y - bounds.min_y) / span_y) * size.y;
  return ImVec2(px, py);
}

Bounds2D ComputeSceneBounds(const ScenePreview& preview, const PlanningOutput* output) {
  Bounds2D bounds;
  bounds.min_x = std::numeric_limits<double>::infinity();
  bounds.max_x = -std::numeric_limits<double>::infinity();
  bounds.min_y = std::numeric_limits<double>::infinity();
  bounds.max_y = -std::numeric_limits<double>::infinity();

  auto expand = [&](double x, double y) {
    bounds.min_x = std::min(bounds.min_x, x);
    bounds.max_x = std::max(bounds.max_x, x);
    bounds.min_y = std::min(bounds.min_y, y);
    bounds.max_y = std::max(bounds.max_y, y);
  };

  for (size_t i = 0; i < preview.x.size(); ++i) {
    expand(preview.x[i], preview.y[i]);
  }
  if (output) {
    for (size_t i = 0; i < output->x.size(); ++i) {
      expand(output->x[i], output->y[i]);
    }
    for (size_t i = 0; i < output->initial_x.size(); ++i) {
      expand(output->initial_x[i], output->initial_y[i]);
    }
  }
  expand(preview.scenario_config.obstacle_center_x - preview.scenario_config.obstacle_radius,
         preview.scenario_config.obstacle_center_y - preview.scenario_config.obstacle_radius);
  expand(preview.scenario_config.obstacle_center_x + preview.scenario_config.obstacle_radius,
         preview.scenario_config.obstacle_center_y + preview.scenario_config.obstacle_radius);

  if (!std::isfinite(bounds.min_x) || !std::isfinite(bounds.max_x)) {
    bounds = Bounds2D{};
  }

  const double margin_x = std::max(1.0, 0.08 * (bounds.max_x - bounds.min_x));
  const double margin_y = std::max(0.8, 0.15 * (bounds.max_y - bounds.min_y));
  bounds.min_x -= margin_x;
  bounds.max_x += margin_x;
  bounds.min_y -= margin_y;
  bounds.max_y += margin_y;
  return bounds;
}

Bounds2D ApplySceneView(const Bounds2D& input_bounds, double zoom, double pan_x, double pan_y) {
  Bounds2D bounds = input_bounds;
  zoom = std::clamp(zoom, 0.25, 20.0);
  const double cx = 0.5 * (bounds.min_x + bounds.max_x) + pan_x;
  const double cy = 0.5 * (bounds.min_y + bounds.max_y) + pan_y;
  const double span_x = std::max(1e-6, (bounds.max_x - bounds.min_x) / zoom);
  const double span_y = std::max(1e-6, (bounds.max_y - bounds.min_y) / zoom);
  bounds.min_x = cx - 0.5 * span_x;
  bounds.max_x = cx + 0.5 * span_x;
  bounds.min_y = cy - 0.5 * span_y;
  bounds.max_y = cy + 0.5 * span_y;
  return bounds;
}

std::vector<ImVec2> MakeObstaclePolyline(const ImVec2& origin,
                                         const ImVec2& size,
                                         const Bounds2D& bounds,
                                         double cx,
                                         double cy,
                                         double radius,
                                         int segments = 48) {
  std::vector<ImVec2> pts;
  pts.reserve(segments + 1);
  for (int i = 0; i <= segments; ++i) {
    const double theta = kTwoPi * static_cast<double>(i) / static_cast<double>(segments);
    pts.push_back(ToCanvas(origin, size, bounds,
                           cx + radius * std::cos(theta),
                           cy + radius * std::sin(theta)));
  }
  return pts;
}

std::vector<ImVec2> MakeVehicleOutline(const VehicleBicycleConfig& config,
                                       const ImVec2& origin,
                                       const ImVec2& size,
                                       const Bounds2D& bounds,
                                       double x,
                                       double y,
                                       double yaw) {
  const double rear = config.body.rear_axle_to_rear;
  const double front = config.body.length - config.body.rear_axle_to_rear;
  const double half_width = 0.5 * config.body.width;
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);

  const std::vector<std::pair<double, double>> body_corners = {
      {front, half_width},
      {front, -half_width},
      {-rear, -half_width},
      {-rear, half_width},
      {front, half_width},
  };

  std::vector<ImVec2> polyline;
  polyline.reserve(body_corners.size());
  for (const auto& corner : body_corners) {
    const double wx = x + c * corner.first - s * corner.second;
    const double wy = y + s * corner.first + c * corner.second;
    polyline.push_back(ToCanvas(origin, size, bounds, wx, wy));
  }
  return polyline;
}

void DrawPolyline(ImDrawList* draw_list, const std::vector<ImVec2>& polyline, ImU32 color, float thickness) {
  if (!draw_list || polyline.size() < 2) {
    return;
  }
  draw_list->AddPolyline(polyline.data(), static_cast<int>(polyline.size()), color, 0, thickness);
}

void FillClosedPolygon(ImDrawList* draw_list, const std::vector<ImVec2>& polygon, ImU32 fill_color) {
  if (!draw_list || polygon.size() < 3) {
    return;
  }
  const int count = static_cast<int>(polygon.size()) - 1;
  if (count >= 3) {
    draw_list->AddConvexPolyFilled(polygon.data(), count, fill_color);
  }
}

void DrawDashedPolyline(ImDrawList* draw_list,
                        const std::vector<ImVec2>& polyline,
                        ImU32 color,
                        float thickness,
                        int draw_stride = 2) {
  if (!draw_list || polyline.size() < 2) {
    return;
  }
  for (size_t i = 0; i + 1 < polyline.size(); ++i) {
    if ((static_cast<int>(i) % draw_stride) != 0) {
      continue;
    }
    draw_list->AddLine(polyline[i], polyline[i + 1], color, thickness);
  }
}

void DrawSignalPlot(const char* title,
                    const std::vector<double>& values,
                    ImU32 color,
                    double fixed_min = std::numeric_limits<double>::quiet_NaN(),
                    double fixed_max = std::numeric_limits<double>::quiet_NaN()) {
  ImGui::BeginChild(title, ImVec2(0.0f, 0.0f), true, ImGuiWindowFlags_NoScrollbar);
  ImGui::TextUnformatted(title);
  const ImVec2 top_left = ImGui::GetCursorScreenPos();
  const ImVec2 canvas_size = ImVec2(std::max(80.0f, ImGui::GetContentRegionAvail().x),
                                    std::max(100.0f, ImGui::GetContentRegionAvail().y - 8.0f));
  const ImVec2 bottom_right(top_left.x + canvas_size.x, top_left.y + canvas_size.y);
  ImDrawList* draw_list = ImGui::GetWindowDrawList();
  draw_list->AddRectFilled(top_left, bottom_right, IM_COL32(19, 23, 31, 255), 6.0f);
  draw_list->AddRect(top_left, bottom_right, IM_COL32(70, 78, 96, 255), 6.0f, 0, 1.0f);

  if (values.size() >= 2) {
    double min_value = std::numeric_limits<double>::infinity();
    double max_value = -std::numeric_limits<double>::infinity();
    for (double value : values) {
      if (!std::isfinite(value)) {
        continue;
      }
      min_value = std::min(min_value, value);
      max_value = std::max(max_value, value);
    }
    if (std::isfinite(fixed_min)) {
      min_value = fixed_min;
    }
    if (std::isfinite(fixed_max)) {
      max_value = fixed_max;
    }
    if (!std::isfinite(min_value) || !std::isfinite(max_value)) {
      min_value = -1.0;
      max_value = 1.0;
    }
    if (std::abs(max_value - min_value) < 1e-6) {
      min_value -= 1.0;
      max_value += 1.0;
    }

    const float pad_x = 10.0f;
    const float pad_y = 10.0f;
    const float plot_w = canvas_size.x - 2.0f * pad_x;
    const float plot_h = canvas_size.y - 2.0f * pad_y;
    const float x_axis_y = bottom_right.y - pad_y;
    const float y_axis_x = top_left.x + pad_x;
    draw_list->AddLine(ImVec2(y_axis_x, top_left.y + pad_y),
                       ImVec2(y_axis_x, x_axis_y),
                       IM_COL32(110, 118, 138, 255), 1.0f);
    draw_list->AddLine(ImVec2(y_axis_x, x_axis_y),
                       ImVec2(bottom_right.x - pad_x, x_axis_y),
                       IM_COL32(110, 118, 138, 255), 1.0f);
    const float zero_t = static_cast<float>((0.0 - min_value) / (max_value - min_value));
    if (zero_t >= 0.0f && zero_t <= 1.0f) {
      const float zero_y = bottom_right.y - pad_y - zero_t * plot_h;
      draw_list->AddLine(ImVec2(top_left.x + pad_x, zero_y),
                         ImVec2(bottom_right.x - pad_x, zero_y),
                         IM_COL32(80, 88, 108, 255), 1.0f);
      draw_list->AddText(ImVec2(bottom_right.x - pad_x - 28.0f, zero_y - 14.0f),
                         IM_COL32(150, 158, 176, 255), "0");
    }

    std::vector<ImVec2> points;
    points.reserve(values.size());
    for (size_t i = 0; i < values.size(); ++i) {
      const float tx = static_cast<float>(i) / static_cast<float>(values.size() - 1);
      const float ty = static_cast<float>((values[i] - min_value) / (max_value - min_value));
      points.emplace_back(top_left.x + pad_x + tx * plot_w,
                          bottom_right.y - pad_y - ty * plot_h);
    }
    DrawPolyline(draw_list, points, color, 2.0f);

    char label[64];
    std::snprintf(label, sizeof(label), "max %.3f", max_value);
    draw_list->AddText(ImVec2(top_left.x + pad_x, top_left.y + 4.0f), IM_COL32(190, 198, 215, 255), label);
    std::snprintf(label, sizeof(label), "min %.3f", min_value);
    draw_list->AddText(ImVec2(top_left.x + pad_x, bottom_right.y - 18.0f), IM_COL32(190, 198, 215, 255), label);
    draw_list->AddText(ImVec2(bottom_right.x - 58.0f, bottom_right.y - 18.0f),
                       IM_COL32(190, 198, 215, 255), "step");
  } else {
    draw_list->AddText(ImVec2(top_left.x + 10.0f, top_left.y + 10.0f), IM_COL32(180, 180, 180, 255), "No data");
  }

  ImGui::Dummy(canvas_size);
  ImGui::EndChild();
}

void DrawLegendEntry(ImDrawList* draw_list,
                     const ImVec2& anchor,
                     ImU32 color,
                     const char* label,
                     bool enabled,
                     bool dashed = false) {
  const ImVec2 p0(anchor.x, anchor.y + 6.0f);
  const ImVec2 p1(anchor.x + 18.0f, anchor.y + 6.0f);
  if (enabled) {
    if (dashed) {
      draw_list->AddLine(p0, ImVec2(anchor.x + 8.0f, anchor.y + 6.0f), color, 2.0f);
      draw_list->AddLine(ImVec2(anchor.x + 10.0f, anchor.y + 6.0f), p1, color, 2.0f);
    } else {
      draw_list->AddLine(p0, p1, color, 2.0f);
    }
  } else {
    draw_list->AddLine(p0, p1, IM_COL32(90, 90, 90, 120), 1.0f);
  }
  draw_list->AddText(ImVec2(anchor.x + 24.0f, anchor.y), enabled ? IM_COL32(230, 230, 230, 255)
                                                                 : IM_COL32(120, 120, 120, 180),
                     label);
}

void DrawSceneView(FrontendState* state) {
  if (!state || !state->preview.valid) {
    return;
  }
  const ScenePreview& preview = state->preview;
  const PlanningOutput* output = state->has_result ? &state->result : nullptr;
  const ImVec2 canvas_pos = ImGui::GetCursorScreenPos();
  const ImVec2 avail = ImGui::GetContentRegionAvail();
  const ImVec2 canvas_size(std::max(200.0f, avail.x), std::max(220.0f, avail.y));
  const ImVec2 canvas_max(canvas_pos.x + canvas_size.x, canvas_pos.y + canvas_size.y);
  ImDrawList* draw_list = ImGui::GetWindowDrawList();

  draw_list->AddRectFilled(canvas_pos, canvas_max, IM_COL32(15, 18, 24, 255), 8.0f);
  draw_list->AddRect(canvas_pos, canvas_max, IM_COL32(65, 72, 88, 255), 8.0f, 0, 1.0f);

  const float pad = 18.0f;
  const ImVec2 plot_origin(canvas_pos.x + pad, canvas_pos.y + pad);
  const ImVec2 plot_size(canvas_size.x - 2.0f * pad, canvas_size.y - 2.0f * pad);
  const bool hovered = ImGui::IsMouseHoveringRect(canvas_pos, canvas_max, true);
  if (hovered) {
    ImGuiIO& io = ImGui::GetIO();
    if (std::abs(io.MouseWheel) > 1e-6f) {
      const double zoom_factor = io.MouseWheel > 0.0f ? 1.12 : 1.0 / 1.12;
      state->scene_zoom = std::clamp(state->scene_zoom * zoom_factor, 0.25, 20.0);
    }
    if (ImGui::IsMouseDragging(ImGuiMouseButton_Right)) {
      const ImVec2 delta = ImGui::GetIO().MouseDelta;
      const Bounds2D raw_bounds = ComputeSceneBounds(preview, output);
      const Bounds2D viewed_bounds =
          ApplySceneView(raw_bounds, state->scene_zoom, state->scene_pan_x, state->scene_pan_y);
      const double span_x = viewed_bounds.max_x - viewed_bounds.min_x;
      const double span_y = viewed_bounds.max_y - viewed_bounds.min_y;
      state->scene_pan_x -= static_cast<double>(delta.x) * span_x / std::max(1.0f, plot_size.x);
      state->scene_pan_y += static_cast<double>(delta.y) * span_y / std::max(1.0f, plot_size.y);
    }
  }
  const Bounds2D bounds =
      ApplySceneView(ComputeSceneBounds(preview, output), state->scene_zoom, state->scene_pan_x, state->scene_pan_y);

  const auto ref_a = ToCanvas(plot_origin, plot_size, bounds, bounds.min_x, 0.0);
  const auto ref_b = ToCanvas(plot_origin, plot_size, bounds, bounds.max_x, 0.0);
  if (state->show_reference_line) {
    draw_list->AddLine(ref_a, ref_b, IM_COL32(242, 198, 65, 220), 2.0f);
    draw_list->AddText(ImVec2(ref_a.x + 8.0f, ref_a.y - 18.0f), IM_COL32(242, 198, 65, 255), "reference line");
  }

  if (state->show_obstacle) {
    const auto obstacle = MakeObstaclePolyline(plot_origin,
                                               plot_size,
                                               bounds,
                                               preview.scenario_config.obstacle_center_x,
                                               preview.scenario_config.obstacle_center_y,
                                               preview.scenario_config.obstacle_radius);
    DrawPolyline(draw_list, obstacle, IM_COL32(230, 84, 84, 255), 2.5f);
  }

  if (state->show_initial_guess && preview.x.size() >= 2) {
    std::vector<ImVec2> initial_polyline;
    initial_polyline.reserve(preview.x.size());
    for (size_t i = 0; i < preview.x.size(); ++i) {
      initial_polyline.push_back(ToCanvas(plot_origin, plot_size, bounds, preview.x[i], preview.y[i]));
    }
    DrawDashedPolyline(draw_list, initial_polyline, IM_COL32(160, 160, 160, 220), 2.0f, 2);
  }

  if (output && state->show_optimized_trajectory && output->x.size() >= 2) {
    std::vector<ImVec2> trajectory_polyline;
    trajectory_polyline.reserve(output->x.size());
    for (size_t i = 0; i < output->x.size(); ++i) {
      trajectory_polyline.push_back(ToCanvas(plot_origin, plot_size, bounds, output->x[i], output->y[i]));
    }
    DrawPolyline(draw_list, trajectory_polyline, IM_COL32(72, 149, 239, 255), 3.6f);

    const auto vehicle_circle =
        my_al_ilqr::BuildSingleCircleVehicleApproximation(output->vehicle_config.body);
    const int sample_step = std::max(1, static_cast<int>(output->x.size() / 8));
    for (size_t i = 0; i < output->x.size(); i += static_cast<size_t>(sample_step)) {
      if (state->show_vehicle_outline) {
        const auto outline = MakeVehicleOutline(output->vehicle_config,
                                              plot_origin,
                                              plot_size,
                                              bounds,
                                              output->x[i],
                                                output->y[i],
                                                output->yaw[i]);
        FillClosedPolygon(draw_list, outline, IM_COL32(180, 100, 255, 72));
      }

      if (state->show_safety_circle) {
        const double c = std::cos(output->yaw[i]);
        const double s = std::sin(output->yaw[i]);
        const double circle_x =
            output->x[i] + c * vehicle_circle.center_x_body - s * vehicle_circle.center_y_body;
        const double circle_y =
            output->y[i] + s * vehicle_circle.center_x_body + c * vehicle_circle.center_y_body;
        const auto circle = MakeObstaclePolyline(plot_origin,
                                                 plot_size,
                                                 bounds,
                                                 circle_x,
                                                 circle_y,
                                                 vehicle_circle.radius,
                                                 32);
        DrawPolyline(draw_list, circle, IM_COL32(46, 204, 113, 128), 1.3f);
      }
    }
  }

  const auto start_pt = ToCanvas(plot_origin, plot_size, bounds, preview.x.front(), preview.y.front());
  draw_list->AddCircleFilled(start_pt, 4.0f, IM_COL32(255, 255, 255, 255));
  if (output && !output->x.empty()) {
    const auto end_pt = ToCanvas(plot_origin, plot_size, bounds, output->x.back(), output->y.back());
    draw_list->AddCircleFilled(end_pt, 4.0f, IM_COL32(72, 149, 239, 255));
  }

  draw_list->AddText(ImVec2(canvas_pos.x + 12.0f, canvas_pos.y + 10.0f),
                     IM_COL32(235, 235, 235, 255),
                     "Reference / initial guess / optimized trajectory");
  char view_label[96];
  std::snprintf(view_label, sizeof(view_label), "zoom %.2fx  RMB drag to pan", state->scene_zoom);
  draw_list->AddText(ImVec2(canvas_max.x - 180.0f, canvas_pos.y + 10.0f),
                     IM_COL32(190, 198, 215, 255), view_label);
  const ImVec2 legend_anchor(canvas_pos.x + 14.0f, canvas_max.y - 86.0f);
  draw_list->AddRectFilled(ImVec2(legend_anchor.x - 6.0f, legend_anchor.y - 6.0f),
                           ImVec2(legend_anchor.x + 170.0f, legend_anchor.y + 74.0f),
                           IM_COL32(10, 12, 18, 170), 6.0f);
  DrawLegendEntry(draw_list, legend_anchor, IM_COL32(242, 198, 65, 220), "Reference", state->show_reference_line);
  DrawLegendEntry(draw_list, ImVec2(legend_anchor.x, legend_anchor.y + 18.0f),
                  IM_COL32(230, 84, 84, 255), "Obstacle", state->show_obstacle);
  DrawLegendEntry(draw_list, ImVec2(legend_anchor.x, legend_anchor.y + 36.0f),
                  IM_COL32(160, 160, 160, 220), "Initial guess", state->show_initial_guess, true);
  DrawLegendEntry(draw_list, ImVec2(legend_anchor.x, legend_anchor.y + 54.0f),
                  IM_COL32(72, 149, 239, 255), "Optimized", state->show_optimized_trajectory);
  ImGui::Dummy(canvas_size);
}

void DrawSolverLogPanel(const FrontendState& state) {
  if (!state.has_result) {
    ImGui::TextWrapped("No solver log yet.");
    return;
  }

  const PlanningOutput& output = state.result;
  ImGui::Text("Planning time: %.1f ms", output.planning_time_ms);
  ImGui::Text("Converged: %s", output.converged ? "true" : "false");
  ImGui::Text("Horizon: %d", output.horizon);
  ImGui::Text("Outer iterations: %d", output.outer_iterations);
  ImGui::Text("Total inner iterations: %d", output.total_inner_iterations);
  ImGui::Text("Initial cost: %.4f", output.initial_cost);
  ImGui::Text("Final cost: %.4f", output.final_cost);
  ImGui::Text("Initial violation: %.6f", output.initial_violation);
  ImGui::Text("Final violation: %.6f", output.final_violation);

  ImGui::SeparatorText("Outer Loop");
  ImGui::BeginChild("outer_logs", ImVec2(0.0f, 180.0f), true);
  for (const auto& log : output.outer_logs) {
    ImGui::Text("it=%d inner=%d base=%.3f aug=%.3f viol=%.5f best=%.5f pen=%.1f %s",
                log.outer_iteration,
                log.inner_iterations,
                log.base_cost,
                log.augmented_cost,
                log.max_violation,
                log.best_violation_so_far,
                log.max_penalty,
                log.penalty_updated ? "penalty++" : "");
  }
  ImGui::EndChild();
}

ScenePreview BuildPreview(const VehicleBicycleConfig& vehicle_config,
                          const StaticObstacleScenarioConfig& scenario_config) {
  ScenePreview preview;
  preview.vehicle_config = vehicle_config;
  preview.scenario_config = scenario_config;
  try {
    const auto scenario = my_al_ilqr::CreateSingleStaticObstacleTestScenario(vehicle_config, scenario_config);
    const auto trajectory =
        scenario.base_problem->Rollout(scenario.initial_state, scenario.initial_controls);
    preview.x.resize(trajectory.Horizon() + 1);
    preview.y.resize(trajectory.Horizon() + 1);
    preview.yaw.resize(trajectory.Horizon() + 1);
    preview.speed.resize(trajectory.Horizon() + 1);
    for (int k = 0; k <= trajectory.Horizon(); ++k) {
      preview.x[k] = trajectory.State(k)(0);
      preview.y[k] = trajectory.State(k)(1);
      preview.yaw[k] = trajectory.State(k)(2);
      preview.speed[k] = trajectory.State(k)(3);
    }
    preview.valid = true;
    preview.status = "Preview updated";
  } catch (const std::exception& e) {
    preview.status = e.what();
  }
  return preview;
}

void RefreshPreviewIfNeeded(FrontendState* state) {
  if (!state || !state->preview_dirty) {
    return;
  }
  state->preview = BuildPreview(state->vehicle_config, state->scenario_config);
  state->preview_dirty = false;
}

PlanningOutput RunPlanningJob(const std::filesystem::path& config_path,
                              const VehicleBicycleConfig& vehicle_config,
                              const StaticObstacleScenarioConfig& scenario_config) {
  PlanningOutput output;
  output.vehicle_config = vehicle_config;
  output.scenario_config = scenario_config;
  try {
    const auto t_start = std::chrono::steady_clock::now();
    const auto scenario = my_al_ilqr::CreateSingleStaticObstacleTestScenario(vehicle_config, scenario_config);
    const auto initial_trajectory =
        scenario.base_problem->Rollout(scenario.initial_state, scenario.initial_controls);
    ALILQRSolver solver(*scenario.constrained_problem, scenario.solver_options);
    const auto result = solver.Solve(scenario.initial_state, scenario.initial_controls);
    const auto t_end = std::chrono::steady_clock::now();

    std::filesystem::create_directories("build");
    output.trajectory_csv_path = "build/imgui_frontend_trajectory.csv";
    my_al_ilqr::WriteTrajectoryCsv(output.trajectory_csv_path, result.trajectory);

    const int horizon = result.trajectory.Horizon();
    output.horizon = horizon;
    output.outer_logs = result.outer_logs;
    output.outer_iterations = static_cast<int>(result.outer_logs.size());
    output.total_inner_iterations = 0;
    for (const auto& log : result.outer_logs) {
      output.total_inner_iterations += log.inner_iterations;
      output.base_cost_history.push_back(log.base_cost);
      output.violation_history.push_back(log.max_violation);
      output.penalty_history.push_back(log.max_penalty);
    }
    output.planning_time_ms =
        std::chrono::duration<double, std::milli>(t_end - t_start).count();
    output.initial_cost = scenario.base_problem->TotalCost(initial_trajectory);
    output.final_cost = scenario.base_problem->TotalCost(result.trajectory);
    output.initial_violation = scenario.constrained_problem->MaxViolation(initial_trajectory);
    output.final_violation = scenario.constrained_problem->MaxViolation(result.trajectory);
    output.converged = result.converged;
    output.x.resize(horizon + 1);
    output.y.resize(horizon + 1);
    output.initial_x.resize(horizon + 1);
    output.initial_y.resize(horizon + 1);
    output.yaw.resize(horizon + 1);
    output.speed.resize(horizon + 1);
    output.accel.resize(horizon);
    output.kappa.resize(horizon);

    for (int k = 0; k <= horizon; ++k) {
      output.initial_x[k] = initial_trajectory.State(k)(0);
      output.initial_y[k] = initial_trajectory.State(k)(1);
      output.x[k] = result.trajectory.State(k)(0);
      output.y[k] = result.trajectory.State(k)(1);
      output.yaw[k] = result.trajectory.State(k)(2);
      output.speed[k] = result.trajectory.State(k)(3);
      if (k < horizon) {
        output.accel[k] = result.trajectory.Control(k)(0);
        output.kappa[k] = std::tan(result.trajectory.Control(k)(1)) / vehicle_config.model.wheelbase;
      }
    }

    std::ostringstream terminal_state;
    terminal_state << result.trajectory.State(horizon).transpose();

    output.success = true;
    output.terminal_state = terminal_state.str();
    output.best_violation = result.best_violation;
    std::ostringstream status;
    status << "Config: " << config_path.string() << "\n"
           << "Planning time: " << output.planning_time_ms << " ms\n"
           << "Outer iterations: " << output.outer_iterations
           << ", total inner iterations: " << output.total_inner_iterations << "\n"
           << "Initial cost -> final cost: " << output.initial_cost
           << " -> " << output.final_cost << "\n"
           << "Initial violation -> final violation: " << output.initial_violation
           << " -> " << output.final_violation << "\n"
           << "Best violation: " << result.best_violation << "\n"
           << "Trajectory CSV: " << output.trajectory_csv_path.string() << "\n"
           << "Pure ImGui frontend: visualization stays in the main window.";
    output.status = status.str();
  } catch (const std::exception& e) {
    output.status = e.what();
  }
  return output;
}

void SetDefaultState(FrontendState* state) {
  if (!state) {
    return;
  }
  const auto default_config = my_al_ilqr::DefaultVehicleBicycleConfigPath().string();
  std::snprintf(state->config_path.data(), state->config_path.size(), "%s", default_config.c_str());
  state->vehicle_config = my_al_ilqr::LoadVehicleBicycleConfig(default_config);
}

void LoadConfigFromPath(FrontendState* state) {
  if (!state) {
    return;
  }
  try {
    state->vehicle_config = my_al_ilqr::LoadVehicleBicycleConfig(state->config_path.data());
    std::ostringstream status;
    status << "Loaded config from " << state->config_path.data();
    state->status = status.str();
  } catch (const std::exception& e) {
    state->status = e.what();
  }
}

bool DrawVehicleConfigPanel(FrontendState* state) {
  if (!state) {
    return false;
  }
  bool changed = false;
  ImGui::SeparatorText("Vehicle Model");
  changed |= SliderInputDouble("Wheelbase", &state->vehicle_config.model.wheelbase, 1.5, 6.0, "%.2f");
  changed |= SliderInputDouble("Min Accel", &state->vehicle_config.model.min_acceleration, -5.0, 5.0, "%.2f");
  changed |= SliderInputDouble("Max Accel", &state->vehicle_config.model.max_acceleration, -5.0, 5.0, "%.2f");
  changed |= SliderInputDouble("Min Steer", &state->vehicle_config.model.min_steering, -1.2, 1.2, "%.2f");
  changed |= SliderInputDouble("Max Steer", &state->vehicle_config.model.max_steering, -1.2, 1.2, "%.2f");
  changed |= SliderInputDouble("Min Speed", &state->vehicle_config.model.min_speed, -2.0, 20.0, "%.2f");
  changed |= SliderInputDouble("Max Speed", &state->vehicle_config.model.max_speed, -2.0, 20.0, "%.2f");

  ImGui::SeparatorText("Vehicle Body");
  changed |= SliderInputDouble("Length", &state->vehicle_config.body.length, 2.0, 8.0, "%.2f");
  changed |= SliderInputDouble("Width", &state->vehicle_config.body.width, 1.0, 3.0, "%.2f");
  changed |= SliderInputDouble("Rear Axle To Rear", &state->vehicle_config.body.rear_axle_to_rear, 0.2, 3.0, "%.2f");
  return changed;
}

bool DrawScenarioPanel(FrontendState* state) {
  if (!state) {
    return false;
  }
  bool changed = false;
  ImGui::SeparatorText("Static Obstacle");
  changed |= SliderInputDouble("Obstacle X", &state->scenario_config.obstacle_center_x, 0.0, 20.0, "%.2f");
  changed |= SliderInputDouble("Obstacle Y", &state->scenario_config.obstacle_center_y, -5.0, 5.0, "%.2f");
  changed |= SliderInputDouble("Obstacle Radius", &state->scenario_config.obstacle_radius, 0.1, 5.0, "%.2f");

  ImGui::SeparatorText("Initial State");
  changed |= SliderInputDouble("Initial X", &state->scenario_config.initial_x, -5.0, 10.0, "%.2f");
  changed |= SliderInputDouble("Initial Y", &state->scenario_config.initial_y, -4.0, 4.0, "%.2f");
  changed |= SliderInputDouble("Initial Yaw", &state->scenario_config.initial_yaw, -3.2, 3.2, "%.2f");
  changed |= SliderInputDouble("Initial Speed", &state->scenario_config.initial_speed, 0.0, 10.0, "%.2f");
  return changed;
}

}  // namespace

int main() {
  FrontendState state;
  SetDefaultState(&state);
  RefreshPreviewIfNeeded(&state);

  if (!glfwInit()) {
    throw std::runtime_error("Failed to initialize GLFW.");
  }

  const char* glsl_version = "#version 130";
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);

  GLFWwindow* window = glfwCreateWindow(1600, 960, "MY-AL-iLQR ImGui Frontend", nullptr, nullptr);
  if (!window) {
    glfwTerminate();
    throw std::runtime_error("Failed to create GLFW window.");
  }

  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO& io = ImGui::GetIO();
  io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
  ImGui::StyleColorsDark();

  ImGui_ImplGlfw_InitForOpenGL(window, true);
  ImGui_ImplOpenGL3_Init(glsl_version);

  std::future<PlanningOutput> planning_future;

  while (!glfwWindowShouldClose(window)) {
    glfwPollEvents();

    if (state.planning &&
        planning_future.valid() &&
        planning_future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
      PlanningOutput output = planning_future.get();
      state.planning = false;
      state.status = output.status;
      state.last_best_violation = output.best_violation;
      state.terminal_state = output.terminal_state;
      state.last_csv_path = output.trajectory_csv_path;
      state.has_result = output.success;
      if (output.success) {
        state.result = std::move(output);
        state.scene_zoom = 1.0;
        state.scene_pan_x = 0.0;
        state.scene_pan_y = 0.0;
      }
    }

    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
    RefreshPreviewIfNeeded(&state);

    const ImGuiViewport* viewport = ImGui::GetMainViewport();
    const float controls_width = 340.0f;
    const float content_padding = 16.0f;
    const float right_x = viewport->Pos.x + controls_width + content_padding;
    const float right_width = viewport->Size.x - controls_width - 2.0f * content_padding;
    const float top_height = viewport->Size.y * 0.60f;
    const float bottom_height = viewport->Size.y - top_height - 3.0f * content_padding;
    const float bottom_third_width = (right_width - 2.0f * content_padding) / 3.0f;

    ImGui::SetNextWindowPos(ImVec2(viewport->Pos.x + content_padding, viewport->Pos.y + content_padding),
                            ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(controls_width - content_padding, viewport->Size.y - 2.0f * content_padding),
                             ImGuiCond_Always);
    ImGui::Begin("Controls");
    ImGui::InputText("Config Path", state.config_path.data(), state.config_path.size());
    if (ImGui::Button("Load Config")) {
      LoadConfigFromPath(&state);
    }
    ImGui::SameLine();
    const bool disable_plan_button = state.planning;
    if (disable_plan_button) {
      ImGui::BeginDisabled();
    }
    if (ImGui::Button("Plan")) {
      state.status = "Planning...";
      state.planning = true;
      const std::filesystem::path config_path = state.config_path.data();
      const VehicleBicycleConfig vehicle_config = state.vehicle_config;
      const StaticObstacleScenarioConfig scenario_config = state.scenario_config;
      planning_future = std::async(std::launch::async,
                                   [config_path, vehicle_config, scenario_config]() {
                                     return RunPlanningJob(config_path, vehicle_config, scenario_config);
                                   });
    }
    if (disable_plan_button) {
      ImGui::EndDisabled();
    }

    ImGui::Separator();
    ImGui::TextWrapped("Status: %s", state.status.c_str());
    ImGui::BulletText("Best violation: %.6f", state.last_best_violation);
    ImGui::BulletText("Trajectory CSV: %s",
                      state.last_csv_path.empty() ? "not written" : state.last_csv_path.string().c_str());
    ImGui::BulletText("Terminal state: %s",
                      state.terminal_state.empty() ? "not available" : state.terminal_state.c_str());
    ImGui::SeparatorText("Solver Logs");
    DrawSolverLogPanel(state);

    bool params_changed = false;
    params_changed |= DrawVehicleConfigPanel(&state);
    params_changed |= DrawScenarioPanel(&state);
    if (params_changed) {
      state.preview_dirty = true;
      state.has_result = false;
      state.scene_zoom = 1.0;
      state.scene_pan_x = 0.0;
      state.scene_pan_y = 0.0;
      RefreshPreviewIfNeeded(&state);
    }
    ImGui::End();

    ImGui::SetNextWindowPos(ImVec2(right_x, viewport->Pos.y + content_padding), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(right_width, top_height - content_padding), ImGuiCond_Always);
    ImGui::Begin("Visualization");
    ImGui::BeginGroup();
    if (ImGui::Button("Reset View")) {
      state.scene_zoom = 1.0;
      state.scene_pan_x = 0.0;
      state.scene_pan_y = 0.0;
    }
    ImGui::SameLine();
    ImGui::TextUnformatted("Mouse wheel: zoom   Right-drag: pan");
    ImGui::EndGroup();
    ImGui::SameLine(ImGui::GetWindowWidth() - 270.0f);
    ImGui::BeginGroup();
    ImGui::Checkbox("Ref", &state.show_reference_line);
    ImGui::SameLine();
    ImGui::Checkbox("Obstacle", &state.show_obstacle);
    ImGui::SameLine();
    ImGui::Checkbox("Init", &state.show_initial_guess);
    ImGui::SameLine();
    ImGui::Checkbox("Opt", &state.show_optimized_trajectory);
    ImGui::Checkbox("Rect", &state.show_vehicle_outline);
    ImGui::SameLine();
    ImGui::Checkbox("Circle", &state.show_safety_circle);
    ImGui::EndGroup();
    ImGui::Separator();
    if (state.has_result) {
      DrawSceneView(&state);
    } else if (state.preview.valid) {
      DrawSceneView(&state);
    } else {
      ImGui::TextWrapped("No preview available.");
    }
    ImGui::End();

    ImGui::SetNextWindowPos(ImVec2(right_x, viewport->Pos.y + top_height + content_padding), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(right_width, bottom_height), ImGuiCond_Always);
    ImGui::Begin("Debug");
    ImGui::Columns(3, "debug_columns_top", false);
    DrawSignalPlot("Speed", state.has_result ? state.result.speed : std::vector<double>{},
                   IM_COL32(74, 201, 120, 255),
                   state.vehicle_config.model.min_speed,
                   state.vehicle_config.model.max_speed);
    ImGui::NextColumn();
    DrawSignalPlot("Kappa", state.has_result ? state.result.kappa : std::vector<double>{},
                   IM_COL32(83, 131, 255, 255));
    ImGui::NextColumn();
    DrawSignalPlot("Acceleration", state.has_result ? state.result.accel : std::vector<double>{},
                   IM_COL32(255, 177, 66, 255),
                   state.vehicle_config.model.min_acceleration,
                   state.vehicle_config.model.max_acceleration);
    ImGui::Columns(1);
    ImGui::Separator();
    ImGui::Columns(3, "debug_columns_bottom", false);
    DrawSignalPlot("Base Cost",
                   state.has_result ? state.result.base_cost_history : std::vector<double>{},
                   IM_COL32(255, 120, 120, 255));
    ImGui::NextColumn();
    DrawSignalPlot("Violation",
                   state.has_result ? state.result.violation_history : std::vector<double>{},
                   IM_COL32(255, 210, 90, 255),
                   0.0);
    ImGui::NextColumn();
    DrawSignalPlot("Penalty",
                   state.has_result ? state.result.penalty_history : std::vector<double>{},
                   IM_COL32(180, 120, 255, 255),
                   0.0);
    ImGui::Columns(1);
    ImGui::End();

    ImGui::Render();
    int display_w = 0;
    int display_h = 0;
    glfwGetFramebufferSize(window, &display_w, &display_h);
    glViewport(0, 0, display_w, display_h);
    glClearColor(0.08f, 0.09f, 0.12f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    glfwSwapBuffers(window);
  }

  if (planning_future.valid()) {
    planning_future.wait();
  }

  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();
  glfwDestroyWindow(window);
  glfwTerminate();
  return 0;
}

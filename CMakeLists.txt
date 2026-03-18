cmake_minimum_required(VERSION 3.16)
project(MY_AL_iLQR LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(FETCHCONTENT_UPDATES_DISCONNECTED ON)
option(MY_AL_ILQR_ENABLE_IMGUI_FRONTEND
       "Build the experimental Dear ImGui frontend."
       ON)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
include(FetchContent)

if(MY_AL_ILQR_ENABLE_IMGUI_FRONTEND)
  find_package(OpenGL REQUIRED)

  set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
  set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
  set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)
  set(GLFW_LOCAL_DIR "${CMAKE_BINARY_DIR}/_deps/glfw-src")
  if(EXISTS "${GLFW_LOCAL_DIR}/CMakeLists.txt")
    add_subdirectory("${GLFW_LOCAL_DIR}" "${CMAKE_BINARY_DIR}/_deps/glfw-build")
  else()
    FetchContent_Declare(
      glfw
      URL https://github.com/glfw/glfw/archive/refs/tags/3.4.tar.gz
    )
    FetchContent_MakeAvailable(glfw)
  endif()

  set(IMGUI_LOCAL_DIR "${CMAKE_BINARY_DIR}/_deps/imgui-src")
  if(EXISTS "${IMGUI_LOCAL_DIR}/imgui.cpp")
    set(imgui_SOURCE_DIR "${IMGUI_LOCAL_DIR}")
  else()
    FetchContent_Declare(
      imgui
      URL https://github.com/ocornut/imgui/archive/refs/tags/v1.91.8.tar.gz
    )
    FetchContent_GetProperties(imgui)
    if(NOT imgui_POPULATED)
      FetchContent_Populate(imgui)
    endif()
  endif()

  add_library(imgui STATIC
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_demo.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
  )

  target_include_directories(imgui
    PUBLIC
    ${imgui_SOURCE_DIR}
    ${imgui_SOURCE_DIR}/backends
  )

  target_link_libraries(imgui
    PUBLIC
    glfw
    OpenGL::GL
  )
endif()

add_library(my_al_ilqr_core
  src/core/trajectory.cpp
  src/autodrive/straight_reference_line.cpp
  src/autodrive/speed_limit_constraint.cpp
  src/autodrive/vehicle_bicycle_config.cpp
  src/autodrive/lane_tracking_cost.cpp
  src/autodrive/circular_obstacle_constraint.cpp
  src/autodrive/demo_scenario.cpp
  src/dynamics/linear_point_mass_model.cpp
  src/dynamics/unicycle_model.cpp
  src/dynamics/kinematic_bicycle_model.cpp
  src/cost/quadratic_cost.cpp
  src/constraints/constraint_function.cpp
  src/constraints/control_box_constraint.cpp
  src/constraints/terminal_goal_constraint.cpp
  src/al/al_ilqr_solver.cpp
  src/al/augmented_lagrangian_cost.cpp
  src/problems/optimal_control_problem.cpp
  src/problems/constrained_optimal_control_problem.cpp
  src/lqr/finite_horizon_lqr_solver.cpp
  src/ilqr/ilqr_solver.cpp
  src/visualization/al_ilqr_log_writer.cpp
  src/visualization/obstacle_prediction_writer.cpp
  src/visualization/trajectory_writer.cpp
)

target_include_directories(my_al_ilqr_core
  PUBLIC
  ${PROJECT_SOURCE_DIR}/include
)

target_compile_definitions(my_al_ilqr_core
  PUBLIC
  MY_AL_ILQR_SOURCE_DIR="${PROJECT_SOURCE_DIR}"
)

target_link_libraries(my_al_ilqr_core
  PUBLIC
  Eigen3::Eigen
)

add_executable(phase0_linear_rollout
  examples/phase0_linear_rollout.cpp
)

target_link_libraries(phase0_linear_rollout
  PRIVATE
  my_al_ilqr_core
)

add_executable(phase1_unicycle_rollout
  examples/phase1_unicycle_rollout.cpp
)

target_link_libraries(phase1_unicycle_rollout
  PRIVATE
  my_al_ilqr_core
)

add_executable(phase1_bicycle_rollout
  examples/phase1_bicycle_rollout.cpp
)

target_link_libraries(phase1_bicycle_rollout
  PRIVATE
  my_al_ilqr_core
)

add_executable(phase2_linear_lqr_demo
  examples/phase2_linear_lqr_demo.cpp
)

target_link_libraries(phase2_linear_lqr_demo
  PRIVATE
  my_al_ilqr_core
)

add_executable(phase3_unicycle_ilqr_demo
  examples/phase3_unicycle_ilqr_demo.cpp
)

target_link_libraries(phase3_unicycle_ilqr_demo
  PRIVATE
  my_al_ilqr_core
)

add_executable(phase4_unicycle_ilqr_stable_demo
  examples/phase4_unicycle_ilqr_stable_demo.cpp
)

target_link_libraries(phase4_unicycle_ilqr_stable_demo
  PRIVATE
  my_al_ilqr_core
)

add_executable(phase5_constraint_evaluation_demo
  examples/phase5_constraint_evaluation_demo.cpp
)

target_link_libraries(phase5_constraint_evaluation_demo
  PRIVATE
  my_al_ilqr_core
)

add_executable(phase6_augmented_lagrangian_demo
  examples/phase6_augmented_lagrangian_demo.cpp
)

target_link_libraries(phase6_augmented_lagrangian_demo
  PRIVATE
  my_al_ilqr_core
)

add_executable(phase7_al_ilqr_demo
  examples/phase7_al_ilqr_demo.cpp
)

target_link_libraries(phase7_al_ilqr_demo
  PRIVATE
  my_al_ilqr_core
)

add_executable(phase8_autodrive_demo
  examples/phase8_autodrive_demo.cpp
)

target_link_libraries(phase8_autodrive_demo
  PRIVATE
  my_al_ilqr_core
)

add_executable(phase9_autodrive_visualization_demo
  examples/phase9_autodrive_visualization_demo.cpp
)

target_link_libraries(phase9_autodrive_visualization_demo
  PRIVATE
  my_al_ilqr_core
)

add_executable(phase10_numerical_enhancement_demo
  examples/phase10_numerical_enhancement_demo.cpp
)

target_link_libraries(phase10_numerical_enhancement_demo
  PRIVATE
  my_al_ilqr_core
)

add_executable(phase11_dynamic_obstacle_demo
  examples/phase11_dynamic_obstacle_demo.cpp
)

target_link_libraries(phase11_dynamic_obstacle_demo
  PRIVATE
  my_al_ilqr_core
)

add_executable(phase12_dynamic_obstacle_speed_sweep_demo
  examples/phase12_dynamic_obstacle_speed_sweep_demo.cpp
)

target_link_libraries(phase12_dynamic_obstacle_speed_sweep_demo
  PRIVATE
  my_al_ilqr_core
)

if(MY_AL_ILQR_ENABLE_IMGUI_FRONTEND)
  add_executable(imgui_frontend
    apps/rerun_imgui_frontend.cpp
  )

  target_link_libraries(imgui_frontend
    PRIVATE
    my_al_ilqr_core
    imgui
  )
endif()

enable_testing()

add_executable(phase0_smoke_test
  tests/phase0_smoke_test.cpp
)

target_link_libraries(phase0_smoke_test
  PRIVATE
  my_al_ilqr_core
)

add_test(NAME phase0_smoke_test COMMAND phase0_smoke_test)

add_executable(phase1_unicycle_test
  tests/phase1_unicycle_test.cpp
)

target_link_libraries(phase1_unicycle_test
  PRIVATE
  my_al_ilqr_core
)

add_test(NAME phase1_unicycle_test COMMAND phase1_unicycle_test)

add_executable(phase1_bicycle_test
  tests/phase1_bicycle_test.cpp
)

target_link_libraries(phase1_bicycle_test
  PRIVATE
  my_al_ilqr_core
)

add_test(NAME phase1_bicycle_test COMMAND phase1_bicycle_test)

add_executable(phase2_lqr_test
  tests/phase2_lqr_test.cpp
)

target_link_libraries(phase2_lqr_test
  PRIVATE
  my_al_ilqr_core
)

add_test(NAME phase2_lqr_test COMMAND phase2_lqr_test)

add_executable(phase3_ilqr_test
  tests/phase3_ilqr_test.cpp
)

target_link_libraries(phase3_ilqr_test
  PRIVATE
  my_al_ilqr_core
)

add_test(NAME phase3_ilqr_test COMMAND phase3_ilqr_test)

add_executable(phase4_ilqr_stability_test
  tests/phase4_ilqr_stability_test.cpp
)

target_link_libraries(phase4_ilqr_stability_test
  PRIVATE
  my_al_ilqr_core
)

add_test(NAME phase4_ilqr_stability_test COMMAND phase4_ilqr_stability_test)

add_executable(phase5_constraints_test
  tests/phase5_constraints_test.cpp
)

target_link_libraries(phase5_constraints_test
  PRIVATE
  my_al_ilqr_core
)

add_test(NAME phase5_constraints_test COMMAND phase5_constraints_test)

add_executable(phase6_al_cost_test
  tests/phase6_al_cost_test.cpp
)

target_link_libraries(phase6_al_cost_test
  PRIVATE
  my_al_ilqr_core
)

add_test(NAME phase6_al_cost_test COMMAND phase6_al_cost_test)

add_executable(phase7_al_ilqr_test
  tests/phase7_al_ilqr_test.cpp
)

target_link_libraries(phase7_al_ilqr_test
  PRIVATE
  my_al_ilqr_core
)

add_test(NAME phase7_al_ilqr_test COMMAND phase7_al_ilqr_test)

add_executable(phase8_autodrive_test
  tests/phase8_autodrive_test.cpp
)

target_link_libraries(phase8_autodrive_test
  PRIVATE
  my_al_ilqr_core
)

add_test(NAME phase8_autodrive_test COMMAND phase8_autodrive_test)

add_executable(phase9_visualization_export_test
  tests/phase9_visualization_export_test.cpp
)

target_link_libraries(phase9_visualization_export_test
  PRIVATE
  my_al_ilqr_core
)

add_test(NAME phase9_visualization_export_test COMMAND phase9_visualization_export_test)

add_executable(phase10_numerical_enhancement_test
  tests/phase10_numerical_enhancement_test.cpp
)

target_link_libraries(phase10_numerical_enhancement_test
  PRIVATE
  my_al_ilqr_core
)

add_test(NAME phase10_numerical_enhancement_test COMMAND phase10_numerical_enhancement_test)

add_executable(phase11_dynamic_obstacle_test
  tests/phase11_dynamic_obstacle_test.cpp
)

target_link_libraries(phase11_dynamic_obstacle_test
  PRIVATE
  my_al_ilqr_core
)

add_test(NAME phase11_dynamic_obstacle_test COMMAND phase11_dynamic_obstacle_test)

add_executable(phase12_dynamic_obstacle_speed_sweep_test
  tests/phase12_dynamic_obstacle_speed_sweep_test.cpp
)

target_link_libraries(phase12_dynamic_obstacle_speed_sweep_test
  PRIVATE
  my_al_ilqr_core
)

add_test(NAME phase12_dynamic_obstacle_speed_sweep_test COMMAND phase12_dynamic_obstacle_speed_sweep_test)

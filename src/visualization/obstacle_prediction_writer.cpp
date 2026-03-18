#include "visualization/obstacle_prediction_writer.hpp"

#include <filesystem>
#include <fstream>
#include <stdexcept>

namespace my_al_ilqr {

void WriteObstaclePredictionCsv(const std::filesystem::path& path,
                                const std::vector<Vector>& obstacle_centers,
                                double radius) {
  if (!path.parent_path().empty()) {
    std::filesystem::create_directories(path.parent_path());
  }
  std::ofstream out(path);
  if (!out.is_open()) {
    throw std::runtime_error("Failed to open obstacle prediction CSV file for writing.");
  }

  out << "k,cx,cy,radius\n";
  for (int k = 0; k < static_cast<int>(obstacle_centers.size()); ++k) {
    if (obstacle_centers[k].size() != 2) {
      throw std::invalid_argument("Obstacle prediction centers must be 2D vectors.");
    }
    out << k << "," << obstacle_centers[k](0) << "," << obstacle_centers[k](1) << "," << radius
        << "\n";
  }
}

}  // namespace my_al_ilqr

#include "visualization/trajectory_writer.hpp"

#include <filesystem>
#include <fstream>
#include <stdexcept>

namespace my_al_ilqr {

void WriteTrajectoryCsv(const std::filesystem::path& path, const Trajectory& trajectory) {
  if (!path.parent_path().empty()) {
    std::filesystem::create_directories(path.parent_path());
  }
  std::ofstream out(path);
  if (!out.is_open()) {
    throw std::runtime_error("Failed to open trajectory CSV file for writing.");
  }

  out << "k";
  for (int i = 0; i < trajectory.StateDim(); ++i) {
    out << ",x" << i;
  }
  for (int i = 0; i < trajectory.ControlDim(); ++i) {
    out << ",u" << i;
  }
  out << "\n";

  for (int k = 0; k <= trajectory.Horizon(); ++k) {
    out << k;
    for (int i = 0; i < trajectory.StateDim(); ++i) {
      out << "," << trajectory.State(k)(i);
    }
    if (k < trajectory.Horizon()) {
      for (int i = 0; i < trajectory.ControlDim(); ++i) {
        out << "," << trajectory.Control(k)(i);
      }
    } else {
      for (int i = 0; i < trajectory.ControlDim(); ++i) {
        out << ",";
      }
    }
    out << "\n";
  }
}

}  // namespace my_al_ilqr

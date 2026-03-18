#include "visualization/al_ilqr_log_writer.hpp"

#include <filesystem>
#include <fstream>
#include <stdexcept>

namespace my_al_ilqr {

void WriteALILQROuterLogCsv(const std::filesystem::path& path,
                            const std::vector<ALILQROuterIterationLog>& outer_log) {
  if (!path.parent_path().empty()) {
    std::filesystem::create_directories(path.parent_path());
  }
  std::ofstream out(path);
  if (!out.is_open()) {
    throw std::runtime_error("Failed to open AL-iLQR outer log CSV file for writing.");
  }

  out << "outer_iteration,inner_iterations,base_cost,augmented_cost,max_violation,"
         "best_violation_so_far,max_penalty,penalty_updated\n";
  for (const auto& log : outer_log) {
    out << log.outer_iteration << "," << log.inner_iterations << "," << log.base_cost << ","
        << log.augmented_cost << "," << log.max_violation << "," << log.best_violation_so_far
        << "," << log.max_penalty << "," << (log.penalty_updated ? 1 : 0) << "\n";
  }
}

}  // namespace my_al_ilqr

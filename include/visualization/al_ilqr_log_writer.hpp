#pragma once

#include <filesystem>
#include <vector>

#include "al/al_ilqr_solver.hpp"

namespace my_al_ilqr {

void WriteALILQROuterLogCsv(const std::filesystem::path& path,
                            const std::vector<ALILQROuterIterationLog>& outer_log);

}  // namespace my_al_ilqr

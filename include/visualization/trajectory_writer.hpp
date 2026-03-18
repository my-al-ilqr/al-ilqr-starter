#pragma once

#include <filesystem>

#include "core/trajectory.hpp"

namespace my_al_ilqr {

void WriteTrajectoryCsv(const std::filesystem::path& path, const Trajectory& trajectory);

}  // namespace my_al_ilqr

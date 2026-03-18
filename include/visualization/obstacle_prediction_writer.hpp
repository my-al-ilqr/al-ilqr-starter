#pragma once

#include <filesystem>
#include <vector>

#include "core/types.hpp"

namespace my_al_ilqr {

void WriteObstaclePredictionCsv(const std::filesystem::path& path,
                                const std::vector<Vector>& obstacle_centers,
                                double radius);

}  // namespace my_al_ilqr

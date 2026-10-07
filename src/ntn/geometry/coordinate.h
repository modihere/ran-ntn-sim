#pragma once

#include <cmath>

namespace ntn::geometry {

struct Position {
    double x_m{0.0};
    double y_m{0.0};
    double z_m{0.0};

    // Helper for Euclidean distance between two positions
    double distance_to(const Position& other) const {
        return std::sqrt(std::pow(x_m - other.x_m, 2) + 
                         std::pow(y_m - other.y_m, 2) + 
                         std::pow(z_m - other.z_m, 2));
    }
};

struct Velocity {
    double vx_mps{0.0};
    double vy_mps{0.0};
    double vz_mps{0.0};
};

} // namespace ntn::geometry


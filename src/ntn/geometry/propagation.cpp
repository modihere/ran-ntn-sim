#include "propagation.h"
#include <cmath>

namespace ntn::geometry {

double PropagationCalculator::calculate_slant_range(const Position& sat, const Position& ue) {
    return sat.distance_to(ue);
}

common::Time_ns PropagationCalculator::calculate_one_way_delay(double slant_range_m) {
    // delay_s = distance / c
    // delay_ns = delay_s * 1e9
    double delay_s = slant_range_m / SPEED_OF_LIGHT_MPS;
    return static_cast<common::Time_ns>(delay_s * 1e9);
}

double PropagationCalculator::calculate_doppler_shift(const Position& sat, const Velocity& sat_v, 
                                                      const Position& ue, const Velocity& ue_v, 
                                                      double carrier_freq_hz) {
    
    double range = calculate_slant_range(sat, ue);
    if (range == 0.0) return 0.0;

    // Relative position vector from UE to Satellite
    double rx = sat.x_m - ue.x_m;
    double ry = sat.y_m - ue.y_m;
    double rz = sat.z_m - ue.z_m;

    // Relative velocity vector (Sat velocity - UE velocity)
    double vx = sat_v.vx_mps - ue_v.vx_mps;
    double vy = sat_v.vy_mps - ue_v.vy_mps;
    double vz = sat_v.vz_mps - ue_v.vz_mps;

    // Dot product (Vrel . Rrel)
    double dot_product = (vx * rx) + (vy * ry) + (vz * rz);

    // Range rate (m/s)
    double range_rate = dot_product / range;

    // Doppler shift: fd = - (fc / c) * range_rate
    return -(carrier_freq_hz / SPEED_OF_LIGHT_MPS) * range_rate;
}

double PropagationCalculator::calculate_elevation_angle_degrees(const Position& sat, const Position& ue) {
    // In our simplified flat-earth model, elevation is the angle above the XY plane.
    // distance projected on XY plane:
    double dx = sat.x_m - ue.x_m;
    double dy = sat.y_m - ue.y_m;
    double ground_dist = std::sqrt(dx*dx + dy*dy);
    
    double dz = sat.z_m - ue.z_m;

    // atan2(y, x) -> atan2(dz, ground_dist) gives radians
    double elevation_rad = std::atan2(dz, ground_dist);
    
    constexpr double pi = 3.14159265358979323846;
    return elevation_rad * (180.0 / pi);
}

} // namespace ntn::geometry

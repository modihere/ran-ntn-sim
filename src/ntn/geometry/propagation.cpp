/**
 * @file propagation.cpp
 * @brief Implementation of RF Propagation and geometry calculations for NTN.
 *
 * Implements line-of-sight distance, propagation delay, Doppler shift calculation,
 * and ground elevation angles.
 */

#include "propagation.h"
#include <cmath>

namespace ntn::geometry {

/**
 * @brief Computes slant range between satellite and UE positions.
 * @param sat Satellite Cartesian position.
 * @param ue UE Cartesian position.
 * @return double Slant range in meters.
 */
double PropagationCalculator::calculate_slant_range(const Position& sat, const Position& ue) {
    return sat.distance_to(ue);
}

/**
 * @brief Computes one-way RF propagation delay in nanoseconds.
 * @param slant_range_m Distance in meters.
 * @return common::Time_ns Delay in nanoseconds.
 */
common::Time_ns PropagationCalculator::calculate_one_way_delay(double slant_range_m) {
    double delay_s = slant_range_m / SPEED_OF_LIGHT_MPS;
    return static_cast<common::Time_ns>(delay_s * 1e9);
}

/**
 * @brief Computes Doppler shift in Hz.
 * @param sat Satellite position.
 * @param sat_v Satellite velocity vector.
 * @param ue UE position.
 * @param ue_v UE velocity vector.
 * @param carrier_freq_hz Carrier frequency in Hz.
 * @return double Doppler shift in Hz.
 */
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

/**
 * @brief Computes elevation angle in degrees above local horizontal plane.
 * @param sat Satellite position.
 * @param ue UE position.
 * @return double Elevation angle in degrees.
 */
double PropagationCalculator::calculate_elevation_angle_degrees(const Position& sat, const Position& ue) {
    double dx = sat.x_m - ue.x_m;
    double dy = sat.y_m - ue.y_m;
    double ground_dist = std::sqrt(dx*dx + dy*dy);
    double dz = sat.z_m - ue.z_m;

    double elevation_rad = std::atan2(dz, ground_dist);
    constexpr double pi = 3.14159265358979323846;
    return elevation_rad * (180.0 / pi);
}

} // namespace ntn::geometry

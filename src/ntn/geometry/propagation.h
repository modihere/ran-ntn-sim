/**
 * @file propagation.h
 * @brief RF Propagation calculator for Non-Terrestrial Network (NTN) service links.
 *
 * Implements physics calculations for line-of-sight slant range, one-way propagation delay,
 * Doppler shift relative to satellite velocity, and ground elevation angle.
 */

#pragma once

#include "coordinate.h"
#include "../../common/simulator.h"

namespace ntn::geometry {

/**
 * @class PropagationCalculator
 * @brief Utility class providing static methods for satellite link geometry and delay calculations.
 */
class PropagationCalculator {
public:
    /// Speed of light in vacuum (m/s) per 3GPP specifications
    static constexpr double SPEED_OF_LIGHT_MPS = 299792458.0;

    /**
     * @brief Computes Euclidean slant range distance between satellite and UE.
     * @param sat Satellite Cartesian position.
     * @param ue UE Cartesian position.
     * @return double Slant range in meters.
     */
    static double calculate_slant_range(const Position& sat, const Position& ue);

    /**
     * @brief Computes one-way RF propagation delay in nanoseconds from distance.
     * @param slant_range_m Line-of-sight distance in meters.
     * @return common::Time_ns One-way propagation delay in nanoseconds.
     */
    static common::Time_ns calculate_one_way_delay(double slant_range_m);

    /**
     * @brief Computes Doppler frequency shift in Hz.
     * @param sat Satellite position.
     * @param sat_v Satellite velocity vector.
     * @param ue UE position.
     * @param ue_v UE velocity vector.
     * @param carrier_freq_hz RF carrier frequency in Hz.
     * @return double Doppler shift in Hz.
     */
    static double calculate_doppler_shift(const Position& sat, const Velocity& sat_v, 
                                          const Position& ue, const Velocity& ue_v, 
                                          double carrier_freq_hz);

    /**
     * @brief Computes ground elevation angle in degrees relative to the horizon.
     * @param sat Satellite position.
     * @param ue UE position.
     * @return double Elevation angle in degrees [0.0, 90.0].
     */
    static double calculate_elevation_angle_degrees(const Position& sat, const Position& ue);
};

} // namespace ntn::geometry

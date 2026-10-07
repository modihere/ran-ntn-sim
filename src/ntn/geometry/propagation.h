#pragma once

#include "coordinate.h"
#include "../../common/simulator.h"

namespace ntn::geometry {

class PropagationCalculator {
public:
    // Speed of light in vacuum (m/s)
    static constexpr double SPEED_OF_LIGHT_MPS = 299792458.0;

    // Calculates the line-of-sight distance between two positions
    static double calculate_slant_range(const Position& sat, const Position& ue);

    // Calculates one-way propagation delay in nanoseconds from distance in meters
    static common::Time_ns calculate_one_way_delay(double slant_range_m);

    // Calculates the Doppler shift in Hz
    // fd = -(fc / c) * (Vrel . Rrel) / |Rrel|
    static double calculate_doppler_shift(const Position& sat, const Velocity& sat_v, 
                                          const Position& ue, const Velocity& ue_v, 
                                          double carrier_freq_hz);

    // Calculates elevation angle of the satellite relative to the UE in degrees.
    // Assumes simplified flat earth where UE is at Z=0.
    static double calculate_elevation_angle_degrees(const Position& sat, const Position& ue);
};

} // namespace ntn::geometry


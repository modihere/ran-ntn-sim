/**
 * @file orbit_propagator.h
 * @brief Linear Orbit Propagator for satellite trajectory modeling in NTN.
 *
 * Implements simplified constant-velocity linear satellite orbit propagation
 * in Cartesian ECEF/ground coordinates for L2/L3 protocol emulation.
 */

#pragma once

#include "coordinate.h"
#include "../../common/simulator.h"

namespace ntn::geometry {

/**
 * @class LinearOrbitPropagator
 * @brief Propagates satellite position over time given initial coordinates and velocity.
 */
class LinearOrbitPropagator {
public:
    /**
     * @brief Constructs the linear orbit propagator.
     * @param initial_pos Initial Cartesian position [m] at initial_time.
     * @param velocity Constant Cartesian velocity vector [m/s].
     * @param initial_time Epoch timestamp in nanoseconds corresponding to initial_pos.
     */
    LinearOrbitPropagator(Position initial_pos, Velocity velocity, common::Time_ns initial_time);

    /**
     * @brief Calculates satellite coordinates at a given simulation timestamp.
     * @param current_time Simulation time in nanoseconds.
     * @return Position Cartesian position vector at current_time.
     */
    Position get_position_at(common::Time_ns current_time) const;

    /**
     * @brief Returns configured constant orbital velocity vector.
     * @return Velocity Cartesian velocity in m/s.
     */
    Velocity get_velocity() const;

private:
    Position initial_pos_;
    Velocity velocity_;
    common::Time_ns initial_time_;
};

} // namespace ntn::geometry

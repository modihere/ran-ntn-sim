/**
 * @file orbit_propagator.cpp
 * @brief Implementation of constant-velocity linear orbit propagator.
 *
 * Linearly propagates Cartesian position components based on elapsed nanoseconds.
 */

#include "orbit_propagator.h"

namespace ntn::geometry {

/**
 * @brief Constructs the linear orbit propagator.
 * @param initial_pos Initial Cartesian position in meters.
 * @param velocity Constant Cartesian velocity in meters per second.
 * @param initial_time Epoch timestamp in nanoseconds.
 */
LinearOrbitPropagator::LinearOrbitPropagator(Position initial_pos, Velocity velocity, common::Time_ns initial_time)
    : initial_pos_(initial_pos), velocity_(velocity), initial_time_(initial_time) {}

/**
 * @brief Calculates satellite coordinates at current_time.
 * @param current_time Simulation time in nanoseconds.
 * @return Position Extrapolated Cartesian position at current_time.
 */
Position LinearOrbitPropagator::get_position_at(common::Time_ns current_time) const {
    // Time delta in seconds (nanoseconds / 1e9)
    double delta_t_s = static_cast<double>(current_time - initial_time_) / 1e9;

    Position pos;
    pos.x_m = initial_pos_.x_m + (velocity_.vx_mps * delta_t_s);
    pos.y_m = initial_pos_.y_m + (velocity_.vy_mps * delta_t_s);
    pos.z_m = initial_pos_.z_m + (velocity_.vz_mps * delta_t_s);

    return pos;
}

/**
 * @brief Returns configured constant velocity.
 * @return Velocity Cartesian velocity in m/s.
 */
Velocity LinearOrbitPropagator::get_velocity() const {
    return velocity_;
}

} // namespace ntn::geometry

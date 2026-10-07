#include "orbit_propagator.h"

namespace ntn::geometry {

LinearOrbitPropagator::LinearOrbitPropagator(Position initial_pos, Velocity velocity, common::Time_ns initial_time)
    : initial_pos_(initial_pos), velocity_(velocity), initial_time_(initial_time) {}

Position LinearOrbitPropagator::get_position_at(common::Time_ns current_time) const {
    // Time delta in seconds
    // (current_time is in nanoseconds, so divide by 1,000,000,000.0)
    double delta_t_s = static_cast<double>(current_time - initial_time_) / 1e9;

    Position pos;
    pos.x_m = initial_pos_.x_m + (velocity_.vx_mps * delta_t_s);
    pos.y_m = initial_pos_.y_m + (velocity_.vy_mps * delta_t_s);
    pos.z_m = initial_pos_.z_m + (velocity_.vz_mps * delta_t_s);

    return pos;
}

Velocity LinearOrbitPropagator::get_velocity() const {
    return velocity_;
}

} // namespace ntn::geometry


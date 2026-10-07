#pragma once

#include "coordinate.h"
#include "../../common/simulator.h"

namespace ntn::geometry {

// Simplified linear orbit propagator for L2/L3 protocol emulation.
// Moves a satellite in a straight line relative to a stationary ground frame.
class LinearOrbitPropagator {
public:
    LinearOrbitPropagator(Position initial_pos, Velocity velocity, common::Time_ns initial_time);

    // Calculate the position of the satellite at the given simulation time
    Position get_position_at(common::Time_ns current_time) const;

    // Get the configured constant velocity
    Velocity get_velocity() const;

private:
    Position initial_pos_;
    Velocity velocity_;
    common::Time_ns initial_time_;
};

} // namespace ntn::geometry


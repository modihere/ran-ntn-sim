#include <gtest/gtest.h>
#include "../../src/ntn/geometry/coordinate.h"
#include "../../src/ntn/geometry/orbit_propagator.h"
#include "../../src/ntn/geometry/propagation.h"

using namespace ntn::geometry;
using namespace ntn::common;

/**
 * @brief Verifies the 3D Euclidean distance calculation between two points.
 * 
 * @param Inputs: Point 1 at (0, 0, 0), Point 2 at (3000, 4000, 0)
 * @param Expected: Distance should be exactly 5000.0 meters (standard 3-4-5 triangle).
 */
TEST(GeometryTest, DistanceCalculation) {
    Position p1{0.0, 0.0, 0.0};
    Position p2{3000.0, 4000.0, 0.0};
    EXPECT_DOUBLE_EQ(p1.distance_to(p2), 5000.0);
}

/**
 * @brief Verifies slant range calculation when the satellite is directly overhead (zenith).
 * 
 * @param Inputs: UE at origin (0,0,0), Satellite at (0,0, 600000) representing 600km altitude.
 * @param Expected: Slant range should equal the altitude (600,000.0 meters).
 */
TEST(GeometryTest, SlantRangeAtZenith) {
    Position ue{0.0, 0.0, 0.0};
    Position sat{0.0, 0.0, 600000.0}; // 600 km up
    EXPECT_DOUBLE_EQ(PropagationCalculator::calculate_slant_range(sat, ue), 600000.0);
}

/**
 * @brief Verifies the one-way propagation delay conversion from distance.
 * 
 * @param Inputs: Slant range of 600,000 meters.
 * @param Expected: Delay in nanoseconds. (600,000 / 299,792,458) * 1e9 = ~2,001,384 ns.
 */
TEST(PropagationTest, DelayCalculation) {
    double slant_range = 600000.0;
    Time_ns expected_delay_ns = 2001384; 
    
    Time_ns actual_delay = PropagationCalculator::calculate_one_way_delay(slant_range);
    
    // Allow +/- 1 ns rounding error
    EXPECT_NEAR(static_cast<double>(actual_delay), static_cast<double>(expected_delay_ns), 1.0);
}

/**
 * @brief Verifies Doppler shift calculation for an approaching LEO satellite.
 * 
 * @param Inputs: 
 *        - Satellite at (600km, 0, 0) moving towards UE at -7.5 km/s on X axis.
 *        - Carrier frequency = 2.0 GHz (S-band).
 * @param Expected: Positive Doppler shift of approximately +50,034.6 Hz.
 */
TEST(PropagationTest, DopplerShiftApproaching) {
    Position ue{0.0, 0.0, 0.0};
    Velocity ue_v{0.0, 0.0, 0.0};
    
    Position sat{600000.0, 0.0, 0.0};
    Velocity sat_v{-7500.0, 0.0, 0.0}; 
    
    double carrier_hz = 2.0e9; // 2 GHz
    
    double doppler = PropagationCalculator::calculate_doppler_shift(sat, sat_v, ue, ue_v, carrier_hz);
    
    EXPECT_GT(doppler, 0.0);
    EXPECT_NEAR(doppler, 50034.6, 0.1);
}

/**
 * @brief Verifies elevation angle calculations for various satellite positions.
 * 
 * @param Inputs: Satellite at zenith, horizon, and 45-degree angle positions relative to UE.
 * @param Expected: 90.0 degrees, 0.0 degrees, and 45.0 degrees respectively.
 */
TEST(PropagationTest, ElevationAngle) {
    Position ue{0.0, 0.0, 0.0};
    
    // Zenith
    Position sat1{0.0, 0.0, 600000.0};
    EXPECT_DOUBLE_EQ(PropagationCalculator::calculate_elevation_angle_degrees(sat1, ue), 90.0);
    
    // Horizon
    Position sat2{600000.0, 0.0, 0.0};
    EXPECT_DOUBLE_EQ(PropagationCalculator::calculate_elevation_angle_degrees(sat2, ue), 0.0);
    
    // 45 degrees
    Position sat3{600000.0, 0.0, 600000.0};
    EXPECT_DOUBLE_EQ(PropagationCalculator::calculate_elevation_angle_degrees(sat3, ue), 45.0);
}

/**
 * @brief Verifies that the LinearOrbitPropagator correctly displaces the satellite over time.
 * 
 * @param Inputs: 
 *        - Initial position (0, 0, 600km).
 *        - Velocity (+7.5km/s on X axis).
 *        - Time elapsed: 10 seconds (10,000,000,000 ns).
 * @param Expected: New position at X = 75km, Y = 0, Z = 600km.
 */
TEST(OrbitPropagatorTest, LinearMovement) {
    Position start{0.0, 0.0, 600000.0};
    Velocity vel{7500.0, 0.0, 0.0};
    
    LinearOrbitPropagator prop(start, vel, 0);
    
    Time_ns t = 10ULL * 1000000000ULL; // 10 seconds later
    Position p2 = prop.get_position_at(t);
    
    EXPECT_DOUBLE_EQ(p2.x_m, 75000.0);
    EXPECT_DOUBLE_EQ(p2.y_m, 0.0);
    EXPECT_DOUBLE_EQ(p2.z_m, 600000.0);
}

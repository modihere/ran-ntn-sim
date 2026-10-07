#include <gtest/gtest.h>
#include "simulator.h"

using namespace ntn::common;

TEST(SimulatorTest, InitialTimeIsZero) {
    Simulator sim;
    EXPECT_EQ(sim.now(), 0);
}

TEST(SimulatorTest, ExecutesScheduledEventAndAdvancesTime) {
    Simulator sim;
    bool executed = false;

    sim.schedule(1000, [&]() {
        executed = true;
        EXPECT_EQ(sim.now(), 1000); // Time should be exactly 1000ns when this runs
    });

    sim.run();

    EXPECT_TRUE(executed);
    EXPECT_EQ(sim.now(), 1000);
}

TEST(SimulatorTest, EventsExecuteInChronologicalOrder) {
    Simulator sim;
    std::vector<int> execution_order;

    // Schedule out of order to ensure Priority Queue sorts them
    sim.schedule(5000, [&]() { execution_order.push_back(3); });
    sim.schedule(1000, [&]() { execution_order.push_back(1); });
    sim.schedule(3000, [&]() { execution_order.push_back(2); });

    sim.run();

    ASSERT_EQ(execution_order.size(), 3);
    EXPECT_EQ(execution_order[0], 1);
    EXPECT_EQ(execution_order[1], 2);
    EXPECT_EQ(execution_order[2], 3);
    EXPECT_EQ(sim.now(), 5000);
}

TEST(SimulatorTest, RunWithSpecificDuration) {
    Simulator sim;
    int execution_count = 0;

    sim.schedule(1000, [&]() { execution_count++; });
    sim.schedule(3000, [&]() { execution_count++; }); // Should NOT run
    sim.schedule(5000, [&]() { execution_count++; }); // Should NOT run

    sim.run(2000); // Run for 2000ns

    EXPECT_EQ(execution_count, 1);
    EXPECT_EQ(sim.now(), 2000); // Clock should pause exactly at the boundary
}

TEST(TimerTest, TimerFiresCorrectly) {
    Simulator sim;
    Timer timer(sim);
    bool fired = false;

    timer.start(5000, [&]() {
        fired = true;
    });

    EXPECT_TRUE(timer.is_running());
    sim.run();

    EXPECT_TRUE(fired);
    EXPECT_FALSE(timer.is_running());
    EXPECT_EQ(sim.now(), 5000);
}

TEST(TimerTest, StoppedTimerDoesNotFire) {
    Simulator sim;
    Timer timer(sim);
    bool fired = false;

    timer.start(5000, [&]() {
        fired = true;
    });

    // Schedule an event at 2000ns that stops the timer early
    sim.schedule(2000, [&]() {
        timer.stop();
    });

    sim.run();

    EXPECT_FALSE(fired);
    EXPECT_FALSE(timer.is_running());
    // The simulator clock will still advance to 5000ns because the lambda
    // wrapper inside Timer is still in the queue, it just becomes a no-op.
    EXPECT_EQ(sim.now(), 5000);
}

TEST(TimerTest, RestartedTimerCancelsPreviousTimeout) {
    Simulator sim;
    Timer timer(sim);
    int fire_count = 0;

    timer.start(2000, [&]() {
        fire_count++;
        // This shouldn't run because it's overwritten before it fires
    });

    // At 1000ns, we restart the timer for another 3000ns
    sim.schedule(1000, [&]() {
        timer.start(3000, [&]() {
            fire_count++;
            EXPECT_EQ(sim.now(), 4000); // 1000 + 3000
        });
    });

    sim.run();

    // The timer should only fire ONCE, at 4000ns.
    EXPECT_EQ(fire_count, 1);
    EXPECT_EQ(sim.now(), 4000);
}

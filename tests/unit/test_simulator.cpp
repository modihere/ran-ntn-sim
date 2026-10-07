#include <gtest/gtest.h>
#include "simulator.h"

using namespace ntn::common;

/**
 * @brief Verifies that a newly instantiated Simulator starts at time 0.
 * 
 * @param Inputs: Default constructed Simulator.
 * @param Expected: now() returns 0.
 */
TEST(SimulatorTest, InitialTimeIsZero) {
    Simulator sim;
    EXPECT_EQ(sim.now(), 0);
}

/**
 * @brief Verifies that the simulator executes a scheduled event and advances its clock.
 * 
 * @param Inputs: An event scheduled at a 1000ns delay.
 * @param Expected: The event's lambda is executed, and the clock jumps to 1000ns.
 */
TEST(SimulatorTest, ExecutesScheduledEventAndAdvancesTime) {
    Simulator sim;
    bool executed = false;

    sim.schedule(1000, [&]() {
        executed = true;
        EXPECT_EQ(sim.now(), 1000); 
    });

    sim.run();

    EXPECT_TRUE(executed);
    EXPECT_EQ(sim.now(), 1000);
}

/**
 * @brief Verifies that the Priority Queue correctly sorts events chronologically.
 * 
 * @param Inputs: Events scheduled out of order (at delays 5000ns, 1000ns, 3000ns).
 * @param Expected: Execution order is exactly 1000ns, then 3000ns, then 5000ns.
 */
TEST(SimulatorTest, EventsExecuteInChronologicalOrder) {
    Simulator sim;
    std::vector<int> execution_order;

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

/**
 * @brief Verifies that run(duration) pauses execution without flushing the entire queue.
 * 
 * @param Inputs: Events at 1000ns, 3000ns, 5000ns. run() is called with a 2000ns duration limit.
 * @param Expected: Only the 1000ns event fires. Clock pauses exactly at 2000ns.
 */
TEST(SimulatorTest, RunWithSpecificDuration) {
    Simulator sim;
    int execution_count = 0;

    sim.schedule(1000, [&]() { execution_count++; });
    sim.schedule(3000, [&]() { execution_count++; }); 
    sim.schedule(5000, [&]() { execution_count++; }); 

    sim.run(2000); 

    EXPECT_EQ(execution_count, 1);
    EXPECT_EQ(sim.now(), 2000); 
}

/**
 * @brief Verifies that a cancelable protocol Timer fires correctly if not stopped.
 * 
 * @param Inputs: A timer started for 5000ns.
 * @param Expected: Timer fires, is_running() transitions to false.
 */
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

/**
 * @brief Verifies that calling stop() on a timer prevents its callback from firing.
 * 
 * @param Inputs: A timer started for 5000ns, but stopped at 2000ns via another event.
 * @param Expected: The timer's timeout lambda does not execute.
 */
TEST(TimerTest, StoppedTimerDoesNotFire) {
    Simulator sim;
    Timer timer(sim);
    bool fired = false;

    timer.start(5000, [&]() {
        fired = true;
    });

    sim.schedule(2000, [&]() {
        timer.stop();
    });

    sim.run();

    EXPECT_FALSE(fired);
    EXPECT_FALSE(timer.is_running());
    EXPECT_EQ(sim.now(), 5000);
}

/**
 * @brief Verifies that restarting a timer overwrites the original timeout.
 * 
 * @param Inputs: Timer started for 2000ns, then restarted at 1000ns for an additional 3000ns.
 * @param Expected: The original 2000ns timeout does not fire. Only the new 4000ns timeout fires.
 */
TEST(TimerTest, RestartedTimerCancelsPreviousTimeout) {
    Simulator sim;
    Timer timer(sim);
    int fire_count = 0;

    timer.start(2000, [&]() {
        fire_count++;
    });

    sim.schedule(1000, [&]() {
        timer.start(3000, [&]() {
            fire_count++;
            EXPECT_EQ(sim.now(), 4000); 
        });
    });

    sim.run();

    EXPECT_EQ(fire_count, 1);
    EXPECT_EQ(sim.now(), 4000);
}

#include <gtest/gtest.h>
#include "../../src/nr/rach/rach_controller.h"
#include "../../src/ntn/timing/timing_engine.h"

using namespace ntn::nr;
using namespace ntn::timing;
using namespace ntn::common;

TEST(RachTest, SuccessfulRach) {
    Simulator sim;
    NtnTimingEngine timing;
    timing.update_geometry(20000000ULL); // 20ms OWD
    
    RachController rach(sim, timing);
    
    // Trigger RACH at t=0
    sim.schedule(0, [&]() { rach.trigger_rach(); });
    
    // Receive RAR (Msg2) at t=25ms
    sim.schedule(25000000ULL, [&]() { rach.receive_msg2(sim.now()); });
    
    // Receive Contention Resolution (Msg4) at t=70ms
    sim.schedule(70000000ULL, [&]() { rach.receive_msg4(true); });
    
    sim.run();
    
    EXPECT_EQ(rach.get_state(), RachState::COMPLETED);
}

TEST(RachTest, RarTimeout) {
    Simulator sim;
    NtnTimingEngine timing;
    timing.update_geometry(20000000ULL); // 20ms OWD -> 40ms RTT
    
    RachController rach(sim, timing);
    
    // Trigger RACH at t=0
    sim.schedule(0, [&]() { rach.trigger_rach(); });
    
    // Let it run. RAR window is 10ms + 40ms = 50ms.
    // It should timeout at 50ms.
    sim.run(60000000ULL);
    
    EXPECT_EQ(rach.get_state(), RachState::FAILED);
}

TEST(RachTest, ContentionLost) {
    Simulator sim;
    NtnTimingEngine timing;
    timing.update_geometry(20000000ULL); 
    
    RachController rach(sim, timing);
    
    sim.schedule(0, [&]() { rach.trigger_rach(); });
    sim.schedule(25000000ULL, [&]() { rach.receive_msg2(sim.now()); });
    
    // Receive Contention Resolution (Msg4) with contention_won = false
    sim.schedule(70000000ULL, [&]() { rach.receive_msg4(false); });
    
    sim.run();
    
    EXPECT_EQ(rach.get_state(), RachState::FAILED);
}


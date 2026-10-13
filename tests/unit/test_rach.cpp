#include <gtest/gtest.h>
#include "../../src/nr/rach/rach_controller.h"
#include "../../src/ntn/timing/timing_engine.h"

using namespace ntn::nr;
using namespace ntn::timing;
using namespace ntn::common;

// Helper: configure timing engine as if SIB19 was received
static void bootstrap_timing(NtnTimingEngine& timing) {
    timing.set_scs_khz(15);                             // 1ms slot
    timing.set_k_offset_slots(60);                      // 60ms RTT capacity
    timing.set_ul_sync_validity_duration(10000000000ULL); // 10s validity
    timing.update_geometry(20000000ULL);                 // 20ms OWD -> 40ms RTT
    timing.update_ta(0);                                 // Sync at t=0
}

TEST(RachTest, SuccessfulRach) {
    Simulator sim;
    NtnTimingEngine timing;
    bootstrap_timing(timing);
    
    RachController rach(sim, timing);
    rach.receive_sib19();
    
    // Trigger RACH at t=0
    sim.schedule(0, [&]() { rach.trigger_rach(); });
    
    // Receive RAR (Msg2) at t=25ms
    sim.schedule(25000000ULL, [&]() { rach.receive_msg2(sim.now()); });
    
    // Receive Contention Resolution (Msg4) at t=100ms
    // (Msg3 sent at ~87ms: 25ms msg2 + 2ms processing + 60ms k_offset)
    sim.schedule(100000000ULL, [&]() { rach.receive_msg4(true); });
    
    sim.run();
    
    EXPECT_EQ(rach.get_state(), RachState::COMPLETED);
}

TEST(RachTest, RarTimeout) {
    Simulator sim;
    NtnTimingEngine timing;
    bootstrap_timing(timing);
    
    RachController rach(sim, timing);
    rach.receive_sib19();
    
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
    bootstrap_timing(timing);
    
    RachController rach(sim, timing);
    rach.receive_sib19();
    
    sim.schedule(0, [&]() { rach.trigger_rach(); });
    sim.schedule(25000000ULL, [&]() { rach.receive_msg2(sim.now()); });
    
    // Receive Contention Resolution (Msg4) with contention_won = false
    sim.schedule(100000000ULL, [&]() { rach.receive_msg4(false); });
    
    sim.run();
    
    EXPECT_EQ(rach.get_state(), RachState::FAILED);
}


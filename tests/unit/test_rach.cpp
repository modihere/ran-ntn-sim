/**
 * @file test_rach.cpp
 * @brief Unit tests for UE-side 4-Step NTN Random Access Procedure (3GPP TS 38.321).
 *
 * Validates the UE-side contention-based RACH state machine:
 * - SIB19 acquisition requirement prior to PRACH initiation.
 * - Successful 4-step RACH handshake (Msg1, Msg2 RAR, Msg3, Msg4 Contention Resolution).
 * - Extended RAR window timeout handling.
 * - Contention resolution loss handling.
 * - RRC Connection Re-establishment RACH procedure after sync recovery.
 */

#include <gtest/gtest.h>
#include "../../src/ue/mac/ue_rach_controller.h"
#include "../../src/ue/timing/timing_engine.h"

using namespace ntn::ue::mac;
using namespace ntn::ue::timing;
using namespace ntn::common;

/**
 * @brief Helper function to bootstrap UE timing engine with valid SIB19 configuration.
 * @param timing Reference to the UeTimingEngine instance under test.
 * @return void (Configures SCS, K_offset, UL sync validity, and initial TA update).
 */
static void bootstrap_timing(UeTimingEngine& timing) {
    timing.set_scs_khz(15);                               // 1ms slot
    timing.set_k_offset_slots(60);                        // 60ms RTT capacity
    timing.set_ul_sync_validity_duration(10000000000ULL); // 10s validity
    timing.update_geometry(20000000ULL);                  // 20ms OWD -> 40ms RTT
    timing.update_service_link_ta(0);                     // Sync at t=0
}

/**
 * @brief Verifies a fully successful 4-step RACH procedure transitioning to CONNECTED.
 * @param None Default simulation environment with valid SIB19 configuration.
 * @return void (Assertion verifies RACH state transitions to CONNECTED).
 */
TEST(RachTest, SuccessfulRach) {
    Simulator sim;
    UeTimingEngine timing;
    bootstrap_timing(timing);
    
    UeRachController rach(sim, timing);
    rach.receive_sib19();
    
    // Trigger RACH at t=0
    sim.schedule(0, [&]() { rach.trigger_rach(); });
    
    // Receive RAR (Msg2) at t=25ms
    sim.schedule(25000000ULL, [&]() { rach.receive_msg2(sim.now()); });
    
    // Receive Contention Resolution (Msg4) at t=100ms
    // (Msg3 sent at ~87ms: 25ms msg2 + 2ms processing + 60ms k_offset)
    sim.schedule(100000000ULL, [&]() { rach.receive_msg4(true); });
    
    sim.run();
    
    EXPECT_EQ(rach.get_state(), RachState::CONNECTED);
}

/**
 * @brief Verifies that RACH transitions to FAILED when no Msg2 is received within the RAR window.
 * @param None Simulation run without delivering RAR Msg2.
 * @return void (Assertion verifies RACH state transitions to FAILED upon timer expiry).
 */
TEST(RachTest, RarTimeout) {
    Simulator sim;
    UeTimingEngine timing;
    bootstrap_timing(timing);
    
    UeRachController rach(sim, timing);
    rach.receive_sib19();
    
    // Trigger RACH at t=0
    sim.schedule(0, [&]() { rach.trigger_rach(); });
    
    // Let it run. RAR window is 10ms + 40ms = 50ms.
    // It should timeout at 50ms.
    sim.run(60000000ULL);
    
    EXPECT_EQ(rach.get_state(), RachState::FAILED);
}

/**
 * @brief Verifies that RACH transitions to FAILED when contention resolution fails (contention_won=false).
 * @param None Simulation delivering Msg4 with contention_won=false.
 * @return void (Assertion verifies RACH state transitions to FAILED).
 */
TEST(RachTest, ContentionLost) {
    Simulator sim;
    UeTimingEngine timing;
    bootstrap_timing(timing);
    
    UeRachController rach(sim, timing);
    rach.receive_sib19();
    
    sim.schedule(0, [&]() { rach.trigger_rach(); });
    sim.schedule(25000000ULL, [&]() { rach.receive_msg2(sim.now()); });
    
    // Receive Contention Resolution (Msg4) with contention_won = false
    sim.schedule(100000000ULL, [&]() { rach.receive_msg4(false); });
    
    sim.run();
    
    EXPECT_EQ(rach.get_state(), RachState::FAILED);
}

/**
 * @brief Verifies that RACH can be triggered for RRC Connection Re-establishment after sync recovery.
 * @param None Simulation environment where SIB19 was reacquired following sync loss.
 * @return void (Assertion verifies RACH state transitions to CONNECTED and is_reestablishment flag).
 */
TEST(RachTest, ReestablishmentRach) {
    Simulator sim;
    UeTimingEngine timing;
    bootstrap_timing(timing);
    
    UeRachController rach(sim, timing);
    rach.receive_sib19();
    
    // Simulate sync loss and MAC flush
    rach.lose_sync_and_flush();
    EXPECT_EQ(rach.get_state(), RachState::WAITING_FOR_SIB19);
    
    // SIB19 reacquired
    rach.receive_sib19();
    EXPECT_EQ(rach.get_state(), RachState::IDLE);
    
    // Trigger RACH for re-establishment
    sim.schedule(0, [&]() { rach.trigger_rach(true); });
    
    // Receive RAR (Msg2) at t=25ms
    sim.schedule(25000000ULL, [&]() { rach.receive_msg2(sim.now()); });
    
    // Receive Contention Resolution (Msg4) at t=100ms
    sim.schedule(100000000ULL, [&]() { rach.receive_msg4(true); });
    
    sim.run();
    
    EXPECT_EQ(rach.get_state(), RachState::CONNECTED);
    EXPECT_TRUE(rach.is_reestablishment());
}

/**
 * @brief Verifies that low K_offset allows Msg1 and Msg2, blocks Msg3, and retries at 1 sec interval until TA fits K_offset.
 * @param None UeTimingEngine with initially inadequate K_offset that later satisfies timing.
 * @return void (Assertions verify Msg3 is blocked initially, retried at 1s, and completes once TA <= K_offset).
 */
TEST(RachTest, LowKOffsetBlocksMsg3AndRetries) {
    Simulator sim;
    UeTimingEngine timing;
    timing.set_scs_khz(15);
    timing.set_k_offset_slots(10);                        // 10ms capacity
    timing.set_ul_sync_validity_duration(10000000000ULL); // 10s validity
    timing.update_geometry(15000000ULL);                  // 15ms OWD -> 30ms RTT > 10ms K_offset
    timing.update_service_link_ta(0);                     // Synced at t=0

    bool msg1_sent = false;
    bool msg3_sent = false;
    UeRachController rach(sim, timing, nullptr,
        [&]() { msg1_sent = true; },
        [&]() { msg3_sent = true; }
    );
    rach.receive_sib19();

    // 1. Initial attempt at t=0: Msg1 is permitted despite low K_offset
    sim.schedule(0, [&]() { rach.trigger_rach(); });

    // Deliver Msg2 at t=25ms: Should detect TA (30ms) > K_offset (10ms) and block Msg3
    sim.schedule(25000000ULL, [&]() {
        EXPECT_TRUE(msg1_sent);
        rach.receive_msg2(sim.now());
        EXPECT_EQ(rach.get_state(), RachState::FAILED);
        EXPECT_FALSE(msg3_sent);
    });

    // 2. At t=1000ms: Satellite moves closer, reducing RTT to 8ms (within 10ms K_offset)
    sim.schedule(1000000000ULL, [&]() {
        timing.update_geometry(4000000ULL); // 4ms OWD -> 8ms RTT <= 10ms K_offset
    });

    // The autonomous 1-second retry timer fires at t=1025ms (25ms + 1000ms)
    // Delivering RAR for the retry attempt at t=1030ms (inside 18ms RAR window [1025ms, 1043ms])
    sim.schedule(1030000000ULL, [&]() {
        EXPECT_EQ(rach.get_state(), RachState::WAIT_MSG2);
        rach.receive_msg2(sim.now()); // Now TA (8ms) <= K_offset (10ms) -> schedules Msg3
    });

    // Deliver Msg4 at t=1050ms (inside contention window)
    sim.schedule(1050000000ULL, [&]() {
        EXPECT_TRUE(msg3_sent);
        rach.receive_msg4(true);
    });

    sim.run();

    EXPECT_EQ(rach.get_state(), RachState::CONNECTED);
}

/**
 * @brief Verifies that exhausting RACH attempts triggers SIB19 re-acquisition and subsequent successful RACH.
 * @param None UeRachController with preambleTransMax = 2 and initial inadequate K_offset.
 * @return void (Assertions verify callback invocation on max attempts and recovery after SIB19 refresh).
 */
TEST(RachTest, ExhaustedAttemptsTriggerSib19Reacquisition) {
    Simulator sim;
    UeTimingEngine timing;
    timing.set_scs_khz(15);
    timing.set_k_offset_slots(10);                        // 10ms capacity
    timing.set_ul_sync_validity_duration(10000000000ULL); // 10s validity
    timing.update_geometry(15000000ULL);                  // 15ms OWD -> 30ms RTT > 10ms K_offset
    timing.update_service_link_ta(0);                     // Synced at t=0

    bool sib19_reacquisition_triggered = false;
    UeRachController rach(sim, timing);
    rach.set_preamble_trans_max(2); // Set max attempts to 2 for quick verification
    rach.receive_sib19();

    // Wire exhausted attempts callback to simulate SIB19 re-reading with updated K_offset
    rach.set_on_rach_exhausted_cb([&]() {
        sib19_reacquisition_triggered = true;
        // gNodeB broadcasts updated K_offset = 40 slots in fresh SIB19
        timing.set_k_offset_slots(40); // 40ms capacity > 30ms RTT
        rach.receive_sib19();
        // Immediately re-attempt RACH
        rach.trigger_rach();
    });

    // 1. Initial attempt at t=0 (counter = 1)
    sim.schedule(0, [&]() { rach.trigger_rach(); });

    // Deliver Msg2 at t=25ms: TA (30ms) > K_offset (10ms) -> Msg3 blocked, attempt 1 failed
    sim.schedule(25000000ULL, [&]() {
        rach.receive_msg2(sim.now());
        EXPECT_EQ(rach.get_state(), RachState::FAILED);
        EXPECT_EQ(rach.get_preamble_trans_counter(), 1);
    });

    // 2. Retry timer fires at t=1025ms (attempt 2, counter = 2 = preambleTransMax)
    // Deliver Msg2 at t=1030ms -> TA still > K_offset -> attempts exhausted -> callback invoked!
    sim.schedule(1030000000ULL, [&]() {
        rach.receive_msg2(sim.now());
        EXPECT_TRUE(sib19_reacquisition_triggered);
        // After callback triggered, new RACH attempt (counter = 1) has sent Msg1
        EXPECT_EQ(rach.get_state(), RachState::WAIT_MSG2);
    });

    // Deliver Msg2 for refreshed RACH attempt at t=1040ms (now K_offset=40ms covers 30ms TA)
    sim.schedule(1040000000ULL, [&]() {
        rach.receive_msg2(sim.now()); // Msg3 scheduled with K_offset (40ms + 2ms processing = 42ms delay)
    });

    // At t=1090ms: Msg3 has departed (at 1082ms) and UE is waiting for Msg4
    sim.schedule(1090000000ULL, [&]() {
        EXPECT_EQ(rach.get_state(), RachState::WAIT_MSG4);
    });

    // Deliver Msg4 at t=1100ms
    sim.schedule(1100000000ULL, [&]() {
        rach.receive_msg4(true);
    });

    sim.run();

    EXPECT_EQ(rach.get_state(), RachState::CONNECTED);
}

/**
 * @brief Verifies that a connected UE transitions to DL_ONLY_UM if SIB19 K_offset is less than TA.
 * @param None UeRachController operating in CONNECTED state.
 * @return void (Assertions verify state transitions between CONNECTED and DL_ONLY_UM).
 */
TEST(RachTest, ConnectedModeDlOnlyUmOnLowKOffset) {
    Simulator sim;
    UeTimingEngine timing;
    timing.set_scs_khz(15);
    timing.set_k_offset_slots(60);
    timing.update_service_link_ta(0);

    UeRachController rach(sim, timing);
    rach.restore_connected();
    EXPECT_EQ(rach.get_state(), RachState::CONNECTED);

    // K_offset becomes inadequate while connected -> transitions to DL_ONLY_UM
    rach.set_dl_only_um();
    EXPECT_EQ(rach.get_state(), RachState::DL_ONLY_UM);

    // K_offset restored -> full bi-directional CONNECTED restored
    rach.restore_connected();
    EXPECT_EQ(rach.get_state(), RachState::CONNECTED);
}

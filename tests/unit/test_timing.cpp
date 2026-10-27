/**
 * @file test_timing.cpp
 * @brief Unit tests for UE NTN Timing Engine (3GPP Release 17 Regenerative Payload).
 *
 * Validates the UE-side autonomous calculation of Service Link Timing Advance (TA),
 * verification of the cell-specific K_offset scheduling margin, and enforcement
 * of the Uplink Synchronization Validity Duration timer broadcast in SIB19.
 */

#include <gtest/gtest.h>
#include "../../src/ue/timing/timing_engine.h"

using namespace ntn::ue::timing;
using namespace ntn::common;

/**
 * @brief Verifies that the K_offset configuration parameter is correctly stored and retrieved.
 * @param None Default constructed UeTimingEngine.
 * @return void (Assertions verify K_offset slot count equals 40).
 */
TEST(TimingEngineTest, KOffsetConfig) {
    UeTimingEngine engine;
    engine.set_scs_khz(15);
    engine.set_k_offset_slots(40);
    
    EXPECT_EQ(engine.get_k_offset_slots(), 40);
}

/**
 * @brief Verifies autonomous Service Link TA computation from one-way propagation delay.
 * @param None UeTimingEngine updated with 20ms one-way delay.
 * @return void (Assertions verify round-trip Service Link TA equals 40ms).
 */
TEST(TimingEngineTest, ServiceLinkTaUpdate) {
    UeTimingEngine engine;
    Time_ns owd = 20000000; // 20 ms
    engine.update_geometry(owd);
    
    EXPECT_EQ(engine.get_service_link_ta(), 2 * owd);
    EXPECT_EQ(engine.get_common_ta(), 2 * owd); // Backward compatibility alias
}

/**
 * @brief Verifies that uplink synchronization is marked invalid before any SIB19 sync occurs.
 * @param None UeTimingEngine configured without SIB19 TA update.
 * @return void (Assertion verifies is_ul_sync_valid returns false).
 */
TEST(TimingEngineTest, SyncValidity_NeverSynced) {
    UeTimingEngine engine;
    engine.set_scs_khz(15);
    engine.set_k_offset_slots(60);
    
    Time_ns owd = 20000000; // 20ms OWD -> 40ms RTT (within 60ms K_offset)
    engine.update_geometry(owd);
    
    // Never called update_service_link_ta
    EXPECT_FALSE(engine.is_ul_sync_valid(100000));
}

/**
 * @brief Verifies that uplink synchronization is valid within the validity timer window.
 * @param None UeTimingEngine synced at t=100ms with a 5000ms validity duration.
 * @return void (Assertion verifies is_ul_sync_valid returns true at t=200ms).
 */
TEST(TimingEngineTest, SyncValidity_ValidAfterSync) {
    UeTimingEngine engine;
    engine.set_scs_khz(15);
    engine.set_k_offset_slots(60);
    engine.set_ul_sync_validity_duration(5000000000ULL); // 5000ms
    
    Time_ns owd = 20000000; // 20ms OWD -> 40ms RTT
    engine.update_geometry(owd);
    engine.update_service_link_ta(100000000ULL); // Synced at t=100ms
    
    EXPECT_TRUE(engine.is_ul_sync_valid(200000000ULL)); // t=200ms, well within 5000ms timer
}

/**
 * @brief Verifies that uplink synchronization expires once the validity duration has elapsed.
 * @param None UeTimingEngine synced at t=100ms evaluated at t=5200ms (timer expired).
 * @return void (Assertion verifies is_ul_sync_valid returns false).
 */
TEST(TimingEngineTest, SyncValidity_TimerExpiry) {
    UeTimingEngine engine;
    engine.set_scs_khz(15);
    engine.set_k_offset_slots(60);
    engine.set_ul_sync_validity_duration(5000000000ULL); // 5000ms
    
    Time_ns owd = 20000000;
    engine.update_geometry(owd);
    engine.update_service_link_ta(100000000ULL); // Synced at t=100ms
    
    // At t=5200ms (5100ms after sync) -> timer expired
    EXPECT_FALSE(engine.is_ul_sync_valid(5200000000ULL));
}

/**
 * @brief Verifies that Service Link TA violates the K_offset scheduling margin when RTT exceeds K_offset capacity.
 * @param None UeTimingEngine with 35ms OWD (70ms RTT) and K_offset=60 slots (60ms).
 * @return void (Assertion verifies is_ta_within_k_offset returns false while is_ul_sync_valid remains true).
 */
TEST(TimingEngineTest, SyncValidity_KOffsetViolation) {
    UeTimingEngine engine;
    engine.set_scs_khz(15);
    engine.set_k_offset_slots(60); // 60ms max RTT capacity
    engine.set_ul_sync_validity_duration(50000000000ULL); // very long timer
    
    Time_ns owd = 35000000; // 35ms OWD -> 70ms RTT > 60ms K_offset
    engine.update_geometry(owd);
    engine.update_service_link_ta(100000000ULL);
    
    // In 3GPP Rel-17: SIB19 ephemeris validity timer is active, so UL sync is valid for Msg1 PRACH
    EXPECT_TRUE(engine.is_ul_sync_valid(200000000ULL));
    // But Msg3 scheduling constraint fails because RTT (70ms) exceeds K_offset capacity (60ms)
    EXPECT_FALSE(engine.is_ta_within_k_offset());
}

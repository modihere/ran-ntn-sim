#include <gtest/gtest.h>
#include "../../src/ntn/timing/timing_engine.h"

using namespace ntn::timing;
using namespace ntn::common;

TEST(TimingEngineTest, KOffsetConfig) {
    NtnTimingEngine engine;
    engine.set_scs_khz(15);
    engine.set_k_offset_slots(40);
    
    EXPECT_EQ(engine.get_k_offset_slots(), 40);
}

TEST(TimingEngineTest, CommonTaUpdate) {
    NtnTimingEngine engine;
    Time_ns owd = 20000000;
    engine.update_geometry(owd);
    
    EXPECT_EQ(engine.get_common_ta(), 2 * owd);
}

TEST(TimingEngineTest, SyncValidity_NeverSynced) {
    NtnTimingEngine engine;
    engine.set_scs_khz(15);
    engine.set_k_offset_slots(60);
    
    Time_ns owd = 20000000; // 20ms OWD -> 40ms RTT (within 60ms K_offset)
    engine.update_geometry(owd);
    
    // Never called update_ta, so last_ta_update_time_ == 0
    EXPECT_FALSE(engine.is_ul_sync_valid(100000));
}

TEST(TimingEngineTest, SyncValidity_ValidAfterSync) {
    NtnTimingEngine engine;
    engine.set_scs_khz(15);
    engine.set_k_offset_slots(60);
    engine.set_ul_sync_validity_duration(5000000000ULL); // 5000ms
    
    Time_ns owd = 20000000; // 20ms OWD -> 40ms RTT
    engine.update_geometry(owd);
    engine.update_ta(100000000ULL); // Synced at t=100ms
    
    EXPECT_TRUE(engine.is_ul_sync_valid(200000000ULL)); // t=200ms, well within 5000ms timer
}

TEST(TimingEngineTest, SyncValidity_TimerExpiry) {
    NtnTimingEngine engine;
    engine.set_scs_khz(15);
    engine.set_k_offset_slots(60);
    engine.set_ul_sync_validity_duration(5000000000ULL); // 5000ms
    
    Time_ns owd = 20000000;
    engine.update_geometry(owd);
    engine.update_ta(100000000ULL); // Synced at t=100ms
    
    // At t=5200ms (5100ms after sync) -> timer expired
    EXPECT_FALSE(engine.is_ul_sync_valid(5200000000ULL));
}

TEST(TimingEngineTest, SyncValidity_KOffsetViolation) {
    NtnTimingEngine engine;
    engine.set_scs_khz(15);
    engine.set_k_offset_slots(60); // 60ms max RTT capacity
    engine.set_ul_sync_validity_duration(50000000000ULL); // very long timer
    
    Time_ns owd = 35000000; // 35ms OWD -> 70ms RTT > 60ms K_offset
    engine.update_geometry(owd);
    engine.update_ta(100000000ULL);
    
    EXPECT_FALSE(engine.is_ul_sync_valid(200000000ULL)); // Fails due to K_offset violation
}

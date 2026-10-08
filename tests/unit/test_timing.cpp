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

TEST(TimingEngineTest, SyncValidity) {
    NtnTimingEngine engine;
    engine.set_scs_khz(15); // 1ms slot
    engine.set_k_offset_slots(60); // 60ms max RTT capacity
    
    Time_ns owd1 = 20000000; // 20ms OWD -> 40ms RTT
    engine.update_geometry(owd1);
    
    EXPECT_TRUE(engine.is_ul_sync_valid(0));
    
    // Change delay beyond drift threshold (e.g., +17us)
    Time_ns owd2 = 20000000 + 17000;
    engine.update_geometry(owd2);
    
    EXPECT_FALSE(engine.is_ul_sync_valid(0)); // Should fail due to CP drift
    
    // Trigger TA update to fix drift
    engine.update_ta();
    EXPECT_TRUE(engine.is_ul_sync_valid(0));
    
    // Change delay beyond K_offset capacity (e.g. OWD = 35ms -> RTT = 70ms > 60ms)
    Time_ns owd3 = 35000000; 
    engine.update_geometry(owd3);
    engine.update_ta(); // Update TA so drift is not the reason for failure
    
    EXPECT_FALSE(engine.is_ul_sync_valid(0)); // Should fail due to K_offset violation
}

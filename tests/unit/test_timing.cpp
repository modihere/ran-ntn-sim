#include <gtest/gtest.h>
#include "../../src/ntn/timing/timing_engine.h"

using namespace ntn::timing;
using namespace ntn::common;

TEST(TimingEngineTest, KOffsetCalculation) {
    NtnTimingEngine engine;
    engine.set_scs_khz(15);
    
    // RTT of 40ms -> OWD of 20ms
    Time_ns owd = 20000000; // 20 ms
    engine.update_geometry(owd);
    
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
    Time_ns owd1 = 20000000;
    engine.update_geometry(owd1);
    
    EXPECT_TRUE(engine.is_ul_sync_valid(0));
    
    // Change delay beyond threshold (e.g., +17us)
    Time_ns owd2 = 20000000 + 17000;
    engine.update_geometry(owd2);
    
    EXPECT_FALSE(engine.is_ul_sync_valid(0));
    
    // Trigger TA update
    engine.update_ta();
    EXPECT_TRUE(engine.is_ul_sync_valid(0));
}


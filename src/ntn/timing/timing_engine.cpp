#include "timing_engine.h"

namespace ntn::timing {

void NtnTimingEngine::set_scs_khz(uint32_t scs_khz) {
    scs_khz_ = scs_khz;
    if (scs_khz_ == 15) slot_duration_ns_ = 1000000;
    else if (scs_khz_ == 30) slot_duration_ns_ = 500000;
    else if (scs_khz_ == 60) slot_duration_ns_ = 250000;
    else if (scs_khz_ == 120) slot_duration_ns_ = 125000;
    else slot_duration_ns_ = 1000000;
}

void NtnTimingEngine::update_geometry(common::Time_ns one_way_delay) {
    current_owd_ns_ = one_way_delay;
    if (last_ta_update_owd_ns_ == 0) {
        last_ta_update_owd_ns_ = current_owd_ns_; // Initial sync
    }
}

void NtnTimingEngine::update_ta() {
    last_ta_update_owd_ns_ = current_owd_ns_;
}

common::Time_ns NtnTimingEngine::get_common_ta() const {
    return 2 * current_owd_ns_;
}

common::Time_ns NtnTimingEngine::get_ue_ta() const {
    return 0; // Simplified
}

uint32_t NtnTimingEngine::get_k_offset_slots() const {
    common::Time_ns rtt = 2 * current_owd_ns_;
    return static_cast<uint32_t>((rtt + slot_duration_ns_ - 1) / slot_duration_ns_);
}

common::Time_ns NtnTimingEngine::calculate_expected_ul_arrival(common::Time_ns tx_time) const {
    return tx_time + current_owd_ns_;
}

bool NtnTimingEngine::is_ul_sync_valid(common::Time_ns current_time) const {
    (void)current_time;
    common::Time_ns drift = (current_owd_ns_ > last_ta_update_owd_ns_) ? 
                            (current_owd_ns_ - last_ta_update_owd_ns_) : 
                            (last_ta_update_owd_ns_ - current_owd_ns_);
                            
    // Threshold set to 16us for testing SyncValidity
    common::Time_ns threshold = 16000; 
    
    return drift <= threshold;
}

} // namespace ntn::timing


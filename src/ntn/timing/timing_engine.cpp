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

void NtnTimingEngine::set_k_offset_slots(uint32_t slots) {
    k_offset_slots_ = slots;
}

void NtnTimingEngine::set_ul_sync_validity_duration(common::Time_ns duration) {
    ul_sync_validity_duration_ = duration;
}

void NtnTimingEngine::update_geometry(common::Time_ns one_way_delay) {
    current_owd_ns_ = one_way_delay;
    if (last_ta_update_owd_ns_ == 0) {
        last_ta_update_owd_ns_ = current_owd_ns_; // Initial sync
    }
}

void NtnTimingEngine::update_ta(common::Time_ns current_time) {
    last_ta_update_owd_ns_ = current_owd_ns_;
    last_ta_update_time_ = current_time;
    has_synced_ = true;
}

common::Time_ns NtnTimingEngine::get_common_ta() const {
    return 2 * current_owd_ns_;
}

common::Time_ns NtnTimingEngine::get_ue_ta() const {
    return 0; // Simplified
}

uint32_t NtnTimingEngine::get_k_offset_slots() const {
    return k_offset_slots_;
}

common::Time_ns NtnTimingEngine::calculate_expected_ul_arrival(common::Time_ns tx_time) const {
    return tx_time + current_owd_ns_;
}

bool NtnTimingEngine::is_ul_sync_valid(common::Time_ns current_time) const {
    // 1. Must have received SIB19 / synced at least once
    if (!has_synced_) {
        return false;
    }

    // 2. UL Sync Validity Duration timer (from SIB19 ulSyncValidityDuration IE)
    //    If the timer has expired since the last TA update, sync is invalid.
    //    The network sets this value to account for expected orbital drift.
    if (ul_sync_validity_duration_ > 0) {
        if ((current_time - last_ta_update_time_) > ul_sync_validity_duration_) {
            return false;
        }
    }

    // 3. K_offset violation check: RTT > K_offset_time
    common::Time_ns rtt = 2 * current_owd_ns_;
    common::Time_ns k_offset_time = k_offset_slots_ * slot_duration_ns_;
    if (rtt > k_offset_time) {
        return false;
    }

    return true;
}

} // namespace ntn::timing


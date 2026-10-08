#pragma once

#include <cstdint>
#include "../../common/simulator.h"

namespace ntn::timing {

class NtnTimingEngine {
public:
    // Config
    void set_scs_khz(uint32_t scs_khz = 15);
    void set_k_offset_slots(uint32_t slots);
    
    // Updates
    void update_geometry(common::Time_ns one_way_delay);
    void update_ta(); // Triggers a TA update, refreshing the sync validity
    
    // 3GPP Parameters
    common::Time_ns get_common_ta() const;
    common::Time_ns get_ue_ta() const;
    uint32_t get_k_offset_slots() const; // Represents static slots configured by network
    
    // Validation
    common::Time_ns calculate_expected_ul_arrival(common::Time_ns tx_time) const;
    bool is_ul_sync_valid(common::Time_ns current_time) const;
    
private:
    uint32_t scs_khz_ = 15;
    uint32_t k_offset_slots_ = 0;
    common::Time_ns slot_duration_ns_ = 1000000; // 1ms for 15kHz
    common::Time_ns current_owd_ns_ = 0;
    common::Time_ns last_ta_update_owd_ns_ = 0;
};

} // namespace ntn::timing


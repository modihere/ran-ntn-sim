#pragma once

#include <cstdint>
#include "../../common/simulator.h"

namespace ntn::nr::rrc {

struct TaInfo {
    common::Time_ns ta_common;
    double ta_common_drift; // Rate of change
};

struct EphemerisInfo {
    double x, y, z;
    double vx, vy, vz;
};

struct NtnConfig {
    common::Time_ns si_window_ms;
    common::Time_ns ul_sync_validity_duration_ms;
    TaInfo ta_info;
    EphemerisInfo ephemeris_info;
    uint32_t k_offset_slots;
    double reacquisition_success_prob{0.5};
    common::Time_ns sync_loss_recovery_gap_ms{400000000ULL}; // 400ms default
};

} // namespace ntn::nr::rrc


/**
 * @file ue_rrc.cpp
 * @brief Implementation of UE-side Radio Resource Control (RRC) entity.
 *
 * Implements SI window monitoring, SIB19 transport block reception from BCCH,
 * ASN.1 SIB19 decoding, delayed activation at SI window boundary in absence of epochTime,
 * and triggering RRC Connection Re-establishment via RACH upon recovery.
 */

#include "ue_rrc.h"
#include <iomanip>
#include <sstream>

namespace ntn::ue::rrc {

/**
 * @brief Constructs the UE RRC entity.
 * @param sim Simulation core reference.
 * @param timing UE timing engine reference.
 * @param rach UE RACH controller reference.
 * @param log_cb Telemetry log sink callback.
 */
UeRrc::UeRrc(
    common::Simulator& sim,
    timing::UeTimingEngine& timing,
    mac::UeRachController& rach,
    std::function<void(const std::string&)> log_cb
) : sim_(sim), timing_(timing), rach_(rach), log_cb_(log_cb) {}

/**
 * @brief Logs active UE monitoring of PDCCH with SI-RNTI in the active SI window.
 */
void UeRrc::start_monitoring_si_window() {
    if (log_cb_) {
        log_cb_("\033[36m[UE-RRC] Monitoring PDCCH with SI-RNTI for SIB19 broadcast in SI window...\033[0m");
    }
}

/**
 * @brief Reads incoming SIB19 transport block from BCCH and decodes parameters during SI window.
 * @param config SIB19 NtnConfig received from satellite broadcast.
 * @param current_time Reception timestamp in nanoseconds.
 * @param owd_ns One-way service link delay in nanoseconds.
 */
void UeRrc::on_sib19_received(const common::NtnConfig& config, common::Time_ns current_time, common::Time_ns owd_ns) {
    (void)current_time;
    staged_config_ = config;
    has_staged_config_ = true;

    if (log_cb_) {
        std::stringstream ss_rx;
        ss_rx << "\033[36m[UE-RRC] Received SIB19 transport block over Service Link (OWD: "
              << (owd_ns / 1000) << " us). Reading BCCH payload...\033[0m";
        log_cb_(ss_rx.str());

        std::stringstream ss_decode;
        ss_decode << "\033[36m[UE-RRC] SIB19 decoded: Ephemeris=["
                  << std::fixed << std::setprecision(0) << config.ephemeris_info.x << ", "
                  << config.ephemeris_info.y << ", " << config.ephemeris_info.z << "] m, K_offset="
                  << config.k_offset_slots << " slots, Validity="
                  << (config.ul_sync_validity_duration_ms / 1000000) << " ms.\033[0m";
        log_cb_(ss_decode.str());

        log_cb_("\033[36m[UE-RRC] (Epoch time absent) SIB19 configuration will be applied on the ending point of the SI window.\033[0m");
    }
}

/**
 * @brief Activates staged SIB19 configuration at the ending point of the SI window.
 * @param current_time Timestamp at SI window closure in nanoseconds.
 * @param is_recovery true if this activation follows a sync-loss recovery cycle.
 */
void UeRrc::on_si_window_ended(common::Time_ns current_time, bool is_recovery) {
    if (!has_staged_config_) {
        return;
    }

    // Activate staged configuration into the UE Timing Engine
    timing_.set_k_offset_slots(staged_config_.k_offset_slots);
    timing_.set_ul_sync_validity_duration(staged_config_.ul_sync_validity_duration_ms);
    timing_.update_service_link_ta(current_time);

    if (log_cb_) {
        std::stringstream ss_apply;
        ss_apply << "\033[36m[UE-RRC] SI Window ended. Activating SIB19 configuration: Service Link TA pre-compensation calculated ("
                 << (timing_.get_service_link_ta() / 1000) << " us), K_offset="
                 << staged_config_.k_offset_slots << " slots. RACH permitted.\033[0m";
        log_cb_(ss_apply.str());
    }

    rach_.receive_sib19();

    if (is_recovery) {
        if (log_cb_) {
            log_cb_("\033[36m[UE-RRC] Triggering RRC Connection Re-establishment via RACH procedure...\033[0m");
        }
        rach_.trigger_rach(true);
    }

    has_staged_config_ = false;
}

/**
 * @brief Handles UL sync timer expiry: instructs MAC to flush buffers and halts UL transmissions.
 */
void UeRrc::on_ul_sync_expired() {
    if (log_cb_) {
        log_cb_("\033[31m[UE-RRC] Uplink Synchronization Validity Duration timer expired! Connection dropped. Flushing MAC buffers.\033[0m");
    }
    rach_.lose_sync_and_flush();
}

} // namespace ntn::ue::rrc

/**
 * @file ue_rrc.h
 * @brief UE-side Radio Resource Control (RRC) entity for NTN.
 *
 * Implements 3GPP TS 38.331 RRC procedures on the UE side:
 * - Active monitoring of the SI window (PDCCH with SI-RNTI).
 * - Reception and reading of SIB19 broadcast transport blocks from BCCH over the service link.
 * - Decoding SIB19 (ntn-Config-r17) parameters during the SI window.
 * - Staging configuration until SI window closure (in absence of epochTime).
 * - Activating SIB19 parameters into UE Timing Engine at the SI window ending point.
 * - Authorizing initial RACH or triggering RRC Connection Re-establishment RACH.
 */

#pragma once

#include <functional>
#include <string>
#include "../../common/simulator.h"
#include "../../common/types.h"
#include "../timing/timing_engine.h"
#include "../mac/ue_rach_controller.h"

namespace ntn::ue::rrc {

/**
 * @class UeRrc
 * @brief Manages UE RRC state, SIB19 reception, and synchronization lifecycle.
 */
class UeRrc {
public:
    /**
     * @brief Constructs the UE RRC entity.
     * @param sim Reference to the simulation core.
     * @param timing Reference to the UE timing engine.
     * @param rach Reference to the UE RACH controller.
     * @param log_cb Log sink callback function.
     */
    UeRrc(
        common::Simulator& sim,
        timing::UeTimingEngine& timing,
        mac::UeRachController& rach,
        std::function<void(const std::string&)> log_cb = nullptr
    );

    /**
     * @brief Logs active UE monitoring of PDCCH scrambled with SI-RNTI during an open SI window.
     */
    void start_monitoring_si_window();

    /**
     * @brief Processes SIB19 transport block received over-the-air from satellite BCCH.
     *
     * Reads and decodes the ASN.1 payload, stages the parameters, and notes that because
     * epochTime is absent, the configuration will take effect at the ending boundary of the SI window.
     *
     * @param config The received SIB19 NtnConfig block.
     * @param current_time Simulation timestamp of reception in nanoseconds.
     * @param owd_ns One-way service link propagation delay in nanoseconds.
     */
    void on_sib19_received(const common::NtnConfig& config, common::Time_ns current_time, common::Time_ns owd_ns = 0);

    /**
     * @brief Applies staged SIB19 configuration at the ending point of the SI window.
     *
     * In accordance with 3GPP TS 38.331, when epochTime is absent, the configuration
     * is activated at the SI window boundary, updating Service Link TA and permitting RACH.
     *
     * @param current_time Timestamp at SI window closure in nanoseconds.
     * @param is_recovery true if this activation follows a sync-loss recovery cycle.
     */
    void on_si_window_ended(common::Time_ns current_time, bool is_recovery = false);

    /**
     * @brief Handles expiration of the Uplink Synchronization Validity Duration timer.
     */
    void on_ul_sync_expired();

private:
    common::Simulator& sim_;                     ///< Simulator core reference
    timing::UeTimingEngine& timing_;             ///< UE timing engine reference
    mac::UeRachController& rach_;                ///< UE RACH controller reference
    std::function<void(const std::string&)> log_cb_; ///< Telemetry log sink

    common::NtnConfig staged_config_;            ///< Decoded SIB19 configuration awaiting SI window closure
    bool has_staged_config_{false};              ///< Flag indicating staged SIB19 awaiting activation
};

} // namespace ntn::ue::rrc

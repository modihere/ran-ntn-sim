/**
 * @file ue_rach_controller.h
 * @brief UE-side Medium Access Control (MAC) 4-Step Random Access (RACH) Controller.
 *
 * Implements the 3GPP TS 38.321 Random Access Procedure on the UE side for NTN:
 * - Gating RACH initiation until SIB19 is acquired over-the-air.
 * - Transmission of PRACH Preamble (Msg1) with autonomous Service Link TA pre-compensation.
 * - Monitoring the extended Random Access Response (RAR) window (Msg2).
 * - Transmission of Msg3 (RRCSetupRequest / RRCReestablishmentRequest) with cell-specific K_offset delay.
 * - Monitoring the extended Contention Resolution Timer (Msg4).
 * - Transitioning to RRC_CONNECTED and handling MAC buffer flushes upon sync loss.
 */

#pragma once

#include <functional>
#include <string>
#include "../../common/simulator.h"
#include "../../common/types.h"
#include "../timing/timing_engine.h"

namespace ntn::ue::mac {

/**
 * @class UeRachController
 * @brief Manages the 4-step contention-based Random Access state machine on the UE side.
 */
class UeRachController {
public:
    /**
     * @brief Constructs the UE RACH Controller.
     * @param sim Reference to the Discrete-Event Simulator core.
     * @param timing Reference to the UE-side NTN Timing Engine.
     * @param log_cb Callback function to deliver telemetry log strings.
     * @param send_msg1_cb Callback to deliver transmitted Msg1 (PRACH Preamble) over the channel.
     * @param send_msg3_cb Callback to deliver transmitted Msg3 over the channel.
     */
    UeRachController(
        common::Simulator& sim,
        timing::UeTimingEngine& timing,
        std::function<void(const std::string&)> log_cb = nullptr,
        std::function<void()> send_msg1_cb = nullptr,
        std::function<void()> send_msg3_cb = nullptr
    );

    /**
     * @brief Triggers the Random Access procedure by transmitting PRACH Msg1.
     *
     * Validates that SIB19 has been acquired and uplink synchronization is valid.
     * Calculates the extended RAR window taking round-trip service link delay into account,
     * and starts the ra_response_window timer.
     *
     * @param is_reestablishment If true, executes RACH for RRC Connection Re-establishment.
     */
    void trigger_rach(bool is_reestablishment = false);

    /**
     * @brief Processes reception of Msg2 (Random Access Response) from the satellite gNodeB.
     * @param rx_time Reception timestamp in nanoseconds.
     */
    void receive_msg2(common::Time_ns rx_time);

    /**
     * @brief Processes reception of Msg4 (Contention Resolution) from the satellite gNodeB.
     * @param contention_won true if contention resolution succeeded; false otherwise.
     */
    void receive_msg4(bool contention_won);

    /**
     * @brief Notifies the controller that SIB19 has been decoded, permitting RACH.
     */
    void receive_sib19();

    /**
     * @brief Handles unexpected loss of uplink synchronization.
     */
    void lose_sync();

    /**
     * @brief Handles UL sync timer expiry: flushes MAC buffers and blocks RACH.
     */
    void lose_sync_and_flush();

    /**
     * @brief Restores the UE state directly to CONNECTED upon SIB19 reacquisition.
     */
    void restore_connected();

    /**
     * @brief Switches the UE into Downlink Unacknowledged Mode (DL RLC-UM) when K_offset < TA while connected.
     */
    void set_dl_only_um();

    /**
     * @brief Returns the current state of the RACH state machine.
     * @return Current RachState enum value.
     */
    common::RachState get_state() const;

    /**
     * @brief Returns whether the active RACH procedure is for RRC Re-establishment.
     * @return bool true if re-establishment procedure, false if initial access.
     */
    bool is_reestablishment() const { return is_reestablishment_; }

    /**
     * @brief Sets callback invoked when RACH attempts reach preambleTransMax without success.
     * @param cb Callback function.
     */
    void set_on_rach_exhausted_cb(std::function<void()> cb) { on_rach_exhausted_cb_ = cb; }

    /**
     * @brief Returns the current preamble transmission counter value.
     * @return Current attempt count.
     */
    uint32_t get_preamble_trans_counter() const { return preamble_trans_counter_; }

    /**
     * @brief Configures maximum preamble transmission counter limit (preambleTransMax).
     * @param max_val Maximum attempt limit (default: 4).
     */
    void set_preamble_trans_max(uint32_t max_val) { preamble_trans_max_ = max_val; }

    /**
     * @brief Returns configured maximum preamble transmission counter limit.
     * @return preambleTransMax limit.
     */
    uint32_t get_preamble_trans_max() const { return preamble_trans_max_; }

    /**
     * @brief Sets the callback function invoked when Msg1 is transmitted.
     * @param cb Callback function.
     */
    void set_send_msg1_cb(std::function<void()> cb) { send_msg1_cb_ = cb; }

    /**
     * @brief Sets the callback function invoked when Msg3 is transmitted.
     * @param cb Callback function.
     */
    void set_send_msg3_cb(std::function<void()> cb) { send_msg3_cb_ = cb; }

private:
    /**
     * @brief Transmits Msg3 (RRCSetupRequest or RRCReestablishmentRequest) scheduled with K_offset delay.
     */
    void send_msg3();

    /**
     * @brief Schedules an autonomous RACH retry after a specified backoff duration.
     * @param delay_ns Backoff duration in nanoseconds (default: 1000 ms / 1 second).
     */
    void schedule_rach_retry(common::Time_ns delay_ns = 1000000000ULL);

    /**
     * @brief Handles expiration of the extended RAR window timer.
     */
    void on_rar_timeout();

    /**
     * @brief Handles expiration of the extended Contention Resolution timer.
     */
    void on_contention_timeout();

    common::Simulator& sim_;                     ///< Simulator core reference
    timing::UeTimingEngine& timing_;             ///< UE timing engine reference
    std::function<void(const std::string&)> log_cb_; ///< Telemetry log sink
    std::function<void()> send_msg1_cb_;         ///< Hook for Msg1 channel transmission
    std::function<void()> send_msg3_cb_;         ///< Hook for Msg3 channel transmission
    common::RachState state_{common::RachState::WAITING_FOR_SIB19}; ///< RACH state machine
    bool is_reestablishment_{false};             ///< Flag indicating re-establishment procedure

    common::Timer ra_response_window_timer_;     ///< Extended RAR response timer
    common::Timer contention_resolution_timer_;  ///< Extended Contention Resolution timer
    common::Timer rach_retry_timer_;             ///< Autonomous 1-second RACH retry timer

    std::function<void()> on_rach_exhausted_cb_{nullptr}; ///< Callback when RACH attempts reach preambleTransMax
    uint32_t preamble_trans_counter_{0};         ///< Counter for transmitted PRACH preambles
    uint32_t preamble_trans_max_{4};             ///< 3GPP preambleTransMax limit (default 4)
};

} // namespace ntn::ue::mac

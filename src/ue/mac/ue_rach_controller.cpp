/**
 * @file ue_rach_controller.cpp
 * @brief Implementation of UE-side Medium Access Control (MAC) RACH Controller.
 *
 * Implements 4-step RACH procedures, pre-compensates Msg1 transmission timing by
 * Service Link TA, schedules Msg3 with K_offset delay, and handles MAC buffer flushing.
 */

#include "ue_rach_controller.h"
#include <spdlog/spdlog.h>
#include <sstream>

namespace ntn::ue::mac {

/**
 * @brief Constructs the UE RACH Controller.
 * @param sim Discrete-Event Simulator reference.
 * @param timing UE timing engine reference.
 * @param log_cb Log callback sink.
 * @param send_msg1_cb Callback to propagate Msg1 onto the channel.
 * @param send_msg3_cb Callback to propagate Msg3 onto the channel.
 */
UeRachController::UeRachController(
    common::Simulator& sim,
    timing::UeTimingEngine& timing,
    std::function<void(const std::string&)> log_cb,
    std::function<void()> send_msg1_cb,
    std::function<void()> send_msg3_cb
) : sim_(sim), timing_(timing), log_cb_(log_cb),
    send_msg1_cb_(send_msg1_cb), send_msg3_cb_(send_msg3_cb),
    ra_response_window_timer_(sim),
    contention_resolution_timer_(sim),
    rach_retry_timer_(sim) {}

/**
 * @brief Initiates PRACH procedure by sending Msg1 preamble.
 * @param is_reestablishment true if triggering RACH for RRC connection re-establishment.
 */
void UeRachController::trigger_rach(bool is_reestablishment) {
    is_reestablishment_ = is_reestablishment;
    rach_retry_timer_.stop();

    if (state_ == common::RachState::WAITING_FOR_SIB19) {
        if (log_cb_) log_cb_("\033[36m[UE-MAC] Blocked: Waiting for SIB19 acquisition.\033[0m");
        else spdlog::warn("[UE-MAC] Blocked: Waiting for SIB19.");
        return;
    }
    
    if (!timing_.is_ul_sync_valid(sim_.now())) {
        if (log_cb_) log_cb_("\033[36m[UE-MAC] Blocked: Uplink Synchronization Invalid or Expired.\033[0m");
        else spdlog::warn("[UE-MAC] Blocked: UL Sync Invalid or Expired.");
        return;
    }

    if (state_ != common::RachState::IDLE && state_ != common::RachState::FAILED) {
        if (log_cb_) log_cb_("\033[36m[UE-MAC] Warning: RACH procedure already in progress.\033[0m");
        else spdlog::warn("[UE-MAC] RACH already in progress.");
        return;
    }

    preamble_trans_counter_++;
    state_ = common::RachState::WAIT_MSG2;
    std::stringstream ss;
    if (is_reestablishment_) {
        ss << "\033[36m[UE-MAC] Msg1 (PRACH Preamble) transmitted with updated Service Link TA pre-compensation ("
           << (timing_.get_service_link_ta() / 1000) << " us).\033[0m";
    } else {
        ss << "\033[36m[UE-MAC] Msg1 (PRACH Preamble) transmitted with Service Link TA pre-compensation ("
           << (timing_.get_service_link_ta() / 1000) << " us).\033[0m";
    }
    if (log_cb_) log_cb_(ss.str());
    else spdlog::info(ss.str());
    
    // In regenerative NTN, RAR window is extended by the round-trip service link propagation delay.
    common::Time_ns rtt = timing_.get_service_link_ta();
    common::Time_ns rar_window = 10000000ULL + rtt; // Base 10ms + Service Link RTT
    
    ra_response_window_timer_.start(rar_window, [this]() { on_rar_timeout(); });

    // Trigger physical uplink propagation hook if configured
    if (send_msg1_cb_) {
        send_msg1_cb_();
    }
}

/**
 * @brief Notifies the controller that SIB19 has been received over-the-air.
 */
void UeRachController::receive_sib19() {
    preamble_trans_counter_ = 0;
    if (state_ == common::RachState::WAITING_FOR_SIB19 || state_ == common::RachState::FAILED) {
        state_ = common::RachState::IDLE;
    }
}

/**
 * @brief Handles sync loss without buffer flush logging.
 */
void UeRachController::lose_sync() {
    rach_retry_timer_.stop();
    preamble_trans_counter_ = 0;
    if (state_ != common::RachState::WAITING_FOR_SIB19) {
        state_ = common::RachState::WAITING_FOR_SIB19;
        if (log_cb_) log_cb_("\033[36m[UE-RRC] Sync Lost. Connection dropped. RACH blocked until SIB19 re-acquired.\033[0m");
    }
}

/**
 * @brief Handles sync validity timer expiry: flushes MAC buffers, cancels retries, and blocks RACH.
 */
void UeRachController::lose_sync_and_flush() {
    state_ = common::RachState::WAITING_FOR_SIB19;
    ra_response_window_timer_.stop();
    contention_resolution_timer_.stop();
    rach_retry_timer_.stop();
    preamble_trans_counter_ = 0;
}

/**
 * @brief Restores UE state to CONNECTED upon SIB19 reacquisition.
 */
void UeRachController::restore_connected() {
    rach_retry_timer_.stop();
    preamble_trans_counter_ = 0;
    state_ = common::RachState::CONNECTED;
}

/**
 * @brief Switches the UE into Downlink Unacknowledged Mode (DL RLC-UM) when K_offset < TA while connected.
 */
void UeRachController::set_dl_only_um() {
    rach_retry_timer_.stop();
    state_ = common::RachState::DL_ONLY_UM;
}

/**
 * @brief Processes reception of Msg2 (Random Access Response) from the gNodeB.
 * @param rx_time Simulation timestamp of reception in nanoseconds.
 */
void UeRachController::receive_msg2(common::Time_ns rx_time) {
    (void)rx_time;
    if (state_ != common::RachState::WAIT_MSG2) {
        return;
    }
    
    ra_response_window_timer_.stop();
    if (is_reestablishment_) {
        if (log_cb_) log_cb_("\033[36m[UE-MAC] Msg2 (RAR) received from satellite gNodeB. Preparing Msg3 (RRCReestablishmentRequest)...\033[0m");
        else spdlog::info("[UE-MAC] Msg2 (RAR) received. Preparing Msg3 (RRCReestablishmentRequest)...");
    } else {
        if (log_cb_) log_cb_("\033[36m[UE-MAC] Msg2 (RAR) received from satellite gNodeB. Preparing Msg3...\033[0m");
        else spdlog::info("[UE-MAC] Msg2 (RAR) received. Preparing Msg3...");
    }

    // 3GPP TS 38.213 §8.3 / TS 38.321 §5.1.4: Validate if Service Link TA fits within K_offset capacity
    if (!timing_.is_ta_within_k_offset()) {
        common::Time_ns rtt_us = timing_.get_service_link_ta() / 1000;
        common::Time_ns capacity_us = timing_.get_k_offset_capacity_ns() / 1000;
        uint32_t k_slots = timing_.get_k_offset_slots();

        std::stringstream ss;
        ss << "\033[33m[UE-MAC] Msg3 transmission blocked: Service Link TA (" << rtt_us
           << " us) exceeds K_offset capacity (" << capacity_us << " us, " << k_slots
           << " slots). Timing requirement cannot be met.\033[0m";
        if (log_cb_) log_cb_(ss.str());
        else spdlog::warn(ss.str());

        state_ = common::RachState::FAILED;

        // 3GPP TS 38.321 §5.1.4: Check if RACH attempts are exhausted (preambleTransMax reached)
        if (preamble_trans_counter_ >= preamble_trans_max_) {
            std::stringstream ss_exhaust;
            ss_exhaust << "\033[33m[UE-MAC] Random Access attempts exhausted ("
                       << preamble_trans_counter_ << " attempts reached preambleTransMax="
                       << preamble_trans_max_ << "). Indicating Random Access Problem to RRC.\033[0m";
            if (log_cb_) log_cb_(ss_exhaust.str());
            else spdlog::warn(ss_exhaust.str());

            if (on_rach_exhausted_cb_) {
                on_rach_exhausted_cb_();
            } else {
                schedule_rach_retry(1000000000ULL);
            }
            return;
        }

        std::stringstream ss_retry;
        ss_retry << "\033[33m[UE-MAC] RACH attempt failed (attempt " << preamble_trans_counter_
                 << "/" << preamble_trans_max_ << "). Retrying RACH in 1000 ms...\033[0m";
        if (log_cb_) log_cb_(ss_retry.str());

        schedule_rach_retry(1000000000ULL); // Schedule retry in 1 second
        return;
    }

    std::stringstream ss_grant;
    ss_grant << "\033[36m[UE-MAC] Service Link TA (" << (timing_.get_service_link_ta() / 1000)
             << " us) within K_offset capacity (" << (timing_.get_k_offset_capacity_ns() / 1000)
             << " us, " << timing_.get_k_offset_slots() << " slots). Scheduling Msg3...\033[0m";
    if (log_cb_) log_cb_(ss_grant.str());

    send_msg3();
}

/**
 * @brief Schedules and transmits Msg3 applying K_offset delay.
 */
void UeRachController::send_msg3() {
    uint32_t k_offset_slots = timing_.get_k_offset_slots();
    common::Time_ns k_offset_delay = k_offset_slots * timing_.get_slot_duration_ns();
    common::Time_ns processing_delay = 2000000ULL; // 2ms UE processing margin
    
    sim_.schedule(processing_delay + k_offset_delay, [this]() {
        state_ = common::RachState::WAIT_MSG4;
        if (is_reestablishment_) {
            if (log_cb_) log_cb_("\033[36m[UE-MAC] Msg3 (RRCReestablishmentRequest) transmitted with K_offset delay.\033[0m");
            else spdlog::info("[UE-MAC] Msg3 (RRCReestablishmentRequest) transmitted with K_offset delay.");
        } else {
            if (log_cb_) log_cb_("\033[36m[UE-MAC] Msg3 (RRCSetupRequest) transmitted with K_offset delay.\033[0m");
            else spdlog::info("[UE-MAC] Msg3 (RRCSetupRequest) transmitted with K_offset delay.");
        }
        
        // Start extended contention resolution timer
        common::Time_ns contention_window = 20000000ULL + timing_.get_service_link_ta();
        contention_resolution_timer_.start(contention_window, [this]() { on_contention_timeout(); });

        if (send_msg3_cb_) {
            send_msg3_cb_();
        }
    });
}

/**
 * @brief Schedules an autonomous RACH retry after a specified backoff duration.
 * @param delay_ns Backoff delay in nanoseconds.
 */
void UeRachController::schedule_rach_retry(common::Time_ns delay_ns) {
    rach_retry_timer_.stop();
    rach_retry_timer_.start(delay_ns, [this]() {
        // Stop retries if UE is already in CONNECTED state
        if (state_ == common::RachState::CONNECTED) {
            return;
        }

        // Check if SIB19 ephemeris / UL sync validity has expired
        if (!timing_.is_ul_sync_valid(sim_.now())) {
            if (log_cb_) {
                log_cb_("\033[33m[UE-MAC] Aborting RACH retries: Uplink Synchronization Validity Duration timer expired.\033[0m");
            }
            state_ = common::RachState::WAITING_FOR_SIB19;
            return;
        }

        if (log_cb_) {
            log_cb_("\033[36m[UE-MAC] Retrying RACH procedure (1 sec interval)...\033[0m");
        }
        state_ = common::RachState::IDLE;
        trigger_rach(is_reestablishment_);
    });
}

/**
 * @brief Processes reception of Msg4 (Contention Resolution).
 * @param contention_won true if contention resolution passed.
 */
void UeRachController::receive_msg4(bool contention_won) {
    if (state_ != common::RachState::WAIT_MSG4) {
        return;
    }
    
    contention_resolution_timer_.stop();
    
    if (contention_won) {
        rach_retry_timer_.stop();
        state_ = common::RachState::CONNECTED;
        if (is_reestablishment_) {
            if (log_cb_) log_cb_("\033[32m[UE-MAC] Msg4 (Contention Resolution / RRCReestablishment) received. Re-establishment SUCCESS -> State: CONNECTED.\033[0m");
            else spdlog::info("[UE-MAC] Msg4 received. Re-establishment SUCCESS.");
        } else {
            if (log_cb_) log_cb_("\033[32m[UE-MAC] Msg4 (Contention Resolution) received. RACH SUCCESS -> State: CONNECTED.\033[0m");
            else spdlog::info("[UE-MAC] Msg4 received. RACH SUCCESS.");
        }
    } else {
        state_ = common::RachState::FAILED;
        if (log_cb_) log_cb_("\033[31m[UE-MAC] Msg4 received, but contention lost. RACH FAILED.\033[0m");
        else spdlog::info("[UE-MAC] Msg4 contention lost. RACH FAILED.");
    }
}

/**
 * @brief Handles expiration of the extended RAR window timer.
 */
void UeRachController::on_rar_timeout() {
    state_ = common::RachState::FAILED;
    if (log_cb_) log_cb_("\033[31m[UE-MAC] Msg2 (RAR) timeout expired. RACH FAILED.\033[0m");
    else spdlog::error("[UE-MAC] Msg2 (RAR) timeout expired.");
}

/**
 * @brief Handles expiration of the extended Contention Resolution timer.
 */
void UeRachController::on_contention_timeout() {
    state_ = common::RachState::FAILED;
    if (log_cb_) log_cb_("\033[31m[UE-MAC] Msg4 (Contention Resolution) timeout expired. RACH FAILED.\033[0m");
    else spdlog::error("[UE-MAC] Msg4 (Contention Resolution) timeout expired.");
}

/**
 * @brief Returns current RACH state.
 * @return RachState
 */
common::RachState UeRachController::get_state() const {
    return state_;
}

} // namespace ntn::ue::mac

#include "rach_controller.h"

namespace ntn::nr {

RachController::RachController(common::Simulator& sim, timing::NtnTimingEngine& timing)
    : sim_(sim), timing_(timing),
      ra_response_window_timer_(sim),
      contention_resolution_timer_(sim) {}

void RachController::trigger_rach() {
    if (state_ != RachState::IDLE && state_ != RachState::FAILED) {
        spdlog::warn("RACH already in progress.");
        return;
    }
    
    state_ = RachState::WAIT_MSG2;
    spdlog::info("[RACH] Msg1 (Preamble) transmitted.");
    
    // In NTN, the RAR window is extended by the round-trip propagation delay.
    // Base window (e.g. 10ms) + RTT
    common::Time_ns rtt = timing_.get_common_ta(); // Approximation of RTT
    common::Time_ns rar_window = 10000000ULL + rtt; 
    
    ra_response_window_timer_.start(rar_window, [this]() { on_rar_timeout(); });
}

void RachController::receive_msg2(common::Time_ns rx_time) {
    (void)rx_time;
    if (state_ != RachState::WAIT_MSG2) {
        return;
    }
    
    ra_response_window_timer_.stop();
    spdlog::info("[RACH] Msg2 (RAR) received. Preparing Msg3...");
    
    send_msg3();
}

void RachController::send_msg3() {
    // Schedule Msg3 using K_offset
    uint32_t k_offset_slots = timing_.get_k_offset_slots();
    common::Time_ns k_offset_delay = k_offset_slots * 1000000ULL; // Assuming 15kHz SCS (1ms slot)
    
    // Add a small processing delay for the UE (e.g. 2ms)
    common::Time_ns processing_delay = 2000000ULL;
    
    sim_.schedule(processing_delay + k_offset_delay, [this]() {
        state_ = RachState::WAIT_MSG4;
        spdlog::info("[RACH] Msg3 (RRCSetupRequest) transmitted with K_offset delay.");
        
        // Start contention resolution timer
        common::Time_ns contention_window = 20000000ULL + timing_.get_common_ta(); // 20ms + RTT
        contention_resolution_timer_.start(contention_window, [this]() { on_contention_timeout(); });
    });
}

void RachController::receive_msg4(bool contention_won) {
    if (state_ != RachState::WAIT_MSG4) {
        return;
    }
    
    contention_resolution_timer_.stop();
    
    if (contention_won) {
        state_ = RachState::COMPLETED;
        spdlog::info("[RACH] Msg4 (Contention Resolution) received. RACH SUCCESS.");
    } else {
        state_ = RachState::FAILED;
        spdlog::info("[RACH] Msg4 received, but contention lost. RACH FAILED.");
    }
}

void RachController::on_rar_timeout() {
    state_ = RachState::FAILED;
    spdlog::error("[RACH] Msg2 (RAR) timeout expired. RACH FAILED.");
}

void RachController::on_contention_timeout() {
    state_ = RachState::FAILED;
    spdlog::error("[RACH] Msg4 Contention Resolution timeout expired. RACH FAILED.");
}

RachState RachController::get_state() const {
    return state_;
}

} // namespace ntn::nr

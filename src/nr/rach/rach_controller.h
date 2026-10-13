#pragma once

#include "../../common/simulator.h"
#include "../../ntn/timing/timing_engine.h"
#include <spdlog/spdlog.h>

namespace ntn::nr {

enum class RachState { 
    WAITING_FOR_SIB19,
    IDLE, 
    WAIT_MSG2, 
    WAIT_MSG4, 
    CONNECTED,
    COMPLETED = CONNECTED, 
    FAILED 
};

class RachController {
public:
    RachController(common::Simulator& sim, timing::NtnTimingEngine& timing, std::function<void(const std::string&)> log_cb = nullptr);
    
    // Starts the RACH procedure (sends Msg1)
    void trigger_rach();
    
    // gNB sends RAR (Msg2)
    void receive_msg2(common::Time_ns rx_time);
    
    // gNB sends Contention Resolution (Msg4)
    void receive_msg4(bool contention_won);
    
    void receive_sib19();
    void lose_sync();
    void lose_sync_and_flush();
    void restore_connected();
    
    RachState get_state() const;

private:
    common::Simulator& sim_;
    timing::NtnTimingEngine& timing_;
    std::function<void(const std::string&)> log_cb_;
    RachState state_{RachState::WAITING_FOR_SIB19};
    
    common::Timer ra_response_window_timer_;
    common::Timer contention_resolution_timer_;
    
    void on_rar_timeout();
    void on_contention_timeout();
    
    void send_msg3();
};

} // namespace ntn::nr


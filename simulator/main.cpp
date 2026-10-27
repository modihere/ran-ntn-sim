/**
 * @file main.cpp
 * @brief Discrete-Event Simulator entry point for 3GPP Release 17 NTN RAN Emulation.
 *
 * Implements a regenerative satellite payload architecture where the gNodeB is on board
 * the satellite (feeder link delay = 0 ns). Simulates UE and gNodeB interactions including:
 * - Over-the-air SIB19 broadcasting and decoding over the service link.
 * - Autonomous UE Service Link Timing Advance (TA) pre-compensation calculation.
 * - 4-step Contention-Based Random Access Procedure (Msg1, Msg2 RAR, Msg3, Msg4).
 * - SIB19 periodic reacquisition, sync validity expiry, MAC buffer flushing, and recovery.
 * - Real-time ASCII TUI Dashboard with in-place freeze pause ('p'/Space) and interactive menu ('m').
 */

#include <iostream>
#include <fstream>
#include <spdlog/spdlog.h>
#include <nlohmann/json.hpp>
#include <thread>
#include <chrono>
#include <sstream>
#include <iomanip>
#include <vector>
#include <string>
#include <cmath>

#include "../src/common/simulator.h"
#include "../src/common/types.h"
#include "../src/ntn/geometry/orbit_propagator.h"
#include "../src/ntn/geometry/propagation.h"
#include "../src/ue/timing/timing_engine.h"
#include "../src/ue/mac/ue_rach_controller.h"
#include "../src/ue/rrc/ue_rrc.h"
#include "../src/gnb/rrc/sib19_broadcaster.h"
#include "../src/gnb/mac/gnb_mac.h"
#include "../src/ui/tui/dashboard.h"
#include "../src/ui/tui/keyboard.h"

using namespace ntn::common;
using namespace ntn::geometry;
using namespace ntn::ue::timing;
using namespace ntn::ue::mac;
using namespace ntn::ue::rrc;
using namespace ntn::gnb::rrc;
using namespace ntn::gnb::mac;
using json = nlohmann::json;

/**
 * @brief Strips ANSI color escape sequences from a string for plain-text file logging.
 * @param str Input string containing ANSI escape codes.
 * @return std::string Clean string without escape sequences.
 */
static std::string strip_ansi(const std::string& str) {
    std::string res;
    bool in_escape = false;
    for (char c : str) {
        if (c == '\033') in_escape = true;
        else if (in_escape && c == 'm') in_escape = false;
        else if (!in_escape) res += c;
    }
    return res;
}

/**
 * @brief Formats a log message ensuring the explicit entity origin is at the very beginning.
 *
 * Ensures all log lines start with [UE-MAC], [gNB-MAC], [UE-RRC], [gNB-RRC], or [SIM],
 * followed immediately by the simulation timestamp.
 *
 * @param msg Raw log message from an entity or simulator.
 * @param now_ns Current simulation timestamp in nanoseconds.
 * @return std::string Formatted and colorized log line.
 */
static std::string format_entity_log(const std::string& msg, Time_ns now_ns) {
    uint64_t now_ms = now_ns / 1000000ULL;
    std::string time_str = "[" + std::to_string(now_ms) + " ms]";

    // Check if msg starts with ANSI sequence followed by '[' e.g. "\033[36m[UE-..."
    if (msg.rfind("\033[", 0) == 0) {
        size_t m_pos = msg.find('m');
        if (m_pos != std::string::npos && m_pos + 1 < msg.size() && msg[m_pos + 1] == '[') {
            size_t tag_end = msg.find(']', m_pos + 1);
            if (tag_end != std::string::npos) {
                std::string ansi_color = msg.substr(0, m_pos + 1);
                std::string tag = msg.substr(m_pos + 1, tag_end - (m_pos + 1) + 1);
                std::string rest = msg.substr(tag_end + 1);

                std::stringstream ss;
                ss << ansi_color << tag << "\033[0m " << time_str << rest;
                return ss.str();
            }
        }
    }

    // Check if msg directly starts with bracketed entity tag e.g. "[UE-" or "[gNB-"
    if (msg.rfind("[UE-", 0) == 0 || msg.rfind("[gNB-", 0) == 0) {
        size_t tag_end = msg.find(']');
        std::string tag = msg.substr(0, tag_end + 1);
        std::string rest = msg.substr(tag_end + 1);
        std::string color = (tag.find("UE") != std::string::npos) ? "\033[36m" : "\033[35m";

        std::stringstream ss;
        ss << color << tag << "\033[0m " << time_str << rest;
        return ss.str();
    }

    // Simulator-level message default prefix
    std::stringstream ss;
    ss << "\033[34m[SIM]\033[0m " << time_str << " " << msg;
    return ss.str();
}

/**
 * @brief Interactive configuration parameter editor menu.
 *
 * Categorized into:
 * - [A] General / Simulation & Channel Modeling Parameters (Applied immediately upon resume).
 * - [B] SIB19 (ntn-Config-r17) Parameters (Broadcast over-the-air, applied at SI window end).
 *
 * @param config Reference to the JSON scenario configuration.
 * @param current_time_ns Current simulation timestamp in nanoseconds (0 if starting).
 * @return void (Updates config in-place).
 */
void modify_config_menu(json& config, Time_ns current_time_ns = 0) {
    while (true) {
        uint64_t current_ms = current_time_ns / 1000000ULL;
        uint32_t current_k_offset = config.contains("sib19") 
            ? config["sib19"].value("k_offset_slots", 6)
            : config.value("k_offset_slots", 6);
        uint64_t current_validity = config.contains("sib19")
            ? config["sib19"].value("ul_sync_validity_duration_ms", 10000)
            : 10000;
        double current_prob = config.contains("sib19")
            ? config["sib19"].value("reacquisition_success_prob", 0.5)
            : 0.5;
        uint64_t current_gap = config.contains("sib19")
            ? config["sib19"].value("sync_loss_recovery_gap_ms", 1500)
            : 1500;
        uint64_t current_si_win = config.contains("sib19")
            ? config["sib19"].value("si_window_ms", 20)
            : 20;

        std::cout << "\n\033[36m======================================================================\033[0m\n";
        std::cout << "\033[36m                  NTN Scenario Configuration Editor                   \033[0m\n";
        std::cout << "\033[36m======================================================================\033[0m\n";
        if (current_ms > 0) {
            std::cout << "\033[33m[Current Simulation Time: " << current_ms << " ms]\033[0m\n\n";
        }

        std::cout << "\033[32m--- [A] General / Simulation & Channel Modeling Parameters (Applied Immediately) ---\033[0m\n";
        std::cout << "[1] duration_ms                  : " << config.value("duration_ms", 50000);
        if (current_ms > 0) {
            int64_t rem = static_cast<int64_t>(config.value("duration_ms", 50000)) - static_cast<int64_t>(current_ms);
            std::cout << " (Remaining: " << (rem > 0 ? rem : 0) << " ms)";
        }
        std::cout << "\n";
        std::cout << "[2] update_interval_ms           : " << config.value("update_interval_ms", 100) << "\n";
        std::cout << "[3] scs_khz                      : " << config.value("scs_khz", 15) << "\n";
        std::cout << "[4] rach_trigger_time_ms         : " << config.value("rach_trigger_time_ms", 1000) << "\n";
        std::cout << "[5] reacquisition_success_prob   : " << std::fixed << std::setprecision(2) << current_prob << "  (Channel Emulation BLER Model)\n";
        std::cout << "[6] sync_loss_recovery_gap_ms    : " << current_gap << "  (Simulation Recovery Gap)\n";

        std::cout << "\n\033[35m--- [B] SIB19 (ntn-Config-r17) Parameters (Broadcast by gNodeB, Applied at SI Window End) ---\033[0m\n";
        std::cout << "[7] k_offset_slots               : " << current_k_offset << "  (cellSpecificKoffset-r17)\n";
        std::cout << "[8] ul_sync_validity_duration_ms : " << current_validity << "  (ntn-UlSyncValidityDuration-r17)\n";
        std::cout << "[9] satellite.vel_x_mps          : " << std::fixed << std::setprecision(1) << config["satellite"].value("vel_x_mps", -7500.0) << "  (ephemerisInfo: velocityVX)\n";
        std::cout << "[10] satellite.start_z_m         : " << std::fixed << std::setprecision(1) << config["satellite"].value("start_z_m", 600000.0) << "  (ephemerisInfo: positionZ)\n";
        std::cout << "[11] si_window_ms                : " << current_si_win << "  (si-WindowLength-r17)\n";

        std::cout << "\n[0] Go Back\n";
        std::cout << "Select parameter to change (0-11): ";

        std::string choice_str;
        std::getline(std::cin, choice_str);
        if (choice_str.empty()) break;

        int choice = -1;
        try {
            choice = std::stoi(choice_str);
        } catch (...) {
            std::cout << "\033[31mInvalid choice.\033[0m\n";
            continue;
        }

        if (choice == 0) break;

        std::string key = "";
        if (choice == 1) key = "duration_ms";
        else if (choice == 2) key = "update_interval_ms";
        else if (choice == 3) key = "scs_khz";
        else if (choice == 4) key = "rach_trigger_time_ms";
        else if (choice == 5) key = "reacquisition_success_prob";
        else if (choice == 6) key = "sync_loss_recovery_gap_ms";
        else if (choice == 7) key = "k_offset_slots";
        else if (choice == 8) key = "ul_sync_validity_duration_ms";
        else if (choice == 9) key = "satellite.vel_x_mps";
        else if (choice == 10) key = "satellite.start_z_m";
        else if (choice == 11) key = "si_window_ms";
        else {
            std::cout << "\033[31mInvalid choice.\033[0m\n";
            continue;
        }

        std::cout << "Enter new value for " << key << ": ";
        double val;
        if (!(std::cin >> val)) {
            std::cin.clear();
            std::cin.ignore(10000, '\n');
            std::cout << "\033[31mInvalid number entered.\033[0m\n";
            continue;
        }
        std::cin.ignore(10000, '\n');

        if (choice == 1) {
            uint64_t dur = static_cast<uint64_t>(val);
            if (current_ms > 0 && dur <= current_ms) {
                std::cout << "\033[31mTarget duration (" << dur 
                          << " ms) must be greater than current time (" 
                          << current_ms << " ms).\033[0m\n";
                continue;
            }
            config["duration_ms"] = dur;
        } else if (choice == 2) {
            config["update_interval_ms"] = static_cast<uint64_t>(val);
        } else if (choice == 3) {
            config["scs_khz"] = static_cast<uint32_t>(val);
        } else if (choice == 4) {
            config["rach_trigger_time_ms"] = static_cast<uint64_t>(val);
        } else if (choice == 5) {
            if (val < 0.0) val = 0.0;
            if (val > 1.0) val = 1.0;
            config["sib19"]["reacquisition_success_prob"] = val;
        } else if (choice == 6) {
            config["sib19"]["sync_loss_recovery_gap_ms"] = static_cast<uint64_t>(val);
        } else if (choice == 7) {
            uint32_t k = static_cast<uint32_t>(val);
            config["k_offset_slots"] = k;
            config["sib19"]["k_offset_slots"] = k;
        } else if (choice == 8) {
            config["sib19"]["ul_sync_validity_duration_ms"] = static_cast<uint64_t>(val);
        } else if (choice == 9) {
            config["satellite"]["vel_x_mps"] = val;
            if (config.contains("sib19") && config["sib19"].contains("ephemeris_info")) {
                config["sib19"]["ephemeris_info"]["vx"] = val;
            }
        } else if (choice == 10) {
            config["satellite"]["start_z_m"] = val;
            if (config.contains("sib19") && config["sib19"].contains("ephemeris_info")) {
                config["sib19"]["ephemeris_info"]["z"] = val;
            }
        } else if (choice == 11) {
            config["sib19"]["si_window_ms"] = static_cast<uint64_t>(val);
        }
    }
}

/**
 * @brief Interactive main menu shown on startup or when 'm' is triggered.
 * @param config Reference to the JSON scenario configuration.
 * @return bool true to start/resume simulation, false to quit.
 */
bool interactive_menu(json& config) {
    while (true) {
        std::cout << "\n\033[36m--- NTN-Aware 5G NR RAN Emulator Menu (Regenerative Satellite Architecture) ---\033[0m\n";
        std::cout << "[1] Start / Resume Simulation\n";
        std::cout << "[2] Modify Config Parameters\n";
        std::cout << "[3] Quit\n";
        std::cout << "Choice: ";

        std::string choice_str;
        std::getline(std::cin, choice_str);

        char choice = '1';
        for (char ch : choice_str) {
            if (!std::isspace(ch)) {
                choice = ch;
                break;
            }
        }

        if (choice == '3') return false;
        if (choice == '2') {
            modify_config_menu(config);
            continue;
        }

        return true;
    }
}

/**
 * @brief Paged terminal viewer for complete simulation log history.
 * @param all_logs Vector of all formatted log entries recorded during simulation.
 * @return void (Renders log lines and pauses between pages).
 */
void view_log_history(const std::vector<std::string>& all_logs) {
    std::cout << "\n\033[36m==================== COMPLETE SIMULATION LOG HISTORY ("
              << all_logs.size() << " entries) ====================\033[0m\n";
    size_t page_size = 20;
    size_t idx = 0;
    while (idx < all_logs.size()) {
        size_t end_idx = std::min(idx + page_size, all_logs.size());
        for (size_t i = idx; i < end_idx; ++i) {
            std::cout << all_logs[i] << "\n";
        }
        idx = end_idx;
        if (idx < all_logs.size()) {
            std::cout << "\033[33m-- Press Enter for next page (" << idx << "/" << all_logs.size() << ") or 'q' to return --\033[0m ";
            std::string input;
            std::getline(std::cin, input);
            if (!input.empty() && (input[0] == 'q' || input[0] == 'Q')) {
                break;
            }
        }
    }
    std::cout << "\033[36m==================== END OF LOG HISTORY ====================\033[0m\n";
    std::cout << "Press Enter to return to menu...";
    std::string tmp;
    std::getline(std::cin, tmp);
}

/**
 * @brief Runs a single simulation instance with UE and satellite gNodeB entities.
 *
 * Sets up 3GPP Release 17 regenerative NTN entities:
 * - gNodeB MAC and SIB19 Broadcaster on board the satellite.
 * - UE Timing Engine, UE RACH Controller, and UE RRC on the ground.
 * - Routes over-the-air packets over the Service Link propagation delay.
 * - Handles freeze pause ('p'/Space) without clearing screen and menu ('m').
 *
 * @param config Scenario configuration JSON object.
 * @param wants_reset Output flag indicating if the user requested a full reset.
 * @param wants_quit Output flag indicating if the user requested to terminate.
 * @return void
 */
void run_simulation_instance(json& config, bool& wants_reset, bool& wants_quit) {
    wants_reset = false;
    wants_quit = false;

    // Show initial menu
    if (!interactive_menu(config)) {
        wants_quit = true;
        return;
    }

    Simulator sim;
    ntn::ui::Dashboard dashboard;

    std::vector<std::string> all_logs;
    std::ofstream log_file("ntn_simulation.log", std::ios::out);

    // Logging Sink with explicit entity tagging and timestamping
    auto log_cb = [&dashboard, &sim, &all_logs, &log_file](const std::string& raw_msg) {
        std::string formatted = format_entity_log(raw_msg, sim.now());
        dashboard.add_log(formatted);
        all_logs.push_back(formatted);
        if (log_file.is_open()) {
            log_file << strip_ansi(formatted) << "\n";
            log_file.flush();
        }
    };

    // 1. UE Timing Engine
    UeTimingEngine timing_engine;
    uint32_t scs = config.value("scs_khz", 15);
    timing_engine.set_scs_khz(scs);

    // 2. Positions and Trajectory
    Position ue_pos{
        config["ue"]["x_m"],
        config["ue"]["y_m"],
        config["ue"]["z_m"]
    };

    Position sat_start{
        config["satellite"]["start_x_m"],
        config["satellite"]["start_y_m"],
        config["satellite"]["start_z_m"]
    };
    Velocity sat_vel{
        config["satellite"]["vel_x_mps"],
        config["satellite"]["vel_y_mps"],
        config["satellite"]["vel_z_mps"]
    };
    LinearOrbitPropagator orbit(sat_start, sat_vel, 0);

    Time_ns simulation_duration = config.value("duration_ms", 50000) * 1000000ULL;
    Time_ns update_interval = config.value("update_interval_ms", 100) * 1000000ULL;
    Time_ns rach_trigger = config.value("rach_trigger_time_ms", 1000) * 1000000ULL;

    // 3. SIB19 Configuration (Regenerative satellite: service_link_ta)
    NtnConfig sib19_cfg;
    if (config.contains("sib19")) {
        auto& s = config["sib19"];
        sib19_cfg.si_window_ms = s.value("si_window_ms", 20) * 1000000ULL;
        sib19_cfg.ul_sync_validity_duration_ms = s.value("ul_sync_validity_duration_ms", 10000) * 1000000ULL;
        sib19_cfg.reacquisition_success_prob = s.value("reacquisition_success_prob", 0.5);
        sib19_cfg.sync_loss_recovery_gap_ms = s.value("sync_loss_recovery_gap_ms", 1500) * 1000000ULL;
        sib19_cfg.k_offset_slots = s.value("k_offset_slots", 6);
        sib19_cfg.ta_info.service_link_ta_ns = s.value("service_link_ta", s.value("ta_common", 4000)) * 1000ULL;
        sib19_cfg.ta_info.service_link_ta_drift = 0.0;
        if (s.contains("ephemeris_info")) {
            sib19_cfg.ephemeris_info.x = s["ephemeris_info"].value("x", sat_start.x_m);
            sib19_cfg.ephemeris_info.y = s["ephemeris_info"].value("y", sat_start.y_m);
            sib19_cfg.ephemeris_info.z = s["ephemeris_info"].value("z", sat_start.z_m);
            sib19_cfg.ephemeris_info.vx = s["ephemeris_info"].value("vx", sat_vel.vx_mps);
            sib19_cfg.ephemeris_info.vy = s["ephemeris_info"].value("vy", sat_vel.vy_mps);
            sib19_cfg.ephemeris_info.vz = s["ephemeris_info"].value("vz", sat_vel.vz_mps);
        }
    } else {
        sib19_cfg.si_window_ms = 20 * 1000000ULL;
        sib19_cfg.ul_sync_validity_duration_ms = 10000 * 1000000ULL;
        sib19_cfg.reacquisition_success_prob = 0.5;
        sib19_cfg.sync_loss_recovery_gap_ms = 1500 * 1000000ULL;
        sib19_cfg.k_offset_slots = config.value("k_offset_slots", 6);
    }

    // Helper: Compute current Service Link OWD
    auto get_current_owd = [&](Time_ns time_now) -> Time_ns {
        Position sat_p = orbit.get_position_at(time_now);
        double slant_range = PropagationCalculator::calculate_slant_range(sat_p, ue_pos);
        return PropagationCalculator::calculate_one_way_delay(slant_range);
    };

    // Helper: Wide sky sweep angle for TUI visualization
    auto get_current_sky_angle = [&](Time_ns time_now) -> double {
        double f = (simulation_duration > 0) ? (static_cast<double>(time_now) / static_cast<double>(simulation_duration)) : 0.0;
        if (f < 0.0) f = 0.0;
        if (f > 1.0) f = 1.0;
        return 15.0 + 150.0 * f; // Wide sweep from 15 deg to 165 deg
    };

    // Helper: Physical elevation angle in degrees
    auto get_current_physical_elevation = [&](Time_ns time_now) -> double {
        Position sat_p = orbit.get_position_at(time_now);
        double slant_range = PropagationCalculator::calculate_slant_range(sat_p, ue_pos);
        if (slant_range <= 0.0) return 90.0;
        double sin_elev = sat_p.z_m / slant_range;
        if (sin_elev > 1.0) sin_elev = 1.0;
        if (sin_elev < 0.0) sin_elev = 0.0;
        return std::asin(sin_elev) * 180.0 / 3.141592653589793;
    };

    // 4. Instantiate gNodeB and UE Protocol Entities
    GnbSib19Broadcaster gnb_sib19(sim, orbit, sib19_cfg, log_cb);

    // Forward declarations for cross-channel callbacks
    std::function<void()> on_gnb_send_msg2;
    std::function<void()> on_gnb_send_msg4;
    std::function<void()> on_ue_send_msg1;
    std::function<void()> on_ue_send_msg3;

    GnbMac gnb_mac(sim, log_cb,
        [&]() { if (on_gnb_send_msg2) on_gnb_send_msg2(); },
        [&]() { if (on_gnb_send_msg4) on_gnb_send_msg4(); }
    );

    UeRachController ue_rach(sim, timing_engine, log_cb,
        [&]() { if (on_ue_send_msg1) on_ue_send_msg1(); },
        [&]() { if (on_ue_send_msg3) on_ue_send_msg3(); }
    );

    UeRrc ue_rrc(sim, timing_engine, ue_rach, log_cb);

    // Helper: Apply modified scenario configuration to active simulation runtime
    auto apply_config = [&](bool is_mid_simulation = false) {
        simulation_duration = config.value("duration_ms", 50000) * 1000000ULL;
        update_interval = config.value("update_interval_ms", 100) * 1000000ULL;

        uint32_t scs = config.value("scs_khz", 15);
        timing_engine.set_scs_khz(scs);

        // Simulation / channel modeling parameters
        if (config.contains("sib19")) {
            sib19_cfg.reacquisition_success_prob = config["sib19"].value("reacquisition_success_prob", 0.5);
            sib19_cfg.sync_loss_recovery_gap_ms = config["sib19"].value("sync_loss_recovery_gap_ms", 1500) * 1000000ULL;
        }

        // Orbital physics update
        Velocity new_vel{
            config["satellite"].value("vel_x_mps", -7500.0),
            config["satellite"].value("vel_y_mps", 0.0),
            config["satellite"].value("vel_z_mps", 0.0)
        };
        Position curr_pos = orbit.get_position_at(sim.now());
        curr_pos.z_m = config["satellite"].value("start_z_m", 600000.0);
        orbit = LinearOrbitPropagator(curr_pos, new_vel, sim.now());

        // Prepare SIB19 configuration payload
        uint32_t k_offset = config.contains("sib19")
            ? config["sib19"].value("k_offset_slots", 6)
            : config.value("k_offset_slots", 6);
        sib19_cfg.k_offset_slots = k_offset;

        if (config.contains("sib19")) {
            sib19_cfg.si_window_ms = config["sib19"].value("si_window_ms", 20) * 1000000ULL;
            sib19_cfg.ul_sync_validity_duration_ms = config["sib19"].value("ul_sync_validity_duration_ms", 10000) * 1000000ULL;
        }

        sib19_cfg.ephemeris_info.x = curr_pos.x_m;
        sib19_cfg.ephemeris_info.y = curr_pos.y_m;
        sib19_cfg.ephemeris_info.z = curr_pos.z_m;
        sib19_cfg.ephemeris_info.vx = new_vel.vx_mps;
        sib19_cfg.ephemeris_info.vy = new_vel.vy_mps;
        sib19_cfg.ephemeris_info.vz = new_vel.vz_mps;

        if (is_mid_simulation) {
            // Per 3GPP TS 38.331: Queue update at satellite gNodeB broadcast entity.
            // UE timing engine is NOT touched directly; it will receive and apply
            // the new configuration at the end of the next SI Window.
            gnb_sib19.queue_config_update(sib19_cfg);
        } else {
            // Initial setup before simulation starts (t = 0)
            gnb_sib19.set_config(sib19_cfg);
        }
    };

    // Apply configuration immediately at startup
    apply_config(false);

    // 5. Connect Over-The-Air Service Link Propagation Channels
    // Channel: UE Msg1 (PRACH Preamble) -> Satellite gNodeB MAC
    on_ue_send_msg1 = [&]() {
        Time_ns owd = get_current_owd(sim.now());
        sim.schedule(owd, [&gnb_mac, &sim, owd]() {
            gnb_mac.on_prach_detected(sim.now(), owd);
        });
    };

    // Channel: Satellite gNodeB Msg2 (RAR) -> UE RACH Controller
    on_gnb_send_msg2 = [&]() {
        Time_ns owd = get_current_owd(sim.now());
        sim.schedule(owd, [&ue_rach, &sim]() {
            ue_rach.receive_msg2(sim.now());
        });
    };

    // Channel: UE Msg3 (RRCSetupRequest / RRCReestablishmentRequest) -> Satellite gNodeB MAC
    on_ue_send_msg3 = [&]() {
        Time_ns owd = get_current_owd(sim.now());
        bool is_reest = ue_rach.is_reestablishment();
        sim.schedule(owd, [&gnb_mac, &sim, owd, is_reest]() {
            gnb_mac.on_msg3_received(sim.now(), owd, is_reest);
        });
    };

    // Channel: Satellite gNodeB Msg4 (Contention Resolution) -> UE RACH Controller
    std::function<void()> schedule_sync_evaluation_cycle;

    on_gnb_send_msg4 = [&]() {
        Time_ns owd = get_current_owd(sim.now());
        bool is_reest = ue_rach.is_reestablishment();
        sim.schedule(owd, [&ue_rach, &schedule_sync_evaluation_cycle, is_reest]() {
            ue_rach.receive_msg4(true);
            if (is_reest) {
                schedule_sync_evaluation_cycle();
            }
        });
    };

    // Helper: Execute complete SI Window transmission and UE reading strictly inside window boundaries
    auto execute_si_window = [&](bool is_recovery, std::function<void()> on_completed = nullptr) {
        // 1. gNB opens SI window
        gnb_sib19.open_si_window(sib19_cfg.si_window_ms);

        // 2. UE actively monitors PDCCH with SI-RNTI in SI window
        ue_rrc.start_monitoring_si_window();

        // 3. Inside SI window (at 2ms offset), gNB broadcasts SIB19 over BCCH
        Time_ns tx_offset = 2000000ULL; // 2 ms inside SI window
        sim.schedule(tx_offset, [&]() {
            NtnConfig broadcast_payload = gnb_sib19.broadcast_sib19(sim.now());
            sib19_cfg = broadcast_payload;
            Time_ns owd = get_current_owd(sim.now());
            // Over-the-air propagation to UE over Service Link
            sim.schedule(owd, [&ue_rrc, &sim, broadcast_payload, owd]() {
                ue_rrc.on_sib19_received(broadcast_payload, sim.now(), owd);
            });
        });

        // 4. At window close (si_window_ms)
        sim.schedule(sib19_cfg.si_window_ms, [&, is_recovery, on_completed]() {
            gnb_sib19.close_si_window();
            ue_rrc.on_si_window_ended(sim.now(), is_recovery);
            if (on_completed) {
                on_completed();
            }
        });
    };

    // Handle Random Access Problem (attempts exhausted): re-acquire SIB19 in SI window and re-attempt RACH
    ue_rach.set_on_rach_exhausted_cb([&]() {
        log_cb("\033[33m[UE-RRC] Random Access Problem detected (RACH attempts exhausted). Reopening SI window to re-read SIB19 and refresh NTN configuration...\033[0m");
        execute_si_window(false, [&]() {
            dashboard.add_event_marker(get_current_sky_angle(sim.now()), 'R');
            log_cb("\033[36m[UE-RRC] SIB19 re-acquired from SI window. Resetting counter and re-attempting RACH...\033[0m");
            ue_rach.trigger_rach();
        });
    });

    // 6. Periodic Uplink Sync Evaluation & SIB19 Reacquisition Lifecycle
    schedule_sync_evaluation_cycle = [&]() {
        Time_ns validity_duration = sib19_cfg.ul_sync_validity_duration_ms;
        Time_ns now = sim.now();
        Time_ns expiry_time = now + validity_duration;
        Time_ns proactive_trigger_time = (expiry_time > 100000000ULL) ? (expiry_time - 100000000ULL) : now;
        Time_ns delay_to_check = (proactive_trigger_time > now) ? (proactive_trigger_time - now) : 0;

        sim.schedule(delay_to_check, [&, expiry_time]() {
            double roll = static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
            if (roll < sib19_cfg.reacquisition_success_prob) {
                // CASE 1: Proactive Reacquisition Success (100 ms prior to UL sync expiry)
                log_cb("\033[36m[UE-RRC] Proactive SIB19 reacquisition triggered (100 ms prior to UL sync expiry). Opening SI window...\033[0m");
                execute_si_window(false, [&]() {
                    dashboard.add_event_marker(get_current_sky_angle(sim.now()), 'R');

                    // 3GPP TS 38.213 §4.2: Check if Service Link TA fits within SIB19 K_offset capacity
                    if (!timing_engine.is_ta_within_k_offset()) {
                        std::stringstream ss_warn;
                        ss_warn << "\033[33m[UE-RRC] Warning: SIB19 K_offset ("
                                << (timing_engine.get_k_offset_capacity_ns() / 1000) << " us, "
                                << timing_engine.get_k_offset_slots() << " slots) is less than Service Link TA ("
                                << (timing_engine.get_service_link_ta() / 1000) << " us). Uplink scheduling margin violated.\033[0m";
                        log_cb(ss_warn.str());
                        log_cb("\033[33m[UE-RRC] Further bi-directional communication cannot happen between UE and gNodeB.\033[0m");
                        log_cb("\033[33m[UE-RRC] Downlink communication operating in Unacknowledged Mode (DL RLC-UM only; UL suspended).\033[0m");
                        ue_rach.set_dl_only_um();
                    } else {
                        if (ue_rach.get_state() == RachState::DL_ONLY_UM) {
                            std::stringstream ss_rest;
                            ss_rest << "\033[32m[UE-RRC] Service Link TA ("
                                    << (timing_engine.get_service_link_ta() / 1000)
                                    << " us) now within K_offset capacity ("
                                    << (timing_engine.get_k_offset_capacity_ns() / 1000)
                                    << " us). Full bi-directional communication restored.\033[0m";
                            log_cb(ss_rest.str());
                        } else {
                            log_cb("\033[36m[UE-RRC] Reacquired SIB19 in SI window. Service Link TA refreshed. UE remains in CONNECTED state.\033[0m");
                        }
                        ue_rach.restore_connected();
                    }

                    schedule_sync_evaluation_cycle();
                });
            } else {
                // CASE 2: Proactive reception failed -> Expiry -> Mac buffer flush -> Recovery
                log_cb("\033[33m[UE-RRC] Warning: Proactive SIB19 reception failed within SI window prior to expiry.\033[0m");
                Time_ns delay_to_expiry = (expiry_time > sim.now()) ? (expiry_time - sim.now()) : 0;
                sim.schedule(delay_to_expiry, [&]() {
                    dashboard.add_event_marker(get_current_sky_angle(sim.now()), 'X');
                    ue_rrc.on_ul_sync_expired();

                    // After recovery gap, reopen SI window and reacquire SIB19
                    sim.schedule(sib19_cfg.sync_loss_recovery_gap_ms, [&]() {
                        log_cb("\033[33m[UE-RRC] Reopening SI window to reacquire SIB19 after sync loss gap...\033[0m");
                        execute_si_window(true, [&]() {
                            dashboard.add_event_marker(get_current_sky_angle(sim.now()), 'R');
                        });
                    });
                });
            }
        });
    };

    log_cb("Simulation Started (Regenerative Payload Architecture). Initializing SI window...");

    // Execute Initial SIB19 Broadcast within SI Window
    execute_si_window(false, [&]() {
        dashboard.add_event_marker(get_current_sky_angle(sim.now()), 'R');
        schedule_sync_evaluation_cycle();
    });

    // Schedule PRACH Trigger on UE
    bool pending_rach = false;
    Time_ns original_rach_trigger = rach_trigger;

    sim.schedule(rach_trigger, [&]() {
        if (!timing_engine.is_ul_sync_valid(sim.now())) {
            pending_rach = true;
            log_cb("\033[33m[UE-MAC] RACH delayed: SIB19 uplink sync not yet valid.\033[0m");
        } else {
            ue_rach.trigger_rach();
        }
    });

    // Periodic physical geometry updater
    std::function<void()> periodic_update;
    periodic_update = [&]() {
        Time_ns now = sim.now();
        Time_ns owd = get_current_owd(now);
        timing_engine.update_geometry(owd);
        sim.schedule(update_interval, periodic_update);
    };
    sim.schedule(0, periodic_update);

    dashboard.clear();

    // 7. Main Real-Time Simulation and Keyboard Event Loop
    while (true) {
        // Check if simulation duration reached
        if (sim.now() >= simulation_duration) {
            std::cout << "\n\033[33m[FINISHED] Simulation duration reached (" << (sim.now() / 1000000) << " ms)! Simulation paused.\033[0m\n";
            while (true) {
                std::cout << "[1] Modify Config (Increase duration to resume)\n[2] Reset\n[3] Quit\n[4] View Full Log History\nChoice (default 1): ";

                std::string choice_str;
                std::getline(std::cin, choice_str);
                char choice = '1';
                for (char ch : choice_str) { if (!std::isspace(ch)) { choice = ch; break; } }

                if (choice == '3') { wants_quit = true; return; }
                else if (choice == '2') { wants_reset = true; return; }
                else if (choice == '4' || choice == 'v' || choice == 'V') {
                    view_log_history(all_logs);
                    continue;
                }
                else if (choice == '1') {
                    modify_config_menu(config, sim.now());
                    apply_config(true);
                    if (sim.now() < simulation_duration) {
                        break;
                    }
                }
            }
            dashboard.clear();
        }

        // --- KEYBOARD INTERACTION & IN-PLACE FREEZE PAUSE ---
        if (ntn::ui::Keyboard::kbhit()) {
            char c = ntn::ui::Keyboard::getch();

            // Option A: In-Place Freeze Pause ('p', 'P', or Space)
            if (c == 'p' || c == 'P' || c == ' ') {
                // Freeze the simulation without clearing the screen
                std::cout << "\r \033[33;1m[PAUSED]\033[0m Simulation frozen in-place. Press 'p' or Space to resume, 'm' for Menu... " << std::flush;
                while (true) {
                    if (ntn::ui::Keyboard::kbhit()) {
                        char pkey = ntn::ui::Keyboard::getch();
                        if (pkey == 'p' || pkey == 'P' || pkey == ' ') {
                            // Resume simulation from the exact same point in-place
                            std::cout << "\r (Press 'p' or Space to Pause, 'm' for Menu)                                    \n" << std::flush;
                            break;
                        } else if (pkey == 'm' || pkey == 'M') {
                            // Switch to interactive menu
                            c = 'm';
                            break;
                        } else if (pkey == 'q' || pkey == 'Q') {
                            wants_quit = true;
                            return;
                        }
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                }
            }

            // Option B: Interactive Configuration Menu ('m' or 'M')
            if (c == 'm' || c == 'M') {
                dashboard.clear();
                while (true) {
                    std::cout << "\n\033[33m[MENU] Simulation paused.\033[0m\n";
                    std::cout << "[1] Resume\n[2] Modify Config\n[3] Reset\n[4] Quit\n[5] View Full Log History\nChoice (default 1): ";

                    std::string choice_str;
                    std::getline(std::cin, choice_str);
                    char choice = '1';
                    for (char ch : choice_str) { if (!std::isspace(ch)) { choice = ch; break; } }

                    if (choice == '4') { wants_quit = true; return; }
                    else if (choice == '3') { wants_reset = true; return; }
                    else if (choice == '5' || choice == 'v' || choice == 'V') {
                        view_log_history(all_logs);
                        continue;
                    }
                    else if (choice == '2') {
                        modify_config_menu(config, sim.now());
                        apply_config(true);
                        continue;
                    } else {
                        break; // Resume
                    }
                }
                dashboard.clear();
            } else if (c == 'q' || c == 'Q') {
                wants_quit = true;
                return;
            }
        }

        // Advance simulation tick
        sim.run(update_interval);

        // Update Dashboard Telemetry
        Time_ns now = sim.now();
        Position sat_pos = orbit.get_position_at(now);
        double slant_range = PropagationCalculator::calculate_slant_range(sat_pos, ue_pos);
        bool sync_valid = timing_engine.is_ul_sync_valid(now);

        std::stringstream ss;
        ss << "[" << std::fixed << std::setprecision(0) << sat_pos.x_m << ", "
           << sat_pos.y_m << ", " << sat_pos.z_m << "]";

        dashboard.update_simulation_time(now);
        double phys_elev = get_current_physical_elevation(now);
        double sky_angle = get_current_sky_angle(now);

        dashboard.update_geometry(slant_range / 1000.0, phys_elev, sky_angle, ss.str());
        dashboard.update_timing(timing_engine.get_k_offset_slots(), timing_engine.get_service_link_ta(), sync_valid);
        dashboard.update_rach_state(rach_state_to_string(ue_rach.get_state()));

        dashboard.render();

        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        // Delayed RACH initiation if sync became valid
        if (sync_valid && pending_rach) {
            pending_rach = false;
            std::stringstream ss_log;
            ss_log << "\033[36m[UE-MAC] Initializing delayed RACH (originally scheduled at "
                   << (original_rach_trigger / 1000000) << " ms) upon Service Link sync availability.\033[0m";
            log_cb(ss_log.str());
            ue_rach.trigger_rach();
        }
    }
}

/**
 * @brief Main entry point of the NTN RAN emulator application.
 * @param argc Argument count.
 * @param argv Argument vector (argv[1] is path to JSON configuration).
 * @return int 0 on normal exit, 1 on error.
 */
int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <config.json>" << std::endl;
        return 1;
    }

    std::ifstream f(argv[1]);
    if (!f.is_open()) {
        std::cerr << "Failed to open config file: " << argv[1] << std::endl;
        return 1;
    }

    json config;
    f >> config;

    // Silence spdlog console output so TUI rendering remains clean
    spdlog::set_level(spdlog::level::off);
    srand(static_cast<unsigned int>(time(nullptr)));

    bool wants_reset = false;
    bool wants_quit = false;

    do {
        run_simulation_instance(config, wants_reset, wants_quit);
    } while (wants_reset && !wants_quit);

    return 0;
}

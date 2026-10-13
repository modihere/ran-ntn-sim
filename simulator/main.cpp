#include <iostream>
#include <fstream>
#include <spdlog/spdlog.h>
#include <nlohmann/json.hpp>
#include "../src/common/simulator.h"
#include "../src/ntn/geometry/orbit_propagator.h"
#include "../src/ntn/geometry/propagation.h"
#include "../src/ntn/timing/timing_engine.h"
#include "../src/nr/rach/rach_controller.h"
#include "../src/ui/tui/dashboard.h"
#include "../src/ui/tui/keyboard.h"
#include "../src/nr/rrc/sib19.h"
#include <thread>
#include <chrono>
#include <sstream>
#include <iomanip>
#include <vector>

using namespace ntn::common;
using namespace ntn::geometry;
using namespace ntn::timing;
using namespace ntn::nr::rrc;
using json = nlohmann::json;

auto rach_state_to_string = [](ntn::nr::RachState state) -> std::string {
    switch (state) {
        case ntn::nr::RachState::WAITING_FOR_SIB19: return "\033[33mWAIT_SIB19\033[0m";
        case ntn::nr::RachState::IDLE: return "IDLE";
        case ntn::nr::RachState::WAIT_MSG2: return "WAIT_MSG2";
        case ntn::nr::RachState::WAIT_MSG4: return "WAIT_MSG4";
        case ntn::nr::RachState::CONNECTED: return "\033[32mCONNECTED\033[0m";
        case ntn::nr::RachState::FAILED: return "\033[31mFAILED\033[0m";
        default: return "UNKNOWN";
    }
};

void modify_config_menu(json& config) {
    while (true) {
        std::cout << "\n\033[36m--- Modify Configuration ---\033[0m\n";
        std::cout << "[1] duration_ms          : " << config.value("duration_ms", 10000) << "\n";
        std::cout << "[2] update_interval_ms   : " << config.value("update_interval_ms", 100) << "\n";
        std::cout << "[3] scs_khz              : " << config.value("scs_khz", 15) << "\n";
        std::cout << "[4] satellite.vel_x_mps  : " << config["satellite"].value("vel_x_mps", -7500.0) << "\n";
        std::cout << "[5] k_offset_slots       : " << config.value("k_offset_slots", 6) << "\n";
        std::cout << "[6] rach_trigger_time_ms : " << config.value("rach_trigger_time_ms", 1000) << "\n";
        std::cout << "[7] satellite.start_z_m  : " << config["satellite"].value("start_z_m", 600000.0) << "\n";
        std::cout << "[0] Go Back\n";
        std::cout << "Select parameter to change (0-7): ";
        
        std::string choice_str;
        std::getline(std::cin, choice_str);
        char choice = choice_str.empty() ? '0' : choice_str[0];
        
        if (choice == '0') break;

        std::string key = "";
        if (choice == '1') key = "duration_ms";
        else if (choice == '2') key = "update_interval_ms";
        else if (choice == '3') key = "scs_khz";
        else if (choice == '4') key = "satellite.vel_x_mps";
        else if (choice == '5') key = "k_offset_slots";
        else if (choice == '6') key = "rach_trigger_time_ms";
        else if (choice == '7') key = "satellite.start_z_m";
        else {
            std::cout << "Invalid choice.\n";
            continue;
        }

        std::cout << "Enter new value for " << key << ": ";
        double val;
        std::cin >> val;
        std::cin.ignore(10000, '\n');

        if (choice == '4') {
            config["satellite"]["vel_x_mps"] = val;
        } else if (choice == '7') {
            config["satellite"]["start_z_m"] = val;
        } else if (choice == '1' || choice == '2' || choice == '6') {
            config[key] = static_cast<uint64_t>(val);
        } else if (choice == '3' || choice == '5') {
            config[key] = static_cast<uint32_t>(val);
        }
    }
}

bool interactive_menu(json& config) {
    while (true) {
        std::cout << "\n\033[36m--- NTN-Aware 5G NR RAN Emulator Menu ---\033[0m\n";
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

        if (choice == '3') return false; // quit
        if (choice == '2') {
            modify_config_menu(config);
            continue;
        }

        return true; // start
    }
}

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

void run_simulation_instance(json& config, bool& wants_reset, bool& wants_quit) {
    wants_reset = false;
    wants_quit = false;

    // Initial Menu
    if (!interactive_menu(config)) {
        wants_quit = true;
        return;
    }

    Simulator sim;
    NtnTimingEngine timing_engine;
    ntn::ui::Dashboard dashboard;

    std::vector<std::string> all_logs;
    std::ofstream log_file("ntn_simulation.log", std::ios::out);

    auto strip_ansi = [](const std::string& str) {
        std::string res;
        bool in_escape = false;
        for (char c : str) {
            if (c == '\033') in_escape = true;
            else if (in_escape && c == 'm') in_escape = false;
            else if (!in_escape) res += c;
        }
        return res;
    };

    auto log_cb = [&dashboard, &sim, &all_logs, &log_file, strip_ansi](const std::string& msg) {
        std::stringstream ss;
        ss << "[LOG] " << (sim.now() / 1000000) << " ms: " << msg;
        std::string formatted = ss.str();
        dashboard.add_log(formatted);
        all_logs.push_back(formatted);
        if (log_file.is_open()) {
            log_file << strip_ansi(formatted) << "\n";
            log_file.flush();
        }
    };

    ntn::nr::RachController rach(sim, timing_engine, log_cb);

    uint32_t scs = config.value("scs_khz", 15);
    timing_engine.set_scs_khz(scs);

    Position ue_pos{
        config["ue"]["x_m"],
        config["ue"]["y_m"],
        config["ue"]["z_m"]
    };
    
    // We will initialize orbit with 0, but update it when SIB19 is received.
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
    
    Time_ns simulation_duration = config.value("duration_ms", 10000) * 1000000ULL;
    Time_ns update_interval = config.value("update_interval_ms", 100) * 1000000ULL;
    Time_ns rach_trigger = config.value("rach_trigger_time_ms", 1000) * 1000000ULL;

    bool pending_rach = false;
    Time_ns original_rach_trigger = 0;

    // SIB19 Config Parsing
    NtnConfig sib19_cfg;
    if (config.contains("sib19")) {
        auto& s = config["sib19"];
        sib19_cfg.si_window_ms = s.value("si_window_ms", 20) * 1000000ULL;
        sib19_cfg.ul_sync_validity_duration_ms = s.value("ul_sync_validity_duration_ms", 10000) * 1000000ULL;
        sib19_cfg.reacquisition_success_prob = s.value("reacquisition_success_prob", 0.5);
        sib19_cfg.sync_loss_recovery_gap_ms = s.value("sync_loss_recovery_gap_ms", 400) * 1000000ULL;
        sib19_cfg.k_offset_slots = s.value("k_offset_slots", 6);
        sib19_cfg.ta_info.ta_common = s.value("ta_common", 4000);
        sib19_cfg.ta_info.ta_common_drift = 0;
        if (s.contains("ephemeris_info")) {
            sib19_cfg.ephemeris_info.x = s["ephemeris_info"].value("x", 600000.0);
            sib19_cfg.ephemeris_info.y = s["ephemeris_info"].value("y", 0.0);
            sib19_cfg.ephemeris_info.z = s["ephemeris_info"].value("z", 600000.0);
            sib19_cfg.ephemeris_info.vx = s["ephemeris_info"].value("vx", -7500.0);
            sib19_cfg.ephemeris_info.vy = s["ephemeris_info"].value("vy", 0.0);
            sib19_cfg.ephemeris_info.vz = s["ephemeris_info"].value("vz", 0.0);
        }
    } else {
        // Fallback
        sib19_cfg.si_window_ms = 20 * 1000000ULL;
        sib19_cfg.ul_sync_validity_duration_ms = 10000 * 1000000ULL;
        sib19_cfg.reacquisition_success_prob = 0.5;
        sib19_cfg.sync_loss_recovery_gap_ms = 400 * 1000000ULL;
        sib19_cfg.k_offset_slots = config.value("k_offset_slots", 6);
    }

    auto get_current_sky_angle = [&](Time_ns time_now) -> double {
        double f = (simulation_duration > 0) ? (static_cast<double>(time_now) / static_cast<double>(simulation_duration)) : 0.0;
        if (f < 0.0) f = 0.0;
        if (f > 1.0) f = 1.0;
        return 15.0 + 150.0 * f; // Wide sweep from 15 deg to 165 deg across the full duration
    };

    auto get_current_physical_elevation = [&](Time_ns time_now) -> double {
        Position sat_p = orbit.get_position_at(time_now);
        double slant_range = PropagationCalculator::calculate_slant_range(sat_p, ue_pos);
        if (slant_range <= 0.0) return 90.0;
        double sin_elev = sat_p.z_m / slant_range;
        if (sin_elev > 1.0) sin_elev = 1.0;
        if (sin_elev < 0.0) sin_elev = 0.0;
        return std::asin(sin_elev) * 180.0 / 3.141592653589793;
    };

    auto update_sib19_params = [&](Time_ns time_now, const std::string& context_tag) {
        Position sat_p = orbit.get_position_at(time_now);
        double slant_range = PropagationCalculator::calculate_slant_range(sat_p, ue_pos);
        Time_ns owd = PropagationCalculator::calculate_one_way_delay(slant_range);
        
        sib19_cfg.ephemeris_info.x = sat_p.x_m;
        sib19_cfg.ephemeris_info.y = sat_p.y_m;
        sib19_cfg.ephemeris_info.z = sat_p.z_m;
        sib19_cfg.ta_info.ta_common = 2 * owd;
        
        timing_engine.update_geometry(owd);
        timing_engine.update_ta(time_now);
        
        std::stringstream ss;
        ss << "\033[36m[" << context_tag << "] Ephemeris updated: Sat Pos=["
           << std::fixed << std::setprecision(0) << sat_p.x_m << ", " << sat_p.y_m << ", " << sat_p.z_m
           << "] m, Common TA=" << (2 * owd / 1000) << " us\033[0m";
        log_cb(ss.str());
    };

    std::function<void()> schedule_sync_evaluation_cycle;
    schedule_sync_evaluation_cycle = [&]() {
        Time_ns validity_duration = sib19_cfg.ul_sync_validity_duration_ms;
        Time_ns now = sim.now();
        Time_ns expiry_time = now + validity_duration;
        Time_ns proactive_trigger_time = (expiry_time > 100000000ULL) ? (expiry_time - 100000000ULL) : now;
        Time_ns delay_to_check = (proactive_trigger_time > now) ? (proactive_trigger_time - now) : 0;
        
        sim.schedule(delay_to_check, [&, expiry_time]() {
            double roll = static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
            if (roll < sib19_cfg.reacquisition_success_prob) {
                // CASE 1: Proactive Reacquisition Success (100ms before expiry)
                log_cb("\033[36m[SIB19] Proactive reacquisition triggered (100 ms prior to UL sync expiry). Opening SI window...\033[0m");
                sim.schedule(sib19_cfg.si_window_ms, [&]() {
                    dashboard.add_event_marker(get_current_sky_angle(sim.now()), 'R');
                    update_sib19_params(sim.now(), "SIB19-REACQUIRED");
                    log_cb("\033[36m[SIB19] Reacquired in SI window! UL sync timer refreshed. UE remains in CONNECTED state.\033[0m");
                    schedule_sync_evaluation_cycle();
                });
            } else {
                // CASE 2: Proactive SIB19 reception failed -> Expiry -> Mac buffer flush -> Delayed recovery
                log_cb("\033[33m[SIB19] Warning: Proactive SIB19 reception failed within SI window prior to expiry.\033[0m");
                Time_ns delay_to_expiry = (expiry_time > sim.now()) ? (expiry_time - sim.now()) : 0;
                sim.schedule(delay_to_expiry, [&]() {
                    dashboard.add_event_marker(get_current_sky_angle(sim.now()), 'X');
                    rach.lose_sync_and_flush();
                    
                    // After gap of recovery gap ms, reopen SI window and reacquire SIB19
                    sim.schedule(sib19_cfg.sync_loss_recovery_gap_ms, [&]() {
                        log_cb("\033[33m[RRC] Reopening SI window to reacquire SIB19 after sync loss gap...\033[0m");
                        sim.schedule(sib19_cfg.si_window_ms, [&]() {
                            dashboard.add_event_marker(get_current_sky_angle(sim.now()), 'R');
                            update_sib19_params(sim.now(), "SIB19-RECOVERED");
                            rach.restore_connected();
                            schedule_sync_evaluation_cycle();
                        });
                    });
                });
            }
        });
    };

    log_cb("Simulation Started. SIB19 configurations will be applied when the SI window ends.");

    sim.schedule(sib19_cfg.si_window_ms / 2, [&log_cb]() {
        log_cb("Receiving SIB19 configuration from broadcast...");
    });

    sim.schedule(sib19_cfg.si_window_ms, [&]() {
        log_cb("\033[32mSI Window ended. Activating initial SIB19 configurations.\033[0m");
        timing_engine.set_k_offset_slots(sib19_cfg.k_offset_slots);
        timing_engine.set_ul_sync_validity_duration(sib19_cfg.ul_sync_validity_duration_ms);
        
        // Update orbit propagator with T0 = current time
        Position e_pos{sib19_cfg.ephemeris_info.x, sib19_cfg.ephemeris_info.y, sib19_cfg.ephemeris_info.z};
        Velocity e_vel{sib19_cfg.ephemeris_info.vx, sib19_cfg.ephemeris_info.vy, sib19_cfg.ephemeris_info.vz};
        orbit = LinearOrbitPropagator(e_pos, e_vel, sim.now());

        update_sib19_params(sim.now(), "RRC-SIB19-INIT");
        dashboard.add_event_marker(get_current_sky_angle(sim.now()), 'R');
        rach.receive_sib19();
        
        schedule_sync_evaluation_cycle();
    });

    auto trigger_rach_sequence = [&rach, &timing_engine, &sim]() {
        rach.trigger_rach();
        Time_ns owd = timing_engine.get_common_ta() / 2;
        sim.schedule(owd + 5000000ULL, [&rach, &timing_engine, &sim, owd]() {
            if (rach.get_state() == ntn::nr::RachState::WAIT_MSG2) {
                rach.receive_msg2(sim.now());
                Time_ns msg3_arrival = sim.now() + owd + (timing_engine.get_k_offset_slots() * 1000000ULL) + 2000000ULL;
                sim.schedule(msg3_arrival - sim.now() + 5000000ULL, [&rach]() {
                    rach.receive_msg4(true);
                });
            }
        });
    };

    sim.schedule(rach_trigger, [&]() {
        if (!timing_engine.is_ul_sync_valid(sim.now())) {
            pending_rach = true;
            original_rach_trigger = rach_trigger;
            log_cb("RACH delayed due to INVALID sync.");
        } else {
            trigger_rach_sequence();
        }
    });

    std::function<void()> periodic_update;
    periodic_update = [&]() {
        Time_ns now = sim.now();
        
        Position sat_pos = orbit.get_position_at(now);
        double slant_range = PropagationCalculator::calculate_slant_range(sat_pos, ue_pos);
        Time_ns owd = PropagationCalculator::calculate_one_way_delay(slant_range);
        
        timing_engine.update_geometry(owd);
        
        sim.schedule(update_interval, periodic_update);
    };

    sim.schedule(0, periodic_update);
    dashboard.clear();

    while (true) {
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
                    modify_config_menu(config);
                    simulation_duration = config.value("duration_ms", 10000) * 1000000ULL;
                    Velocity new_vel{config["satellite"]["vel_x_mps"], config["satellite"]["vel_y_mps"], config["satellite"]["vel_z_mps"]};
                    Position curr_pos = orbit.get_position_at(sim.now());
                    curr_pos.z_m = config["satellite"].value("start_z_m", 600000.0);
                    orbit = LinearOrbitPropagator(curr_pos, new_vel, sim.now());
                    timing_engine.set_k_offset_slots(config.value("k_offset_slots", 6));
                    
                    if (sim.now() < simulation_duration) {
                        break; // Successfully increased, resume
                    } else {
                        std::cout << "\n\033[31mDuration must be greater than current time (" << (sim.now() / 1000000) << " ms) to resume.\033[0m\n";
                    }
                }
            }
            dashboard.clear();
        }
        // --- INTERRUPT LOGIC ---
        if (ntn::ui::Keyboard::kbhit()) {
            char c = ntn::ui::Keyboard::getch();
            if (c == '\n' || c == '\r') { } // Ignore lone newlines
            
            dashboard.clear();
            while (true) {
                std::cout << "\n\033[33m[INTERRUPT] Simulation paused.\033[0m\n";
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
                    modify_config_menu(config);
                    // Dynamically apply vel, height & K_offset
                    Velocity new_vel{config["satellite"]["vel_x_mps"], config["satellite"]["vel_y_mps"], config["satellite"]["vel_z_mps"]};
                    Position curr_pos = orbit.get_position_at(sim.now());
                    curr_pos.z_m = config["satellite"].value("start_z_m", 600000.0);
                    orbit = LinearOrbitPropagator(curr_pos, new_vel, sim.now());
                    timing_engine.set_k_offset_slots(config.value("k_offset_slots", 6));
                    continue; // Re-show interrupt menu
                } else {
                    break; // Resume
                }
            }
            dashboard.clear();
        }

        // Run one tick
        sim.run(update_interval);
        
        // Render Dashboard
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
        dashboard.update_timing(timing_engine.get_k_offset_slots(), timing_engine.get_common_ta(), sync_valid);
        dashboard.update_rach_state(rach_state_to_string(rach.get_state()));
        
        dashboard.render();
        
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        
        if (sync_valid && pending_rach) {
            pending_rach = false;
            std::stringstream ss;
            ss << "RACH started (Originally config " << (original_rach_trigger / 1000000) << " ms) due to sync availability.";
            log_cb(ss.str());
            sim.schedule(0, trigger_rach_sequence);
        }
    }
}

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
    
    // Silence spdlog to keep TUI clean
    spdlog::set_level(spdlog::level::off);
    srand(static_cast<unsigned int>(time(nullptr)));

    bool wants_reset = false;
    bool wants_quit = false;

    do {
        run_simulation_instance(config, wants_reset, wants_quit);
    } while (wants_reset && !wants_quit);

    return 0;
}

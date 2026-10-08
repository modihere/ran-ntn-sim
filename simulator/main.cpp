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
#include <thread>
#include <chrono>
#include <sstream>
#include <iomanip>
#include <vector>

using namespace ntn::common;
using namespace ntn::geometry;
using namespace ntn::timing;
using json = nlohmann::json;

auto rach_state_to_string = [](ntn::nr::RachState state) -> std::string {
    switch (state) {
        case ntn::nr::RachState::IDLE: return "IDLE";
        case ntn::nr::RachState::WAIT_MSG2: return "WAIT_MSG2";
        case ntn::nr::RachState::WAIT_MSG4: return "WAIT_MSG4";
        case ntn::nr::RachState::COMPLETED: return "\033[32mCOMPLETED\033[0m";
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
        std::cout << "[0] Go Back\n";
        std::cout << "Select parameter to change (0-6): ";
        
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

    auto log_cb = [&dashboard, &sim](const std::string& msg) {
        std::stringstream ss;
        ss << "[LOG] " << (sim.now() / 1000000) << " ms: " << msg;
        dashboard.add_log(ss.str());
    };

    ntn::nr::RachController rach(sim, timing_engine, log_cb);

    uint32_t scs = config.value("scs_khz", 15);
    timing_engine.set_scs_khz(scs);
    timing_engine.set_k_offset_slots(config.value("k_offset_slots", 6));

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
    
    Time_ns simulation_duration = config.value("duration_ms", 10000) * 1000000ULL;
    Time_ns update_interval = config.value("update_interval_ms", 100) * 1000000ULL;
    Time_ns rach_trigger = config.value("rach_trigger_time_ms", 1000) * 1000000ULL;

    sim.schedule(rach_trigger, [&]() {
        rach.trigger_rach();
        Time_ns owd = timing_engine.get_common_ta() / 2;
        sim.schedule(owd + 5000000ULL, [&]() {
            if (rach.get_state() == ntn::nr::RachState::WAIT_MSG2) {
                rach.receive_msg2(sim.now());
                Time_ns msg3_arrival = sim.now() + owd + (timing_engine.get_k_offset_slots() * 1000000ULL) + 2000000ULL;
                sim.schedule(msg3_arrival - sim.now() + 5000000ULL, [&]() {
                    rach.receive_msg4(true);
                });
            }
        });
    });

    std::function<void()> periodic_update;
    periodic_update = [&]() {
        Time_ns now = sim.now();
        if (now > simulation_duration) return;

        Position sat_pos = orbit.get_position_at(now);
        double slant_range = PropagationCalculator::calculate_slant_range(sat_pos, ue_pos);
        Time_ns owd = PropagationCalculator::calculate_one_way_delay(slant_range);
        
        timing_engine.update_geometry(owd);
        if ((now % (500 * 1000000ULL)) == 0) {
            timing_engine.update_ta();
        }
        
        sim.schedule(update_interval, periodic_update);
    };

    sim.schedule(0, periodic_update);
    dashboard.clear();

    while (sim.now() < simulation_duration) {
        // --- INTERRUPT LOGIC ---
        if (ntn::ui::Keyboard::kbhit()) {
            char c = ntn::ui::Keyboard::getch();
            if (c == '\n' || c == '\r') { } // Ignore lone newlines
            
            dashboard.clear();
            while (true) {
                std::cout << "\n\033[33m[INTERRUPT] Simulation paused.\033[0m\n";
                std::cout << "[1] Resume\n[2] Modify Config\n[3] Reset\n[4] Quit\nChoice (default 1): ";
                
                std::string choice_str;
                std::getline(std::cin, choice_str);
                char choice = '1';
                for (char ch : choice_str) { if (!std::isspace(ch)) { choice = ch; break; } }

                if (choice == '4') { wants_quit = true; return; } 
                else if (choice == '3') { wants_reset = true; return; } 
                else if (choice == '2') {
                    modify_config_menu(config);
                    // Dynamically apply vel & K_offset
                    Velocity new_vel{config["satellite"]["vel_x_mps"], config["satellite"]["vel_y_mps"], config["satellite"]["vel_z_mps"]};
                    orbit = LinearOrbitPropagator(orbit.get_position_at(sim.now()), new_vel, sim.now());
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
        double elevation = 90.0 - ((slant_range - 600000.0) / 400000.0) * 45.0;
        if (elevation < 0) elevation = 0;
        if (elevation > 180) elevation = 180;
        
        dashboard.update_geometry(slant_range / 1000.0, elevation, ss.str());
        dashboard.update_timing(timing_engine.get_k_offset_slots(), timing_engine.get_common_ta(), sync_valid);
        dashboard.update_rach_state(rach_state_to_string(rach.get_state()));
        
        dashboard.render();
        
        if (!sync_valid) {
            std::cout << "\n\033[33m[SYNC DROPPED] Uplink Sync is INVALID! Simulation paused.\033[0m\n";
            std::cout << "K_offset is too small for the current physical RTT or TA drift exceeded limits.\n";
            while (true) {
                std::cout << "[1] Resume (Force)\n[2] Modify Config (Fix K_offset/Vel)\n[3] Reset\n[4] Quit\nChoice (default 2): ";
                
                std::string choice_str;
                std::getline(std::cin, choice_str);
                char choice = '2';
                for (char ch : choice_str) { if (!std::isspace(ch)) { choice = ch; break; } }

                if (choice == '4') { wants_quit = true; return; } 
                else if (choice == '3') { wants_reset = true; return; } 
                else if (choice == '2') {
                    modify_config_menu(config);
                    Velocity new_vel{config["satellite"]["vel_x_mps"], config["satellite"]["vel_y_mps"], config["satellite"]["vel_z_mps"]};
                    orbit = LinearOrbitPropagator(orbit.get_position_at(sim.now()), new_vel, sim.now());
                    timing_engine.set_k_offset_slots(config.value("k_offset_slots", 6));
                    continue; 
                } else {
                    break; // Resume
                }
            }
            dashboard.clear();
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
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

    bool wants_reset = false;
    bool wants_quit = false;

    do {
        run_simulation_instance(config, wants_reset, wants_quit);
    } while (wants_reset && !wants_quit);

    return 0;
}

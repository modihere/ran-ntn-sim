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

bool interactive_menu(json& config) {
    while (true) {
        std::cout << "\n\033[36m--- NTN-Aware 5G NR RAN Emulator Menu ---\033[0m\n";
        std::cout << "[s] Start Simulation\n";
        std::cout << "[m] Modify Config Parameter\n";
        std::cout << "[q] Quit\n";
        std::cout << "Choice: ";
        
        std::string choice_str;
        std::getline(std::cin, choice_str);
        
        char choice = 's';
        for (char ch : choice_str) {
            if (!std::isspace(ch)) {
                choice = ch;
                break;
            }
        }

        if (choice == 'q') return false; // stop

        if (choice == 'm') {
            std::string key;
            double val;
            std::cout << "Available keys: duration_ms, update_interval_ms, scs_khz, satellite.vel_x_mps\n";
            std::cout << "Enter key: ";
            std::cin >> key;
            std::cout << "Enter new value: ";
            std::cin >> val;

            if (key == "duration_ms" || key == "update_interval_ms" || key == "scs_khz") {
                config[key] = val;
            } else if (key == "satellite.vel_x_mps") {
                config["satellite"]["vel_x_mps"] = val;
            } else {
                std::cout << "Unknown key.\n";
            }
            std::cin.ignore(10000, '\n');
            continue; // show menu again
        }

        return true;
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
    ntn::nr::RachController rach(sim, timing_engine);
    ntn::ui::Dashboard dashboard;

    uint32_t scs = config.value("scs_khz", 15);
    timing_engine.set_scs_khz(scs);

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

    sim.schedule(1000000000ULL, [&]() {
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

    bool is_paused = false;
    while (sim.now() < simulation_duration) {
        if (is_paused) {
            dashboard.clear(); // Clear before menu
            if (!interactive_menu(config)) {
                wants_quit = true;
                return;
            }
            // User might have chosen reset
            // We should check if they actually pressed 'r'
            // Oh, wait, interactive_menu doesn't easily convey 'reset' vs 'resume'.
            // Let's refactor interactive_menu logic here for interrupt!
        }
        
        // --- INTERRUPT LOGIC ---
        if (ntn::ui::Keyboard::kbhit()) {
            char c = ntn::ui::Keyboard::getch();
            // Ignore any leftover newlines
            if (c == '\n' || c == '\r') {
                // If it's just a newline in buffer, we can either pause or just continue.
                // Let's pause to be safe.
            }

            dashboard.clear();
            while (true) {
                std::cout << "\n\033[33m[INTERRUPT] Simulation paused.\033[0m\n";
                std::cout << "[r] Reset  |  [q] Quit  |  [m] Modify config  |  [s] Resume\nChoice (default 's'): ";
                
                std::string choice_str;
                std::getline(std::cin, choice_str);
                
                char choice = 's';
                for (char ch : choice_str) {
                    if (!std::isspace(ch)) {
                        choice = ch;
                        break;
                    }
                }

                if (choice == 'q') {
                    wants_quit = true;
                    return;
                } else if (choice == 'r') {
                    wants_reset = true;
                    return;
                } else if (choice == 'm') {
                    std::string key; double val;
                    std::cout << "Available keys: duration_ms, update_interval_ms, scs_khz, satellite.vel_x_mps\nKey: ";
                    std::cin >> key;
                    std::cout << "New value: ";
                    std::cin >> val;
                    std::cin.ignore(10000, '\n'); // flush
                    if (key == "duration_ms" || key == "update_interval_ms" || key == "scs_khz") config[key] = val;
                    else if (key == "satellite.vel_x_mps") {
                        config["satellite"]["vel_x_mps"] = val;
                        Velocity sat_vel{config["satellite"]["vel_x_mps"], config["satellite"]["vel_y_mps"], config["satellite"]["vel_z_mps"]};
                        orbit = LinearOrbitPropagator(orbit.get_position_at(sim.now()), sat_vel, sim.now());
                    }
                    continue; // Re-show interrupt menu
                } else {
                    // Resume
                    break;
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
        if (elevation > 90) elevation = 90;
        
        dashboard.update_geometry(slant_range / 1000.0, elevation, ss.str());
        dashboard.update_timing(timing_engine.get_k_offset_slots(), timing_engine.get_common_ta(), sync_valid);
        dashboard.update_rach_state(rach_state_to_string(rach.get_state()));
        
        dashboard.render();
        
        if (!sync_valid) {
            std::cout << "\n\033[33m[INTERRUPT] Uplink Sync is INVALID! Simulation paused.\033[0m\n";
            std::cout << "Press ENTER to resume simulation... " << std::flush;
            std::cin.ignore(10000, '\n'); 
            std::cin.get();
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

#include <iostream>
#include <fstream>
#include <spdlog/spdlog.h>
#include <nlohmann/json.hpp>
#include "../src/common/simulator.h"
#include "../src/ntn/geometry/orbit_propagator.h"
#include "../src/ntn/geometry/propagation.h"
#include "../src/ntn/timing/timing_engine.h"
#include "../src/nr/rach/rach_controller.h"

using namespace ntn::common;
using namespace ntn::geometry;
using namespace ntn::timing;
using json = nlohmann::json;

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

    spdlog::set_level(spdlog::level::info);
    spdlog::info("Starting NTN RAN Emulator");

    Simulator sim;
    NtnTimingEngine timing_engine;
    ntn::nr::RachController rach(sim, timing_engine);
    
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

    // Trigger RACH at 1 second
    sim.schedule(1000000000ULL, [&]() {
        rach.trigger_rach();
        
        // Mock gNB sending Msg2 (RAR) after half RTT + processing time
        Time_ns owd = timing_engine.get_common_ta() / 2;
        sim.schedule(owd + 5000000ULL, [&]() {
            if (rach.get_state() == ntn::nr::RachState::WAIT_MSG2) {
                rach.receive_msg2(sim.now());
                
                // Mock gNB sending Msg4 after Msg3 arrives (which takes RTT + K_offset + proc time)
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
        
        // Emulate periodic TA update
        if ((now % (500 * 1000000ULL)) == 0) {
            timing_engine.update_ta();
        }

        bool sync_valid = timing_engine.is_ul_sync_valid(now);

        spdlog::info("Time: {}ms | Sat: [{:.0f}, {:.0f}, {:.0f}] | Range: {:.2f}km | K_offset: {} slots | Common TA: {}ns | Sync: {}",
                     now / 1000000,
                     sat_pos.x_m, sat_pos.y_m, sat_pos.z_m,
                     slant_range / 1000.0,
                     timing_engine.get_k_offset_slots(),
                     timing_engine.get_common_ta(),
                     sync_valid ? "VALID" : "INVALID");

        sim.schedule(update_interval, periodic_update);
    };

    sim.schedule(0, periodic_update);
    sim.run(simulation_duration + 1000000);

    spdlog::info("Simulation Complete.");
    return 0;
}


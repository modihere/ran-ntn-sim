#include "dashboard.h"
#include <iostream>
#include <iomanip>

namespace ntn::ui {

Dashboard::Dashboard() = default;

void Dashboard::update_simulation_time(uint64_t time_ns) {
    current_time_ns_ = time_ns;
}

void Dashboard::update_geometry(double slant_range_km, double elevation_deg, const std::string& sat_pos_str) {
    slant_range_km_ = slant_range_km;
    elevation_deg_ = elevation_deg;
    sat_pos_str_ = sat_pos_str;
}

void Dashboard::update_timing(uint32_t k_offset, uint64_t common_ta_ns, bool sync_valid) {
    k_offset_ = k_offset;
    common_ta_ns_ = common_ta_ns;
    sync_valid_ = sync_valid;
}

void Dashboard::update_rach_state(const std::string& state_str) {
    rach_state_ = state_str;
}

void Dashboard::render() const {
    // Clear screen and move cursor to top-left using ANSI escape codes
    std::cout << "\033[2J\033[H";
    
    std::cout << "========================================================================\n";
    std::cout << "                   NTN-Aware 5G NR RAN Emulator                         \n";
    std::cout << "========================================================================\n";
    
    std::cout << " [ Clock ]\n";
    std::cout << " Simulation Time: " << (current_time_ns_ / 1000000) << " ms\n";
    std::cout << "------------------------------------------------------------------------\n";
    
    std::cout << " [ NTN Geometry ]\n";
    std::cout << " Satellite Pos: " << sat_pos_str_ << "\n";
    std::cout << " Slant Range:   " << std::fixed << std::setprecision(2) << slant_range_km_ << " km\n";
    std::cout << " Elevation:     " << std::fixed << std::setprecision(2) << elevation_deg_ << " deg\n";
    std::cout << "------------------------------------------------------------------------\n";
    
    std::cout << " [ Timing Engine ]\n";
    std::cout << " K_offset:      " << k_offset_ << " slots\n";
    std::cout << " Common TA:     " << (common_ta_ns_ / 1000) << " us\n";
    std::cout << " Sync Status:   " << (sync_valid_ ? "\033[32mVALID\033[0m" : "\033[31mINVALID\033[0m") << "\n";
    std::cout << "------------------------------------------------------------------------\n";
    
    std::cout << " [ NR Protocol ]\n";
    std::cout << " RACH State:    " << rach_state_ << "\n";
    std::cout << "========================================================================\n";
    std::cout << std::flush;
}

void Dashboard::clear() const {
    std::cout << "\033[2J\033[H" << std::flush;
}

} // namespace ntn::ui

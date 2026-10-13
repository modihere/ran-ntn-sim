#pragma once

#include <string>
#include <cstdint>
#include <deque>
#include <vector>

namespace ntn::ui {

class Dashboard {
public:
    Dashboard();
    
    // Updates
    void update_simulation_time(uint64_t time_ns);
    void update_geometry(double slant_range_km, double elevation_deg, double sky_angle_deg, const std::string& sat_pos_str);
    void update_timing(uint32_t k_offset, uint64_t common_ta_ns, bool sync_valid);
    void update_rach_state(const std::string& state_str);
    
    // Render
    void render();
    void clear() const;
    void add_log(const std::string& msg);
    void add_event_marker(double sky_angle_deg, char type);

private:
    void enable_virtual_terminal();

    struct EventMarker {
        double sky_angle_deg;
        char type; // 'R' = reacquired, 'X' = sync lost
    };

    bool first_render_{true};
    std::deque<std::string> logs_;
    std::vector<EventMarker> event_markers_;
    const size_t MAX_LOGS = 14;
    uint64_t current_time_ns_{0};
    double slant_range_km_{0.0};
    double elevation_deg_{0.0};
    double sky_angle_deg_{15.0};
    std::string sat_pos_str_;
    uint32_t k_offset_{0};
    uint64_t common_ta_ns_{0};
    bool sync_valid_{true};
    std::string rach_state_{"IDLE"};
    bool line_persistent_{false};
};

} // namespace ntn::ui

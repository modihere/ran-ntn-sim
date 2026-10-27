/**
 * @file dashboard.cpp
 * @brief Implementation of Terminal User Interface (TUI) Dashboard for regenerative NTN.
 *
 * Implements dual-pane ANSI telemetry rendering, drawing the orbital arc,
 * positioning the satellite and RF beam, plotting event markers ('R' and 'X'),
 * and displaying the real-time event log buffer with in-place freeze pause controls.
 */

#include "dashboard.h"
#include <iostream>
#include <iomanip>
#include <sstream>
#include <cmath>
#include <vector>

#ifdef _WIN32
#include <windows.h>
#endif

namespace ntn::ui {

/**
 * @brief Constructs Dashboard and enables Windows ANSI terminal processing.
 */
Dashboard::Dashboard() {
    enable_virtual_terminal();
}

/**
 * @brief Enables Windows virtual terminal sequences for ANSI escape code support.
 */
void Dashboard::enable_virtual_terminal() {
#ifdef _WIN32
    HANDLE hOut = GetStdHandle(STD_OUTPUT_HANDLE);
    if (hOut != INVALID_HANDLE_VALUE) {
        DWORD dwMode = 0;
        if (GetConsoleMode(hOut, &dwMode)) {
            dwMode |= ENABLE_VIRTUAL_TERMINAL_PROCESSING;
            SetConsoleMode(hOut, dwMode);
        }
    }
#endif
}

/**
 * @brief Updates current simulation time.
 * @param time_ns Simulation time in nanoseconds.
 */
void Dashboard::update_simulation_time(uint64_t time_ns) {
    current_time_ns_ = time_ns;
}

/**
 * @brief Updates satellite geometric and sky angle coordinates.
 * @param slant_range_km Slant range in kilometers.
 * @param elevation_deg Geometric elevation angle in degrees above horizon.
 * @param sky_angle_deg Visual sky dome arc angle in degrees [15, 165].
 * @param sat_pos_str 3D position string [x, y, z] in meters.
 */
void Dashboard::update_geometry(double slant_range_km, double elevation_deg, double sky_angle_deg, const std::string& sat_pos_str) {
    slant_range_km_ = slant_range_km;
    elevation_deg_ = elevation_deg;
    sky_angle_deg_ = sky_angle_deg;
    sat_pos_str_ = sat_pos_str;
}

/**
 * @brief Updates uplink timing parameters for regenerative NTN architecture.
 * @param k_offset Cell-specific scheduling offset in slots.
 * @param service_link_ta_ns Round-trip Service Link Timing Advance in nanoseconds.
 * @param sync_valid Uplink synchronization validity status.
 */
void Dashboard::update_timing(uint32_t k_offset, uint64_t service_link_ta_ns, bool sync_valid) {
    k_offset_ = k_offset;
    service_link_ta_ns_ = service_link_ta_ns;
    sync_valid_ = sync_valid;
}

/**
 * @brief Updates current RACH protocol state string.
 * @param state_str Formatted state string.
 */
void Dashboard::update_rach_state(const std::string& state_str) {
    rach_state_ = state_str;
}

/**
 * @brief Appends a telemetry message string to the circular display buffer.
 * @param msg Formatted log message string.
 */
void Dashboard::add_log(const std::string& msg) {
    logs_.push_back(msg);
    if (logs_.size() > MAX_LOGS) {
        logs_.pop_front();
    }
}

/**
 * @brief Records an event marker along the visual orbital sky arc.
 * @param sky_angle_deg Normalized arc angle in degrees [15, 165].
 * @param type Marker type: 'R' (reacquisition on arc), 'X' (sync loss below arc).
 */
void Dashboard::add_event_marker(double sky_angle_deg, char type) {
    event_markers_.push_back({sky_angle_deg, type});
}

/**
 * @brief Bresenham's line algorithm for drawing the connection beam in the character grid.
 * @param grid Character grid buffer.
 * @param x0 Starting X coordinate.
 * @param y0 Starting Y coordinate.
 * @param x1 Destination X coordinate.
 * @param y1 Destination Y coordinate.
 * @param ch Character to draw.
 */
void draw_line(std::vector<std::string>& grid, int x0, int y0, int x1, int y1, char ch) {
    int dx = std::abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -std::abs(y1 - y0), sy = y0 < y1 ? 1 : -1; 
    int err = dx + dy, e2;

    for (;;) {
        if (y0 >= 0 && y0 < static_cast<int>(grid.size()) && x0 >= 0 && x0 < static_cast<int>(grid[0].size())) {
            if (grid[y0][x0] == ' ') grid[y0][x0] = ch;
        }
        if (x0 == x1 && y0 == y1) break;
        e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

/**
 * @brief Renders the complete dashboard view to stdout.
 */
void Dashboard::render() {
    if (first_render_) {
        std::cout << "\033[2J";
        first_render_ = false;
    }
    std::cout << "\033[H";
    
    std::cout << "====================================================================================================\n";
    std::cout << "               NTN-Aware 5G NR RAN Emulator (Regenerative Satellite gNodeB)                         \n";
    std::cout << "====================================================================================================\n";
    
    auto format_row = [](const std::string& label, const std::string& val) {
        std::stringstream ss;
        ss << " " << std::left << std::setw(16) << label << ": " << val;
        std::string res = ss.str();
        return res;
    };

    std::vector<std::string> left_pane = {
        " [ Clock ]                              ",
        format_row("Simulation Time", std::to_string(current_time_ns_ / 1000000) + " ms"),
        "----------------------------------------",
        " [ NTN Geometry ]                       ",
        format_row("Satellite Pos", sat_pos_str_),
        format_row("Slant Range", [this]() { std::stringstream ss; ss << std::fixed << std::setprecision(2) << slant_range_km_ << " km"; return ss.str(); }()),
        format_row("Elevation", [this]() { std::stringstream ss; ss << std::fixed << std::setprecision(2) << elevation_deg_ << " deg"; return ss.str(); }()),
        "----------------------------------------",
        " [ Timing Engine ]                      ",
        format_row("K_offset", std::to_string(k_offset_) + " slots"),
        format_row("Service Link TA", std::to_string(service_link_ta_ns_ / 1000) + " us"),
        format_row("Sync Status", std::string(sync_valid_ ? "\033[32mVALID\033[0m  " : "\033[31mINVALID\033[0m")),
        "----------------------------------------",
        " [ NR Protocol ]                        ",
        format_row("RACH State", rach_state_),
        "                                        "
    };

    // Ensure left pane has 18 lines to match right pane
    left_pane.push_back("                                        ");
    left_pane.push_back("                                        ");

    // Pad left pane properly so ANSI codes don't distort column width
    for (auto& s : left_pane) {
        size_t visible_len = 0;
        bool in_ansi = false;
        for (char c : s) {
            if (c == '\033') in_ansi = true;
            else if (in_ansi && c == 'm') in_ansi = false;
            else if (!in_ansi) visible_len++;
        }
        while (visible_len < 38) { s += " "; visible_len++; }
    }

    const int W = 58;
    const int H = 18;
    std::vector<std::string> right_pane(H, std::string(W, ' '));
    
    int cx = 29;
    int cy = 16;
    int rx = 27;
    int ry = 13;

    // Dynamically draw sky dome arc
    for (int deg = 0; deg <= 180; deg += 3) {
        double r = deg * 3.141592653589793 / 180.0;
        int ax = cx - static_cast<int>(std::cos(r) * rx);
        int ay = cy - static_cast<int>(std::sin(r) * ry);
        if (ax >= 0 && ax < W && ay >= 0 && ay < H) {
            if (right_pane[ay][ax] == ' ') right_pane[ay][ax] = '.';
        }
    }

    // Draw Ground exactly centered for W=58 (26 '=' + 6 char UE + 26 '=')
    right_pane[16] = "==========================[ UE ]==========================";

    double rad = sky_angle_deg_ * 3.141592653589793 / 180.0;
    int sat_x = cx - static_cast<int>(std::cos(rad) * rx);
    int sat_y = cy - static_cast<int>(std::sin(rad) * ry);
    
    if (sat_x < 0) sat_x = 0; 
    if (sat_x >= W) sat_x = W - 1;
    if (sat_y < 0) sat_y = 0; 
    if (sat_y > 15) sat_y = 15;

    // If RACH is active or connected, draw persistent connection beam if sync is valid
    if (sync_valid_ && (rach_state_.find("WAIT") != std::string::npos || 
                        rach_state_.find("COMPLETED") != std::string::npos || 
                        rach_state_.find("CONNECTED") != std::string::npos || 
                        rach_state_.find("DL-ONLY") != std::string::npos || 
                        line_persistent_)) {
        char beam_char = '*';
        if (rach_state_.find("DL-ONLY") != std::string::npos) {
            beam_char = 'v'; // Unidirectional downlink beam for DL-only Unacknowledged Mode
        }
        draw_line(right_pane, cx, 16, sat_x, sat_y, beam_char);
    }

    // Draw Event Markers on arc ('R' on arc row, 'X' directly below arc row so they never collide)
    for (const auto& marker : event_markers_) {
        double r = marker.sky_angle_deg * 3.141592653589793 / 180.0;
        int mx = cx - static_cast<int>(std::cos(r) * rx);
        int my = cy - static_cast<int>(std::sin(r) * ry);
        if (mx >= 0 && mx < W) {
            if (marker.type == 'R') {
                if (my >= 0 && my < 16) {
                    right_pane[my][mx] = 'R';
                }
            } else if (marker.type == 'X') {
                int y_below = my + 1;
                if (y_below >= 0 && y_below < 16) {
                    right_pane[y_below][mx] = 'X';
                }
            }
        }
    }

    // Draw satellite label [SAT]
    if (sat_y >= 0 && sat_y < H && sat_x >= 0 && sat_x < W) {
        if (sat_x > 2 && sat_x < W - 3) {
            right_pane[sat_y].replace(sat_x - 2, 5, "[SAT]");
        }
    }

    // Legend line below ground
    if (rach_state_.find("DL-ONLY") != std::string::npos) {
        right_pane[17] = "Arc: '.' Orbit  'v' DL-UM  'R' Reacquire  'X' Sync Lost";
    } else {
        right_pane[17] = "Arc: '.' Orbit  '*' Beam  'R' Reacquire  'X' Sync Lost";
    }

    for (size_t i = 0; i < left_pane.size() && i < right_pane.size(); ++i) {
        std::cout << left_pane[i] << " | " << right_pane[i] << "\n";
    }

    std::cout << "====================================================================================================\n";
    
    // Render Logs
    for (const auto& log : logs_) {
        std::cout << log << "\n";
    }
    // Pad to ensure clear screen below logs
    for (size_t i = logs_.size(); i < MAX_LOGS; ++i) {
        std::cout << "                                                                                                    \n";
    }
    
    std::cout << "====================================================================================================\n";
    std::cout << " (Press 'p' or Space to Pause, 'm' for Menu)\n";
    std::cout << std::flush;
}

/**
 * @brief Clears the entire terminal screen.
 */
void Dashboard::clear() const {
    std::cout << "\033[2J\033[H" << std::flush;
}

} // namespace ntn::ui

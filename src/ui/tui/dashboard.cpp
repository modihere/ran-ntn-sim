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

Dashboard::Dashboard() {
    enable_virtual_terminal();
}

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

void Dashboard::add_log(const std::string& msg) {
    logs_.push_back(msg);
    if (logs_.size() > MAX_LOGS) {
        logs_.pop_front();
    }
}

// Bresenham's line algorithm
void draw_line(std::vector<std::string>& grid, int x0, int y0, int x1, int y1, char ch) {
    int dx = std::abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -std::abs(y1 - y0), sy = y0 < y1 ? 1 : -1; 
    int err = dx + dy, e2;

    for (;;) {
        if (y0 >= 0 && y0 < (int)grid.size() && x0 >= 0 && x0 < (int)grid[0].size()) {
            if (grid[y0][x0] == ' ') grid[y0][x0] = ch;
        }
        if (x0 == x1 && y0 == y1) break;
        e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

void Dashboard::render() {
    if (first_render_) {
        std::cout << "\033[2J";
        first_render_ = false;
    }
    std::cout << "\033[H";
    
    std::cout << "====================================================================================================\n";
    std::cout << "                             NTN-Aware 5G NR RAN Emulator                                           \n";
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
        format_row("Common TA", std::to_string(common_ta_ns_ / 1000) + " us"),
        format_row("Sync Status", std::string(sync_valid_ ? "\033[32mVALID\033[0m  " : "\033[31mINVALID\033[0m")),
        "----------------------------------------",
        " [ NR Protocol ]                        ",
        format_row("RACH State", rach_state_),
        "                                        "
    };

    // Pad left pane properly so ANSI codes don't mess up width calculation
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
    const int H = 16;
    std::vector<std::string> right_pane(H, std::string(W, ' '));
    
    // Draw Arc
    right_pane[2] =  "                 .   -   '   -   .                  ";
    right_pane[3] =  "             '                       '              ";
    right_pane[4] =  "           '                           '            ";
    right_pane[5] =  "         '                               '          ";
    right_pane[6] =  "       .                                   .        ";

    // Draw Ground
    right_pane[12] = "======================[ UE ]======================";

    double rad = elevation_deg_ * 3.14159265 / 180.0;
    int cx = 28;
    int cy = 10;
    int rx = 24;
    int ry = 8;
    int sat_x = cx - (int)(std::cos(rad) * rx);
    int sat_y = cy - (int)(std::sin(rad) * ry);
    
    if (sat_x < 0) sat_x = 0; 
    if (sat_x >= W) sat_x = W - 1;
    if (sat_y < 0) sat_y = 0; 
    if (sat_y > 11) sat_y = 11;

    // If RACH is active or has happened, draw persistent beam
    if (rach_state_.find("WAIT") != std::string::npos || rach_state_.find("COMPLETED") != std::string::npos || line_persistent_) {
        draw_line(right_pane, cx, 12, sat_x, sat_y, '*');
    }

    // Draw SAT
    if (sat_y >= 0 && sat_y < H && sat_x >= 0 && sat_x < W) {
        if (sat_x > 2 && sat_x < W - 3) {
            right_pane[sat_y].replace(sat_x - 2, 5, "[SAT]");
        }
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
    std::cout << " (Press 'm' or 'Enter' for Menu / Pause) \n";
    std::cout << std::flush;
}
void Dashboard::clear() const {
    std::cout << "\033[2J\033[H" << std::flush;
}

} // namespace ntn::ui

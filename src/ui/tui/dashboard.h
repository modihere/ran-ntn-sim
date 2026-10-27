/**
 * @file dashboard.h
 * @brief Terminal User Interface (TUI) Dashboard for the NTN RAN Emulator.
 *
 * Implements a dual-pane ANSI terminal visualizer formatted for 100x32 terminals:
 * - Left pane: High-resolution clock, orbital geometry, Service Link TA (regenerative),
 *   K_offset, Uplink Synchronization status, and RACH protocol state.
 * - Right pane: Dynamic 180-degree sky dome arc with real-time satellite position,
 *   visual connection beam, and dedicated event markers for SIB19 reacquisition ('R')
 *   and sync loss ('X').
 * - Bottom pane: Real-time telemetry log feed with in-place freeze pause controls.
 */

#pragma once

#include <string>
#include <cstdint>
#include <deque>
#include <vector>

namespace ntn::ui {

/**
 * @class Dashboard
 * @brief Renders real-time telemetry, orbital visualization, and event logs to the terminal.
 */
class Dashboard {
public:
    /**
     * @brief Constructs the Dashboard and initializes virtual terminal processing.
     */
    Dashboard();
    
    /**
     * @brief Updates the current simulation time display.
     * @param time_ns Current simulation time in nanoseconds.
     */
    void update_simulation_time(uint64_t time_ns);

    /**
     * @brief Updates the satellite geometric position, physical elevation, and sky arc angle.
     * @param slant_range_km Slant range between satellite and ground UE in kilometers.
     * @param elevation_deg Physical geometric elevation angle in degrees above horizon.
     * @param sky_angle_deg Normalized sky arc visualization angle in degrees [15, 165].
     * @param sat_pos_str Formatted string containing satellite 3D coordinates.
     */
    void update_geometry(double slant_range_km, double elevation_deg, double sky_angle_deg, const std::string& sat_pos_str);

    /**
     * @brief Updates uplink timing parameters for regenerative NTN architecture.
     * @param k_offset Cell-specific scheduling K_offset in slots.
     * @param service_link_ta_ns Round-trip Service Link Timing Advance in nanoseconds.
     * @param sync_valid Uplink synchronization validity status.
     */
    void update_timing(uint32_t k_offset, uint64_t service_link_ta_ns, bool sync_valid);

    /**
     * @brief Updates the displayed RACH protocol state string.
     * @param state_str ANSI-formatted state string (e.g. CONNECTED, WAIT_MSG2).
     */
    void update_rach_state(const std::string& state_str);
    
    /**
     * @brief Renders the entire dashboard frame to stdout using ANSI escape sequences.
     */
    void render();

    /**
     * @brief Clears the terminal screen and resets cursor position.
     */
    void clear() const;

    /**
     * @brief Appends a formatted log string to the circular display buffer.
     * @param msg Formatted log message string.
     */
    void add_log(const std::string& msg);

    /**
     * @brief Plots an event marker onto the visual orbital arc trajectory.
     * @param sky_angle_deg Sky arc angle in degrees [15, 165] where event occurred.
     * @param type Character marker ('R' = SIB19 reacquired on arc, 'X' = sync lost below arc).
     */
    void add_event_marker(double sky_angle_deg, char type);

private:
    /**
     * @brief Enables ANSI virtual terminal processing on Windows platforms.
     */
    void enable_virtual_terminal();

    /**
     * @brief Event marker structure positioned along the orbital sky arc.
     */
    struct EventMarker {
        double sky_angle_deg{15.0}; ///< Sky arc angle in degrees
        char type{' '};             ///< Marker character ('R' or 'X')
    };

    bool first_render_{true};                        ///< Flag to trigger full screen clear on initial draw
    std::deque<std::string> logs_;                   ///< Circular buffer of active log lines
    std::vector<EventMarker> event_markers_;         ///< Stored historical event markers
    const size_t MAX_LOGS{14};                       ///< Maximum number of visible log rows
    uint64_t current_time_ns_{0};                    ///< Simulation timestamp [ns]
    double slant_range_km_{0.0};                     ///< Slant range [km]
    double elevation_deg_{0.0};                      ///< Geometric elevation angle [deg]
    double sky_angle_deg_{15.0};                     ///< Visual sky dome arc angle [deg]
    std::string sat_pos_str_;                        ///< Formatted position string
    uint32_t k_offset_{0};                           ///< K_offset slots
    uint64_t service_link_ta_ns_{0};                 ///< Service Link TA [ns] (feeder link = 0 in regenerative)
    bool sync_valid_{true};                          ///< Synchronization status
    std::string rach_state_{"IDLE"};                 ///< Protocol state string
    bool line_persistent_{false};                    ///< Persistence flag for RF beam
};

} // namespace ntn::ui

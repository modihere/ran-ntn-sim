/**
 * @file sib19_broadcaster.cpp
 * @brief Implementation of satellite gNodeB SIB19 broadcaster for regenerative NTN.
 *
 * Implements SIB19 generation, SI window boundary management, staging of pending
 * configuration updates for the next SI window, and explicit [gNB-RRC] telemetry logging.
 */

#include "sib19_broadcaster.h"
#include <iomanip>
#include <sstream>

namespace ntn::gnb::rrc {

/**
 * @brief Constructs the gNodeB SIB19 Broadcaster.
 * @param sim Simulation core reference.
 * @param orbit Orbit propagator reference.
 * @param config Initial SIB19 parameters.
 * @param log_cb Telemetry log sink callback.
 */
GnbSib19Broadcaster::GnbSib19Broadcaster(
    common::Simulator& sim,
    geometry::LinearOrbitPropagator& orbit,
    const common::NtnConfig& config,
    std::function<void(const std::string&)> log_cb
) : sim_(sim), orbit_(orbit), config_(config), pending_config_(config), log_cb_(log_cb) {}

/**
 * @brief Announces the opening of the SI window for SIB19 transmission.
 * @param duration_ns Duration of the SI window in nanoseconds.
 */
void GnbSib19Broadcaster::open_si_window(common::Time_ns duration_ns) {
    if (log_cb_) {
        std::stringstream ss;
        ss << "\033[35m[gNB-RRC] Opening SI Window (duration: " 
           << (duration_ns / 1000000) << " ms) for SIB19 transmission.\033[0m";
        log_cb_(ss.str());
    }
}

/**
 * @brief Announces the closure of the SI window.
 */
void GnbSib19Broadcaster::close_si_window() {
    if (log_cb_) {
        log_cb_("\033[35m[gNB-RRC] SI Window closed.\033[0m");
    }
}

/**
 * @brief Queues updated SIB19 configuration parameters to take effect on the next SI window broadcast.
 * @param new_config Updated NtnConfig parameters.
 */
void GnbSib19Broadcaster::queue_config_update(const common::NtnConfig& new_config) {
    pending_config_ = new_config;
    has_pending_config_ = true;

    if (log_cb_) {
        std::stringstream ss;
        ss << "\033[35m[gNB-RRC] Updated SIB19 broadcast configuration queued: K_offset="
           << pending_config_.k_offset_slots << " slots, Validity="
           << (pending_config_.ul_sync_validity_duration_ms / 1000000)
           << " ms. Will be broadcast in the next SI Window.\033[0m";
        log_cb_(ss.str());
    }
}

/**
 * @brief Generates and broadcasts the current SIB19 payload over the air within the SI window.
 * @param current_time Simulation timestamp in nanoseconds.
 * @return SIB19 NtnConfig with updated ephemeris information.
 */
common::NtnConfig GnbSib19Broadcaster::broadcast_sib19(common::Time_ns current_time) {
    // If pending updates exist from mid-run config modifications, activate them for this SI window
    if (has_pending_config_) {
        config_ = pending_config_;
        has_pending_config_ = false;
    }

    // Obtain latest propagated satellite position and velocity from physical orbit
    common::Position sat_pos = orbit_.get_position_at(current_time);
    common::Velocity sat_vel = orbit_.get_velocity();
    
    // Update SIB19 ephemeris block
    config_.ephemeris_info.x = sat_pos.x_m;
    config_.ephemeris_info.y = sat_pos.y_m;
    config_.ephemeris_info.z = sat_pos.z_m;
    config_.ephemeris_info.vx = sat_vel.vx_mps;
    config_.ephemeris_info.vy = sat_vel.vy_mps;
    config_.ephemeris_info.vz = sat_vel.vz_mps;
    
    if (log_cb_) {
        std::stringstream ss;
        ss << "\033[35m[gNB-RRC] Broadcasting SIB19 on BCCH inside SI Window. Ephemeris=["
           << std::fixed << std::setprecision(0) << sat_pos.x_m << ", "
           << sat_pos.y_m << ", " << sat_pos.z_m << "] m, K_offset="
           << config_.k_offset_slots << " slots, Validity="
           << (config_.ul_sync_validity_duration_ms / 1000000) << " ms.\033[0m";
        log_cb_(ss.str());
    }
    
    return config_;
}

} // namespace ntn::gnb::rrc

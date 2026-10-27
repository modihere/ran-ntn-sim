/**
 * @file sib19_broadcaster.h
 * @brief Satellite gNodeB RRC entity for broadcasting System Information Block 19 (SIB19).
 *
 * In 3GPP Release 17 NTN (TS 38.331), SIB19 carries the ntn-Config Information Element.
 * In a regenerative satellite architecture, the gNodeB is on board the satellite.
 * It broadcasts orbital ephemeris coordinates, scheduling K_offset, and the
 * ulSyncValidityDuration timer over-the-air inside periodic System Information (SI) windows.
 */

#pragma once

#include <functional>
#include <string>
#include "../../common/simulator.h"
#include "../../common/types.h"
#include "../../ntn/geometry/orbit_propagator.h"

namespace ntn::gnb::rrc {

/**
 * @class GnbSib19Broadcaster
 * @brief Manages the periodic generation and over-the-air broadcast of SIB19 from the satellite gNodeB.
 */
class GnbSib19Broadcaster {
public:
    /**
     * @brief Constructs the satellite gNodeB SIB19 Broadcaster.
     * @param sim Reference to the simulation core.
     * @param orbit Reference to the satellite orbit propagator.
     * @param config Initial SIB19 NTN configuration.
     * @param log_cb Log sink callback function.
     */
    GnbSib19Broadcaster(
        common::Simulator& sim,
        geometry::LinearOrbitPropagator& orbit,
        const common::NtnConfig& config,
        std::function<void(const std::string&)> log_cb = nullptr
    );

    /**
     * @brief Announces the opening of the System Information (SI) window for SIB19.
     * @param duration_ns Duration of the SI window in nanoseconds.
     */
    void open_si_window(common::Time_ns duration_ns);

    /**
     * @brief Announces the closing of the System Information (SI) window.
     */
    void close_si_window();

    /**
     * @brief Queues updated SIB19 configuration parameters to be broadcast in the next SI window.
     * @param new_config Updated NtnConfig parameters.
     */
    void queue_config_update(const common::NtnConfig& new_config);

    /**
     * @brief Generates and broadcasts the current SIB19 payload over the satellite BCCH.
     *
     * Queries the satellite orbit propagator for the latest 3D position and velocity
     * vectors at the broadcast timestamp, updates the SIB19 ephemeris block, and delivers
     * the payload to the channel.
     *
     * @param current_time Current simulation time in nanoseconds.
     * @return common::NtnConfig The broadcast SIB19 configuration block.
     */
    common::NtnConfig broadcast_sib19(common::Time_ns current_time);

    /**
     * @brief Returns the active SIB19 configuration block.
     * @return Current NtnConfig struct.
     */
    const common::NtnConfig& get_config() const { return config_; }

    /**
     * @brief Updates the SIB19 configuration parameters immediately (used at initial setup).
     * @param new_config Updated NtnConfig parameters.
     */
    void set_config(const common::NtnConfig& new_config) { 
        config_ = new_config; 
        pending_config_ = new_config;
    }

private:
    common::Simulator& sim_;                     ///< Simulator core reference
    geometry::LinearOrbitPropagator& orbit_;      ///< Satellite orbit propagator
    common::NtnConfig config_;                   ///< Active broadcast SIB19 configuration
    common::NtnConfig pending_config_;           ///< Queued SIB19 configuration for next SI window
    bool has_pending_config_{false};             ///< Flag indicating queued updates waiting for next SI window
    std::function<void(const std::string&)> log_cb_; ///< Telemetry log sink
};

} // namespace ntn::gnb::rrc

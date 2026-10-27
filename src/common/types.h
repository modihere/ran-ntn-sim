/**
 * @file types.h
 * @brief Common 3GPP and Non-Terrestrial Network (NTN) data types and structures.
 *
 * This header defines core mathematical and protocol data types shared across
 * the UE (User Equipment), gNodeB (satellite base station), and physics engines,
 * including 3D Cartesian coordinates, orbital ephemeris structures, regenerative
 * Service Link Timing Advance (TA) information, and RACH protocol state enumerations.
 */

#pragma once

#include <cstdint>
#include <string>
#include <cmath>

namespace ntn::common {

/**
 * @brief 3D Cartesian position vector in meters [m].
 */
struct Position {
    double x_m{0.0}; ///< Position along X axis in meters
    double y_m{0.0}; ///< Position along Y axis in meters
    double z_m{0.0}; ///< Position along Z axis (altitude) in meters

    /**
     * @brief Computes Euclidean distance to another Cartesian point.
     * @param other Target Position.
     * @return double Distance in meters.
     */
    double distance_to(const Position& other) const {
        return std::sqrt(std::pow(x_m - other.x_m, 2) + 
                         std::pow(y_m - other.y_m, 2) + 
                         std::pow(z_m - other.z_m, 2));
    }
};

/**
 * @brief 3D Cartesian velocity vector in meters per second [m/s].
 */
struct Velocity {
    double vx_mps{0.0}; ///< Velocity along X axis in m/s
    double vy_mps{0.0}; ///< Velocity along Y axis in m/s
    double vz_mps{0.0}; ///< Velocity along Z axis in m/s
};

/**
 * @brief Orbital ephemeris state vector broadcast by gNodeB in SIB19.
 *
 * In 3GPP Rel-17 NTN (TS 38.331), ephemeris information describes the satellite's
 * position and velocity vectors at a reference epoch time, allowing the UE to
 * predict satellite motion and calculate service link propagation delay.
 */
struct EphemerisInfo {
    double x{0.0};  ///< Satellite X coordinate in meters
    double y{0.0};  ///< Satellite Y coordinate in meters
    double z{0.0};  ///< Satellite Z coordinate in meters
    double vx{0.0}; ///< Satellite velocity along X axis in m/s
    double vy{0.0}; ///< Satellite velocity along Y axis in m/s
    double vz{0.0}; ///< Satellite velocity along Z axis in m/s
};

/**
 * @brief Service Link Timing Advance (TA) Information for Regenerative Payloads.
 *
 * In a regenerative satellite architecture, the gNodeB is located on board the
 * satellite, meaning the feeder link delay is zero. The Timing Advance corresponds
 * entirely to the round-trip propagation delay over the Service Link (satellite to UE).
 */
struct ServiceLinkTaInfo {
    uint64_t service_link_ta_ns{0}; ///< Round-trip service link delay in nanoseconds [ns]
    double service_link_ta_drift{0.0}; ///< Rate of change of service link delay [ns/s]
};

/**
 * @brief SIB19 NTN Configuration Block broadcast by the satellite gNodeB.
 *
 * Encapsulates System Information Block 19 parameters per 3GPP TS 38.331,
 * including System Information (SI) window duration, ephemeris information,
 * Service Link Timing Advance parameters, scheduling K_offset, and validity duration.
 */
struct NtnConfig {
    uint64_t si_window_ms{20};                   ///< SI window duration in milliseconds
    uint64_t ul_sync_validity_duration_ms{10000};///< Uplink sync validity timer in milliseconds
    ServiceLinkTaInfo ta_info;                  ///< Service link Timing Advance information
    EphemerisInfo ephemeris_info;               ///< Orbital ephemeris state vector
    uint32_t k_offset_slots{6};                 ///< Cell-specific K_offset in slots
    double reacquisition_success_prob{0.5};     ///< Autonomous SIB19 reacquisition success rate
    uint64_t sync_loss_recovery_gap_ms{1500};   ///< Gap before re-opening SI window after sync loss
};

/**
 * @brief Random Access Channel (RACH) procedure states per 3GPP TS 38.321.
 */
enum class RachState {
    WAITING_FOR_SIB19, ///< UE has not acquired SIB19; all uplink transmissions blocked
    IDLE,              ///< SIB19 acquired; UE is ready to initiate PRACH
    WAIT_MSG2,         ///< Msg1 (PRACH Preamble) sent; UE monitoring RAR window
    WAIT_MSG4,         ///< Msg3 (RRCSetupRequest) sent with K_offset; monitoring contention timer
    CONNECTED,         ///< Msg4 received; Contention resolution successful; RRC Connected
    DL_ONLY_UM,        ///< Connected in Downlink Unacknowledged Mode (K_offset < TA, UL suspended)
    FAILED,            ///< RACH attempt timed out or contention lost
    COMPLETED = CONNECTED ///< Alias for backward compatibility
};

/**
 * @brief Converts a RachState enumeration value to an ANSI-colored string representation.
 * @param state The RachState enum to convert.
 * @return std::string Formatted string suitable for terminal display.
 */
inline std::string rach_state_to_string(RachState state) {
    switch (state) {
        case RachState::WAITING_FOR_SIB19: return "\033[33mWAIT_SIB19\033[0m";
        case RachState::IDLE:              return "IDLE";
        case RachState::WAIT_MSG2:         return "WAIT_MSG2";
        case RachState::WAIT_MSG4:         return "WAIT_MSG4";
        case RachState::CONNECTED:         return "\033[32mCONNECTED\033[0m";
        case RachState::DL_ONLY_UM:        return "\033[33mDL-ONLY (UM)\033[0m";
        case RachState::FAILED:            return "\033[31mFAILED\033[0m";
        default:                           return "UNKNOWN";
    }
}

} // namespace ntn::common


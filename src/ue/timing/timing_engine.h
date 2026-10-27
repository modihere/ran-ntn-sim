/**
 * @file timing_engine.h
 * @brief UE-side NTN Timing Engine for autonomous Service Link TA pre-compensation.
 *
 * Implements 3GPP Release 17 NTN uplink timing alignment on the UE side.
 * In a regenerative satellite architecture (where gNodeB is on board the satellite),
 * the feeder link delay is absent (0 ns). The UE autonomously calculates its Service
 * Link Timing Advance (T_service = 2 * tau_service) from satellite ephemeris and
 * GNSS coordinates, enforces the cell-specific K_offset scheduling margin, and monitors
 * the Uplink Synchronization Validity Duration timer broadcast in SIB19.
 */

#pragma once

#include <cstdint>
#include "../../common/simulator.h"
#include "../../common/types.h"

namespace ntn::ue::timing {

/**
 * @class UeTimingEngine
 * @brief Computes and validates UE uplink transmission timing in regenerative NTN networks.
 */
class UeTimingEngine {
public:
    /**
     * @brief Configures the Subcarrier Spacing (SCS) in kHz and updates slot duration.
     * @param scs_khz Subcarrier spacing in kHz (default: 15 kHz -> 1 ms slot).
     */
    void set_scs_khz(uint32_t scs_khz = 15);

    /**
     * @brief Configures the cell-specific scheduling offset (K_offset) in slots.
     * @param slots Number of scheduling offset slots broadcast in SIB19.
     */
    void set_k_offset_slots(uint32_t slots);

    /**
     * @brief Configures the Uplink Synchronization Validity Duration timer from SIB19.
     * @param duration Validity duration in nanoseconds.
     */
    void set_ul_sync_validity_duration(common::Time_ns duration);

    /**
     * @brief Updates the physical service link geometry with the current one-way delay.
     * @param one_way_delay Current one-way propagation delay in nanoseconds.
     */
    void update_geometry(common::Time_ns one_way_delay);

    /**
     * @brief Triggers a Service Link TA update and resets the uplink sync validity timer.
     * @param current_time Current simulation time in nanoseconds.
     */
    void update_service_link_ta(common::Time_ns current_time);

    /**
     * @brief Returns the total Service Link Timing Advance (round-trip delay).
     * @return Service Link Timing Advance in nanoseconds (2 * one_way_delay).
     */
    common::Time_ns get_service_link_ta() const;

    /**
     * @brief Backward compatibility alias for get_service_link_ta().
     * @return Service Link Timing Advance in nanoseconds.
     */
    common::Time_ns get_common_ta() const { return get_service_link_ta(); }

    /**
     * @brief Returns the cell-specific K_offset in slots.
     * @return uint32_t Number of K_offset slots.
     */
    uint32_t get_k_offset_slots() const;

    /**
     * @brief Returns the configured Uplink Sync Validity Duration.
     * @return Validity duration in nanoseconds.
     */
    common::Time_ns get_ul_sync_validity_duration() const { return ul_sync_validity_duration_; }

    /**
     * @brief Returns the simulation timestamp of the last successful TA update.
     * @return Timestamp in nanoseconds.
     */
    common::Time_ns get_last_ta_update_time() const { return last_ta_update_time_; }

    /**
     * @brief Computes expected arrival timestamp at the satellite gNodeB for a transmission.
     * @param tx_time Uplink transmission departure time in nanoseconds.
     * @return Expected arrival time at satellite in nanoseconds (tx_time + one_way_delay).
     */
    common::Time_ns calculate_expected_ul_arrival(common::Time_ns tx_time) const;

    /**
     * @brief Validates if SIB19 uplink synchronization is currently valid per 3GPP TS 38.331.
     *
     * Validates:
     * 1. SIB19 has been received and initial sync has been performed.
     * 2. Uplink Sync Validity Duration timer (ulSyncValidityDuration-r17) has not expired.
     *
     * In 3GPP Rel-17, valid uplink sync permits PRACH (Msg1) transmissions regardless
     * of cellSpecificKoffset.
     *
     * @param current_time Current simulation time in nanoseconds.
     * @return true if uplink synchronization is valid, false if expired or unacquired.
     */
    bool is_ul_sync_valid(common::Time_ns current_time) const;

    /**
     * @brief Validates whether the current round-trip Service Link TA fits within the K_offset scheduling capacity.
     *
     * Per 3GPP TS 38.213 §8.3 and TS 38.321 §5.1.4, Msg3 PUSCH scheduling is delayed by K_offset slots.
     * If the round-trip Service Link TA exceeds K_offset * slot_duration, the timing requirement cannot
     * be met and Msg3 transmission must be blocked.
     *
     * @return true if Service Link TA <= K_offset * slot_duration; false otherwise.
     */
    bool is_ta_within_k_offset() const;

    /**
     * @brief Returns the total scheduling offset capacity in nanoseconds (K_offset_slots * slot_duration).
     * @return Offset capacity in nanoseconds.
     */
    common::Time_ns get_k_offset_capacity_ns() const;

    /**
     * @brief Returns the duration of an NR slot in nanoseconds for the active numerology (SCS).
     * @return Slot duration in nanoseconds.
     */
    common::Time_ns get_slot_duration_ns() const { return slot_duration_ns_; }

private:
    uint32_t scs_khz_{15};                           ///< Subcarrier spacing in kHz
    uint32_t k_offset_slots_{0};                     ///< K_offset in slots
    common::Time_ns slot_duration_ns_{1000000};      ///< Slot duration in nanoseconds (1 ms for 15 kHz)
    common::Time_ns current_owd_ns_{0};              ///< Current one-way service link delay [ns]
    common::Time_ns last_ta_update_owd_ns_{0};       ///< OWD recorded at last TA update [ns]
    common::Time_ns ul_sync_validity_duration_{0};   ///< SIB19 UL sync validity duration [ns]
    common::Time_ns last_ta_update_time_{0};         ///< Timestamp of last TA update [ns]
    bool has_synced_{false};                         ///< Flag indicating if initial sync has occurred
};

} // namespace ntn::ue::timing


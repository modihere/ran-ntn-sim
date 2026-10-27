/**
 * @file timing_engine.cpp
 * @brief Implementation of UE-side NTN Timing Engine for regenerative payload architecture.
 *
 * Implements autonomous Service Link TA pre-compensation calculation, slot duration
 * sizing, K_offset margin verification, and SIB19 uplink sync validity timer tracking.
 */

#include "timing_engine.h"

namespace ntn::ue::timing {

/**
 * @brief Configures Subcarrier Spacing (SCS) and updates the internal slot duration.
 * @param scs_khz SCS in kHz (15, 30, 60, or 120 kHz).
 */
void UeTimingEngine::set_scs_khz(uint32_t scs_khz) {
    scs_khz_ = scs_khz;
    if (scs_khz_ == 15) slot_duration_ns_ = 1000000;
    else if (scs_khz_ == 30) slot_duration_ns_ = 500000;
    else if (scs_khz_ == 60) slot_duration_ns_ = 250000;
    else if (scs_khz_ == 120) slot_duration_ns_ = 125000;
    else slot_duration_ns_ = 1000000;
}

/**
 * @brief Configures cell-specific K_offset slots.
 * @param slots Number of K_offset slots broadcast in SIB19.
 */
void UeTimingEngine::set_k_offset_slots(uint32_t slots) {
    k_offset_slots_ = slots;
}

/**
 * @brief Sets the Uplink Sync Validity Duration timer.
 * @param duration Validity duration in nanoseconds.
 */
void UeTimingEngine::set_ul_sync_validity_duration(common::Time_ns duration) {
    ul_sync_validity_duration_ = duration;
}

/**
 * @brief Updates service link geometry with the latest calculated one-way delay.
 * @param one_way_delay One-way propagation delay in nanoseconds.
 */
void UeTimingEngine::update_geometry(common::Time_ns one_way_delay) {
    current_owd_ns_ = one_way_delay;
    if (last_ta_update_owd_ns_ == 0) {
        last_ta_update_owd_ns_ = current_owd_ns_;
    }
}

/**
 * @brief Records a Service Link TA update event, resetting the validity timer.
 * @param current_time Current simulation time in nanoseconds.
 */
void UeTimingEngine::update_service_link_ta(common::Time_ns current_time) {
    last_ta_update_owd_ns_ = current_owd_ns_;
    last_ta_update_time_ = current_time;
    has_synced_ = true;
}

/**
 * @brief Returns total Service Link Timing Advance (2 * one_way_delay).
 * @return Round-trip service link delay in nanoseconds.
 */
common::Time_ns UeTimingEngine::get_service_link_ta() const {
    return 2 * current_owd_ns_;
}

/**
 * @brief Returns K_offset in slots.
 * @return Number of K_offset slots.
 */
uint32_t UeTimingEngine::get_k_offset_slots() const {
    return k_offset_slots_;
}

/**
 * @brief Calculates the expected arrival time of an uplink transmission at the satellite.
 * @param tx_time Departure time in nanoseconds.
 * @return Expected arrival time in nanoseconds (tx_time + current_owd_ns_).
 */
common::Time_ns UeTimingEngine::calculate_expected_ul_arrival(common::Time_ns tx_time) const {
    return tx_time + current_owd_ns_;
}

/**
 * @brief Validates if uplink synchronization is valid at the given simulation time.
 * @param current_time Current simulation time in nanoseconds.
 * @return true if SIB19 ephemeris sync is valid; false if timer expired or not yet synced.
 */
bool UeTimingEngine::is_ul_sync_valid(common::Time_ns current_time) const {
    // 1. Must have received SIB19 and performed initial synchronization
    if (!has_synced_) {
        return false;
    }

    // 2. UL Sync Validity Duration timer (from SIB19 ulSyncValidityDuration IE)
    //    In 3GPP NTN, if the timer expires, the UE cannot assume its TA is accurate.
    if (ul_sync_validity_duration_ > 0) {
        if ((current_time - last_ta_update_time_) > ul_sync_validity_duration_) {
            return false;
        }
    }

    return true;
}

/**
 * @brief Validates whether the current round-trip Service Link TA fits within K_offset capacity.
 * @return true if Service Link TA <= K_offset * slot_duration; false otherwise.
 */
bool UeTimingEngine::is_ta_within_k_offset() const {
    common::Time_ns rtt = get_service_link_ta();
    common::Time_ns k_offset_capacity = get_k_offset_capacity_ns();
    return (rtt <= k_offset_capacity);
}

/**
 * @brief Returns the total scheduling offset capacity in nanoseconds (K_offset_slots * slot_duration).
 * @return Capacity in nanoseconds.
 */
common::Time_ns UeTimingEngine::get_k_offset_capacity_ns() const {
    return static_cast<common::Time_ns>(k_offset_slots_) * slot_duration_ns_;
}

} // namespace ntn::ue::timing


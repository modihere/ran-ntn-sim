/**
 * @file gnb_mac.h
 * @brief Satellite gNodeB Medium Access Control (MAC) entity for NTN Random Access.
 *
 * Implements gNodeB-side MAC functions for 3GPP TS 38.321 4-step RACH:
 * - Detecting PRACH Preamble (Msg1) on physical PRACH occasions.
 * - Generating and scheduling Random Access Response (Msg2) over the service link.
 * - Processing Msg3 (RRCSetupRequest or RRCReestablishmentRequest) and resolving contention.
 * - Generating and scheduling Contention Resolution PDU (Msg4) downlink over the service link.
 */

#pragma once

#include <functional>
#include <string>
#include "../../common/simulator.h"
#include "../../common/types.h"

namespace ntn::gnb::mac {

/**
 * @class GnbMac
 * @brief Models the satellite gNodeB MAC layer Random Access reception and response handling.
 */
class GnbMac {
public:
    /**
     * @brief Constructs the satellite gNodeB MAC entity.
     * @param sim Reference to the simulation core.
     * @param log_cb Telemetry log sink callback.
     * @param send_msg2_cb Callback to deliver Msg2 (RAR) downlink to UE.
     * @param send_msg4_cb Callback to deliver Msg4 (Contention Resolution) downlink to UE.
     */
    GnbMac(
        common::Simulator& sim,
        std::function<void(const std::string&)> log_cb = nullptr,
        std::function<void()> send_msg2_cb = nullptr,
        std::function<void()> send_msg4_cb = nullptr
    );

    /**
     * @brief Handles physical detection of PRACH Preamble (Msg1) at the satellite gNodeB.
     * @param rx_time Arrival timestamp at satellite in nanoseconds.
     * @param service_link_owd One-way service link delay in nanoseconds.
     */
    void on_prach_detected(common::Time_ns rx_time, common::Time_ns service_link_owd);

    /**
     * @brief Handles physical reception of Msg3 at the satellite gNodeB.
     * @param rx_time Arrival timestamp at satellite in nanoseconds.
     * @param service_link_owd One-way service link delay in nanoseconds.
     * @param is_reestablishment true if Msg3 carries an RRCReestablishmentRequest.
     */
    void on_msg3_received(common::Time_ns rx_time, common::Time_ns service_link_owd, bool is_reestablishment = false);

private:
    common::Simulator& sim_;                     ///< Simulator core reference
    std::function<void(const std::string&)> log_cb_; ///< Telemetry log sink
    std::function<void()> send_msg2_cb_;         ///< Downlink Msg2 transmission callback
    std::function<void()> send_msg4_cb_;         ///< Downlink Msg4 transmission callback
};

} // namespace ntn::gnb::mac

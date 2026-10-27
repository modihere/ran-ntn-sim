/**
 * @file gnb_mac.cpp
 * @brief Implementation of satellite gNodeB MAC Random Access processing.
 *
 * Implements preamble detection, formulation of Random Access Response (Msg2),
 * and generation of Contention Resolution (Msg4) over the regenerative service link.
 */

#include "gnb_mac.h"
#include <spdlog/spdlog.h>
#include <sstream>

namespace ntn::gnb::mac {

/**
 * @brief Constructs the gNodeB MAC entity.
 * @param sim Simulation core reference.
 * @param log_cb Log callback sink.
 * @param send_msg2_cb Hook for downlink Msg2 transmission.
 * @param send_msg4_cb Hook for downlink Msg4 transmission.
 */
GnbMac::GnbMac(
    common::Simulator& sim,
    std::function<void(const std::string&)> log_cb,
    std::function<void()> send_msg2_cb,
    std::function<void()> send_msg4_cb
) : sim_(sim), log_cb_(log_cb),
    send_msg2_cb_(send_msg2_cb), send_msg4_cb_(send_msg4_cb) {}

/**
 * @brief Handles detection of PRACH Msg1 preamble at the satellite gNodeB.
 * @param rx_time Preamble arrival timestamp in nanoseconds.
 * @param service_link_owd One-way service link delay in nanoseconds.
 */
void GnbMac::on_prach_detected(common::Time_ns rx_time, common::Time_ns service_link_owd) {
    (void)rx_time;
    if (log_cb_) {
        std::stringstream ss;
        ss << "\033[35m[gNB-MAC] Detected PRACH Preamble (Msg1) on physical occasion. One-Way Delay="
           << (service_link_owd / 1000) << " us. Generating RAR (Msg2)...\033[0m";
        log_cb_(ss.str());
    } else {
        spdlog::info("[gNB-MAC] Detected PRACH Preamble. Generating RAR.");
    }

    // Schedule downlink transmission of Msg2 (RAR) back to UE
    // Internal gNB L2 processing delay (5ms)
    common::Time_ns gnb_proc_delay = 5000000ULL;
    sim_.schedule(gnb_proc_delay, [this]() {
        if (log_cb_) {
            log_cb_("\033[35m[gNB-MAC] Msg2 (RAR) transmitted downlink with Timing Advance Command and Uplink Grant.\033[0m");
        }
        if (send_msg2_cb_) {
            send_msg2_cb_();
        }
    });
}

/**
 * @brief Handles reception of Msg3 at the satellite gNodeB.
 * @param rx_time Arrival timestamp in nanoseconds.
 * @param service_link_owd One-way service link delay in nanoseconds.
 * @param is_reestablishment true if Msg3 is an RRCReestablishmentRequest.
 */
void GnbMac::on_msg3_received(common::Time_ns rx_time, common::Time_ns service_link_owd, bool is_reestablishment) {
    (void)rx_time;
    if (log_cb_) {
        std::stringstream ss;
        if (is_reestablishment) {
            ss << "\033[35m[gNB-MAC] Received Msg3 (RRCReestablishmentRequest). Validating UE identity and re-establishing context...\033[0m";
        } else {
            ss << "\033[35m[gNB-MAC] Received Msg3 (RRCSetupRequest). Resolving contention for UE identity...\033[0m";
        }
        log_cb_(ss.str());
    } else {
        spdlog::info("[gNB-MAC] Received Msg3.");
    }

    // Schedule downlink transmission of Msg4 (Contention Resolution)
    // Internal gNB L2 processing delay (5ms)
    common::Time_ns gnb_proc_delay = 5000000ULL;
    sim_.schedule(gnb_proc_delay, [this, service_link_owd, is_reestablishment]() {
        (void)service_link_owd;
        if (log_cb_) {
            if (is_reestablishment) {
                log_cb_("\033[35m[gNB-MAC] Msg4 (Contention Resolution / RRCReestablishment) transmitted downlink.\033[0m");
            } else {
                log_cb_("\033[35m[gNB-MAC] Msg4 (Contention Resolution) transmitted downlink: Contention Won.\033[0m");
            }
        }
        if (send_msg4_cb_) {
            send_msg4_cb_();
        }
    });
}

} // namespace ntn::gnb::mac

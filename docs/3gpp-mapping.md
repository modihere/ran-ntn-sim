# 3GPP Specification to Implementation Mapping

This document provides explicit traceability between **3GPP Release 17/18 specifications** and the emulator implementation components, procedures, and observable KPIs.

---

## 1. Release Baseline
* **3GPP Target Release**: **Release 17** (frozen baseline with Release 18 NTN evolution notes).
* **Reference Specifications**:
  * 3GPP TS 38.300 v17.5.0: Overall architecture and NTN support
  * 3GPP TS 38.321 v17.5.0: MAC protocol specification & NTN extensions
  * 3GPP TS 38.331 v17.5.0: RRC protocol specification & SIB19 information elements
  * 3GPP TS 38.211 / TS 38.213 v17.5.0: Physical layer timing relationships
  * 3GPP TS 38.322 v17.5.0: RLC protocol specification

---

## 2. Traceability Matrix

### 2.1. NTN Configuration & RRC (TS 38.331)

| 3GPP Clause | 3GPP Information Element | Meaning / Description | C++ Model Entity | Observable Trace / KPI |
|---|---|---|---|---|
| **6.3.2** | `SIB19` (`ntn-Config`) | Broadcast NTN system information | `struct Sib19Config` | SIB19 reception event, ephemeris decode |
| **6.3.2** | `epoch` | Reference time for orbital ephemeris | `std::chrono::microseconds epoch` | Ephemeris validity evaluation |
| **6.3.2** | `ntn-UlSyncValidityDuration` | Duration for which UL synchronization remains valid without GNSS/drift update | `uint32_t ulSyncValidityDurationMs` | `UL_SYNC_EXPIRY` event, UL transmission halt |
| **6.3.2** | `cellSpecificKoffset` | Minimum delay offset between DL reception and UL transmission ($K_{\text{offset}}$) | `uint32_t kOffsetSlots` / `std::chrono::microseconds` | Slant range margin, PUSCH scheduling time |
| **6.3.2** | `ta-Info` | Common Timing Advance parameters ($T_{\text{common}}$, $\dot{T}_{\text{common}}$) | `struct TaInfo` | Common TA compensation curves |

---

### 2.2. Medium Access Control (MAC) & RACH (TS 38.321)

| 3GPP Clause | Procedure / Mechanism | Description in 3GPP | C++ Model Entity | Observable Trace / KPI |
|---|---|---|---|---|
| **5.1.1 - 5.1.5** | Random Access Procedure | 4-step contention-based RACH (Msg1 to Msg4) | `class RachController` | `RACH_MSG1_SENT`, `RACH_RAR_RECV`, `RACH_MSG3_SENT`, `RACH_MSG4_SUCCESS` |
| **5.1.4** | `ra-ResponseWindow` | Window to monitor for RAR. In NTN, extended by round-trip propagation delay | `Timer raResponseWindowTimer` | RAR window expiry vs successful reception |
| **5.1.5** | `ra-ContentionResolutionTimer` | Contention resolution timer extended by service-link RTT | `Timer contentionTimer` | Msg4 resolution success / backoff / retry count |
| **5.4.1** | Uplink Grant & $K_{\text{offset}}$ | Timing relation $K_2 + K_{\text{offset}}$ for PUSCH scheduling | `MacScheduler::calculateUplinkSlot()` | PUSCH arrival alignment vs slot boundary |
| **5.4.2** | HARQ Process & Disabling | Extended HARQ RTT or per-process disabling in NTN | `class HarqManager` | Process stall time, ACK/NACK latency, retransmission count |

---

### 2.3. Physical Layer Timing Relationships (TS 38.211 / 38.213)

| 3GPP Clause | Formula / Specification | Physical Meaning | C++ Implementation |
|---|---|---|---|
| **TS 38.211 § 4.3.1** | $T_{\text{TA}} = (N_{\text{TA}} + N_{\text{TA,offset}} + N_{\text{TA,common}}) \cdot T_c$ | Total Timing Advance applied by UE | `NtnTimingEngine::computeTotalTimingAdvance()` |
| **TS 38.213 § 4.2** | Transmission timing with $K_{\text{offset}}$ | Scheduling gap $n + K_2 + K_{\text{offset}}$ | `NtnTimingEngine::calculateUplinkTransmitTime()` |
| **TR 38.821 § 6.2** | $T_{\text{common}} = 2 \cdot \frac{R_{\text{ref}}}{c}$ | Service link delay between satellite and reference point | `NtnTimingEngine::calculateCommonTa()` |

---

### 2.4. Radio Link Control (RLC) (TS 38.322)

| 3GPP Clause | Procedure | Description | C++ Model Entity | Observable Trace / KPI |
|---|---|---|---|---|
| **5.2.2** | RLC UM Transmit / Receive | Unacknowledged transmission, segmentation | `class RlcUm` | SDU throughput, segmentation overhead |
| **5.2.3** | RLC AM Sliding Window | Acknowledged transmission with TX/RX windows | `class RlcAm` | Window stall events, buffer occupancy vs RTT |
| **5.2.3.2** | `t-PollRetransmit` | Timer preventing premature poll retransmission over large NTN RTT | `Timer pollRetransmitTimer` | False retransmissions vs timely ARQ recovery |
| **5.2.3.3** | STATUS PDU Reporting | ACK/NACK reporting format for lost PDUs | `RlcAm::generateStatusPdu()` | STATUS PDU overhead, ARQ recovery latency |

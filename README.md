# NTN-Aware 5G NR RAN Mobility & Timing Emulator

A 3GPP Release 17/18 aligned C++17 emulator connecting satellite orbital mechanics, non-terrestrial propagation physics, and RAN Layer 2/Layer 3 protocol procedures into a unified, executable, real-time discrete-event system.

---

## 🛰️ 1. Project Overview

Non-Terrestrial Networks (NTN) in 3GPP (Release 17 & 18) introduce fundamental challenges to cellular radio access:
* **Large Propagation Delays & Dynamic RTT**: Slant range varying rapidly with Low Earth Orbit (LEO) satellite speed (~7.6 km/s).
* **Severe Doppler Shift & Drift**: Carrier frequency shifts requiring continuous autonomous compensation.
* **Timing Relationships & TA Pre-compensation**: Uplink timing alignment requires Common Timing Advance ($T_{\text{common}}$), UE-specific Timing Advance ($T_{\text{UE}}$), Scheduling Offset ($K_{\text{offset}}$), and drift tracking before timing validity expires.
* **Protocol Impact on Access & Data Planes**: Large RTT profoundly alters 4-Step Random Access (Msg1–Msg4 contention), RLC Acknowledged Mode (AM) buffer sizing, and HARQ retransmission timelines.

This project bridges **satellite orbital geometry** directly into **3GPP protocol state machines**, offering an end-to-end discrete-event RAN emulator with real-time terminal-based telemetry and runtime scenario controls.

---

## 🏛️ 2. High-Level Architecture

```text
+--------------------------------------------------------------------------------+
|                   ANSI / Unicode Terminal UI (TUI) Dashboard                   |
|--------------------------------------------------------------------------------|
| Live Orbit View | Slant Range & Delay | NTN Timing (Koffset/TA) | RACH / HARQ  |
| Event Log Timeline | KPI Telemetry (Throughput, Latency, BLER, Retry Rates)    |
+---------------------------------------+----------------------------------------+
                                        |
+---------------------------------------v----------------------------------------+
|                          Simulation Core Controller                            |
|--------------------------------------------------------------------------------|
| High-Resolution Clock | Deterministic Event Queue | Timers | KPI Collector     |
+-------------------+------------------------------------+-----------------------+
                    |                                    |
+-------------------v--------------------+  +------------v-----------------------+
|              NTN Engine                |  |            RAN Protocols           |
|----------------------------------------|  |------------------------------------|
| • Orbit & Coordinate Propagator (LEO)  |  | • RRC & SIB19 NTN Config Model     |
| • Slant Range, Elevation & Visibility  |  | • 4-Step RACH (Msg1 to Msg4)       |
| • Dynamic Propagation Delay & Doppler  |  | • MAC (BSR, Grants, LCH Prioritize)|
| • Common TA, UE TA, Koffset, TA Drift  |  | • HARQ Process State Machine       |
| • Uplink Sync Validity Monitor         |  | • RLC UM & AM (ARQ, Retx, STATUS)  |
+----------------------------------------+  | • Terrestrial vs NTN Scheduler     |
                                            +------------------------------------+
```

---

## 📖 3. 3GPP Specification Alignment

| 3GPP Specification | Title / Area | Emulated Procedures & Parameters |
|---|---|---|
| **TS 38.300** | NR Overall Description | NTN Network Architecture, Transparent Payload |
| **TS 38.321** | MAC Protocol Specification | 4-Step RACH, RAR timing window, $K_{\text{offset}}$ UL scheduling, HARQ disablement/process count |
| **TS 38.331** | RRC Protocol Specification | SIB19 (`ntn-Config`), Ephemeris representation, Uplink sync validity timer |
| **TS 38.211 / 38.213** | NR Physical Channels & Timing | Timing Advance formula $T_{\text{TA}} = (N_{\text{TA}} + N_{\text{TA,offset}} + N_{\text{TA,common}}) \cdot T_c$ |
| **TS 38.322** | RLC Protocol Specification | RLC AM sliding window, Status PDU, Poll retransmit timer under NTN RTT |
| **TR 38.821 / 38.811** | Solutions for NR to support NTN | LEO/GEO reference orbits, Doppler profiles, path loss models |

Detailed clause-by-clause mapping is documented in [`docs/3gpp-mapping.md`](docs/3gpp-mapping.md).

---

## 🛠️ 4. Build & Development

### Prerequisites
* **C++ Compiler**: GCC 11+ / Clang 13+ / MSVC 2022 supporting **C++17**
* **Build System**: CMake (≥ 3.20) and Ninja
* **Git**

### Building with CMake & Ninja
```bash
# Configure build
cmake -B build -G Ninja

# Compile
cmake --build build

# Run Unit & Integration Tests
ctest --test-dir build --output-on-failure
```

---

## ⚙️ 5. Configuration & Interactivity

The emulator uses **JSON** for static configuration (e.g., `config/ntn_scenario.json`), utilizing the lightweight `nlohmann_json` library.

### Interactive Terminal Parameters
During the live Terminal UI (TUI) simulation, the following parameters can be dynamically edited to observe real-time system responses:
*   **$K_{\text{offset}}$**: Modify scheduling delay and observe RACH success/failures.
*   **Satellite Velocity / Altitude**: Simulate faster passes and watch Doppler/Delay shift.
*   **Common TA & Drift**: Inject drift and watch sync validity timers expire.

> **Note on Coordinate System**: For protocol layer isolation and deterministic reproducibility, the emulator uses a simplified 3D Cartesian system (stationary ground UE, straight-line satellite pass) rather than a full SGP4/ECEF orbital model. This is highlighted dynamically in the TUI.

---

## 📁 6. Repository Structure

* `src/common/` — Event engine, discrete clock, timers, logging, and statistics.
* `src/ntn/` — Orbital dynamics, geometry, slant range delay, Doppler, timing advance, and SIB19 model.
* `src/nr/` — 5G NR L2/L3 protocol abstractions (RRC, RACH, MAC, HARQ, RLC AM/UM, Scheduler).
* `simulator/` — Simulation harness, scenario runners, and terminal UI visualizer.
* `docs/` — 3GPP mappings, architectural design, state machines, and decision records.
* `tests/` — GoogleTest unit, integration, and scenario regression suites.


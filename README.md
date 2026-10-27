# NTN-Aware 5G NR RAN Mobility & Timing Emulator

A 3GPP Release 17/18 aligned C++17 emulator connecting satellite orbital mechanics, non-terrestrial propagation physics, and RAN Layer 2/Layer 3 protocol procedures into a unified, executable, real-time discrete-event system.

---

## 🛰️ 1. Project Overview

Non-Terrestrial Networks (NTN) in 3GPP (Release 17 & 18) introduce fundamental challenges to cellular radio access:
* **Large Propagation Delays & Dynamic RTT**: Slant range varying rapidly with Low Earth Orbit (LEO) satellite speed (~7.5 km/s).
* **Severe Doppler Shift & Drift**: Carrier frequency shifts requiring continuous autonomous compensation.
* **Regenerative Satellite Architecture**: On-board gNodeB processing eliminates feeder link latency (feeder link delay = 0 ns); the Timing Advance corresponds entirely to the round-trip propagation delay over the Service Link (satellite to UE).
* **Timing Relationships & TA Pre-compensation**: Uplink timing alignment requires autonomous UE-calculated Service Link Timing Advance ($T_{\text{service\_link}}$), scheduling offset ($K_{\text{offset}}$), and drift tracking before uplink synchronization validity expires (`ntn-UlSyncValidityDuration-r17`).
* **SIB19 (ntn-Config-r17) SI Window Broadcast Lifecycle**: Broadcast of orbital ephemeris coordinates, $K_{\text{offset}}$, and sync validity timers within periodic System Information (SI) windows over BCCH, decoded by UE monitoring PDCCH scrambled with `SI-RNTI`.
* **Absence of Epoch Time (§5.2.2.4.20)**: When `epochTime-r17` is absent from SIB19, parameters are applied strictly at the ending boundary of the SI window.
* **4-Step RACH & RRC Connection Re-establishment**: Contention-based Random Access (Msg1 $\to$ Msg2 RAR $\to$ Msg3 $\to$ Msg4) for initial access, and full RRC Connection Re-establishment following sync-loss recovery.

---

## 📺 2. Real-Time Terminal UI (TUI) Dashboard Visualization

The emulator features a real-time ASCII/ANSI Terminal UI rendering live telemetry at 100 ms intervals:

```text
====================================================================================================
               NTN-Aware 5G NR RAN Emulator (Regenerative Satellite gNodeB)                         
====================================================================================================
 [ Clock ]                               |                                                           
 Simulation Time : 4050 ms              |                              .                            
---------------------------------------- |                     .. ... ..R.. ... ..                   
 [ NTN Geometry ]                        |                 .. . X                 . ..               
 Satellite Pos   : [569625, 0, 600000] |              ...                           ...            
 Slant Range     : 827.27 km           |           ..                                   ..         
 Elevation       : 46.50 deg           |          .                                       .        
---------------------------------------- |        ..                                         ..      
 [ Timing Engine ]                       |       .                                             .     
 K_offset        : 6 slots             |      .                                               .    
 Service Link TA : 5519 us             |     .                                                 .   
 Sync Status     : VALID               |    .                                                   .  
---------------------------------------- |   [SAT]                                                .  
 [ NR Protocol ]                         |    . *                                                 .  
 RACH State      : WAIT_MSG4           |    .   *                                               .  
                                         |    .     *                                             .  
                                         | ==========================[ UE ]==========================
                                         | Arc: '.' Orbit  '*' Beam  'R' Reacquire  'X' Sync Lost
====================================================================================================
[gNB-RRC] [4020 ms] Opening SI Window (duration: 20 ms) for SIB19 transmission.
[UE-RRC] [4020 ms] Monitoring PDCCH with SI-RNTI for SIB19 broadcast in SI window...
[gNB-RRC] [4022 ms] Broadcasting SIB19 on BCCH inside SI Window. Ephemeris=[569835, 0, 600000] m...
[UE-RRC] [4024 ms] Received SIB19 transport block over Service Link (OWD: 2760 us). Reading BCCH payload...
[UE-RRC] [4024 ms] SIB19 decoded: Ephemeris=[569835, 0, 600000] m, K_offset=6 slots, Validity=3000 ms.
[UE-RRC] [4024 ms] (Epoch time absent) SIB19 configuration will be applied on the ending point of the SI window.
[gNB-RRC] [4040 ms] SI Window closed.
[UE-RRC] [4040 ms] SI Window ended. Activating SIB19 configuration: Service Link TA pre-compensation calculated (5521 us), K_offset=6 slots. RACH permitted.
[UE-RRC] [4040 ms] Triggering RRC Connection Re-establishment via RACH procedure...
[UE-MAC] [4040 ms] Msg1 (PRACH Preamble) transmitted with updated Service Link TA pre-compensation (5521 us).
[gNB-MAC] [4042 ms] Detected PRACH Preamble (Msg1) on physical occasion. One-Way Delay=2759 us. Generating RAR (Msg2)...
[gNB-MAC] [4047 ms] Msg2 (RAR) transmitted downlink with Timing Advance Command and Uplink Grant.
[UE-MAC] [4050 ms] Msg2 (RAR) received from satellite gNodeB. Preparing Msg3 (RRCReestablishmentRequest)...
[UE-MAC] [4058 ms] Msg3 (RRCReestablishmentRequest) transmitted with K_offset delay.
[gNB-MAC] [4061 ms] Received Msg3 (RRCReestablishmentRequest). Validating UE identity and re-establishing context...
[gNB-MAC] [4066 ms] Msg4 (Contention Resolution / RRCReestablishment) transmitted downlink.
[UE-MAC] [4069 ms] Msg4 (Contention Resolution / RRCReestablishment) received. Re-establishment SUCCESS -> State: CONNECTED.
====================================================================================================
 (Press 'p' or Space to Pause, 'm' for Menu)
```

---

## 🏛️ 3. High-Level System Architecture

```text
+---------------------------------------------------------------------------------------------------+
|                            Real-Time ANSI / Unicode TUI Visualizer                                |
|---------------------------------------------------------------------------------------------------|
|  Simulation Clock  |  3D Geometry & Elevation  |  Timing Engine & TA  |  Protocol State Machine   |
|  Orbital Arc Display with Real-Time Event Tracking ('R' Reacquire, 'X' Sync Loss, '*' RF Beam)     |
|  Scrolling Multi-Entity Colorized Telemetry Log Sink ([SIM], [gNB-RRC], [UE-RRC], [gNB-MAC], ...) |
+-------------------------------------------------+-------------------------------------------------+
                                                  |
+-------------------------------------------------v-------------------------------------------------+
|                                 Discrete-Event Simulation Core                                    |
|---------------------------------------------------------------------------------------------------|
|  Deterministic Event Queue  |  Nanosecond Time Resolution  |  Async Event Scheduling Engine       |
+------------------------+---------------------------------------------------+----------------------+
                         |                                                   |
+------------------------v------------------------+  +-----------------------v----------------------+
|             Satellite gNodeB Entities           |  |                 Ground UE Entities           |
|-------------------------------------------------|  |----------------------------------------------|
| • GnbSib19Broadcaster (SI Windows, BCCH tx)    |  | • UeRrc (SI-RNTI PDCCH rx, SIB19 decode)     |
| • GnbMac (PRACH detection, RAR, Contention Res) |  | • UeTimingEngine (LEO Ephemeris TA calc)     |
| • Orbital Physics Engine (Linear/Kepler orbit)  |  | • UeRachController (4-Step RACH, K_offset)  |
+-------------------------------------------------+  +----------------------------------------------+
                         ^                                                   ^
                         +=================[ Service Link ]==================+
                                         Delay = Slant Range / c
```

---

## 📖 4. 3GPP Specification Alignment

| 3GPP Spec | Title / Area | Emulated Procedures & Implementation Details |
|---|---|---|
| **TS 38.300** | NR Overall Description | Regenerative satellite architecture with on-board gNodeB; Feeder link delay = 0 ns. |
| **TS 38.331** | RRC Protocol (§5.2.2, §5.2.2.4.20) | SIB19 `ntn-Config-r17`, `EphemerisInfo-r17` state vectors, `cellSpecificKoffset-r17`, `ntn-UlSyncValidityDuration-r17`, `si-WindowLength-r17`. Absence of `epochTime-r17` delays activation to SI window closure. |
| **TS 38.321** | MAC Protocol (§5.1) | 4-step Contention-Based Random Access; Msg3 scheduling with $K_{\text{offset}}$ slots; RRC Connection Re-establishment RACH procedure; MAC buffer flush upon sync loss. |
| **TS 38.211 / 213** | NR Physical Channels & Timing | Autonomous UE Timing Advance pre-compensation: $T_{\text{TA}} = 2 \cdot \frac{R(t)}{c}$; Doppler shift and carrier frequency offset tracking. |
| **TR 38.821** | Solutions for NR NTN | LEO circular orbit propagation, slant range delay, Doppler profiles, and wide-beam elevation modeling. |

---

## 🔄 5. SIB19 Transmission, Timing & Decoupled Update Lifecycle

### 3GPP SIB19 Broadcast Sequence
```mermaid
sequenceDiagram
    autonumber
    participant GNB as Satellite gNodeB (RRC & MAC)
    participant CH as Service Link Channel (OWD = R/c)
    participant UE as Ground User Equipment (RRC & MAC)

    Note over GNB: SI Window Opens (e.g. duration = 20 ms)
    GNB->>GNB: Open SI Window
    UE->>UE: Monitor PDCCH scrambled with SI-RNTI
    GNB->>CH: Broadcast SIB19 on BCCH (Ephemeris, K_offset, Validity)
    CH->>UE: Propagate over Service Link
    UE->>UE: Decode SIB19 Transport Block
    Note over UE: (Epoch time absent) SIB19 staged; will activate at SI window end
    GNB->>GNB: SI Window Closes
    UE->>UE: Activate SIB19: Compute TA pre-compensation & apply K_offset
    UE->>UE: RACH Permitted -> Ready for Msg1
```

### Decoupled Runtime Reconfiguration (3GPP Spec Compliant)
In cellular and non-terrestrial networks, broadcast parameters are **never** applied instantaneously to UE timing engines when modified at the network:
1. **Mid-Run Modifications**: SIB19 parameters updated via the interactive menu are queued at the satellite gNodeB (`GnbSib19Broadcaster::queue_config_update`).
2. **UE Stays Untouched**: The UE continues operating with its current valid SIB19 parameters without interruption.
3. **Next SI Window Broadcast**: When the next SI window opens, the gNodeB transmits the queued parameters over the BCCH.
4. **Window Boundary Activation**: The UE decodes the transport block inside the window, logs the absence of epoch time, and activates the parameters **at SI window closure**.
5. **RRC Re-establishment**: If SIB19 was reacquired following sync validity expiration, the UE immediately triggers the 4-step RRC Connection Re-establishment RACH procedure (`Msg1` $\to$ `Msg2` $\to$ `Msg3` $\to$ `Msg4`).

### $K_{\text{offset}}$ Timing Constraint & 1-Second RACH Retry Loop (TS 38.213 §8.3 & TS 38.321 §5.1.4)
* **Msg1 & Msg2 Independence from $K_{\text{offset}}$**: Once SIB19 is acquired and the `ntn-UlSyncValidityDuration-r17` timer is active, the UE transmits Msg1 (PRACH Preamble) autonomously pre-compensated by Service Link TA. Msg2 (RAR) is scheduled downlink by the gNodeB. Both Msg1 and Msg2 proceed regardless of the broadcast $K_{\text{offset}}$.
* **Msg3 $K_{\text{offset}}$ Feasibility Check**: When Msg2 is received, the UE validates whether its active Service Link TA fits within the $K_{\text{offset}}$ capacity:
  $$\text{Service Link TA} \le K_{\text{offset}} \cdot T_{\text{slot}}$$
  - **If TA exceeds $K_{\text{offset}}$ (Low $K_{\text{offset}}$)**: Msg3 transmission cannot satisfy the grant window. The UE logs:
    ```text
    [UE-MAC] Msg3 transmission blocked: Service Link TA (5643 us) exceeds K_offset capacity (4000 us, 4 slots). Timing requirement cannot be met.
    [UE-MAC] RACH attempt failed. Retrying RACH in 1000 ms...
    ```
  - **1-Second Retry Loop**: The UE retries RACH every 1 second (1000 ms). Because the LEO satellite moves continuously along its orbital trajectory (~7.5 km/s), slant range and Service Link TA decrease as the satellite approaches the ground station.
  - **Resolution**: As soon as TA drops within $K_{\text{offset}}$ (or $K_{\text{offset}}$ is updated via SIB19), Msg3 is scheduled with $K_{\text{offset}}$ delay, Msg4 is received, and RACH completes (`CONNECTED`). If the satellite passes without TA satisfying $K_{\text{offset}}$, the `ulSyncValidityDuration` timer expires, halting RACH retries and initiating SIB19 reacquisition in the SI window.

---

## 🎛️ 6. Runtime Interaction & Interactive Configuration Editor

### Live Simulation Controls
* **In-Place Freeze Pause (`'p'`, `'P'`, or `Space`)**: Freezes simulation clock in-place without wiping or clearing the terminal. Press `'p'` or `Space` again to resume smoothly.
* **Interactive Menu (`'m'` or `'M'`)**: Pauses the simulation and provides access to the parameter editor, log history viewer, reset, or quit.

### Categorized Scenario Configuration Menu
The configuration editor explicitly separates immediate simulation parameters from over-the-air SIB19 parameters:

```text
======================================================================
                  NTN Scenario Configuration Editor                   
======================================================================
[Current Simulation Time: 8,000 ms]

--- [A] General / Simulation & Channel Modeling Parameters (Applied Immediately upon Resume) ---
[1] duration_ms                  : 50000 (Remaining: 42000 ms)
[2] update_interval_ms           : 100
[3] scs_khz                      : 15
[4] rach_trigger_time_ms         : 1000
[5] reacquisition_success_prob   : 0.50  (Channel Emulation BLER Model)
[6] sync_loss_recovery_gap_ms    : 1500  (Simulation Recovery Gap)

--- [B] SIB19 (ntn-Config-r17) Parameters (Broadcast by gNodeB, Applied at SI Window End) ---
[7] k_offset_slots               : 6  (cellSpecificKoffset-r17)
[8] ul_sync_validity_duration_ms : 10000  (ntn-UlSyncValidityDuration-r17)
[9] satellite.vel_x_mps          : -7500.0  (ephemerisInfo: velocityVX)
[10] satellite.start_z_m         : 600000.0  (ephemerisInfo: positionZ)
[11] si_window_ms                : 20  (si-WindowLength-r17)

[0] Go Back
======================================================================
Select parameter to change (0-11): 
```

* **Target Duration Validation**: Setting option `[1] duration_ms` validates that the new target duration is strictly greater than the current simulation time before accepting the change.
* **Full Log History Viewer (`'v'` or Menu Option `[4]` / `[5]`)**: Paged interactive terminal viewer allowing inspection of all timestamped entity logs from $t = 0$.

---

## 🛠️ 7. Build, Test & Run

### Prerequisites
* **C++ Compiler**: GCC 11+, Clang 13+, or MSVC 2022 supporting **C++17**
* **Build System**: CMake (≥ 3.20) and Ninja / Make
* **Git**

### Compilation
```powershell
# Configure CMake build with Ninja
cmake -B build -G Ninja

# Compile simulator executable and test runner
cmake --build build
```

Target binaries generated:
* `build/bin/simulator.exe` — Main interactive NTN RAN emulator application
* `build/bin/unit_tests.exe` — Comprehensive GoogleTest test suite

### Running Unit Tests
```powershell
./build/bin/unit_tests.exe
```
Executes all **26 unit tests** covering:
* `RachTest`: Successful 4-step RACH, RAR timeout, contention resolution loss, RRC Connection Re-establishment RACH, Low $K_{\text{offset}}$ Msg3 blocking & autonomous 1-second retries, Exhausted attempts SIB19 re-acquisition & RACH recovery, Connected mode DL-only UM fallback when $K_{\text{offset}} < \text{TA}$.
* `TimingEngineTest`: $K_{\text{offset}}$ slot calculations, Service Link TA pre-compensation, UL sync validity timer, $K_{\text{offset}}$ capacity validation.
* `GeometryTest`: 3D Cartesian Euclidean distances, slant range at zenith.
* `PropagationTest`: One-way delay, Doppler shift profiles, physical elevation angles.
* `OrbitPropagatorTest`: Linear orbital propagation and kinematics.
* `SimulatorTest`: Event scheduling order, timestamp fidelity, discrete clock progression.
* `TimerTest`: One-shot timers, cancellations, timeouts.

### Running the Simulator
```powershell
# Run with default LEO Scenario 1 configuration
./build/bin/simulator.exe config/scenario_1.json
```

All formatted logs are saved simultaneously to `ntn_simulation.log` for post-run analysis.

---

## 📁 8. Repository Structure

```text
ntn-ran-emulator/
├── CMakeLists.txt               # Top-level CMake configuration
├── config/
│   └── scenario_1.json          # Default 3GPP Rel-17 LEO regenerative scenario
├── simulator/
│   ├── CMakeLists.txt           # Simulator executable target definition
│   └── main.cpp                 # Main simulation loop, TUI loop, and menu handlers
├── src/
│   ├── CMakeLists.txt           # Core libraries target definition
│   ├── common/
│   │   ├── simulator.h / .cpp   # Discrete-event engine and priority event queue
│   │   ├── timer.h              # Nanosecond-precision event timers
│   │   └── types.h              # Common 3GPP & NTN data structures (NtnConfig, TA, etc.)
│   ├── gnb/
│   │   ├── rrc/                 # SIB19 broadcaster & SI window management
│   │   └── mac/                 # gNodeB MAC (PRACH detection, RAR, Msg4 Contention)
│   ├── ue/
│   │   ├── rrc/                 # UE RRC (PDCCH SI-RNTI monitoring, SIB19 decode)
│   │   ├── mac/                 # UE RACH controller (Msg1-Msg4, Re-establishment)
│   │   └── timing/              # UE Timing Engine (LEO TA pre-compensation)
│   ├── ntn/
│   │   └── geometry/            # Orbit propagator, slant range, Doppler calculations
│   └── ui/
│       └── tui/                 # ANSI terminal dashboard, sky dome arc, keyboard input
└── tests/
    └── unit/                    # GoogleTest test suites (23 unit tests)
```

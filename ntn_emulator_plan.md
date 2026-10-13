## Goal Description
We are evaluating our trajectory against the original project specification to ensure we are building the **First Vertical Slice** efficiently. 

**Current Status: On Track.**
* ✅ **Phase 0 (Foundation)**: Git, CMake, C++17, GoogleTest, spdlog, architecture docs.
* ✅ **Phase 1 (Simulation Core)**: A deterministic, nanosecond-precision Discrete-Event Simulator (DES) has been implemented and unit-tested.
* ✅ **Phase 2 (NTN Geometry Engine)**: Simple Cartesian coordinate system implemented, enabling slant range, propagation delay, and Doppler shift calculations. Unit tests fully documented and committed.
* ✅ **Phase 3 (NTN Timing Engine)**: Maps the physical propagation delays into 3GPP uplink timing pre-compensation parameters ($K_{\text{offset}}$, Common TA, UE TA).
* ✅ **Phase 4 (4-Step RACH Procedure with Timing Sensitivity)**: Random Access Procedure state machine modeled in `src/nr/rach/rach_controller.h/cpp` using the common DES engine, with extended `ra-ResponseWindow`.

* ✅ **Phase 5 (Real-time Terminal Dashboard (TUI))**: Interactive loop implemented. Features dynamic RACH beam, satellite tracking, real-time config modification (speed, K_offset, height), persistent event logs, and dynamic interrupt logic based on uplink sync state.
* ✅ **Phase 6 (SIB19 over-the-air broadcasting & Uplink Sync Lifecycle)**: SIB19 `ntn-Config` model implemented. Over-the-air broadcast in SI window, ephemeris-based TA updates, RACH gating, autonomous reacquisition vs. UL sync validity expiration lifecycle (with MAC buffer flush and recovery), wide sky arc sweep, dedicated TUI event markers (`'R'`/`'X'`), and complete log history viewer.

**Next Immediate Goal: Phase 7 (MAC Layer & HARQ Process Management)**
Model the L2 data plane procedures following initial access:
- Uplink data scheduling ($K_2 + K_{\text{offset}}$).
- Extended HARQ RTT feedback timelines over satellite delay.
- 3GPP Rel-17 HARQ disabling switch (comparing process stall vs throughput).
- Buffer Status Reporting (BSR) and Logical Channel Prioritization (LCP).

## Current Context
- The emulator codebase is fully verified and stable. All 22 unit tests pass.
- Autonomous simulation runs end-to-end with persistent disk streaming (`ntn_simulation.log`).

## Verification Plan
1. Unit tests: `./build/bin/unit_tests.exe` (22/22 pass).
2. Live emulator: `./build/bin/simulator.exe config/scenario_1.json`.

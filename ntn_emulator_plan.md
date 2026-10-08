## Goal Description
We are evaluating our trajectory against the original project specification to ensure we are building the **First Vertical Slice** efficiently. 

**Current Status: On Track.**
* ✅ **Phase 0 (Foundation)**: Git, CMake, C++17, GoogleTest, spdlog, architecture docs.
* ✅ **Phase 1 (Simulation Core)**: A deterministic, nanosecond-precision Discrete-Event Simulator (DES) has been implemented and unit-tested.
* ✅ **Phase 2 (NTN Geometry Engine)**: Simple Cartesian coordinate system implemented, enabling slant range, propagation delay, and Doppler shift calculations. Unit tests fully documented and committed.
* ✅ **Phase 3 (NTN Timing Engine)**: Maps the physical propagation delays into 3GPP uplink timing pre-compensation parameters ($K_{\text{offset}}$, Common TA, UE TA).
* ✅ **Phase 4 (4-Step RACH Procedure with Timing Sensitivity)**: Random Access Procedure state machine modeled in `src/nr/rach/rach_controller.h/cpp` using the common DES engine, with extended `ra-ResponseWindow`.

* ✅ **Phase 5 (Real-time Terminal Dashboard (TUI))**: Interactive loop implemented. Features dynamic RACH beam, satellite tracking, real-time config modification (speed, K_offset, height), persistent event logs, and dynamic interrupt logic based on uplink sync state.

**Next Immediate Goal: Phase 6 (SIB19 over-the-air broadcasting)**
We need to model the reading of System Information Block 19 (SIB19) by the UE over-the-air before it initiates the RACH procedure. 
- The UE must first successfully receive a simulated SIB19 payload (with ephemeris data).
- The `K_offset` and Feeder Link Delay (Common TA component) should be passed via this block.
- The UE uses the ephemeris data to calculate its specific Service Link Delay.
- Once configured with SIB19, the UE is permitted to trigger RACH.

## Current Context
- The emulator codebase is currently stable. All 19 unit tests pass.
- We have 3 local commits pending to be pushed to `origin/develop`. (The earliest 8 were successfully pushed).
- There are uncommitted, local modifications in `config/scenario_1.json`.
- The simulation loop handles interactive interruption natively and is fully prepared for event-driven extensions like SIB19 broadcasting.

## Verification Plan
1. Ensure the new SIB19 event correctly blocks RACH initiation until successfully processed.
2. Run `./build/bin/simulator.exe config/scenario_1.json` to manually verify the dashboard accurately reflects the new timing structure.

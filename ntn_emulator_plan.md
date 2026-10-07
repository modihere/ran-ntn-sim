## Goal Description
We are evaluating our trajectory against the original project specification to ensure we are building the **First Vertical Slice** efficiently. 

**Current Status: On Track.**
* ✅ **Phase 0 (Foundation)**: Git, CMake, C++17, GoogleTest, spdlog, architecture docs.
* ✅ **Phase 1 (Simulation Core)**: A deterministic, nanosecond-precision Discrete-Event Simulator (DES) has been implemented and unit-tested.
* ✅ **Phase 2 (NTN Geometry Engine)**: Simple Cartesian coordinate system implemented, enabling slant range, propagation delay, and Doppler shift calculations. Unit tests fully documented and committed.

**Next Immediate Goal: Phase 3 (NTN Timing Engine)**
We must map the physical propagation delays computed in Phase 2 into 3GPP uplink timing pre-compensation parameters ($K_{\text{offset}}$, Common TA, UE TA). We will explicitly configure it for 15 kHz Subcarrier Spacing (SCS), meaning 1 slot = 1 millisecond. We will structure the state such that changes in delay gracefully dynamically translate to slots for $K_{\text{offset}}$.

## Proposed Changes

---

### NTN Timing Engine (`src/ntn/timing`)
This maps the physical propagation delay into 3GPP parameters and handles validity windows.

#### [NEW] `timing_engine.h` & `.cpp`
Maintains the state of $K_{\text{offset}}$ and Timing Advance.
```cpp
namespace ntn::timing {
class NtnTimingEngine {
public:
    // Config
    void set_scs_khz(uint32_t scs_khz = 15);
    
    // Updates
    void update_geometry(common::Time_ns one_way_delay);
    
    // 3GPP Parameters
    common::Time_ns get_common_ta() const;
    common::Time_ns get_ue_ta() const;
    uint32_t get_k_offset_slots() const; // Represents slots at configured SCS
    
    // Validation
    common::Time_ns calculate_expected_ul_arrival(common::Time_ns tx_time) const;
    bool is_ul_sync_valid(common::Time_ns current_time) const;
    
private:
    uint32_t scs_khz_ = 15;
    common::Time_ns slot_duration_ns_ = 1000000; // 1ms for 15kHz
    common::Time_ns current_owd_ns_ = 0;
};
}
```

---

### Simulator Integration & TUI Foundation (`simulator/`)
Connecting the geometry and timing engines to the simulation clock, initializing with JSON config, and setting up the TUI logging.

#### [NEW] `main.cpp`
Initializes the simulation, creates a UE and Satellite based on parameters from a JSON configuration file.
It schedules a periodic event (e.g., every 100ms) to evaluate geometry, update the Timing Engine, and heavily log critical interactive parameters like $K_{\text{offset}}$, slant range, and TA to the terminal via `spdlog`.

## Verification Plan

### Automated Tests
Run GoogleTest suite via `ctest --test-dir build`:
1. `TimingEngineTest.KOffsetCalculation`: Verify that a delay of exactly 40ms yields a $K_{\text{offset}}$ of 40 slots at 15kHz SCS.
2. `TimingEngineTest.CommonTaUpdate`: Verify Common TA equals $2 \times$ one-way delay to the reference point.
3. `TimingEngineTest.SyncValidity`: Verify that if the delay changes drastically beyond a threshold without TA update, sync is marked invalid.

### Manual Verification
1. Run `./build/bin/simulator.exe config/scenario_1.json`.
2. Observe the terminal output prominently highlighting $K_{\text{offset}}$, Common TA, and Satellite Position.
3. Confirm real-time dynamic changes to $K_{\text{offset}}$ as the satellite passes overhead.


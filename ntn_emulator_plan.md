## Goal Description
We are evaluating our trajectory against the original project specification to ensure we are building the **First Vertical Slice** efficiently. 

**Current Status: On Track.**
* ✅ **Phase 0 (Foundation)**: Git, CMake, C++17, GoogleTest, spdlog, architecture docs.
* ✅ **Phase 1 (Simulation Core)**: A deterministic, nanosecond-precision Discrete-Event Simulator (DES) has been implemented and unit-tested.
* ✅ **Phase 2 (NTN Geometry Engine)**: Simple Cartesian coordinate system implemented, enabling slant range, propagation delay, and Doppler shift calculations. Unit tests fully documented and committed.
* ✅ **Phase 3 (NTN Timing Engine)**: Maps the physical propagation delays into 3GPP uplink timing pre-compensation parameters ($K_{\text{offset}}$, Common TA, UE TA).
* ✅ **Phase 4 (4-Step RACH Procedure with Timing Sensitivity)**: Random Access Procedure state machine modeled in `src/nr/rach/rach_controller.h/cpp` using the common DES engine, with extended `ra-ResponseWindow`.

**Next Immediate Goal: Phase 5 (Real-time Terminal Dashboard (TUI))**
We need to implement a Real-time Terminal Dashboard displaying orbit, timing error, and RACH state transitions. The TUI will visualize the simulation state (e.g., satellite position, slant range, $K_{\text{offset}}$, RACH state) in a non-wrapping formatted terminal dashboard as the simulation runs.

## Proposed Changes

---

### Terminal UI Dashboard (`src/ui/tui`)
This module handles formatting and displaying the simulation metrics dynamically in the terminal.

#### [NEW] `dashboard.h` & `.cpp`
Creates an ANSI-based dashboard view that updates in-place.
```cpp
namespace ntn::ui {
class Dashboard {
public:
    Dashboard();
    
    // Updates
    void update_simulation_time(common::Time_ns time);
    void update_geometry(double slant_range_km, double elevation_deg);
    void update_timing(uint32_t k_offset, common::Time_ns common_ta);
    void update_rach_state(const std::string& state_str);
    
    // Render
    void render() const;
    void clear() const;
private:
    // Internal state variables for rendering
};
}
```

---

### Simulator Integration (`simulator/`)
Connecting the TUI to the main simulator loop.

#### `main.cpp`
Update `main.cpp` to instantiate the `Dashboard` and periodically trigger `render()` (e.g. every 100ms or 1s of simulation time) with the latest stats from the NTN Timing Engine and RACH Controller. It will also need to slow down the simulation to "real-time" (or a scaled wall-clock time) so the user can observe the dashboard, rather than completing the entire simulation in a fraction of a second.

## Verification Plan

### Automated Tests
Run GoogleTest suite via `ctest --test-dir build`:
1. Add tests for `ui::Dashboard` if needed, although TUI testing is mostly visual. Basic string formatting can be tested.

### Manual Verification
1. Run `./build/bin/simulator.exe config/scenario_1.json`.
2. Observe a structured dashboard updating in place with real-time values for Slant Range, $K_{\text{offset}}$, and RACH state.
3. The simulation should run at a human-observable speed.

# Design Decision Records (ADR)

## ADR-001: Choice of Simulation Paradigm (Discrete Event vs Continuous Time)
* **Status**: Accepted
* **Context**: Cellular PHY simulations often run time-domain slot or sample processing. However, full PHY signal processing (channel estimation, FFT, LDPC/Polar decoding) consumes high computational power and obscures higher-layer protocol interactions.
* **Decision**: Implement a **Discrete-Event Simulation (DES)** engine with microsecond precision.
* **Consequences**:
  * Enables fast-than-realtime or synchronized real-time execution.
  * Allows exact modelling of 3GPP protocol timers, state machines, and message exchanges.
  * Preserves timing sensitivity without CPU-intensive RF DSP.

---

## ADR-002: Language Standard and Compiler Toolchain
* **Status**: Accepted
* **Context**: Need maximum cross-platform compatibility, modern C++ type safety, and standard library concurrency/chrono primitives.
* **Decision**: Target **C++17** using GCC / MinGW-w64, Clang, or MSVC.
* **Consequences**:
  * Utilizes `std::optional`, `std::variant`, `std::chrono`, and structured bindings.
  * Compatible with standard enterprise RAN developer environments.

---

## ADR-003: Dependency Management Strategy
* **Status**: Accepted
* **Context**: Need standard libraries for testing (`GoogleTest`), logging (`spdlog`), and serialization (`nlohmann_json`) without requiring users to install package managers like vcpkg or Conan.
* **Decision**: Use CMake's built-in `FetchContent` with shallow git clones.
* **Consequences**:
  * Fully self-contained repository setup: `cmake -B build` automatically downloads and configures dependencies.
  * Zero external tool prerequisites beyond CMake and a C++ compiler.

---

## ADR-004: User Interface & Visualization Architecture
* **Status**: Accepted
* **Context**: Visualizing orbit trajectories, timing drift, and protocol states during live runs.
* **Decision**: Implement a high-performance **ANSI / Unicode Terminal UI (TUI)** formatted for standard terminal dimensions (100 columns x 32 rows) without line wrapping.
* **Consequences**:
  * Single self-contained binary execution without web browser or external web server dependencies.
  * Immediate real-time visualization directly in the developer terminal or VS Code.
  * Optional popup plotting capability for high-resolution 2D orbit curves.

---

## ADR-005: Breadth-First Vertical Slice Strategy
* **Status**: Accepted
* **Context**: For interview demonstration and visible GitHub contributions, completing deep protocol layers (e.g. months on RRC ASN.1 or RLC AM) in isolation prevents early end-to-end testing.
* **Decision**: Build a complete vertical slice first:
  $$\text{Simulation Clock} \longrightarrow \text{Orbit Geometry} \longrightarrow \text{Slant Range / Delay} \longrightarrow K_{\text{offset}} \text{ Timing} \longrightarrow \text{4-Step RACH} \longrightarrow \text{Terminal Dashboard}$$
* **Consequences**:
  * Early demonstrable milestone with observable KPIs (e.g., Koffset insufficiency leading to RACH failure).
  * Subsequent modules (RLC AM, MAC, Schedulers) build upon a proven, end-to-end running core.


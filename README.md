# AVR-Map: Adaptive Variable-Resolution 2.5D LiDAR Mapping

**SIH Problem Statement:** SIH26053 — Adaptive Variable-Resolution 2.5D LiDAR Mapping for Dynamic Environment Perception  
**Current Status:** Tested persistent C++ mapping core, frame-local dynamic clustering, frozen frontend demo, and M11A offline benchmark harness. Real recorded-data validation is pending.

See [offline benchmark instructions](docs/benchmark/README.md) and [M11 source audit](docs/benchmark/M11_AUDIT.md).

---

## Overview

**AVR-Map** is a prototype for allocating a 2.5D LiDAR map at 5, 10, 20 and 40 cm resolution. The implemented C++ core supports persistent adaptive mapping, semantic and motion evidence, replay-safe refinement, hysteresis and coarsening. Memory savings, real-time throughput and accuracy on recorded data have not yet been established. The current pipeline does not supply uncertainty observations.

> **Core Principle:**  
> *"The map is the product; the neural network is a component."*

---

## Architectural Runtimes

- **Planned Deployment Runtime:** Ubuntu 24.04 LTS (x86_64/ARM64) with **ROS 2 Jazzy Jalisco**. ROS2 adapters and production neural inference are planned; no model/runtime integration is claimed by the current portable core.
- **Portable Development Platform:** macOS is supported for pure C++ algorithmic library development (`backend/cpp/`), unit/property testing, offline dataset tooling, and full frontend UI engineering (`frontend/`).

---

## Repository Structure

- `docs/`: Authoritative project specifications (PRD, TRD, Backend Spec, Frontend Spec) and architecture decision logs (`05_IMPLEMENTATION_DECISIONS.md`).
- `config/`: Reserved configuration directories; current core defaults are in C++ types.
- `backend/cpp/`: Standalone, ROS-independent pure C++20 algorithms, spatial data structures, and unit tests.
- `backend/ros2_ws/`: Reserved for future ROS2 package adapters and launch orchestration.
- `frontend/`: AVR Console — an operator and technical inspection interface built with React, Vite, and Three.js.
- `datasets/`: Ignored local dataset storage; no real sequence is bundled.
- `models/`: Reserved for separately acquired model assets.
- `tools/`: Automated experiment runners, data converters, and offline utilities.
- `results/`: Ignored local benchmark JSON/CSV outputs and validation logs.
- `scripts/`: Development automation scripts (environment check, build, test, clean).

---

## Building Pure C++ Core (Portable)

Prerequisites: CMake $\ge 3.22$ and a C++20-compliant compiler (Apple Clang or GCC 13+).

```bash
# 1. Verify development environment
./scripts/check_env.sh

# 2. Build Debug preset (with ASan/UBSan sanitizers)
./scripts/build_cpp.sh debug

# 3. Run test suite
./scripts/test_cpp.sh debug

# 4. Build Release preset
./scripts/build_cpp.sh release
./scripts/test_cpp.sh release
```

# AVR-Map: Adaptive Variable-Resolution 2.5D LiDAR Mapping

**SIH Problem Statement:** SIH26053 — Adaptive Variable-Resolution 2.5D LiDAR Mapping for Dynamic Environment Perception  
**Current Status:** Milestone 0 (Environment & Repository Bootstrap)

---

## Overview

**AVR-Map** is a real-time autonomous-perception framework that transforms incoming 3D LiDAR point clouds into a hierarchical, semantic, variable-resolution 2.5D local environment map. Traditional mapping forces a trade-off between memory-prohibitive uniform fine grids and risk-prone coarse grids that lose safety-critical hazards. AVR-Map addresses this by dynamically allocating spatial resolution based on distance, semantic safety relevance, uncertainty, and geometry, preserving navigation-critical detail while dramatically reducing computational and memory overhead.

> **Core Principle:**  
> *"The map is the product; the neural network is a component."*

---

## Architectural Runtimes

- **Canonical Deployed Runtime:** Ubuntu 24.04 LTS (x86_64/ARM64) with **ROS 2 Jazzy Jalisco**. This environment hosts the end-to-end multi-node perception pipeline, hardware-accelerated neural segmentation (SalsaNext), and formal benchmarking.
- **Portable Development Platform:** macOS is supported for pure C++ algorithmic library development (`backend/cpp/`), unit/property testing, offline dataset tooling, and full frontend UI engineering (`frontend/`).

---

## Repository Structure

- `docs/`: Authoritative project specifications (PRD, TRD, Backend Spec, Frontend Spec) and architecture decision logs (`05_IMPLEMENTATION_DECISIONS.md`).
- `config/`: Declarative YAML configurations for mapping bounds, resolution bands, tracking, and vehicle parameters.
- `backend/cpp/`: Standalone, ROS-independent pure C++20 algorithms, spatial data structures, and unit tests.
- `backend/ros2_ws/`: Canonical ROS 2 Jazzy workspace containing package adapters and launch orchestration.
- `frontend/`: AVR Console — an operator and technical inspection interface built with React, Vite, and Three.js.
- `datasets/`: Dataset storage, calibration files, and evaluation sequences (SemanticKITTI, RELLIS-3D).
- `models/`: Pretrained model checkpoints and neural provider assets.
- `tools/`: Automated experiment runners, data converters, and offline utilities.
- `results/`: Benchmark run databases (SQLite), metrics CSVs, and validation artifacts.
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

# AVR-Map Architecture & Implementation Decision Log

This document records architectural and implementation decisions for the **AVR-Map** prototype (SIH26053). It serves as the living log of engineering choices that refine and operationalize the authoritative specifications located in `/docs` without modifying or replacing them.

---

## ADR-001: Physical Codebase Partitioning (`backend/cpp/` and `backend/ros2_ws/src/`)

- **Status**: Accepted
- **Context**: The Backend Architecture Specification mandates that core algorithms must be implemented as pure C++ libraries independent of ROS wherever practical, with ROS nodes acting strictly as thin adapters. However, the original documents displayed a unified package layout under `ros2_ws/src/`. Furthermore, early development is conducted on macOS workstations where native ROS 2 Jazzy is unavailable.
- **Decision**: Partition the backend repository physically into two distinct directories:
  1. `backend/cpp/`: Standalone, ROS-independent pure C++ algorithm and data structure libraries (`avr_common`, `avr_spatial`, `avr_mapping_core`, `avr_tracking_core`, `avr_traversability_core`), buildable with standard CMake.
  2. `backend/ros2_ws/src/`: Canonical ROS 2 packages and composable node adapters (`avr_msgs`, `avr_input`, `avr_preprocess`, `avr_semantics`, `avr_mapping_ros`, etc.).
  *(Note: This partitioning is an implementation refinement established at project bootstrap to guarantee strict decoupling and enable portable testing on macOS.)*
- **Reason**: Enables immediate native compilation, property testing, and benchmarking on developer workstations (macOS/Linux) without requiring a full ROS 2 environment, while preserving seamless integration into the ROS 2 workspace on canonical Ubuntu machines.
- **Consequences**: Pure C++ libraries must maintain zero compile-time dependencies on `rclcpp` or ROS msg packages. ROS adapter packages in `ros2_ws/src/` will link against the pure C++ libraries.

---

## ADR-002: Target Runtimes and Platform Allocation

- **Status**: Accepted
- **Context**: Autonomous vehicle stacks and robotics perception frameworks rely heavily on Linux kernel interfaces, ROS 2 distributions, and NVIDIA GPU acceleration. Development takes place across diverse environments including macOS.
- **Decision**: Designate **Ubuntu 24.04 LTS (x86_64/ARM64) + ROS 2 Jazzy Jalisco** as the sole canonical production and evaluation runtime for AVR-Map. Designate **macOS** as a development-only platform supported exclusively for:
  - Documentation and architectural modeling;
  - Pure C++ library implementation and unit/property testing (`backend/cpp/`);
  - Offline dataset tools, converters, and schema validation;
  - Full frontend UI/UX development (`AVR Console`).
- **Reason**: ROS 2 Jazzy and NVIDIA CUDA are not natively supported on macOS. Decoupling the pure algorithms and UI enables rapid local iteration while locking system reproducibility to Ubuntu 24.04.
- **Consequences**: Full pipeline integration, multi-node ROS launches, hardware-accelerated neural inference, and formal system benchmarking must be verified inside Ubuntu 24.04 environments (native host or container).

---

## ADR-003: C++ Language Standard Baseline (C++20)

- **Status**: Accepted
- **Context**: The specifications suggest C++17/20. Modern concurrent architecture in AVR-Map specifies cooperative thread cancellation (`std::stop_token`, `std::jthread`), `std::span` for zero-copy view of buffer arrays, and structured concepts.
- **Decision**: Standardize on **C++20** (`CMAKE_CXX_STANDARD 20`, `CMAKE_CXX_EXTENSIONS OFF`) for all pure C++ code and nodes.
- **Reason**: C++20 is fully supported by GCC 13+ (standard in Ubuntu 24.04) and modern Apple Clang (Xcode 15+). It provides standard lifecycle and concurrency primitives needed for robust worker thread management without third-party thread wrapper dependencies.
- **Consequences**: Compilers must support full C++20 core language and standard library features.

---

## ADR-004: Empirical Benchmarking and Truthful Reporting

- **Status**: Accepted
- **Context**: Early documentation discussions referred to potential theoretical reductions in memory and cell count.
- **Decision**: No theoretical or unmeasured numerical threshold (e.g. arbitrary percentages) shall be adopted as a hard acceptance criterion. Benchmark success is strictly defined by empirical, repeatable measurements:
  - Identical sensor input stream;
  - Identical coordinate extents and temporal windows;
  - Identical compute hardware;
  - Formal comparison against a real $5\text{ cm}$ uniform grid baseline;
  - Explicit separation of logical map memory, container capacity, and process RSS;
  - Elimination of startup warm-up frames from steady-state statistics.
- **Reason**: Fabricated or theoretical metrics compromise engineering integrity. True algorithmic value is demonstrated only by fair, reproducible side-by-side execution.
- **Consequences**: All performance claims in presentations or documentation must link directly to reproducible artifact logs in `results/`.

---

## ADR-005: Ground-Truth First Semantic Integration

- **Status**: Accepted
- **Context**: Neural LiDAR semantic segmentation models (such as SalsaNext) introduce heavy dependencies (PyTorch, CUDA, GPU drivers) and inference stochasticity that can impede development of spatial map data structures.
- **Decision**: Implement and thoroughly validate the `GroundTruthSemanticProvider` using ground-truth dataset labels (SemanticKITTI, RELLIS-3D) before integrating deep neural inference.
- **Reason**: Isolates mapping logic, boundary correctness, small-object retention, and spatial indexing from neural model error or GPU environment issues. Ensures the core map engine is robust and provably correct before introducing perception uncertainty.
- **Consequences**: Development milestones can proceed with full test coverage on non-GPU machines.

---

## ADR-006: Document Hierarchy and Specification Authority

- **Status**: Accepted
- **Context**: The repository contains four authoritative specification PDFs in `/docs`:
  1. Product Requirements Document (PRD)
  2. Technical Requirements Document (TRD)
  3. Backend Architecture & Engineering Specification
  4. Frontend UX-UI Design & Implementation Specification
- **Decision**: The existing four specification files in `/docs` remain immutable sources of truth. Any necessary implementation details, configuration refinements, or operational decisions that arise during engineering must be documented in this Implementation Decisions log (`05_IMPLEMENTATION_DECISIONS.md`) rather than editing or overwriting the original PDFs.
- **Reason**: Preserves project requirement provenance and provides clear traceability for judges and reviewers.
- **Consequences**: All team members and coding agents must respect the original specifications as authoritative and check this log for active implementation refinements.

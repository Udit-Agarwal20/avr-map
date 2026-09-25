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

---

## ADR-007: Canonical Integer-Aligned Grid Origin, Key Layout, and Rejection of Fuzzy Boundary Snapping

- **Status**: Accepted
- **Context**: Milestone 1.0 and 1.1 investigated floating-point quantization behavior for non-zero origins. In Milestone 1.1, machine-precision boundary snapping tolerance (`ulp_bound = 4 * (|coord| + |origin|) * 20 * epsilon`) was introduced to bridge floating-point addition cancellation for non-zero double origins. An audit in Milestone 1.2 revealed that any fuzzy snapping tolerance creates a non-zero tolerance envelope that snaps legitimate representable floating-point values strictly below a cell boundary (`std::nextafter(boundary, -infinity)`) into the upper cell, directly violating the mathematical invariant that cell ownership is strictly half-open $[min, max)$. Furthermore, the memory layout of `CellKey` and `TileKey` required empirical verification.
- **Decision**:
  1. **Strict Half-Open Ownership**: Revert and permanently reject fuzzy boundary snapping tolerances. Spatial quantization in AVR-Map strictly follows exact mathematical floor division $[min, max)$:
     $$\text{quantizeToBaseGrid}(x) = \lfloor x \times \text{BASE\_CELLS\_PER\_METER} \rfloor$$
     Every representable floating-point coordinate strictly less than a boundary $B$ remains in the previous cell; $B$ belongs to the intended cell.
  2. **Canonical Origin Anchoring**: The canonical AVR-Map grid is permanently anchored to the `odom` frame origin $(0.0, 0.0)$. In the canonical frame, all dyadic boundaries $k \times 0.05$ have bit-for-bit exact half-open boundaries without cancellation.
  3. **Integer-Aligned Grid Shifts**: Submaps, rolling local windows, and grid shifts are represented as discrete integer base-cell offsets $(ox, oy)$ on the canonical grid. Relative cell coordinates are computed via exact integer subtraction $(gx - ox, gy - oy)$, eliminating floating-point origin subtraction cancellation.
  4. **Key Memory Layout**: Verify and accept that `CellKey` and `TileKey` have natural 8-byte alignment (`alignof = 8`) and a total size of 16 bytes (`sizeof = 16`, composed of 8-byte `uint64_t morton` + 1-byte `uint8_t level` + 7 bytes alignment padding). Neither struct shall be artificially packed (`#pragma pack` or `__attribute__((packed))`). Natural alignment avoids unaligned hardware access penalties on ARM64/x86_64 and maintains `std::is_trivially_copyable_v<CellKey> == true`.
- **Reason**: Preserves mathematical integrity of spatial partitioning, eliminates fuzzy boundary edge cases, and provides stable CPU-side memory layout without premature or unsafe packing.
- **Consequences**: Map scrolling must advance in discrete integer base-cell / tile increments rather than continuous floating-point translations.

---

## ADR-008: GridOrigin2D Integer Representation, Strict Metric Conversion, and Internalization of floorDivPositive

- **Status**: Accepted
- **Context**: Milestone 1.2 completed the quantization and key memory audit. Milestone 1.3 audited the public origin contract and division helpers. The spatial API previously permitted arbitrary floating-point origins and silently rounded non-aligned origins via `round(origin * BASE_CELLS_PER_METER)`. A non-aligned origin (e.g. 0.01 m, 1.001 m) silently shifted the coordinate system identity. Additionally, `floorDivPositive` was exposed in the public `avr::spatial` namespace despite being intended strictly for internal hierarchy math with verified positive constant divisors.
- **Decision**:
  1. **Canonical GridOrigin2D**: Establish `struct GridOrigin2D { int32_t x{0}; int32_t y{0}; }` as the first-class origin type for spatial queries, where $(x, y)$ are integer base-cell offsets on the canonical grid. Canonical map origin is `GridOrigin2D::canonical() = GridOrigin2D{0, 0}`.
  2. **Checked Origin Arithmetic**: All origin offset subtractions ($global\_cell - origin\_cell$) must use checked 64-bit intermediate arithmetic (`checkedCellOffset`) and return `std::nullopt` on 32-bit integer overflow/underflow. Arithmetic never wraps.
  3. **Strict Metric Origin Conversion**: Any continuous metric origin passed as a double or `WorldPoint2D` must be explicitly validated via `metricOriginToGridCell` / `metricOriginToGridOrigin`. Conversion succeeds ONLY if the metric coordinate lies exactly on a canonical 5 cm base-grid boundary (`static_cast<double>(cell) / BASE_CELLS_PER_METER == origin`). Non-aligned origins (e.g., 0.01, 0.02, 0.033, 1.001, -0.014) fail explicitly (`std::nullopt`). Silent rounding or coordinate-system mutation is strictly prohibited.
  4. **Internalization of floorDivPositive**: Move `floorDivPositive` to `avr::spatial::detail`. Public callers must use the safe, fallible `avr::spatial::floorDiv(...)` returning `std::optional<T>`. All internal call sites are verified to use strictly positive compile-time constant divisors (1, 2, 4, 8, 16). In release mode, invalid non-positive divisors defensively return `std::numeric_limits<T>::min()` without undefined behavior and without returning a believable fake spatial coordinate.
- **Reason**: Guarantees unambiguous coordinate frame contracts, prevents silent coordinate frame drift, hardens integer arithmetic against overflow UB, and hides positive-divisor specialization from external consumers.
- **Consequences**: External callers requiring local grid frames must supply integer `GridOrigin2D` offsets or verified grid-aligned metric coordinates.

---

## ADR-009: Resolution Controller Prototype Thresholds Are Configurable and Non-Optimal Defaults

- **Status**: Accepted
- **Context**: Milestone 4 implements the pure C++ `ResolutionController` and information-density gate. Authoritative project specifications (PRD Sections 14–18, TRD Sections 32–43, Backend Architecture Specification Sections 56–61) define the canonical distance-based baseline bands ($0 \le d < 10\text{ m} \rightarrow L0$, $10 \le d < 30\text{ m} \rightarrow L1$, $30 \le d < 60\text{ m} \rightarrow L2$, $d \ge 60\text{ m} \rightarrow L3$) and the multi-factor refinement logic for semantics, dynamic objects, uncertainty, and geometry complexity. However, numerical thresholds for continuous signals (such as normalized density score, minimum observation count, high uncertainty threshold, and terrain complexity threshold) are not scientifically optimized or fixed in the source specifications.
- **Decision**:
  1. **Canonical Distance Source**: Distance baseline thresholds ($10.0\text{ m}, 30.0\text{ m}, 60.0\text{ m}$) originate directly from PRD Section 14 and TRD Section 34.
  2. **Conservative Prototype Defaults**: Adopt the following explicit, configurable prototype defaults in `ResolutionPolicyConfig`:
     - `min_observations = 4`
     - `min_density_score = 0.25`
     - `high_uncertainty_threshold = 0.60`
     - `high_geometry_threshold = 0.50`
     - `semantic_elevated_target = L1` ($0.10\text{ m}$)
     - `semantic_critical_target = L0` ($0.05\text{ m}$)
     - `confirmed_dynamic_target = L0` ($0.05\text{ m}$)
     - `uncertainty_refinement_steps = 1`
     - `geometry_refinement_steps = 1`
  3. **No Optimality Claim**: These values are declared strictly as configurable prototype defaults. No claim of theoretical or empirical optimality is made. Formal parameter calibration will be conducted on real-world datasets (SemanticKITTI, RELLIS-3D) during later benchmarking milestones.
  4. **Information-Density Gate**: Refinement driven by uncertainty or geometry complexity strictly requires sufficient evidence (`observation_count >= min_observations` and `density_score >= min_density_score`). When evidence is insufficient, uncertainty/geometry refinement is blocked and flagged via `EvidenceState::INSUFFICIENT` and reason bit `DENSITY_GATE_BLOCKED`, preventing the fabrication of high-resolution cells from sparse or unconfirmed measurements.
  5. **Safety Signal Independence**: Safety-critical semantic cues (`SAFETY_CRITICAL`) and confirmed dynamic objects (`CONFIRMED`) act as maximum-coarseness caps and are not suppressed by the density gate. If an obstacle is detected in sparse returns, the map preserves fine representation for vehicle safety while truthfully reporting `EvidenceState::INSUFFICIENT`.
  6. **Stateless and Mutation-Free**: The controller is 100% stateless and does not mutate `AdaptiveHierarchy`. Hysteresis and temporal smoothing are separate concerns belonging to the future observation aggregation and map integration pipeline.
  7. **Independent-Target Combination Algebra**: The decision engine computes independent resolution targets for each active signal:
     - `baseline_target`: distance baseline derived from continuous metric distance ($L0$–$L3$);
     - `continuous_target`: distance baseline refined by the SUM of evidence-approved uncertainty and geometry refinement steps ($L_{\text{stepped}} = \max(L0, L_{\text{base}} - (S_{\text{unc}} + S_{\text{geom}}))$);
     - `semantic_target`: configured semantic maximum-coarseness cap if semantic signal is active;
     - `dynamic_target`: configured confirmed-dynamic maximum-coarseness cap if confirmed dynamic signal is active.
     The final requested resolution is the fine-most (minimum level integer) across all active targets:
     $$\text{requested\_level} = \min(\text{baseline\_target}, \text{continuous\_target}, \text{semantic\_target}, \text{dynamic\_target})$$
     This guarantees commutativity across independent signal categories, monotonicity toward finer resolution, and avoids procedural sequential double-counting.
  8. **Configuration Validation Contracts**: Policy validation strictly enforces:
     - `min_observations >= 1` (zero observations is unconditionally insufficient evidence);
     - `semantic_critical_target <= semantic_elevated_target` (safety critical hazards cannot be configured coarser than elevated structures).
- **Reason**: Provides an unambiguous, testable, and deterministic decision engine for M4 while respecting the boundaries of the source documentation and ensuring full configurability.
- **Consequences**: All threshold values must be loaded from configuration rather than hard-coded into decision logic, and invalid configuration parameters must be rejected at controller construction time.

---

## ADR-010: Frame Evidence Spatial-Coverage Proxy and Vertical-Relief Geometry Signal

- **Status**: Accepted
- **Context**: Milestone 5 introduces `FrameEvidence` and signal extraction to feed the M4 `ResolutionController`. The decision policy requires an information density indicator and a geometry complexity indicator for its two-part evidence gate. Authoritative project documentation refers to "information density" and "geometry/terrain complexity" conceptually, but does not provide a fixed physical formula for raw unsegmented LiDAR returns.
- **Decision**:
  1. **Frame-Local Evidence Lifecycle**: `FrameEvidence` is temporary, frame-local measurement support for one logical sensor frame. It is NOT `UniformMap`, `AdaptiveHierarchy`, persistent map state, or replay history. It is replaced on each `buildFrame` call and does not perform cross-frame temporal fusion.
  2. **Density Score as Spatial-Coverage Proxy**: Define `density_score` as a normalized frame-local spatial coverage proxy:
     $$\text{density\_score} = \frac{\text{occupied L0 base cells inside region}}{\text{total possible L0 base cells in region}}$$
     Capacity per level footprint: $L0 = 1$, $L1 = 4$, $L2 = 16$, $L3 = 64$. Range is $[0.0, 1.0]$.
     This metric measures spatial support across the cell footprint, not raw return volume. 100 points in one L0 cell yield $\text{density\_score} = 1/64 \approx 0.0156$ at L3, correctly preventing dense clusters on isolated features from masquerading as broad surface coverage.
  3. **No Calibrated Physical Density Claim**: `density_score` is explicitly a dimensionless prototype coverage proxy. It is NOT claimed to be physical points-per-square-metre or sensor-calibrated beam density.
  4. **Geometry Complexity as Vertical-Relief Proxy**: Define `geometry_complexity` from normalized vertical relief:
     $$\text{vertical\_span} = z_{\max} - z_{\min}$$
     $$\text{region\_width} = \text{resolutionMeters}(\text{region.level})$$
     $$\text{geometry\_complexity} = \text{clamp}\left(\frac{\text{vertical\_span}}{\text{region\_width}}, 0.0, 1.0\right)$$
  5. **Geometry Support Gating**: To prevent isolated points from fabricating geometric surface roughness, `geometry_complexity` is evaluated ONLY if the region contains at least 2 observations AND at least 2 distinct occupied L0 base cells. Otherwise, `geometry_complexity = std::nullopt`. If intermediate arithmetic overflows or produces non-finite values, it fails safely to `std::nullopt` without corrupting summary elevation statistics.
  6. **No Traversability or Learned Feature Claim**: Vertical relief is a transparent geometric cue. It is NOT claimed to represent traversability, ground roughness, slope classification, or learned neural features.
  7. **No Neural / Model Uncertainty Invention**: `FrameEvidence` does NOT infer model uncertainty from elevation variance. Upstream model uncertainty is passed through if supplied externally; otherwise, `uncertainty = std::nullopt`.
  8. **Numerically Stable Parallel Welford Aggregation**: Aggregation across arbitrary $L0$–$L3$ footprints uses integer base-grid coordinates (bounded at $\le 64$ lookups) and Chan, Golub, LeVeque (1979) parallel Welford merging. Updates are strictly transactional: arithmetic overflow leaves prior statistics untouched.
  9. **Transactional Sensor Origin Contract**: `buildFrame` validates finite sensor coordinates before clearing existing frame evidence. If the sensor origin is non-finite, previous valid frame evidence is preserved intact.
  10. **Observation Count Saturation**: Region observation count is stored as `uint64_t` in `EvidenceSummary` and saturated to `UINT32_MAX` (never wrapped) when constructing `ResolutionSignals`.
  11. **EvidenceSummary Variance Semantics**: `EvidenceSummary::elevation_variance` explicitly represents the population variance ($M2 / n$) of observations present inside the requested frame-local region footprint (nullopt if empty, 0.0 for $n=1$). Sample variance remains available separately via `ElevationStats::sampleVariance()`.
- **Reason**: Implements a transparent, deterministic, and numerically safe evidence extraction pipeline that satisfies the M4 evidence gate without fabricating uncalibrated physical metrics or mutating the persistent map hierarchy.
- **Consequences**: Physical LiDAR calibration and learned terrain segmentation remain deferred to future perception modules; downstream consumers must interpret prototype signals according to their defined geometric proxies.

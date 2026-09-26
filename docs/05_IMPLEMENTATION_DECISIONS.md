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

---

## ADR-011: Geometry-Only Adaptive Frame Topology Application Policy

- **Status**: Accepted
- **Context**: Milestone 6 introduces `AdaptiveFrameProcessor`, connecting `FrameEvidence`, `ResolutionSignals`, `ResolutionController`, and `AdaptiveHierarchy` for an end-to-end geometry-only frame processing pipeline. A concrete policy is required to govern how frame-local evidence decisions are translated into hierarchical spatial partitioning without compromising M3 topological safety or pre-empting the persistent adaptive map.
- **Decision**:
  1. **Frame-Derived Topology Rebuild**: Topology is rebuilt per valid sensor frame. Each incoming valid frame resets the previous frame's adaptive hierarchy and instantiates new topology derived exclusively from current frame measurements.
  2. **Sparse Evidence-Driven Root Seeding**: Sparse $L3$ roots are allocated in `AdaptiveHierarchy` ONLY where active frame evidence exists (discovered from `FrameEvidence::baseCellSnapshot()`). Unobserved world space contains zero allocated roots.
  3. **Empty Structural Sibling Rule**: When an evidence-containing node splits, all four direct children are created to satisfy M3's quadtree topology coverage invariants. Any child containing zero frame observations remains a structural coverage LEAF at that level and is NEVER recursively evaluated or refined from distance baseline alone. This strictly prevents the generation of dense near-field hierarchies over unobserved space.
  4. **Monotonic Effective Target Propagation**: For any evaluated evidence-containing region:
     $$\text{local\_target} = \text{local\_decision.requested\_level}$$
     $$\text{effective\_target} = \min(\text{local\_target}, \text{inherited\_target})$$
     $$\text{inherited\_target}_{\text{root}} = L3$$
     Descendants inherit the finest target required by their ancestors. A child may request even finer representation than its ancestor, but cannot undo or coarsen an ancestor requirement down an observed branch.
  5. **Diagnostic Reason Lineage Semantics**: Decision diagnostics record both local reasons and ancestor reasons:
     $$\text{effective\_reasons} = \text{local\_reasons} \cup \text{inherited\_reasons}$$
     `effective_reasons` is a cumulative diagnostic lineage union representing all ancestral and local cues encountered down the branch. It is NOT guaranteed to be the minimal mathematical causal set of bits necessary to reproduce `effective_target`. Future inspection tools must present it as ancestral context rather than asserting that every individual bit caused the local numeric level.
  6. **Fixed Geometry-Only Upstream Defaults**: The M6 production pipeline supplies fixed defaults for unsegmented perception cues (`SemanticRelevance::NONE`, `DynamicRelevance::NONE`, `uncertainty = std::nullopt`). Resolution decisions depend exclusively on distance baseline, spatial support density, and M5 vertical-relief geometry complexity.
  7. **Transactional Frame Guard**: If global frame parameters are invalid (e.g., non-finite sensor origin), `processFrame` fails immediately with `std::nullopt` and preserves previous `FrameEvidence`, `AdaptiveHierarchy`, diagnostics, and statistics completely intact.
  8. **Single-Writer Processor and Reset Capacity**: `AdaptiveFrameProcessor` is a single-writer object. Frame rebuilds call `AdaptiveHierarchy::reset()`, which clears logical hierarchy nodes and tiles while retaining allocated `TilePool` blocks and container capacities for zero-reallocation reuse.
  9. **Not Final Persistent Adaptive Map**: M6 generates frame-local adaptive topology only. It does not implement cross-frame temporal fusion, cell observation payloads (elevation/variance per adaptive cell), persistence, coarsening hysteresis, or raw point replay.
- **Reason**: Provides a rigorous, deterministic, and allocation-controlled bridge between frame evidence and sparse hierarchy topology while preserving clear architectural boundaries ahead of persistent map integration.
- **Consequences**: Downstream consumers must treat M6 topology as a single-frame geometric view; persistent mapping, cell payloads, and temporal hysteresis will be introduced in subsequent milestones.

---

## ADR-012: Persistent Adaptive Leaf Payload and Split-History Contract

- **Status**: Accepted
- **Context**: Milestone 7 introduces `PersistentAdaptiveMap` and `AdaptivePayloadStore`, establishing the foundation for cross-frame persistence in the adaptive hierarchy. Up to M6, adaptive hierarchy topology was rebuilt per frame and did not persist cell observation payloads. To persist elevation statistics across sequential frames without corrupting spatial resolution, explicit rules are required governing payload ownership, transactional fusion, split transitions, and history conservation.
- **Decision**:
  1. **Separation of Topology and Payload Storage**: `AdaptiveHierarchy` remains the frozen, pure structural topology authority (managing cell state bits and tile pools). Persistent measurement state is managed separately by `AdaptivePayloadStore` using sparse 16x16 `AdaptivePayloadTile` blocks. The higher-level `PersistentAdaptiveMap` owns and coordinates both subsystems under strict single-writer thread confinement.
  2. **Active-Leaf-Only Observation Fusion**: Only active hierarchy LEAF nodes may receive new LiDAR observation fusion. INTERNAL nodes, ABSENT nodes, and retired parent caches must NEVER receive direct observation updates.
  3. **Strict Monotonic Frame Ordering**: Incoming frames must satisfy `frame_seq > last_fused_frame_seq`. Duplicate and out-of-order frame sequences are rejected immediately with zero map mutation to prevent double fusion and temporal inconsistencies.
  4. **No Aggregate Redistribution on Split**: When an active LEAF bearing persistent payload is split, its aggregate statistics (count, mean, variance, min, max) are NEVER divided by 4, duplicated into children, or used to fabricate child payloads. Aggregate statistics do not contain directional or sub-cell spatial distributions.
  5. **Retired Parent Cache Semantics**: Upon split, the parent node becomes INTERNAL and its active payload transitions to an immutable `RETIRED_PARENT_CACHE`. This cache serves exclusively as a diagnostic and historical summary and never participates in subsequent observation fusion.
  6. **Replay-Required Leaf State**: When a payload-bearing parent splits, all four newly instantiated child leaves are marked with the `REPLAY_REQUIRED` flag. Children begin with zero fabricated observations. Their historical state represents an information gap that can only be reconstructed through raw observation replay (scheduled for M8).
  7. **History-Gap Propagation**: If a leaf that is already marked `REPLAY_REQUIRED` is further refined, the history-gap propagates downward: all of its direct child descendants inherit `REPLAY_REQUIRED = true`. If the parent also had active observations, its retired cache is marked with `retired_cache_incomplete_mask` to truthfully record that its cache is historically partial.
  8. **Fusion into Replay-Required Leaves**: New observation frames may fuse into active leaves that are marked `REPLAY_REQUIRED`. The leaf updates its active payload normally while retaining `REPLAY_REQUIRED = true` until raw replay explicitly resolves its historical gap.
  9. **Side-Effect-Free PREPARE / COMMIT Transaction Protocol and Observation Conservation**: Frame evidence fusion uses a strict two-phase PREPARE / COMMIT transaction protocol.
     - **Side-Effect-Free PREPARE Phase**: PREPARE is completely side-effect free with respect to persistent topology and payload state (`AdaptiveHierarchy`, `AdaptivePayloadStore`, frame sequencing, and cumulative statistics). Unseen $L3$ root regions with valid evidence are PLANNED as prospective roots rather than inserted into `AdaptiveHierarchy`. Prospective root keys serve as candidate owners during PREPARE. Existing active leaves are queried via `findLeafContainingBase` and evaluated on temporary candidate copies. All ordinary logical and numerical candidate validations (monotonic frame ordering, valid spatial bounds, candidate `tryFuse` Welford merge arithmetic finiteness, and exact observation conservation where $\sum \text{candidate observations} == \text{accepted\_points}$) occur during PREPARE. If any check fails, fusion aborts immediately with `std::nullopt` and produces ZERO persistent mutation.
     - **COMMIT Phase**: Executed only after all candidates have been validated. COMMIT creates planned missing $L3$ roots in canonical `CellKey` order, writes candidate payloads into tiles, updates `last_fused_frame_seq`, increments `total_frames_fused`, updates `total_observations_fused`, and publishes cumulative statistics. Under valid invariant state, non-allocation failure during COMMIT is impossible; memory allocation failure (`std::bad_alloc`) during dynamic container or tile growth remains outside strong rollback guarantees.
  10. **Active vs Retired Observation Accounting**: Cumulative active payload observation counts represent measurements on currently active LEAF nodes (`active_observation_count`). Observations retired into parent caches upon split are historical summaries and accounted for separately (`retired_cache_observation_count`). Their sum (`active + retired`) must NOT be treated as a deduplicated current active-world observation count. Furthermore, cumulative active payload points do not equal cumulative historical accepted points after structural splits occur, because retired parent observations are intentionally decoupled from active child leaves pending M8 raw observation replay.
  11. **No Coarsening or Merge API in M7**: `PersistentAdaptiveMap` does not expose merge or coarsening primitives in M7. Reconciling child payloads and resolving replay status during coarsening requires explicit hysteresis semantics, deferred to subsequent milestones.
  12. **M8 Raw Replay Dependency**: M7 introduces the structural and state foundation for replay tracking (`REPLAY_REQUIRED`), but deliberately does not retain raw point caches or replay buffers. Raw point replay will be introduced in Milestone 8.
- **Reason**: Guarantees scientific and numerical integrity by refusing to invent synthetic high-resolution data from low-resolution aggregate statistics, while establishing complete traceability and conservation for persistent multi-frame mapping.
- **Consequences**: Downstream consumers and planning tools must check `requiresReplay(leaf)` to determine whether a fine leaf currently contains full multi-frame history or only recent observations pending replay reconstruction.

---

## ADR-013: Bounded Recent Raw Observation Cache and L3 Spatial Replay Index

- **Status**: Accepted
- **Context**: Milestone 7 established the persistent adaptive map and marked newly split leaves with `REPLAY_REQUIRED` to prevent fraudulent aggregate subdivision. Reconstructing fine child observations requires access to historical raw sensor measurements. Milestone 8A introduces the foundational replay data source (`RecentObservationCache` and `CachedObservationFrame`) before connecting replay to persistent map state in Milestone 8B.
- **Decision**:
  1. **Configurable Bounded Retention Window and Strict Validation**: `RecentObservationCache` maintains a small configurable window of recent raw observation frames (canonical prototype default: `max_frames = 3`, not claimed optimal; requires `max_frames >= 1`). Invalid configuration (`max_frames = 0`) is explicitly rejected during factory construction (`RecentObservationCache::create` returns `std::nullopt`), never silently clamped or reinterpreted. Older frames are evicted strictly in FIFO sequence once the capacity is exceeded.
  2. **Single Raw Point Storage per Cached Frame**: Each raw `PointXYZ` frame is copied exactly once into an immutable contiguous vector (`raw_points`). There are ZERO duplicate `PointXYZ` copies per map tile, quadtree node, or spatial bucket.
  3. **Strict Preservation of Original Input Order**: The original raw point sequence and index order are strictly preserved in `raw_points`, including individually invalid measurements (non-finite coordinates, out-of-grid bounds). This ensures M8B replay can faithfully reconstruct `FrameEvidence` with identical input order and identical acceptance/rejection semantics.
  4. **Coarse L3 Spatial Bucket Indexing**: Spatial indexing is performed at Level $L3$ (40 cm / 8x8 base cells). Spatial buckets store compact 32-bit integer indices (`uint32_t`) into `raw_points`.
  5. **Replay Candidate Semantics**: A raw point is indexed as a replay candidate if and only if $x, y, z$ are strictly finite and canonical base-grid quantization succeeds. Replay candidates are NOT labeled "accepted map observations", as downstream Welford fusion or sensor model gates may subsequently reject candidates numerically during aggregation.
  6. **Invalid Points Retained but Unindexed**: Non-finite and out-of-range points remain preserved in `raw_points` but receive no spatial index entry, satisfying exact input conservation: $\text{input\_points} \equiv \text{replay\_candidates} + \text{non\_finite\_unindexed} + \text{out\_of\_range\_unindexed}$.
  7. **Finite Extreme Z Indexing**: Finite extreme elevations (e.g., $z = +1\times 10^{200}$) remain spatially indexable candidates because spatial indexing is purely planar; numerical Welford admissibility is deferred to frame evidence aggregation.
  8. **Strict Monotonic Frame Ordering and Transactional Push**: Cached frames require strictly increasing `frame_seq > newest_frame_seq`. New frames are prepared transactionally in temporary memory; existing frames are never evicted if frame preparation fails (e.g., non-finite sensor origin).
  9. **Borrowed Frame Lookup Lifetime**: `findFrame(frame_seq)` returns a borrowed read-only pointer (`const CachedObservationFrame*`). Callers must not retain or use this borrowed pointer across any mutating cache operations that could evict or invalidate frames (such as `pushFrame`, `reset`, or cache destruction). No long-term pointer stability is guaranteed.
  10. **Query Failure Contract and Valid-Empty Query Distinction**: `collectObservationIndices` unconditionally clears the output vector before returning. If the queried frame is missing, the region level is invalid ($> L3$), or parent navigation fails, the query returns `false` with an empty output. If the query parameters and frame are valid but no observations fall within the containing $L3$ bucket, the query succeeds and returns `true` with an empty output. This explicitly distinguishes query error from valid unobserved space.
  11. **Hierarchical Exact Region Membership and Ordering**: Queries for any valid $L0$–$L3$ `CellKey` inspect exclusively the single containing $L3$ bucket and filter points via exact integer canonical cell coordinate matching (zero floating epsilon, zero hand-written division). Returned point indices are strictly sorted in original ascending input arrival order ($output[i] < output[i+1]$). Concatenating canonical child query results in spatial quadrant order is not guaranteed to equal the parent's raw temporal arrival order, but set-union conservation across children ($\bigcup \text{children} \equiv \text{parent}$) holds exactly.
  12. **Telemetry Arithmetic Overflow Protection**: Payload byte telemetry (`raw_point_payload_bytes` and `index_payload_bytes`) and aggregate counters employ saturating arithmetic at limit boundaries to guarantee that large statistics never wrap.
  13. **Zero Persistent Map Mutation in M8A**: `RecentObservationCache` is an isolated replay data source. It does NOT own, modify, or interact with `PersistentAdaptiveMap`, `AdaptiveHierarchy`, or `AdaptivePayloadStore`.
  14. **No Replay Execution or Completeness Claims in M8A**: M8A makes zero claims that a bounded cache is sufficient to reconstruct complete persistent history, nor does it clear `REPLAY_REQUIRED` or modify retired parent caches. Proof of completeness and payload reconstruction are deferred to Milestone 8B.
- **Reason**: Provides an allocation-bounded, high-throughput, and index-preserving raw measurement cache that allows subsequent replay pipelines to query local region history without ballooning RAM through redundant point copies.
- **Consequences**: Downstream replay engines in M8B must query cached frames by region, re-run evidence aggregation, and verify against parent retired caches to determine whether local historical reconstruction is partial or complete.

---

## ADR-014: Direct-Parent Raw Replay Reconstruction and Completeness Verification

- **Status**: Accepted
- **Context**: Milestone 8A introduced the bounded recent raw observation cache and spatial replay index. Milestone 8B performs the first actual use of cached raw observations to repair `REPLAY_REQUIRED` child leaves after an internal parent splits. Because bounded cache capacity may have evicted earlier history, and because raw measurements may not match historical invariants, a rigorous completeness verification contract is mandatory before modifying persistent map state or clearing replay gap markers.
- **Decision**:
  1. **Direct-Parent Scope Boundary**: M8B exclusively supports direct-parent replay. The replay anchor must be an `INTERNAL` parent hierarchy node possessing a complete `RETIRED_PARENT_CACHE` whose four direct children are all currently valid `LEAF` nodes. Deep replay through already-split children, inherited gaps with no direct retired cache, incomplete retired caches, and multi-ancestor replay are deferred to later milestones.
  2. **Bounded Cache Window Screening**: Replay does not assume the cache is complete. Before reconstruction, fast early screening rejects attempts where the cache is empty, `oldest_frame_seq > parent.first_seen_frame`, or `newest_frame_seq < parent.last_seen_frame`, returning `CACHE_HISTORY_INSUFFICIENT` without persistent mutation. Replay sequence iteration examines only retained frames within the parent's historical interval $[first\_seen\_frame, last\_seen\_frame]$ using `cache.frameSequenceSnapshot()` without assuming consecutive frame numbers.
  3. **Verification Meaning and Parent Replay Equivalence Contract**: Before any child reconstruction or persistent mutation, a candidate parent payload is reconstructed from raw measurements within the parent's historical interval using scratch `FrameEvidence` and identical numeric aggregation semantics. In this contract, `APPLIED_COMPLETE` explicitly denotes "complete replay verified against the complete retired-parent measurement/temporal payload contract"; it does NOT claim mathematical proof that the retained raw point multiset is identical to the original raw input history (measurement collision limitation: exact aggregate and temporal equality is the strongest verification possible from the M7 persistent payload schema; raw history hashing/fingerprinting is deferred). Completeness is verified by requiring exact bit-for-bit equality against all 8 persistent measurement and temporal invariants of the complete retired parent cache:
     $$\text{elevation.count}, \text{elevation.mean}, \text{elevation.m2}, \text{elevation.min}, \text{elevation.max}, \text{first\_seen\_frame}, \text{last\_seen\_frame}, \text{observed\_frame\_count}$$
     If any invariant differs, replay fails with `PARENT_RECONSTRUCTION_MISMATCH` and produces ZERO persistent map mutation. No loose epsilon matching is permitted.
  4. **Exclusion of Decision Reason Bits from Completeness Match**: `last_update_reasons` is intentionally excluded from the parent completeness check because raw cached `PointXYZ` measurements do not retain historical runtime resolution decision provenance. The parent's decision reasons remain preserved in the retired cache.
  5. **Replay Entry State Contract**: If `replayDirectChildren` is invoked on a parent whose four direct children are already not replay-required, it returns an explicit `ALREADY_COMPLETE` status with ZERO persistent mutation, preventing double reconstruction or double prepending. If direct children have mixed replay-required states (some true, some false), it returns `INCONSISTENT_REPLAY_STATE` with ZERO persistent mutation.
  6. **Independent Child Reconstruction from Raw Measurements**: Child payloads are reconstructed exclusively from raw observations queried independently for each child region across the parent's historical frame interval. Child statistics are NEVER derived or redistributed from parent aggregate statistics. Zero-contribution frames inside the interval do not increment a child's `observed_frame_count`, so children may legitimately have narrower temporal spans than their parent.
  7. **Historical Observation Conservation**: Before commit, the sum of reconstructed historical child observation counts must strictly equal the parent's retired observation count:
     $$\sum_{c=0}^3 \text{reconstructed\_child}_c.\text{count} \equiv \text{retired\_parent}.\text{count}$$
     Any discrepancy immediately fails replay with `CONSERVATION_FAILURE`. Post-split child observations are excluded from this historical conservation equation.
  8. **Preservation and Prepending of Post-Split Child Payloads**: If a child leaf already received valid active observations post-split, those observations must strictly satisfy $\text{child.first\_seen\_frame} > \text{parent.last\_seen\_frame}$ (temporal overlap returns `TEMPORAL_OVERLAP`). Historical observations are transactionally prepended into the existing child payload via `AdaptiveCellPayload::tryPrependHistory`, merging elevation statistics via parallel Welford while preserving the child's latest post-split `last_update_reasons` and `last_seen_frame`. If a child has zero historical points but existing post-split payload, its active payload is preserved bit-for-bit while clearing `REPLAY_REQUIRED`.
  9. **Reconstructed-Only Child Reason Policy**: For children with historical replay payload but zero post-split active observations, `last_update_reasons` is initialized to empty zero bits (`ResolutionReasonBits{}`). Parent decision reasons are NOT copied into children to prevent implying false child-local decision provenance.
  10. **Empty Historical Child Clearance**: A direct child that legitimately contained zero historical observations and zero post-split observations has its `REPLAY_REQUIRED` flag cleared without creating a fabricated empty payload, correctly reflecting complete knowledge that no observations fell within its quadrant.
  11. **Two-Phase Atomic Commit and All-or-Nothing Mutation**: Replay mutation is strictly all-or-nothing. Complete parent equivalence, 4-child historical reconstruction, conservation, post-split merge validation, and topology checks must ALL succeed before any persistent state is modified. If any check fails, zero mutation occurs and all persistent map snapshots before and after remain bit-for-bit identical.
  12. **Global Map Time and Ingestion Counters Unaffected**: Replay is a spatial redistribution and verification of already-ingested measurements. Replay does NOT advance `last_fused_frame_seq` and does NOT modify cumulative ingestion counters (`total_frames_fused`, `total_observations_fused`). Current payload store metrics (`active_observation_count`, `replay_required_count`) update accurately upon commit.
  13. **Retired Parent Cache Immutability and Retention**: Successful replay does NOT modify, convert, or delete the parent's retired cache. The retired cache is preserved in perpetuity as immutable diagnostic, audit, and coarsening reference state.
  14. **Explicit Replay Orchestration**: Replay is not called automatically inside `splitLeaf` or `refineToLevel` in M8B. Replay remains an explicit, decoupled API call (`replayDirectChildren`) to maintain transactional determinism and testability.
- **Reason**: Guarantees absolute mathematical integrity, observation conservation, and temporal consistency when reconstructing fine-scale leaf history from bounded raw observation caches.
- **Consequences**: Downstream split orchestration can safely repair resolution gaps on direct children whenever adequate raw history is retained, while remaining immune to partial corruption or heuristic extrapolation when history has been evicted.

---

## ADR-015: Verified Persistent Sibling Coarsening and Temporal Metadata Reconstruction

- **Status**: Accepted
- **Context**: Milestone 7 established the persistent adaptive map with leaf payload fusion, M8A introduced bounded raw observation caching, and M8B introduced verified direct-parent raw observation replay. Milestone 9A introduces explicit, manual, and safe persistent coarsening (`mergeDirectChildren`), allowing four direct sibling leaf nodes to merge back into a single active parent leaf. To prevent fabricating temporal history, double-counting frames, or corrupting invariant multi-frame state, a rigorous transaction and reconstruction contract is required.
- **Decision**:
  1. **Direct-Parent Scope Boundary**: M9A exclusively supports merging four direct `LEAF` siblings into an existing `INTERNAL` parent with `level > L0`. Deep recursive coarsening of grandchildren or descendants is explicitly deferred.
  2. **Replay Completeness Requirement**: All four direct children must satisfy `requiresReplay(child) == false`. If any direct child retains unresolved historical gaps (`requiresReplay == true`), coarsening is rejected immediately with `REPLAY_INCOMPLETE` and zero persistent map mutation. Replay and merge remain strictly decoupled.
  3. **Empty Children Semantics and Free-Space Disclaimers**: Direct children with zero active payload and `requiresReplay == false` represent payload-free leaves with no unresolved historical replay gaps (leaves with no active elevation observations and complete known history state). This does NOT prove physical free space or imply occupancy, traversability, or empty-space semantics; M9A does not represent occupancy or dynamic states. If all four direct children are payload-free with complete replay, coarsening succeeds without requiring raw cache: the parent becomes a `LEAF` with no active payload, no retired cache, and `requiresReplay == false`.
  4. **Parent Reconstructed from Current Child History, NOT Old Retired Cache**: Merging is NOT the restoration of the parent's old split-era retired cache. Children may have received replayed historical observations AND subsequent post-split observations. The coarsened parent payload must reflect current active child states through the present merge point.
  5. **The observed_frame_count Non-Summability Problem**: Because multiple sibling children may receive measurements from the same global sensor frame, child `observed_frame_count` values CANNOT safely be summed. Summing child frame counts would severely double-count multi-child frames.
  6. **Raw Cache Reconstruction of Exact Parent Temporal Metadata**: For non-empty merges, `RecentObservationCache` is used to reconstruct exact parent state across the historical interval $[min\_child\_first\_seen, max\_child\_last\_seen]$. Because parent evidence aggregates all contributing points per frame, the parent's `observed_frame_count` naturally increments exactly once per contributing global frame, eliminating double-counting.
  7. **Bounded-Cache Coarsening Limitation and HOLD-FINE Semantics**: For non-empty coarsening, `RecentObservationCache` must be able to reproduce the child payload history required by the merge. Because the raw cache is bounded, long-lived regions whose earlier raw frames have been evicted may become temporarily unmergeable. This is a SAFE FAILURE, NOT map corruption. Under no circumstances may coarsening fall back to approximate temporal metadata (e.g. summing child frame counts or averaging means) or restoring the stale retired parent cache. When cache history is insufficient, merge fails cleanly with `CACHE_HISTORY_INSUFFICIENT` with zero mutation; automatic M9B coarsening engines must interpret this status as "remain at current finer resolution" (HOLD-FINE).
  8. **Four-Child Cache Equivalence Verification**: Raw cache cannot be blindly trusted solely by timestamp bounds. Each of the four direct children is independently reconstructed from the raw cache over the same interval. For every child with active payload, the reconstructed candidate must match the current active payload bit-for-bit across all 8 measurement and temporal invariants (`count`, `mean`, `M2`, `min`, `max`, `first_seen`, `last_seen`, `observed_frame_count`). Payload-free children must reconstruct zero points. If the cache cannot faithfully reproduce all four current children, the merge fails with `CACHE_HISTORY_INSUFFICIENT` or `CHILD_RECONSTRUCTION_MISMATCH` with zero mutation.
  9. **Observation Conservation**: The reconstructed parent observation count must strictly equal the sum of the four verified child observation counts:
     $$\sum_{c=0}^3 \text{reconstructed\_child}_c.\text{count} \equiv \text{reconstructed\_parent}.\text{count}$$
     Discrepancies fail immediately with `CONSERVATION_FAILURE`.
  10. **Retired Parent Cache Removal**: Upon coarsening, the parent transitions from `INTERNAL` to `LEAF`. In accordance with cross-store invariants prohibiting retired caches on non-internal nodes, any existing `RETIRED_PARENT_CACHE` on the parent is permanently removed and superseded by the newly installed active payload.
  11. **Resolution Reason Policy**: Raw measurement replay cannot reconstruct runtime decision provenance. The coarsened parent receives the explicit `reasons` bitmask supplied to `mergeDirectChildren`. Historical child reason bits are NOT ORed together, nor is the old retired parent reason copied.
  12. **Two-Phase Side-Effect-Free PREPARE / COMMIT Protocol**: Replay screening, child reconstruction, child verification, parent reconstruction, temporal validation, and conservation are evaluated entirely on local scratch memory during PREPARE. If all checks pass, COMMIT atomically merges hierarchy topology via `hierarchy_.mergeChildren(parent)`, removes child active payloads and replay flags, clears the old parent retired cache, and installs the candidate active payload. Under valid invariant state, non-allocation failure during COMMIT is impossible; memory allocation failure (`std::bad_alloc`) during parent active payload installation remains outside strong rollback guarantees.
  13. **Invariance of Global Frame Time and Ingestion Metrics**: Manual structural coarsening is a spatial reorganization of persistent state, not an ingestion event. Replay and coarsening do NOT advance `last_fused_frame_seq` and do NOT modify cumulative ingestion counters (`total_frames_fused`, `total_observations_fused`, `total_roots_created`). In contrast, current structural and payload metrics (`node_count`, `leaf_count`, `internal_count`, `active_payload_count`, `retired_cache_count`) accurately update to reflect the newly coarsened representation.
  14. **Full Compatibility with Subsequent Splitting**: A merged parent is a standard active leaf. It may subsequently undergo `splitLeaf` (transitioning active payload to retired cache) and `replayDirectChildren` again without stale metadata or state corruption.
  15. **No Automatic Hysteresis in M9A**: M9A provides only the explicit, manual coarsening transaction primitive. Automatic coarsening heuristics, stability counters, coarsening timers, and resolution hysteresis policies are deferred to Milestone 9B.
- **Reason**: Guarantees absolute temporal correctness, observation conservation, and cross-subsystem invariant validity when coarsening fine dyadic leaves back to their parent.
- **Consequences**: Downstream resolution engines can safely coarsen uniform terrain whenever raw cache retains sufficient history, with complete mathematical assurance that cell elevations and frame counts remain truthful.

---

## ADR-016: Fast Refinement and Observation-Count-Based Delayed Coarsening Hysteresis

- **Status**: Accepted
- **Context**: Milestone 4 (`ResolutionController`) established the instantaneous resolution decision engine based on geometry, semantics, dynamics, and uncertainty. Milestone 9A (`mergeDirectChildren`) established the safe persistent sibling coarsening primitive. However, direct instantaneous coupling between M4 and M9A would induce rapid resolution chatter and oscillation at decision boundaries. A deterministic temporal hysteresis policy state engine is required to decouple instantaneous spatial demand from physical topology mutation.
- **Decision**:
  1. **Separation of Temporal Policy and Physical Mutation**: Milestone 9B1 implements ONLY the deterministic temporal policy state engine (`ResolutionHysteresisController`). It does NOT mutate `PersistentAdaptiveMap`, does NOT call `mergeDirectChildren`, `splitLeaf`, or `refineToLevel`, does NOT build `FrameEvidence`, and does NOT inspect replay cache capacity. All persistent map integration is deferred to M9B2.
  2. **Instantaneous Authority Retained by M4**: M4 remains the sole instantaneous spatial resolution authority. `ResolutionHysteresisController` does not recompute distance bands, surface roughness, semantic classification, or uncertainty; it consumes sequential, evidence-backed `ResolutionDecision` objects produced by M4 for active spatial regions.
  3. **Fast Safety-Dominant Refinement**: If `decision.requested_level < region.level()`, safety considerations dominate immediately. The controller emits `REFINE_NOW` with `target_level = decision.requested_level` on the very first observation. Refinement may jump multiple levels at once (e.g. L3 to L0). Any ongoing coarsening streak or pending coarsen-ready state is immediately cancelled and reset to 0. Fast refinement always vetoes pending coarsening.
  4. **Equal-Level Request Resets Coarsening**: If `decision.requested_level == region.level()`, the region is satisfied with its current resolution. The action returned is `HOLD_CURRENT`, any accumulated coarsening streak is reset to 0, and any pending coarsening action is cancelled.
  5. **Delayed One-Level-at-a-Time Coarsening**: If `decision.requested_level > region.level()`, the region desires coarsening. The controller tracks a stable coarsening streak strictly targeting the immediate parent level (`region.level() + 1`). Even if the instantaneous decision requests multiple levels coarser (e.g. current L0, requested L3), the immediate candidate is strictly one level coarser (L1). Further coarsening from L1 to L2 or L2 to L3 requires an actual future topology merge followed by a fresh full hysteresis window at each intermediate level.
  6. **Coarser Target Variation**: Because coarsening progresses one level at a time, any request at or coarser than `region.level() + 1` (e.g., requests alternating between L1, L2, L3 for a current L0 cell) represents valid evidence that the region desires to be at least one level coarser. Such variations contribute to the same coarsening streak without resetting.
  7. **Evidence-Backed Observation-Count Hysteresis**: Streak accumulation is based strictly on the number of evidence-backed temporal observations received for that specific region (`CellKey`), NOT on wall-clock time and NOT on elapsed global sensor frames. Unobserved global frames do not increment the streak.
  8. **No Inactivity Timeout & Stale Pending Limitation**: M9B1 defines no inactivity timeout. If a region reaches `COARSEN_READY` and subsequently receives no sensor evidence, its readiness state does not automatically expire in M9B1. This is a prototype limitation; M9B2 must define sibling-freshness and temporal group-coordination semantics before triggering physical merges.
  9. **Explicit First-Frame Initialization & Frame Zero Contract**: Frame sequence 0 is a legally valid sensor frame identifier. For a previously unseen region, any `uint64_t` frame sequence is accepted on initial observation, including 0. Subsequent accepted observations for that region must have `frame_seq` strictly greater than `last_frame_seq` ($f_{k+1} > f_k$). Non-consecutive frame sequences (e.g. 10, 42, 105) are accepted.
  10. **Zero State Insertion on Invalid Observation**: Observations with invalid region levels, invalid decision levels (`requested_level` or `baseline_level`), or duplicate/out-of-order frame sequences are rejected with `INVALID_OBSERVATION` before creating or inserting any tracking state. No rejected observation creates ghost hysteresis entries.
  11. **Configurable Default-10 Prototype Threshold (Uncalibrated)**: The controller defaults to `coarsen_stable_observations = 10`. This value is strictly an initial prototype default parameter, NOT calibrated, validated, or claimed optimal for production sensor pipelines. Config validation strictly requires `coarsen_stable_observations >= 1`; invalid values are rejected without silent clamping.
  12. **Single-Emission and Pending Ack Protocol**: Upon reaching the threshold on observation $N$, the controller emits `COARSEN_READY` and transitions to `coarsen_action_pending = true`. Subsequent compatible coarser observations return `COARSEN_READY_PENDING_ACK` with `already_pending = true`, streak saturating at the threshold, and updated `last_frame_seq` and latest `trigger_reasons`.
  13. **Fallible Acknowledgement Lifecycle**: Callers acknowledge coarsening outcomes via `acknowledgeCoarsenResult(CellKey, CoarsenAttemptOutcome)`. Acknowledgement is strictly fallible: unknown regions or regions with `coarsen_action_pending == false` return `false` with zero state mutation. For pending regions:
      - `APPLIED`: The merge succeeded. The old child `CellKey` state is erased from the controller because it is no longer an active leaf. The new parent key begins with a fresh zero state (no streak inheritance).
      - `BLOCKED_HOLD_FINE`: The merge failed safely (e.g., M9A returned `CACHE_HISTORY_INSUFFICIENT` or `REPLAY_INCOMPLETE`). The pending state is cleared and the coarsening streak resets to 0, but accepted-frame ordering history is preserved (`last_frame_seq` is not reset). The region must accumulate a full new hysteresis window (e.g. 10 observations) before retrying, bounding retry frequency and enforcing HOLD-FINE behavior.
      - `CANCELLED`: The merge attempt was cancelled (e.g. topology changed externally). Pending state is cleared, streak resets to 0, and frame ordering history is preserved.
  14. **Sibling Group Coordination Responsibilities (Deferred to M9B2)**: M9A merges four direct sibling leaves into one parent, while M9B1 tracks readiness per region. One child's `COARSEN_READY` is NOT permission to merge the sibling group. M9B2 must verify all four siblings are ready. Furthermore, upon a successful merge, M9B2 is responsible for acknowledging/erasing all four retired sibling states. If a group merge is blocked, M9B2 must acknowledge/reset all participating siblings to avoid stale pending deadlocks.
  15. **Independence of Hysteresis Threshold (10) and Replay Cache Capacity (3)**: The prototype hysteresis threshold is 10 observations, while the replay cache retains 3 raw frames by default. There is no requirement that `coarsen_stable_observations <= max_frames`. Hysteresis answers *when to attempt* coarsening, whereas M9A answers *whether the map can physically and safely execute* the coarsening. With long-lived payloads, reaching a 10-observation threshold may frequently occur after required raw history has already been evicted from the 3-frame cache. The safe and deliberate result is that M9A rejects with `CACHE_HISTORY_INSUFFICIENT`, M9B2 reports `BLOCKED_HOLD_FINE`, and the cell safely remains at fine resolution (HOLD-FINE). This is a known current integration limitation to be reconciled during system tuning.
  16. **Attempt Permission, Not Merge Guarantee**: `COARSEN_READY` represents only temporal policy readiness to attempt coarsening. It does NOT guarantee that raw history is available, replay is complete, siblings are ready, or topology is mergeable.
  17. **Latest Reason Context Only**: `trigger_reasons` in `HysteresisResult` contains the `ResolutionReasonBits` from the single most recent instantaneous `ResolutionDecision` that produced the returned action. Reason bits are NEVER ORed across the multi-observation streak, ensuring clean, truthful latest-transition context for downstream M9A logging.
  18. **Deterministic Sparse State Management & Single-Writer Contract**: Temporal state is stored sparsely per `CellKey` without dense spatial allocation. Public `stateSnapshot()` returns records deterministically sorted in canonical `CellKey` order, never exposing internal hash map ordering. The controller is single-writer without internal locks, matching `PersistentAdaptiveMap` concurrency semantics.
- **Reason**: Decouples instantaneous spatial resolution demand from physical map coarsening mutations, eliminating high-frequency resolution flutter while preserving immediate safety-critical multi-level refinement.
- **Consequences**: Downstream pipelines gain a robust, deterministic policy engine that safely drives coarsening attempts without risk of infinite retry loops or map corruption.

---

## ADR-017: Exact Internal Coarsening Shadow State and Long-Lived Coarsening

- **Status**: Accepted (Explicitly supersedes ADR-015 item 7 requirement that routine non-empty coarsening depends on full raw-cache history)
- **Context**: Milestone 9A established safe persistent coarsening (`mergeDirectChildren`), but required `RecentObservationCache` to reproduce the entire child payload history across $[min\_child\_first\_seen, max\_child\_last\_seen]$ for non-empty merges. Because raw observation caches are bounded in memory, fine-resolution regions active for more than a few frames quickly lose early raw frames. Consequently, routine coarsening of long-lived regions failed with `CACHE_HISTORY_INSUFFICIENT`, creating a permanent hold-fine stall. An architectural audit revealed that simply expanding raw cache size cannot guarantee correctness for arbitrary operational lifetimes, and that summing child `observed_frame_count` remains mathematically invalid due to multi-child frames. Milestone 9B2A resolves this limitation by maintaining an exact sequential internal-node coarsening shadow during frame ingestion.
- **Decision**:
  1. **Bounded Raw Cache Cannot Support Arbitrary-Lifetime Merges**: A raw observation cache storing point clouds is strictly bounded in memory (e.g. 3–10 frames). Fine regions in operational autonomy may remain split for hundreds or thousands of frames before terrain uniformity warrants coarsening. Relying on bounded raw cache retention for routine coarsening fundamentally prevents long-lived coarsening.
  2. **Enlarging Cache Is Not a Correctness Guarantee**: Arbitrarily increasing raw cache capacity merely delays cache exhaustion and causes unbounded heap consumption. It does not provide an architectural guarantee of coarsening capability for arbitrary lifetimes.
  3. **Non-Summability of Child `observed_frame_count`**: Multiple sibling children frequently receive measurements from the same global sensor frame. Summing child frame counts would severely double-count multi-child frames (e.g. 4 children receiving points from 1 frame would yield 4 frames instead of 1). Temporal frame metadata must be maintained at the parent level.
  4. **Rejection of Base + Aggregated Delta Representation**: The pre-implementation audit considered storing an aggregated post-split delta and performing a single Welford merge with the retired base at coarsening time ($B \oplus D$). However, floating-point addition and Welford parallel combination are non-associative: $(((B \oplus F_1) \oplus F_2) \oplus F_3) \neq B \oplus ((F_1 \oplus F_2) \oplus F_3)$ down to the last bit. Milestone 8B requires exact bit-for-bit equality across all 8 measurement/temporal fields upon re-splitting and raw replay. Performing one-shot Base + Delta merge at coarsening time breaks M8B replay equivalence.
  5. **Current Parent Shadow Architecture**: Instead of Base + Delta, each internal node maintains a `current_parent_shadow` representing "the full currently known measurement/temporal state the INTERNAL node would possess if it had remained an active LEAF and received the same sequence of real sensor-frame observations." It is NOT only post-split data, an aggregated delta, a replay cache, or a second retired cache. New sensor frames fuse sequentially into the shadow using identical `AdaptiveCellPayload::tryFuse` frame-order semantics, guaranteeing bit-for-bit equivalence with unsplit leaf evolution.
  6. **Immutable Retired Parent Cache Remains Historical Anchor**: The `RETIRED_PARENT_CACHE` created on split remains completely immutable and separate from the shadow. It continues serving its role as the ground-truth anchor for M8B historical child replay and split verification.
  7. **Internal Current-Parent Shadow Is Separate and Mutable**: The shadow state (`InternalCoarseningState`) is stored separately from the retired cache. It updates online as real sensor frames arrive, representing the current exact coarsened candidate.
  8. **Sequential Real-Frame Update Contract**: For each genuinely new sensor frame, an internal ancestor's shadow is updated at most once per frame. The update is computed directly from `FrameEvidence::aggregateElevationStats(ancestor)` for that frame. If the frame contributes zero observations to the ancestor region, the shadow is not touched.
  9. **Replay Never Updates Shadow**: M8B `replayDirectChildren` redistributes pre-split historical observations that were already part of the parent state before split. Replay operations MUST NEVER update the internal parent shadow.
  10. **Structural Merge Never Updates Ancestor Shadows**: Merging children is a topological reorganization of existing persistent state, not a real sensor observation. Successful structural merge does not update grandparent or higher ancestor shadows.
  11. **Coarsening Base State Model**: Internal coarsening history explicitly defines `CoarseningBaseState`:
      - `COMPLETE_EMPTY`: Parent had no active elevation payload when split and no replay gap.
      - `COMPLETE_RETIRED_CACHE`: Parent had complete active payload when split; immutable retired cache is canonical historical base.
      - `INCOMPLETE`: Parent was already `REPLAY_REQUIRED` when split; historical completeness is unknown. An `INCOMPLETE` internal node must never produce a complete coarsened leaf. The only valid enum identifier is `CoarseningBaseState::INCOMPLETE`.
  12. **Universal Internal Node State Invariant**: In `PersistentAdaptiveMap`, every `INTERNAL` hierarchy node owns exactly one `InternalCoarseningState`. Nodes with state `LEAF` or `ABSENT` own none.
  13. **Operational No-Cache Merge Path**: Milestone 9B2A supersedes ADR-015 item 7 for routine coarsening: `mergeDirectChildren(parent, reasons)` without cache is the canonical operational coarsening API. For complete internal states, it installs the candidate derived from `current_shadow` (or retired cache if no post-split frames) without requiring historical raw cache.
  14. **Strict Cache Overload Preserved for Diagnostic Verification**: `mergeDirectChildren(parent, cache, reasons)` is preserved as a strict diagnostic verification mode. When raw cache contains the entire required current history, strict verification proves BOTH: (A) each current child payload matches raw reconstruction, and (B) the canonical current parent candidate derived from internal coarsening state matches raw parent reconstruction across all 8 measurement/temporal fields (`elevation.count`, `elevation.mean`, `elevation.M2`, `elevation.min`, `elevation.max`, `first_seen_frame`, `last_seen_frame`, `observed_frame_count`). If raw parent reconstruction mismatches the canonical candidate, merge fails cleanly with `PARENT_RECONSTRUCTION_MISMATCH` with zero mutation. If cache history is insufficient, it returns `CACHE_HISTORY_INSUFFICIENT`. Operational pipelines must use the canonical no-cache overload.
  15. **Reason Metadata Decoupling**: Shadow `last_update_reasons` is internal and non-authoritative. The merged parent receives the explicit `reasons` passed to `mergeDirectChildren` as its final `last_update_reasons`.
  16. **Derived Shadow Counts Are Structural Provenance**: Ancestor shadow statistics summarize the same sensor observations already counted by active leaves. Shadow observation counts are derived duplicates and MUST NOT increment global cumulative sensor counters (`total_observations_fused`).
  17. **Full Lifecycle Integrity (Reset, Split, Re-Split)**: `reset()` clears all internal coarsening states. Merging cleans up children, parent retired cache, and parent internal coarsening state, installing the active payload. A subsequent `splitLeaf` on the merged parent installs a fresh retired cache and fresh coarsening state, successfully passing M8B raw replay.
  18. **M9B1 Frozen and Sibling Automation Deferred**: Milestone 9B1 (`ResolutionHysteresisController`) remains frozen and unmodified. Group-sibling readiness policies, freshness windows, and automatic pipeline integration are deferred to Milestone 9B3.
- **Reason**: Enables long-lived fine-resolution regions to be coarsened safely and deterministically after arbitrary sensor frame counts without requiring unbounded raw observation history.
- **Consequences**: Downstream autonomous resolution pipelines can execute routine coarsening operations online without cache capacity constraints, while preserving exact bitwise equivalence for split/replay verification.

---

## ADR-018: Staged Raw Observation Cache Commit and Replay Window Isolation

- **Status**: Accepted (Narrow, non-breaking extension to RecentObservationCache ingestion interface)
- **Context**: In Milestone 8A, `RecentObservationCache::pushFrame` combined raw point validation, candidate construction, deque insertion, and FIFO capacity eviction into a single immediate mutating call. The Milestone 9B3A pre-implementation integration audit identified a critical eviction hazard when coordinating automatic persistent adaptation loops: when an incoming sensor frame $F$ triggers structural leaf refinement, any newly split parent node requires an immediate M8B historical replay (`replayDirectChildren`) to reconstruct child leaf history. If raw frame $F$ is pushed directly into the bounded cache before this replay runs, pushing $F$ can prematurely evict the oldest retained frame $[F - \text{max\_frames}]$. If the newly split parent's historical interval relies on that oldest frame, replay fails with `CACHE_HISTORY_INSUFFICIENT`, stranding children with unresolved historical gaps. Milestone 9B3B resolves this hazard by separating raw frame ingestion into a staged two-phase PREPARE and COMMIT interface.
- **Decision**:
  1. **Finite Sensor Origin Required**: Sensor origin coordinates must be finite (`std::isfinite(sensor_origin.x)` and `std::isfinite(sensor_origin.y)`). A non-finite sensor origin is frame-fatal and immediately rejects `prepareFrame` without cache mutation.
  2. **Individual Non-Finite and Out-of-Range Raw Measurements Retained but Unindexed**: In accordance with frozen M8A contracts, individual non-finite (`NaN`, `Inf`) and out-of-range raw points do NOT reject the frame; they remain retained in the frame's `raw_points` vector in exact original arrival order and are classified as `non_finite_unindexed` or `out_of_range_unindexed`, excluded from spatial replay buckets.
  3. **Separation of Frame Preparation from Cache Commit**: Raw observation frame processing is split into `prepareFrame(...)` and `commitPreparedFrame(...)`. The legacy `pushFrame(...)` method is preserved as an inline convenience wrapper that delegates directly to `prepareFrame` followed by `commitPreparedFrame`.
  4. **Move-Only Staged Representation (`PreparedCachedObservationFrame`)**: Preparation returns a move-only wrapper (`PreparedCachedObservationFrame`) that completely owns the built `CachedObservationFrame`, raw point vectors, L3 spatial bucket index, sensor origin, and frame-local metrics. It retains zero references or pointers into caller input spans, `RecentObservationCache`, or `PersistentAdaptiveMap`.
  5. **Strict Zero-Mutation Preparation**: `prepareFrame` is a `const` member function. It performs zero mutations on `RecentObservationCache`: retained frame count, oldest/newest sequence numbers, bucket indices, and aggregate byte telemetry remain bit-for-bit unchanged.
  6. **Zero FIFO Eviction During Prepare**: Even when the cache is at maximum capacity (`frameCount() == max_frames`), calling `prepareFrame` does not evict the oldest retained frame, reserve a logical cache slot, or modify container storage.
  7. **Pointer Lifetime Preservation**: A borrowed pointer previously obtained via `findFrame(...)` is not invalidated by `prepareFrame(...)`.
  8. **Replay Window Isolation**: Downstream pipelines can safely execute M8B historical replay (`replayDirectChildren`) against the un-evicted raw cache window while current frame $F$ is prepared and held in staged memory.
  9. **Query Invisibility Prior to Commit**: Uncommitted prepared frames are completely invisible to cache inspection: `findFrame(prepared_seq)` returns `nullptr`, `containsFrame` returns `false`, `frameSequenceSnapshot` excludes the prepared sequence, and spatial index queries return missing-frame results.
  10. **Zero Side-Effects on Discard**: If a prepared frame is destroyed without calling `commitPreparedFrame` (e.g. due to downstream frame rejection or pipeline rollback), the cache experiences zero mutations, zero stat changes, and zero memory leaks.
  11. **Commit Frame Sequence Validation Against Current Cache State**: `commitPreparedFrame` enforces strictly increasing sequence numbers against the CURRENT cache state at commit time (`prepared.frameSeq() > newestFrameSeq()`). It does not assume cache state remained unchanged since prepare.
  12. **Stale Prepared Rejection with Zero Mutation**: If a prepared frame becomes stale (e.g. another frame committed in the interim, or frame sequence is $\le$ current newest), `commitPreparedFrame` rejects the operation with `std::nullopt` and leaves cache state completely untouched.
  13. **Insertion Succeeds Before Destructive FIFO Eviction**: To maintain strong exception safety, `commitPreparedFrame` inserts the newly moved frame into the backing storage before performing FIFO eviction (`pop_front`). If container insertion fails (e.g. throwing `std::bad_alloc`), the previous retained cache remains intact and has not lost its oldest frame.
  14. **Cumulative Statistics Advance Only After Successful Commit**: Cache lifetime counters (`total_frames_pushed`) and aggregate retained metrics increment and publish strictly after frame insertion and eviction have completed.
  15. **Public M8B Staged Replay vs Early-Push Insufficiency**: Public `PersistentAdaptiveMap::replayDirectChildren` succeeds completely when current frame $F$ is held staged, whereas legacy early push of $F$ would evict required history and explicitly fail with `ReplayStatus::CACHE_HISTORY_INSUFFICIENT` with zero map mutation.
  16. **Reset Independence**: Resetting `RecentObservationCache` clears retained frames and resets `total_frames_pushed_`, but does not invalidate the self-contained state of an independently prepared `PreparedCachedObservationFrame`. If the prepared frame satisfies sequence validity relative to the post-reset cache, it may be committed cleanly.
  17. **Automatic Adaptation and Refinement Policy Deferred**: Milestone 9B3B introduces zero automatic pipeline coordination, zero hysteresis alterations, zero cascaded refinement, and zero gap-closing merge changes.
- **Reason**: Guarantees that current-frame raw point ingestion never prematurely evicts historical frames needed for same-cycle leaf refinement replay, providing robust cache transaction isolation for future automatic adaptation pipelines.
- **Consequences**: Downstream mapping pipelines can safely stage incoming raw sensor frames, execute structural refinements and historical replays against intact previous cache windows, and commit raw frames afterward with zero historical data loss.

---

## ADR-019: Complete-Shadow Subtree Collapse and Replay-Gap Recovery

- **Status**: Accepted (Explicit structural recovery primitive in `PersistentAdaptiveMap`)
- **Context**: Safety-driven multi-level refinement can refine a coarse cell (such as an L3 or L2 node) across multiple levels down to L0. If raw sensor frames spanning the cell's earliest observations have been evicted from the bounded `RecentObservationCache`, historical replay (`replayDirectChildren`) cannot be completed for deep descendants, leaving leaf nodes with `requiresReplay == true` and intermediate nodes with `base_state == INCOMPLETE`.
  Ordinary sibling coarsening (`mergeDirectChildren`) is insufficient to recover from this condition because:
  1. `mergeDirectChildren` strictly requires all direct children to be `LEAF` nodes, whereas deep refinement introduces intermediate `INTERNAL` nodes.
  2. `mergeDirectChildren` requires direct children to be either replay-complete or derived from an intact single-parent base, preventing coarsening across multi-level incomplete subtrees.
  However, the coarse ancestor itself may still possess an exact, complete historical representation (`COMPLETE_EMPTY` or `COMPLETE_RETIRED_CACHE`) and an exact `current_shadow` that has accumulated all subsequent sensor frames fused into the subtree without loss.
- **Decision**:
  1. **Explicit Recovery Primitive (`collapseSubtreeFromCompleteShadow`)**: Introduce an explicit/manual API `collapseSubtreeFromCompleteShadow(ancestor, reasons)` on `PersistentAdaptiveMap`. This is a dedicated structural gap-recovery primitive and NOT an ordinary M9B1 gradual coarsening policy.
  2. **Ordinary Merge Kept Frozen**: Ordinary sibling coarsening (`mergeDirectChildren`) remains frozen and unmodified with all of its existing eligibility and leaf-child preconditions intact.
  3. **Selected Ancestor Base Must Be Complete**: The targeted ancestor must be an `INTERNAL` node whose historical base state is strictly `COMPLETE_EMPTY` or `COMPLETE_RETIRED_CACHE`. If the ancestor's base state is `INCOMPLETE`, the operation immediately returns `ANCESTOR_BASE_INCOMPLETE` with zero mutation.
  4. **Intermediate Descendants May Be Incomplete**: Intermediate descendant internal nodes are permitted to have `base_state == INCOMPLETE`, and active descendant leaf nodes are permitted to have `requiresReplay == true`. These gaps reflect missing historical raw frames at fine resolutions, which is the exact condition this recovery primitive resolves.
  5. **Canonical Parent Candidate Derivation**: The canonical restored ancestor payload is derived strictly from the ancestor's own base and shadow in accordance with frozen M9B2A rules:
     - Case A: `COMPLETE_RETIRED_CACHE` and `has_current_shadow == false` $\to$ candidate is the retired parent cache.
     - Case B: `COMPLETE_EMPTY` and `has_current_shadow == false` $\to$ candidate is empty (no active payload).
     - Case C: `has_current_shadow == true` $\to$ candidate is `current_shadow`.
     The candidate is NEVER constructed by aggregating descendant Welford accumulators or querying `RecentObservationCache`. The caller-supplied `reasons` are assigned as `candidate.last_update_reasons`.
  6. **Subtree Topology and Closure Validation**: The entire descendant subtree under `ancestor` is enumerated via deterministic traversal. All intermediate nodes must have valid child pointers, correct level decrements ($L - 1$), and zero duplicate keys. Any structural defect aborts with `INVALID_SUBTREE_STATE` and zero mutation.
  7. **Descendant Measurement and Coverage Consistency Checks**: Active descendant leaf payloads are validated against the canonical parent candidate:
     - Checked sum of active descendant leaf observation counts must satisfy $\sum \text{count}_{\text{leaf}} \le \text{count}_{\text{parent}}$ using checked 64-bit unsigned arithmetic. Strict inequality is expected when fine historical replay was incomplete. Descendant count exceeding parent count immediately aborts with `DESCENDANT_COVERAGE_INCONSISTENT`.
     - Empty parent candidate: if the parent candidate has no observations, no active descendant may have observations.
     - Elevation range consistency: descendant $\min(z) \ge \text{parent } \min(z)$ and descendant $\max(z) \le \text{parent } \max(z)$ under exact comparison.
     - Temporal consistency: descendant $\text{first\_seen} \ge \text{parent } \text{first\_seen}$, descendant $\text{last\_seen} \le \text{parent } \text{last\_seen}$, and descendant $\text{observed\_frame\_count} \le \text{parent } \text{observed\_frame\_count}$.
     - Newest-frame coverage: no active descendant may contain a frame newer than the parent shadow ($\text{descendant } \text{last\_seen} \le \text{parent } \text{last\_seen}$).
  8. **Provenance Guarantee vs Consistency Checks**: Temporal, count, and elevation-range subset checks are necessary consistency guards against gross corruption or divergence; they are NOT by themselves a formal proof of observation-set membership. The actual coarse-history completeness guarantee derives from the frozen M9B2A provenance contract: every genuine sensor frame fused into active descendants is also aggregated into each relevant internal ancestor shadow exactly once from the same `FrameEvidence` in the same successful persistent fusion transaction. Therefore, complete ancestor shadow + valid M9B2A invariants is authoritative.
  9. **Trust Boundary and No Hash/Fingerprint Claim**: Collapse trusts the exact online provenance of a `COMPLETE` ancestor shadow established by M9B2A. M9B3C does NOT independently reconstruct every descendant point from raw history, which would defeat the bounded-memory architecture. There is currently no persistent multiset fingerprint linking each active descendant observation to the ancestor shadow; normal correctness relies on transactional M9B2A fusion and validator invariants.
  10. **Mean/M2 Policy**: No floating-point reconstruction of parent mean or $M_2$ from descendants is attempted. The complete coarse ancestor shadow is authoritative.
  11. **Deterministic Bottom-Up Hierarchy Merge Plan**: To respect `AdaptiveHierarchy` invariants without manually mutating node state flags, internal nodes are merged bottom-up: sorted by level ascending (finest levels first, e.g. L1 before L2, L2 before L3), then canonical `CellKey` order. At each step, direct children are guaranteed to be leaves.
  12. **Complete Metadata and Subtree Cleanup**: On commit, all descendants (internal and leaf) are removed from the hierarchy, their payload store entries (`active_payload`, `retired_cache`, `replay_required`, and `retired_cache_incomplete` metadata) are erased, and their `InternalCoarseningState` entries are removed. Ancestor retired cache, incomplete markers, and coarsening state are cleared, the canonical parent payload is installed as active (if non-empty), and ancestor `requiresReplay` is set to `false`. Restored ancestor is strictly a `LEAF`.
  13. **Deterministic Removed Descendant Keys API**: `SubtreeCollapseResult` returns a deterministic value-owned vector `removed_descendant_keys` sorted in canonical `CellKey` order. It contains EVERY removed descendant key (both former active leaves and intermediate internal nodes) and excludes the restored ancestor. On failure, this vector is empty. The vector size strictly equals `descendant_nodes_removed`.
  14. **Future Hysteresis Cleanup Contract**: Returning all removed descendant keys enables the future pipeline coordinator to execute defensive cleanup: `for (const auto& key : result.removed_descendant_keys) hysteresis.eraseRegion(key);`, ensuring stale M9B1 hysteresis state is purged even for former leaves that had become internal. The restored ancestor starts with fresh hysteresis state. M9B3C itself has zero dependency on M9B1.
  15. **Grandparent and Higher Ancestor Shadows Immutability**: If the collapsed ancestor is nested under an internal grandparent, the grandparent's `current_shadow` and coarsening state remain bit-for-bit unchanged, as subtree collapse is a purely structural reconfiguration and not a sensor observation frame.
  16. **Ingestion Counters Preserved**: Successful collapse leaves untouched ALL cumulative sensor ingestion counters: `total_frames_fused`, `total_observations_fused`, `last_fused_frame_seq`, and `has_fused_frames`. Structural and current-state metrics update appropriately. Recovered observations are never counted as new ingestion.
  17. **Result Status and Diagnostic Semantics**:
      - `APPLIED_GAP_CLOSED` means the complete-shadow subtree-collapse recovery path applied successfully. It does NOT assert that at least one replay gap existed (replay-complete descendants are allowed, where `replay_required_leaves_closed == 0`). The exact number of closed gaps is observable via `replay_required_leaves_closed`.
      - `levels_collapsed` reports the maximum hierarchy depth removed: $\text{ancestor\_level} - \text{finest\_removed\_level}$. For mixed-depth subtrees, not every branch necessarily reached this depth.
  18. **Commit Failure and bad_alloc Contract**: During commit, hierarchy merge calls are audited; unexpected failure classifies as `INTERNAL_ERROR`. `std::bad_alloc` during payload tile allocation occurs after structural commit begins and remains outside strong rollback guarantees.
  19. **Scientific Semantics of Gap Closure**: Returning `APPLIED_GAP_CLOSED` means:
      *"An incomplete finer active representation was discarded and the map returned to a complete canonical ancestor representation."*
      It does NOT mean:
      *"Missing historical fine measurements were reconstructed."*
      Coarse ancestor history is preserved exactly, but fine spatial redistribution of old history is permanently abandoned.
  20. **Raw Cache Independence and Deferred Policy**: The recovery primitive has zero dependency on `RecentObservationCache` and operates cleanly even after old raw history is completely evicted. Automatic pipeline triggering, automatic adaptation, and cascaded coarsening policies are strictly deferred.
- **Reason**: Provides an architecturally sound and scientifically honest mechanism to resolve stranded deep refinement replay gaps by collapsing incomplete descendant subtrees back into complete coarse ancestors whose shadows have continuously tracked full observation history.
- **Consequences**: Autonomous systems can recover from unreplayable deep refinement branches safely and deterministically without unbounded raw frame retention, restoring full invariant compliance and unblocking subsequent refinement cycles.

---

## ADR-020: Staged Hysteresis Observation Preview and Commit

- **Status**: Accepted (Transaction-isolated staged observation processing for `ResolutionHysteresisController`)
- **Context**: In Milestone 9B1, `ResolutionHysteresisController::observe` combined transition evaluation, candidate installation, streak advancement, and pending action generation into a single immediate mutating call. The Milestone 9B3D coordinator architecture audit identified a fundamental transaction-isolation problem: an autonomous adaptation coordinator must evaluate temporal resolution hysteresis decisions BEFORE executing structural hierarchy refinement and raw sensor point fusion, but the candidate temporal observation state must persist ONLY AFTER the incoming sensor frame successfully fuses into `PersistentAdaptiveMap`. If frame fusion later fails (e.g. numerical failure, invalid sensor data), an immediately mutated hysteresis state would cause irreversible policy drift. Furthermore, unconditional candidate overwrite upon delayed commit would allow stale candidates to overwrite newer state mutations (such as intervening acknowledgements, erasures, or clears). Milestone 9B3E resolves this transaction boundary by introducing non-mutating preview and staged commit with stale-precondition validation.
- **Decision**:
  1. **Immediate Mutation Incompatible with Staged Pipelines**: Immediate mutation in `observe(...)` violates the transaction isolation required by multi-phase mapping pipelines. Staged preview and explicit commit decouple decision inspection from persistent policy state advancement.
  2. **Non-Mutating Pure Preview (`previewObservation`)**: `ResolutionHysteresisController::previewObservation(...)` is a `const` member function. It performs zero mutations on controller state: zero insertions into `regions_`, zero erasures, zero streak updates, zero acknowledgement mutations, and zero revision advances.
  3. **Value-Owned Self-Contained Staged Object (`HysteresisPreparedObservation`)**: Preview returns a value-owned struct containing the evaluated `HysteresisResult`, the expected controller revision token, expected prior presence and state, candidate presence, and candidate `RegionHysteresisState`. It retains zero references or pointers into controller storage, requires zero dynamic heap allocation, and has trivial copy/move semantics (`std::is_trivially_copyable_v<HysteresisPreparedObservation> == true`, `sizeof = 128`, `alignof = 8`).
  4. **Single Internal Transition Evaluator**: The core state-transition logic of Milestone 9B1 is unified into a single pure evaluator function (`evaluateTransition`) shared between `previewObservation` and `observe`. There is exactly ONE copy of frame ordering logic, finer/equal/coarser branching, stable streak counting, pending threshold gating, and reason-bit policies. Legacy `observe` delegates directly to `previewObservation` followed by `commitPreparedObservation`, ensuring identical algorithmic behavior and zero code fork.
  5. **Global Controller Revision Token (`state_revision_`)**: A 64-bit monotonically advancing revision token tracks controller mutations. `state_revision_` is a GLOBAL transaction token. Consequently, ANY successful controller mutation invalidates EVERY staged observation prepared under an earlier revision, including mutations to completely unrelated CellKeys. This intentionally conservative behavior prevents hidden concurrent policy desynchronization.
  6. **Clear as a Deliberate Revision Barrier**: When `state_revision_ < UINT64_MAX`, `clear()` removes all tracked region states, preserves configuration, and ALWAYS increments `state_revision_` by exactly 1, even if `regions_` was already empty. This deliberate revision barrier invalidates all staged observations across the entire controller.
  7. **Revision Exhaustion Terminal Contract**: When `state_revision_ == UINT64_MAX`, the controller is terminally mutation-frozen. No further logical controller mutation may occur:
     - `commitPreparedObservation` returns `PreparedCommitStatus::REVISION_EXHAUSTED` with zero mutation;
     - `commitPreparedBatch` returns `PreparedBatchCommitStatus::REVISION_EXHAUSTED` with zero mutation;
     - legacy `observe` returns `HysteresisAction::INVALID_OBSERVATION` with zero mutation;
     - `acknowledgeCoarsenResult` returns `false` with zero mutation;
     - `eraseRegion` returns `false` with zero mutation;
     - `clear()` returns immediately with zero mutation (leaving tracked `regions_` intact and revision at `UINT64_MAX`).
     Under no circumstances does `state_revision_` ever wrap from `UINT64_MAX` to 0.
  8. **Preview at Revision Exhaustion**: `previewObservation` remains a read-only inspectable query even at `state_revision_ == UINT64_MAX`, returning a valid candidate transition with `expected_controller_revision = UINT64_MAX`. However, because there is no revision headroom to advance, attempting to commit this candidate returns `REVISION_EXHAUSTED`.
  9. **Successful Single-Commit Ordering**: Single commit executes ordinary validation BEFORE mutation:
     - Candidate is committable and valid;
     - `prepared.expected_controller_revision == state_revision_`;
     - Expected prior presence matches current container presence;
     - Expected prior state matches current container state under semantic field equality (no raw `memcmp`);
     - Candidate invariants hold (`has_last_frame == true`, `last_frame_seq == prepared.frame_seq`, streak $\le$ threshold, pending target consistency);
     - Revision headroom exists (`state_revision_ < UINT64_MAX`).
     Only after all checks pass is the candidate installed in `regions_`, followed by advancing `state_revision_` by exactly 1.
  10. **Absent Region Insertion OOM Contract**: For an absent region commit, `unordered_map` insertion may allocate memory. If allocation throws `std::bad_alloc`, the revision has not yet advanced, and existing controller state remains valid per standard container exception guarantees.
  11. **Batch Commit API (`commitPreparedBatch`)**: For applying multiple regional transitions resulting from a single fused frame, `commitPreparedBatch(std::span<const HysteresisPreparedObservation>)` executes a single atomic policy transaction.
  12. **Batch Prevalidation and All-or-None Guarantee**: Before any state modification, the entire batch is strictly prevalidated:
     - Every entry must have `expected_controller_revision == state_revision_`;
     - Every entry must be committable and valid;
     - All entries must have distinct `CellKey` identifiers (`DUPLICATE_REGION` rejection if duplicate keys exist);
     - Every entry's prior-state presence and semantic field equality must match current storage;
     - Revision headroom must exist.
     If ANY entry fails validation, the entire batch is rejected with `committed_count = 0` and zero controller state is modified.
  13. **Batch Canonical Ordering and Single Revision Advance**: On prevalidation success, batch candidates are installed in canonical Morton `CellKey` order. `state_revision_` increments exactly ONCE for the entire non-empty batch.
  14. **Batch Subset Use Case**: In future pipeline execution, a frame may preview regions $\{A, B, C, D\}$. If spatial refinement splits $B$ and $D$ into internal nodes before fusion, only surviving active leaves $\{A, C\}$ are submitted to `commitPreparedBatch({A, C})`. Because all surviving prepared observations share the original revision $R$ and no intermediate hysteresis mutations occurred, the subset batch commits cleanly against revision $R$.
  15. **Batch vs Sequential Revision Semantics**: $N$ sequential legacy `observe` calls advance `state_revision_` $N$ times. One staged batch containing $N$ region transitions advances `state_revision_` ONCE. This difference is intentional: a batch represents a single atomic policy transaction for a fused frame. The per-region semantic states resulting from both methods are identical.
  16. **Empty Batch Contract**: An empty batch (`commitPreparedBatch({})`) is a successful no-op (`APPLIED`, `committed_count = 0`) that leaves `state_revision_` and container state completely unchanged.
  17. **Batch Out-of-Memory (OOM) Contract and Future Pipeline Consequence**: New region insertions in a batch allocate hashtable nodes. While all logical conditions are prevalidated, `std::bad_alloc` during multi-entry insertion may occur after an earlier candidate has been inserted. Therefore, batch atomicity is NOT guaranteed under OOM. If `std::bad_alloc` is caught during staged hysteresis batch installation, the future pipeline coordinator must treat controller state as potentially partially applied and enter a fatal internal policy-error path; it must NOT silently continue normal adaptive operation.
  18. **Zero Side-Effects on Discard**: A prepared observation or batch can be discarded without calling commit or cancel. Uncommitted candidates have zero side effects on controller state, read-only lookups, snapshots, or acknowledgement eligibility.
  19. **Acknowledgement and Erase Contracts Unchanged**: `acknowledgeCoarsenResult` and `eraseRegion` preserve their frozen Milestone 9B1 semantics:
     - `APPLIED` erases the region state and increments `state_revision_` by 1;
     - `BLOCKED_HOLD_FINE` and `CANCELLED` clear the pending flag, reset stable coarsen count to 0, preserve frame ordering history (`has_last_frame`, `last_frame_seq`), and increment `state_revision_` by 1;
     - Unsuccessful acks/erases leave `state_revision_` unchanged;
     - At revision exhaustion, all acks and erases fail closed with zero mutation.
  20. **Critical Future Success-Path Ordering**: Because `state_revision_` is a global transaction token, any mutation invalidates the staged batch. In the future adaptive pipeline, after sensor frame fusion succeeds:
     - **FIRST**: Commit the surviving `HysteresisPreparedObservation` batch against revision $R$ $\rightarrow$ advances controller to $R+1$;
     - **ONLY AFTER THAT**: Call `eraseRegion` on hysteresis states belonging to former leaves that became `INTERNAL` during structural refinement;
     - **THEN**: Post-fusion coarsening acknowledgements, cancellations, or ordinary merge cleanups may mutate the controller.
     DO NOT call `eraseRegion` or `acknowledgeCoarsenResult` BEFORE staged batch commit, because any prior mutation advances `state_revision_` and causes the batch commit to fail with `STALE_CONTROLLER_REVISION`.
  21. **Future Success-Path Ordering Example**:
     - Pre-frame controller tracks active leaves $A, B, C$ at revision $R$.
     - Coordinator previews $A, B, C$, all recording expected revision $R$.
     - Spatial refinement splits leaf $B$ into an `INTERNAL` node with 4 new child leaves.
     - Sensor fusion into `PersistentAdaptiveMap` succeeds.
     - *Correct Execution*:
       1. `commitPreparedBatch({A, C})` against revision $R$ $\rightarrow$ succeeds, advances revision to $R+1$.
       2. `eraseRegion(B)` $\rightarrow$ purges obsolete prior leaf state of $B$, advances revision to $R+2$.
     - *Incorrect Execution*:
       1. `eraseRegion(B)` $\rightarrow$ advances revision to $R+1$.
       2. `commitPreparedBatch({A, C})` $\rightarrow$ REJECTED with `STALE_CONTROLLER_REVISION`.
  22. **Future Fusion-Failure Ordering**: If structural refinement converted $B$ to `INTERNAL` but downstream sensor fusion fails:
     - Discard ALL prepared observations immediately (do NOT commit $A$ or $C$);
     - Call `eraseRegion(B)` because $B$ is no longer an active leaf;
     - Controller temporal state for surviving unchanged regions remains completely uncorrupted.
  23. **No Premature Coarsening Acknowledgements**: The future pipeline must never call `acknowledgeCoarsenResult` before committing the staged batch generated from the same revision. Post-fusion coarsening ACKs belong strictly AFTER batch commit.
  24. **Decoupling and Zero Upstream Dependencies**: `ResolutionHysteresisController` contains zero dependencies on `PersistentAdaptiveMap`, `AdaptiveHierarchy`, `RecentObservationCache`, `FrameEvidence`, subtree collapse, replay, or the pipeline coordinator.
  25. **Automatic Adaptation Engine Deferred**: Milestone 9B3E implements only the hysteresis transaction staging foundation. Automatic adaptation, recursive spatial refinement, sibling freshness, processing generation, and pipeline coordination loops remain deferred.
- **Reason**: Provides transaction isolation for temporal resolution hysteresis, allowing multi-phase mapping pipelines to inspect decision outcomes during planning while committing temporal state strictly upon successful frame fusion.
- **Consequences**: Autonomous mapping pipelines can safely stage incoming frames, plan multi-level refinement cascades, and guarantee zero policy drift if downstream sensor fusion fails.

### ADR-021: Persistent Adaptive Frame Pipeline V1 (Ingestion, Localized Refinement, Replay, Fusion & Transactional Auxiliary-State Commit)

- **Status**: Decided and Implemented in Milestone 9B3F1.
- **Context**:
  Following the completion of the Milestone 9B3D architecture audit and Milestone 9B3E staged hysteresis transaction preview/commit foundation, Milestone 9B3F1 implements `PersistentAdaptiveFramePipeline`. This is the first real autonomous persistent mapping pipeline in AVR-Map, serving as the single-writer coordinator connecting five frozen components:
  1. `FrameEvidence` (M5 evidence aggregation & signal extraction);
  2. `ResolutionController` (M4 pure mathematical resolution decision engine);
  3. `ResolutionHysteresisController` (M9B1/M9B3E staged temporal resolution hysteresis);
  4. `PersistentAdaptiveMap` (M7 payload store, M8A split/replay store, M9B2A coarsening shadows);
  5. `RecentObservationCache` (M8A/M9B3B bounded raw observation window & staged commit).
- **Decisions**:
  1. **Single-Writer Coordinator Responsibility**: `PersistentAdaptiveFramePipeline` acts exclusively as the single-writer coordinator. It does NOT duplicate mathematical or spatial algorithms from `FrameEvidence`, `ResolutionController`, `ResolutionHysteresisController`, `PersistentAdaptiveMap`, `RecentObservationCache`, or M8B replay. The coordinator owns strictly ordering, phase sequencing, and transactional fault boundaries.
  2. **Geometry-Only V1 Contract**: In M9B3F1, sensor frames ingest 3D LiDAR point clouds (`PointXYZ`) and sensor origin (`WorldPoint2D`). Semantic relevance defaults to `NONE`, dynamic relevance to `NONE`, and uncertainty to `std::nullopt`. Semantic and dynamic pipelines will hook into this geometry-only foundation in future milestones.
  3. **Strict Top-Level Preflight Ordering**: All known fatal/rejection conditions are evaluated before building evidence, allocating roots, or mutating topology:
     - Check 1: `fatal_error_latched_` $\rightarrow$ returns `PIPELINE_FATAL_LATCHED` immediately with zero mutation.
     - Check 2: `processing_generation_ == UINT64_MAX` $\rightarrow$ returns `GENERATION_EXHAUSTED` with zero mutation.
     - Check 3: `hysteresis_.stateRevision() == UINT64_MAX` $\rightarrow$ returns `POLICY_REVISION_EXHAUSTED` with zero mutation (detecting terminal hysteresis freeze before prospective root creation, splitting, replay, or fusion).
     - Check 4: `frame_seq <= persistent_map_.lastFusedFrameSeq()` $\rightarrow$ returns `FRAME_REJECTED_SEQUENCE` with zero mutation.
  4. **Mixed Invalid Raw Measurement Semantics**: Preserves underlying component contracts without artificial strengthening:
     - Non-finite continuous `sensor_origin` (NaN or Inf) fails global frame validation and triggers whole-frame rejection (`FRAME_REJECTED_INPUT`) with zero persistent mutation.
     - Individual invalid LiDAR point returns (NaN, Inf, or coordinate values out of bounds) are skipped/unindexed in `FrameEvidence` and retained in raw arrival order in `RecentObservationCache` as unindexed points (`non_finite_unindexed` and `out_of_range_unindexed`). They do NOT reject the frame or produce corrupt spatial evidence.
  5. **Staged Raw-Cache Preparation (Non-Mutating)**: After `FrameEvidence::buildFrame` validates points and origin, `RecentObservationCache::prepareFrame` stages the raw point buffer. This preparation is strictly non-mutating: the staged frame is NOT present in the cache window, and the older cache window remains intact for all historical replay triggered during the current frame cycle.
  6. **Pre-Existing Leaf & Prospective-Root Discovery**: Active base cells from `FrameEvidence::baseCellSnapshot()` are mapped to either an active leaf via `findLeafContainingBase` or an unallocated L3 root via `AdaptiveHierarchy::rootKeyForBase`. Touched regions are deduplicated and sorted into canonical Morton `CellKey` order (`pre_existing_touched_leaves` and `prospective_l3_roots`).
  7. **One Decision per Touched Pre-Existing Leaf**: Each touched pre-existing leaf evaluates resolution signals and M4 decisions exactly once. If signal building or controller evaluation fails unexpectedly, the frame aborts with `FRAME_REJECTED_DECISION` before any hierarchy mutation.
  8. **Staged M9B1 Hysteresis Preview**: Each touched pre-existing leaf previews its decision via `ResolutionHysteresisController::previewObservation`. Hysteresis state is NOT mutated during preview. If any preview returns `INVALID_OBSERVATION` or a non-committable state, the frame aborts with `FRAME_REJECTED_POLICY` before structural mutation.
  9. **Prospective Roots Bypass M9B1 Initially**: Unallocated L3 roots did not exist prior to frame $F$. They have no prior hysteresis tracking and bypass M9B1 preview. Their initial safety refinement is driven directly by M4. Surviving newly created leaves start with fresh hysteresis state (`ABSENT`).
  10. **Preview Action Classification**: Pre-existing leaves classify into `REFINE_NOW` (eligible for immediate recursive refinement), `HOLD_CURRENT` (no structural mutation), `COARSEN_PENDING` (recorded, deferred), and `COARSEN_READY` (recorded, deferred). M9B3F1 intentionally does NOT auto-merge coarsening candidates.
  11. **Structurally Removed Preview Tracking**: A vector `structurally_removed_preexisting_leaves` tracks pre-existing leaves converted to `INTERNAL` nodes during spatial refinement. These prepared observations are excluded from post-fusion batch commit, and their obsolete prior hysteresis states are erased after batch commit.
  12. **Prospective Root Creation Counts as Structural Mutation**: Allocating an L3 root via `ensureRootForBase` creates persistent hierarchy topology (`structure_changed = true`) even if M4 requests L3 and no split occurs. If downstream fusion fails, this structural change path is preserved under conservative structural commit (`FUSION_FAILED_AFTER_STRUCTURAL_CHANGE`).
  13. **Recursive Localized Evidence-Bearing Child Refinement**: Refinement helper `refineRegionFromEvidence` operates recursively up to L0. It splits only one level at a time and inspects the 4 children in canonical Morton order. Children with zero observation count remain coarse coverage leaves and are NEVER refined (no-evidence child gate).
  14. **Target Bound & Child Early Stop**: Authorized target bounds both directions:
      - `effective_child_target = std::max(child_decision.requested_level, authorized_target)`.
      - If a child requests finer resolution than authorized target, it is bounded by the authorized target.
      - If a child requests coarser resolution than authorized target, it stops early at that coarser level.
      - Prospective roots pass their root-level M4 requested level as the initial authorized target, bounding all descendant cascades.
  15. **No Uniform Full-Subtree Refinement & No Arbitrary Single-Base Path**: The pipeline strictly prohibits uniform 64-leaf expansions and avoids calling `refineToLevel(base_coord, target_level)`. All automatic adaptation occurs through localized evidence-bearing splits.
  16. **Direct Historical Replay Before Deeper Split**: For pre-existing payload/history-bearing parents, after `splitLeaf` and BEFORE splitting any newly created child further, `replayDirectChildren(parent, replay_cache_)` is called while all 4 direct children are still leaves, maximizing exact historical redistribution across the old cache window.
  17. **Replay Completion vs Replay Insufficient**:
      - `APPLIED_COMPLETE` or `ALREADY_COMPLETE` marks children complete and records `REPLAY_COMPLETE`.
      - `CACHE_HISTORY_INSUFFICIENT` or `PARENT_CACHE_INCOMPLETE` records `REPLAY_PENDING` and continues safety-driven refinement. Descendants retain `REPLAY_REQUIRED` without fake completion, and ancestor complete shadows guarantee future coarse reversibility.
      - `NO_RETIRED_PARENT_CACHE` on unobserved intermediate descendants is treated as a clean no-op, not internal corruption.
  18. **Replay Corruption Fail-Close**: Replay statuses indicating payload mismatch, child reconstruction mismatch, invalid temporal state, or invariant corruption return `REPLAY_INTERNAL_ERROR`, latch fatal error state, clean up obsolete hysteresis states for already-split leaves, and halt the frame immediately with zero cache commit and zero generation advance.
  19. **Child Decision Diagnostics & Explainability**: To ensure explainability in AVR Console, every evaluated child in `refineRegionFromEvidence` records a `PipelineDecisionRecord` with `existed_pre_frame = false`, `used_hysteresis = false`, and `hysteresis_action = INVALID_OBSERVATION` sentinel.
  20. **Final-Topology Persistent Map Fusion**: Persistent sensor fusion (`fuseFrameEvidence`) executes ONLY AFTER all structural refinement and direct replay for frame $F$ completes. Points fuse directly into final leaf nodes; no intermediate coarse fusion occurs. Neutral fusion reason bits ensure structural reasons from one region do not corrupt unrelated payload cells.
  21. **Conservative Structural Commit on Fusion Failure**: If persistent map fusion fails after structural mutations have occurred (root creation, splits, or replays), completed structural changes are NOT rolled back. Prepared hysteresis batch and staged raw cache are discarded, processing generation does not advance, but obsolete hysteresis states for structurally removed leaves are erased via `eraseRegion` (which may advance controller revision $R \rightarrow R+k$ for cleanup). Status `FUSION_FAILED_AFTER_STRUCTURAL_CHANGE` is reported and map invariants are verified.
  22. **Processing Generation Exact Contract**: `processing_generation` strictly counts the number of sensor frames successfully fused into `PersistentAdaptiveMap`. It increments by exactly 1 immediately upon successful `fuseFrameEvidence`. Subsequent auxiliary policy or cache commit failures do not decrement or roll back `processing_generation`.
  23. **Surviving Hysteresis Batch & Batch-Before-Erase Contract**: Following successful map fusion, prepared observations for regions that are STILL active leaves are collected into `surviving_batch`. `commitPreparedBatch(surviving_batch)` executes first. ONLY AFTER batch commit succeeds are old hysteresis states erased for leaves that became `INTERNAL`.
  24. **Policy Commit std::bad_alloc OOM Path**: If `commitPreparedBatch` throws `std::bad_alloc` post-fusion, the failure is fatal: `processing_generation` remains incremented (map is fused), `fatal_error_latched_` is set to `true`, raw cache is NOT committed, and status `POLICY_COMMIT_FATAL` is returned.
  25. **Cache Commit Last & Non-Fatal Allocation Degradation**: `commitPreparedFrame` commits the staged raw point cloud into `RecentObservationCache` only after map fusion and policy commit complete. If cache commit fails due to `std::bad_alloc`, it is non-fatal: `fatal_error_latched_` remains `false`, the frame is fused, policy is committed, and status `SUCCESS_CACHE_DEGRADED` is returned. Logical cache commit failure latches fatal error and returns `CACHE_COMMIT_INTERNAL_ERROR`.
  26. **Deterministic Status Precedence**:
      - For successful/degraded outcomes:
        1. If cache degraded: `SUCCESS_CACHE_DEGRADED`
        2. Else if replay pending: `SUCCESS_REPLAY_PENDING`
        3. Else: `SUCCESS`
      - For failure outcomes: the first causally terminal failure wins.
      - Final cross-subsystem invariant failure discovered during post-processing overrides any otherwise-success status with `INVARIANT_FAILURE` and latches fatal error.
  27. **Fatal-State Latch Terminality**: The pipeline latches `fatal_error_latched_ = true` on invariant validation failure, replay corruption/internal error, policy commit desynchronization/OOM, or unexpected logical cache commit failure. The latch is strictly terminal for the pipeline instance (no partial reset); caller must reconstruct/create a new pipeline instance.
  28. **FrameEvidence Ephemeral Scratch Accessor**: The `frameEvidence()` accessor returns the last attempted successfully-built `FrameEvidence` scratch state, which is ephemeral and does not represent persistent map state if the frame was rejected or uncommitted.
  29. **Deferred to Milestone 9B3F2**: Automatic sibling coarsening (`mergeDirectChildren`), sibling freshness windows, automatic subtree-collapse recovery (`collapseSubtreeFromCompleteShadow`), latest ready generation tracking, and map revision remain strictly deferred to Milestone 9B3F2.
- **Reason**: Implements the complete autonomous frame adaptation loop while upholding strict transaction isolation, preventing catastrophic policy drift, and ensuring safe localized spatial refinement.
- **Consequences**: Provides robust, high-performance LiDAR frame processing that scales dynamically from coarse coverage to fine localized representation while guaranteeing bit-for-bit repeatability and strict memory safety.

---

## ADR-022: Post-Fusion Automatic Sibling Coarsening, Sibling Freshness Windows, and Complete-Shadow Subtree Recovery Pipeline

- **Status**: Accepted
- **Context**:
  Milestone 9B3F1 implemented the first persistent adaptive frame pipeline (`PersistentAdaptiveFramePipeline`), integrating ingestion, localized refinement, direct replay reconstruction, payload fusion, staged hysteresis, and bounded raw caching. However, automatic coarsening was intentionally deferred to Milestone 9B3F2 to maintain strict transaction isolation and prevent fine topology destruction without an au  Milestone 9B3F2 implements the complete post-fusion coarsening half of the autonomous loop:
  1. Automatic sibling direct merge (`mergeDirectChildren`);
  2. Bounded sibling readiness freshness windows ($W = \text{coarsen\_group\_max\_age\_generations}$ for stale expiration only);
  3. Automatic complete-shadow subtree recovery collapse (`collapseSubtreeFromCompleteShadow`);
  4. Multi-candidate conflict resolution and non-overlapping execution;
  5. Policy metadata lifecycle and cross-frame state synchronization.
- **Decisions**:
  1. **Resolution Level Constants**: Dyadic hierarchy levels strictly define physical metric resolutions: $L0 = 0.05\,\text{m}$ (5 cm), $L1 = 0.10\,\text{m}$ (10 cm), $L2 = 0.20\,\text{m}$ (20 cm), and $L3 = 0.40\,\text{m}$ (40 cm).
  2. **Post-Fusion Phase Placement (Phase 7)**: Automatic coarsening executes strictly in Phase 7 after persistent map frame evidence fusion (Phase 4), staged hysteresis policy batch commit (Phase 5), `LeafPolicyMetadata` lifecycle updates, stale readiness expiration, and staged raw cache commit (Phase 6). Coarsening operates strictly on the freshly fused, post-refinement persistent topology.
  3. **Cache Degraded and Empty Frame Coarsening Invariants**:
     - When raw cache commit experiences allocation degradation (`SUCCESS_CACHE_DEGRADED`), candidate discovery and coarsening execution are skipped to preserve conservative coarse cache reversibility. Prior to skipping, current metadata is updated and stale readiness expiration executes.
     - Valid empty frames (0 accepted points) execute stale readiness expiration and metadata invariant validation, but bypass candidate discovery and execution.
  4. **Model A Evidence Invariant & Candidate Seeding**:
     - **Current-Generation Evidence Mandate**: Automatic merge and recovery require CURRENT-GENERATION observational evidence (`latest_decision_generation == processing_generation` and `latest_ready_generation == processing_generation`) from EVERY participating leaf. The freshness window $W$ is expiry-only, never authorization to merge unobserved leaves.
     - **Direct Merge Candidates**: Seeded from parent keys of active leaves in `surviving_batch` that reached `COARSEN_READY` or `COARSEN_READY_PENDING_ACK`. Candidate parent $P$ must be `INTERNAL`, must not conflict with any region split during frame $F$, and all 4 direct children must be active `LEAF` nodes, must not require replay, must exist in `leaf_policy_metadata_`, must have current-generation evidence, must have hysteresis `pending_target_level == P.level` exactly, and `latest_requested_level >= P.level`.
     - **Complete-Shadow Recovery Candidates**: Seeded from ancestors of frontier leaves in `surviving_batch` with `COARSEN_READY` or `COARSEN_READY_PENDING_ACK` and a real provenance gap (`requiresReplay == true` on a descendant leaf or `base_state == INCOMPLETE` on an intermediate ancestor). Deep topology alone is not a gap. Candidate ancestor $A$ must be `INTERNAL`, must contain a verified provenance gap, must not conflict with split regions, and all descendant frontier leaves must have current-generation evidence and target level $\ge A.\text{level}$.
  5. **Sibling Readiness Freshness Window ($W$)**:
     - Configurable via `coarsen_group_max_age_generations` (default: 2; $W=0$ supported).
     - Age defined as $\text{current\_generation} - \text{latest\_ready\_generation}$.
     - When an active leaf remains ready without merging for $\text{age} > W$, stale expiration acknowledges the controller with `CoarsenAttemptOutcome::CANCELLED` and resets `latest_ready_generation = 0`, incrementing `stale_ready_cancelled`. Expiration executes before candidate discovery.
  6. **Restored Parent & Same-Frame Refinement Clean State**:
     - Restored direct-merge parents and restored recovery ancestors start with NO hysteresis state and NO `LeafPolicyMetadata` until observed in a subsequent frame.
     - Same-frame newly refined leaves start with NO hysteresis state and NO `LeafPolicyMetadata` until observed in a subsequent frame.
  7. **Conflict Resolution & Non-Overlapping Deterministic Ordering**:
     - Candidates are deduplicated and canonically sorted: finer candidates first (`level` ascending: $L1 \rightarrow L2 \rightarrow L3$), direct merge before recovery (`type` ascending), and tie-broken by Morton `CellKey` ascending.
     - Execution is strictly non-overlapping: committed candidate regions mark their spatial subtrees, precluding conflicting ancestor/descendant candidate executions in the same frame.
  8. **Safe Block vs Fatal Error Classification**:
     - **Direct Merge**: `MergeStatus::APPLIED` installs coarsened parent payload, erases children from payload store and hierarchy, cleans child hysteresis states, and records `DIRECT_MERGE`. Safe blocks (`CHILD_NOT_LEAF`, `REPLAY_INCOMPLETE`, `CACHE_HISTORY_INSUFFICIENT`) safely increment `direct_merges_blocked` without fatal latching. Structural corruption, conservation violations, reconstruction mismatches, temporal metadata errors, or payload inconsistencies trigger fatal error latching with `COARSENING_INTERNAL_ERROR`.
     - **Subtree Recovery Collapse**: `SubtreeCollapseStatus::APPLIED_GAP_CLOSED` restores ancestor leaf, removes fine descendant topology, erases descendant hysteresis states, and records `SUBTREE_RECOVERY_COLLAPSE`. Safe blocks increment `recovery_blocked`. Non-success collapse statuses trigger fatal error latching with `COARSENING_INTERNAL_ERROR`.
  9. **Metadata OOM Contract**: Allocation failure (`std::bad_alloc`) during post-fusion metadata insertion is FATAL: it latches `fatal_error_latched_ = true` and returns `COARSENING_INTERNAL_ERROR`. Metadata is never silently discarded to continue automatic adaptation.
  10. **LeafPolicyMetadata Invariants & Snapshot**:
      - `leafPolicyMetadataSnapshot()` returns a deterministic, canonically sorted vector of key-value pairs (`std::vector<std::pair<CellKey, LeafPolicyMetadata>>`).
      - Validated before and after coarsening:
        1. Key must be an active `LEAF` in hierarchy;
        2. `latest_decision_generation <= processing_generation`;
        3. `latest_ready_generation <= processing_generation`;
        4. `latest_requested_level` must be a valid level ($L0 \le l \le L3$);
        5. If `latest_ready_generation > 0`, region state must exist in hysteresis with `coarsen_action_pending == true`.
  11. **Zero Out-of-Scope Features**: Semantics, dynamic obstacles, traversability scoring, ROS nodes, and frontend visualizations remain deferred.
- **Reason**: Completes the autonomous, closed-loop multi-resolution persistent frame pipeline while guaranteeing mathematical exactness, non-overlapping transaction safety, and provable complete-shadow reversibility.
- **Consequences**: Downstream autonomous navigation and localization modules can ingest continuous LiDAR streams indefinitely without unbounded fine topology memory growth or unverified topology destruction.

---

## ADR-023: Replay-Safe Static-Fusion Routing Provenance

- **Status**: Accepted
- **Context**:
  Milestone 10A completed the architecture audit for semantic perception and dynamic obstacle integration. A critical architectural invariant identified in that audit is that points classified as dynamic must not enter the persistent static map. Furthermore, because fine map regions may undergo adaptive structural refinement (`splitLeaf`) and historical cache replay (`replayDirectChildren`) long after original frame ingestion, any point excluded from static map fusion during frame ingestion must remain excluded during all future historical replays.
  Milestone 10B1 implements this exact routing provenance primitive in isolation, without coupling to semantic inference, machine learning models, or dynamic obstacle tracking pipelines.
- **Decisions**:
  1. **Strict Separation of Geometry Validity and Static-Fusion Routing**:
     - Geometry validity remains exclusively determined by finite coordinates (`!isnan`, `!isinf`), bounding box range (`kMinValidWorldCoord` to `kMaxValidWorldCoord`), and spatial grid bounds.
     - Static-fusion routing is orthogonal to geometry validity: it answers strictly whether a geometrically valid observation was authorized to enter the persistent static map.
     - A routing flag can never make invalid geometry replayable. Points with NaN, Inf, or out-of-range coordinates remain unindexed and unrepayable regardless of routing value.
     - Refinement from M10A audit: `INVALID` geometry is NOT an enum state of `PointRouting`. Geometry validation owns invalid/unindexed status independently.
  2. **PointRouting Enum Definition**:
     ```cpp
     enum class PointRouting : uint8_t {
       STATIC_FUSABLE = 0,
       DYNAMIC_EXCLUDED = 1,
     };
     ```
     Only values `0` and `1` are valid. Any other value fails validation immediately during frame preparation with zero cache mutation.
  3. **Legacy API Preservation and Default Routing**:
     - Existing APIs (`prepareFrame`, `pushFrame`) that omit routing parameters default to `PointRouting::STATIC_FUSABLE` for all raw points.
     - Geometrically invalid points still remain unindexed.
     - Frozen M8A/M9B3 callers and test suites retain 100% behavioral equivalence.
  4. **Routed Frame Preparation API & Strict Validation**:
     - New overloads accept `std::span<const PointRouting> routing`.
     - Invariant: `routing.size() == raw_points.size()`. Any mismatch causes immediate prepare failure with zero cache mutation.
     - Every element in `routing` is verified using `isValidPointRouting`. Any invalid enum cast fails preparation with zero mutation.
  5. **Routing Provenance Ownership and Storage**:
     - `PreparedCachedObservationFrame` and `CachedObservationFrame` own routing provenance by value (`std::vector<PointRouting>`).
     - Caller may mutate or destroy original routing inputs after `prepareFrame` without affecting the prepared or committed frame.
     - Raw points and routing vectors maintain identical indexing and arrival order: `routing_[i]` describes `raw_points_[i]`.
  6. **Raw Point Retention**:
     - `DYNAMIC_EXCLUDED` points are retained in `raw_points_` to preserve raw frame fidelity for future perception and diagnostics.
  7. **Spatial Indexing Policy**:
     - Geometrically valid `DYNAMIC_EXCLUDED` points remain indexed in existing multi-resolution spatial buckets (`CellKey` to `PointIndex`). The spatial bucket index remains an authoritative geometry index.
     - Frozen `collectObservationIndices` semantics are strictly preserved for general spatial retrieval and diagnostics.
  8. **Explicit Static-Fusion Collection API**:
     - Added `collectStaticFusableObservationIndices(region, output)` on `CachedObservationFrame` and `RecentObservationCache`.
     - Returns only indices that are both geometrically valid/indexed and have `PointRouting::STATIC_FUSABLE`.
  9. **Persistent Replay Uses Static-Fusion Collection Exclusively**:
     - `PersistentAdaptiveMap::replayDirectChildren` and `PersistentAdaptiveMap::mergeDirectChildren` query `collectStaticFusableObservationIndices`.
     - `DYNAMIC_EXCLUDED` observations can never enter `ElevationStats`, `AdaptiveCellPayload`, coarsening shadows, or reconstructed child payloads.
  10. **Immutable Provenance at Ingestion**:
      - Once committed to `RecentObservationCache`, routing provenance is immutable. No API supports in-place reclassification or rewriting of historical frames.
  11. **Temporal Payload Preservation**:
      - Frames containing only `DYNAMIC_EXCLUDED` points for a parent cell return empty static indices and are bypassed during replay, preserving `first_seen_frame`, `last_seen_frame`, and `observed_frame_count` invariants.
  12. **Bounded Cache and Eviction Limitations Preserved**:
      - Routing provenance shares the bounded FIFO lifetime of the raw observation cache. If historical frames have been evicted, replay returns `CACHE_HISTORY_INSUFFICIENT`, and ancestor complete shadows remain the authoritative recovery mechanism.
  13. **Zero Semantic Code / Geometry-Only Pipeline**:
      - `PersistentAdaptiveFramePipeline` continues operating in geometry-only mode using default static routing. Semantic providers, PointNet++, clustering, tracking, and dynamic sinks remain deferred to M10B2+.
- **Reason**: Guarantees that dynamic point exclusion decisions are preserved immutably across all future multi-resolution map adaptations, resolving the dynamic replay contamination hazard at the root provenance level.
- **Consequences**: Enables safe integration of upstream dynamic obstacle filters while ensuring existing frozen geometry pipelines, replay reconstruction, and shadow coarsening remain 100% bitwise and behaviorally identical.

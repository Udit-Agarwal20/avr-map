# Offline AVR benchmark harness (M11A)

IMPLEMENTED NOW: a standalone C++20 runner, strict labeled-sequence loader, replay
semantic provider, three resolution strategies, OS RSS/latency telemetry and a
fresh-process comparison tool. No ROS2, network, inference model or frontend is
needed at runtime. The build uses the repository's existing cached/FetchContent
GoogleTest when tests are enabled; Python 3 is required for integration tests and
the orchestration tool. Only macOS and Linux RSS backends are supported.

TEST FIXTURE ONLY: the bundled 20-frame sequence is hand-authored synthetic data.
Every run is marked **HARNESS VALIDATION FIXTURE ONLY / NOT REAL PERFORMANCE DATA**.
There is no real recorded dataset in this repository. No SIH memory-savings,
accuracy or throughput claim follows from the fixture timings.

## Run

From the repository root:

```sh
./scripts/build_cpp.sh release
python3 tools/benchmark_compare.py \
  --runner backend/cpp/build/release/avr_benchmark/avr_benchmark_runner \
  --sequence backend/cpp/avr_benchmark/tests/fixtures/validation \
  --output results/validation-001 --warmup-frames 3
```

The comparison tool launches sequential, fresh subprocesses, one per policy. All
artifacts and subprocess logs are retained. Reusing an output directory is rejected.
Use a new directory for each repeat; preserve unsuccessful runs too. `--verify-existing`
rechecks artifacts without rerunning and does not independently prove process isolation.
The runner also accepts `--mode uniform-l0-observed|distance-only|full-avr`,
`--sequence DIRECTORY`, `--output NEW_DIRECTORY`, and optional `--warmup-frames N`.
N defaults to zero and must leave at least one measured frame. No frames are hidden.

Reconfigure/rebuild with the canonical script after changing source or committing:
git commit and dirty state are **build provenance captured at CMake configuration**,
not a claim about the checkout when an old binary is later executed. The manifest
records compiler/version, build type, sanitizer flag, OS and CPU architecture;
it omits hostname, username and absolute dataset paths.

## Experimental control

| Mode | Resolution selection | Evidence, routing and map operations |
|---|---|---|
| UNIFORM_L0_OBSERVED | L0 for regions with current observed support | Common pipeline |
| DISTANCE_ONLY | Planar XY distance baseline only | Common pipeline |
| FULL_AVR | Existing distance, semantic/dynamic caps and evidence-gated geometry/uncertainty controller | Common pipeline |

Uniform is labeled **Uniform 5 cm — observed support**. Required dyadic siblings
are created by normal splits, but empty sibling subtrees are not recursively
expanded. This is not a dense 5 cm grid across the bounding rectangle. Regions
without current evidence do not gain invented observations. Normal hysteresis,
replay and coarsening remain active in every policy. Uniform's mode identifies the
L0 override; canonical reason bits retain their meanings, with no new reason bit.

All modes use the same provider and independently thresholded motion evidence.
Distance-only ignores semantic/dynamic signals only for selecting resolution;
confirmed moving returns still never fuse into static terrain. The default strategy
is FULL_AVR; existing callers retain their previous behavior. Existing status codes
0–21, eight semantic classes, reason bits, CellKey and hierarchy levels are unchanged.

The persistent pipeline currently supplies **no uncertainty observation**. FULL_AVR
uses its implemented signals as-is; no artificial uncertainty or future tracking is
added. Clustering is frame-local and runs after processFrame, with identical defaults.

The runner validates actual cached raw positions/order and routing against input
labels on every frame, checks accepted/static/dynamic counts and static fusion
increments, and emits per-point routing SHA-256 fingerprints (including source
ordinal and XYZ). The orchestrator compares hashes, per-frame identities/counts,
sequence hash, configurations and build provenance. Any discrepancy or invalid
frame makes the comparison INVALID and returns nonzero. Cache-degraded results
cannot establish routing identity and are invalid benchmark runs. Replay-pending
success is retained and named explicitly. Failures stop replay immediately; available
attempted-frame rows and an invalid summary are preserved. Input preflight failure
exits before creating a run directory.

## Native sequence schema v1

A directory contains `sequence.avr`, a frame index CSV and point CSVs. See the tiny
fixture for complete examples. Files are UTF-8/ASCII with LF newlines; CSV fields
are unquoted, with exact headers, no comments or blank records. Paths are relative
to the sequence directory; parent traversal and escaping symlinks are rejected.
Repeated point-file references are allowed. The preflight streams one frame at a
time, validates all input and stores index metadata/file hashes. Each frame is
reloaded and verified against its preflight hash during replay.

`sequence.avr` starts with `AVR_SEQUENCE_V1` then exactly these nonempty key=value fields:

- name; kind (`HARNESS_VALIDATION_FIXTURE` or `REAL_RECORDED`)
- semantic_provenance (`GROUND_TRUTH_LABELS`, `PRECOMPUTED_LABELS` or `SYNTHETIC_VALIDATION_LABELS`)
- source_description; license; pose_provenance (human-readable provenance, not machine identifiers)
- coordinate_frame (`FIXED_MAP_FRAME` only)
- bounds (`min_x,max_x,min_y,max_y`, finite nonempty half-open XY bounds in metres)
- frames (relative frame index filename)

Synthetic provenance is permitted only for fixture kind. The harness trusts real-data
provenance declarations; it cannot prove a recording's origin. Bounds validate the
input domain, not a dense allocation, rolling-map crop or clipping/filtering rule.
Core grid origin remains (0,0). The caller must first express XYZ and sensor origins
in one common fixed map frame. Z is preserved but does not enter planar policy distance.

Frame index header:

```text
frame_seq,timestamp_ns,sensor_x,sensor_y,sensor_z,points_file
```

Frame sequence and timestamps are uint64 and strictly increasing. Timestamps are
recording time in nanoseconds, not wall-clock processing time. Gaps are allowed.
Origins are finite metres. Point-file header:

```text
source_index,x,y,z,intensity,semantic_class,semantic_confidence,motion_state,motion_confidence
```

Source index is exactly the original consecutive 0-based ordinal; no sorting,
filtering, dropping or renumbering occurs. XYZ is finite, quantizable and within
the declared bounds. Intensity is retained as a finite double in original units;
the current PointXYZ mapping API does not consume it. Semantic and motion labels
are mandatory for every point in **all modes**, so missing FULL_AVR labels fail fast.
Empty point frames have a header and no records. An empty sequence is rejected.
Canonical class codes are 0 UNKNOWN, 1 GROUND, 2 ROAD, 3 VEGETATION, 4 BUILDING,
5 VEHICLE, 6 PEDESTRIAN, 7 OBSTACLE_OTHER. Motion codes are 0 UNKNOWN, 1 STATIC,
2 CANDIDATE, 3 CONFIRMED_DYNAMIC. Confidences are finite [0,1], stored using the
core's float contract. Semantic class alone is not evidence of confirmed motion.

Sequence identity: SHA-256 over exact manifest bytes, index bytes, then each point
file's bytes in frame order, each preceded by its decimal byte length and `:`.
No absolute path enters the hash. Whitespace/intensity/metadata edits change it.
SHA-256 is tested against known vectors and independently against Python hashlib.

## Artifacts and measurement definitions

Each process writes schema-version-1 `run_manifest.json`, `frames.csv`, `summary.json`.
The parent writes `comparison.json` plus each subprocess's diagnostic log. Configuration
values are current compiled defaults, explicitly emitted, and identical except for
selection mode. Timestamps and all source data participate in sequence identity.

CSV includes every attempted frame's sequence/time, status number/name, raw and
accepted points, static/dynamic counts, clusters, processFrame microseconds, four
leaf-level counts, internals, tiles, refinement requests/splits, direct/subtree merges,
generation, current RSS, invariant/routing validation and topology/routing hashes.
Topology fingerprints use sorted canonical nodes, not pointer addresses.

Latency uses steady_clock immediately around processFrame. Input I/O, provider
binding, standalone clustering, external invariant validation, topology hashing and
RSS/artifact work are outside the timer. Semantic inference replay/copies and all
pipeline-internal work are inside. Summaries contain both all-frame and post-warmup
mean, p50, p95, p99, max and sample count. Quantiles linearly interpolate q*(n-1).
Failed-frame timings are retained but the run is invalid. Deterministic reproducibility
means identical input/routing/topology/accounting, not identical timings/RSS/UTC.

Memory is **whole-process OS RSS in bytes**, including harness overhead:

- macOS current: mach task_info MACH_TASK_BASIC_INFO resident_size.
- Linux current: /proc/self/statm resident pages times sysconf(_SC_PAGESIZE).
- Peak: getrusage RUSAGE_SELF lifetime high-water (macOS bytes; Linux KiB converted).
- Baseline: after input preflight, before pipeline construction.
- Final: after replay, before pipeline destruction and JSON summary creation.
- Peak delta = peak − baseline; final delta = final − baseline (signed).

Lifetime peak includes preflight/allocator history **within this single process**;
it is not isolated map memory or a sampled maximum. Actual compiled sizeof values
and allocated structural components are emitted separately. They omit allocator
and container overhead; overlapping categories cannot be summed as RSS. No 32-byte
analytical cell model is substituted. No timing claims should use debug/sanitizer builds.

Dynamic contamination count is `null`: general recordings do not provide per-fused-
observation terrain provenance. Routing/fusion conservation is checked at runtime;
the fixture and existing core tests verify Z=3 dynamic points cannot contaminate
Z=0.1 static elevations during normal fusion, replay, splitting and coarsening.
No RMSE, tracking accuracy or hazard preservation score is invented.

## Validation and boundaries

Run `scripts/build_cpp.sh PRESET` then `scripts/test_cpp.sh PRESET` for dev, debug
(ASan+UBSan) and release. CTest discovers all existing core tests plus the harness
tests. Integration tests validate artifact schemas, fresh processes, independent
hash calculation, deterministic topology, malformed/missing inputs and deliberate
routing mismatch rejection. Unit tests exercise invariant-failure termination.

PLANNED: real recorded dataset adapter/provenance workflow (M11B), followed by an
explicit evidence review. FUTURE: production inference, temporal tracking, terrain
analysis, multi-surface representation, ROS2 and transport. No frontend files change.

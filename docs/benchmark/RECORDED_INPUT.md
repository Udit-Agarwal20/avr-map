# M11B recorded dataset boundary — AWAITING DATASET INPUT

IMPLEMENTED NOW: `tools/recorded_sequence_adapter.py`, a Python 3.9+ standard-library
adapter for explicitly labeled, timestamped, posed LiDAR CSV exports. It converts
sensor-frame points into the M11A fixed-map sequence schema. No real recording was
found in the repository. No dataset was selected, downloaded or invented. No real
performance or accuracy results exist. The M11B experiment remains pending.

This is the boundary for a future dataset-specific reader, not a claim to decode
SemanticKITTI, RELLIS-3D, ROS bags or arbitrary vendor formats. Such a reader should
produce the format below after inspecting the chosen dataset's official format,
license, calibration, ontology and motion-label meaning. The adapter does not infer
motion, produce semantics, estimate poses, synchronize sensors or deskew scans.

## Required input from the recording owner

Provide a local sequence/export path and its exact recording ID, source and license.
It must include original-order finite XYZ/intensity returns, frame sequence/timestamps,
semantic labels/confidences (ground truth or precomputed with documented source),
independent motion evidence/confidence, and explicit sensor-to-map transforms.

If motion evidence is unavailable, state that limitation and explicitly export
UNKNOWN with confidence 0; do not turn vehicle/pedestrian class membership into
confirmed motion. If reliable poses are unavailable, a fixed sensor recording may
use an explicitly justified identity transform. A moving recording without poses
cannot establish a persistent fixed-frame mapping benchmark. Do not fabricate poses.
Ground-truth and precomputed labels must be identified separately. No model weights
or runtime inference are needed to ingest existing labeled recordings.

Choose bounds that contain every input point after transformation. Out-of-bounds,
NaN/Inf or non-quantizable returns fail conversion; there is no silent clipping,
filtering or renumbering. If a dataset needs curation, define and document that
upstream protocol explicitly before making performance claims.

## Export schema v1

Source directory:

```text
recording.json
frames.jsonl
<original point CSV files>
```

`recording.json` must contain exactly these fields (replace descriptions with real
provenance; this is a schema description, not a fabricated recording):

| Field | Contract |
|---|---|
| schema_version | integer 1 |
| name | human-readable sequence name |
| kind | REAL_RECORDED; tests must use HARNESS_VALIDATION_FIXTURE |
| recording_id | exact dataset release/sequence/segment identifier |
| source_uri | source citation/location; avoid personal paths or credentials |
| license | applicable license and acquisition/use provenance |
| semantic_provenance | GROUND_TRUTH_LABELS or PRECOMPUTED_LABELS; fixture tests use SYNTHETIC_VALIDATION_LABELS |
| semantic_label_source | annotation release or model/weights/runtime provenance |
| motion_label_source | annotation/measurement method; explicitly state UNKNOWN if unavailable |
| pose_provenance | pose/calibration source and composition; whether scan deskewing was performed |
| coordinate_convention | RIGHT_HANDED_Z_UP_METRES |
| bounds | [min_x, max_x, min_y, max_y] in map metres, half-open |
| semantic_class_map | object mapping each input label string to a canonical integer code 0–7 |
| frames | relative path to frame JSONL index |

Metadata text must be nonempty, single-line text. Duplicate JSON keys and unknown
fields are errors. Native semantic codes: UNKNOWN=0, GROUND=1, ROAD=2, VEGETATION=3,
BUILDING=4, VEHICLE=5, PEDESTRIAN=6, OBSTACLE_OTHER=7. There is **no automatic unknown
fallback** for an unmapped dataset label. Mapping a known dataset label to UNKNOWN
is allowed only when explicitly entered in semantic_class_map.

Each nonempty JSONL frame record has exactly:

- frame_seq: uint64, strictly increasing; gaps allowed.
- timestamp_ns: uint64 recording time in nanoseconds, strictly increasing.
- points_file: relative point CSV path; no parent traversal or escaping symlinks.
- transform_map_from_sensor: 16 row-major numbers describing T_map_sensor.

T_map_sensor is an explicit right-handed rigid transform: the 3×3 rotation is
orthonormal with determinant +1 (absolute tolerance 1e-6), and bottom row is exactly
[0,0,0,1]. For each return, `p_map = R * p_sensor + t`; the sensor origin is `t`.
Use the actual LiDAR sensor frame. A vehicle/body pose must first be composed with
its calibrated LiDAR extrinsics. Input sensor coordinates must already follow the
declared axes/units; no undocumented axis swapping or unit conversion occurs.
One transform per scan does not correct intra-scan motion distortion.

Point CSV exact header:

```text
source_index,x,y,z,intensity,label,semantic_confidence,motion_state,motion_confidence
```

source_index equals the original consecutive 0-based ordinal; the adapter keeps this
order. XYZ/intensity are finite numbers; intensity retains its original numerical
units and values (double precision). Label is a string present in semantic_class_map.
Both confidences are explicit finite [0,1]. Motion codes follow the core:
UNKNOWN=0, STATIC=1, CANDIDATE=2, CONFIRMED_DYNAMIC=3. Semantic confidence does not
replace motion confidence. Empty scans may have only the CSV header. Empty sequences
are rejected. Native output numbers use 17 significant digits to preserve doubles.

## Convert, inspect, then run

From the repository root, using actual local paths:

```sh
python3 tools/recorded_sequence_adapter.py \
  --source datasets/recording-export \
  --output datasets/recording-avr

./scripts/build_cpp.sh release
python3 tools/benchmark_compare.py \
  --runner backend/cpp/build/release/avr_benchmark/avr_benchmark_runner \
  --sequence datasets/recording-avr \
  --output results/recording-repeat-001 --warmup-frames 0
```

These paths are command templates, not bundled data. The converter never overwrites
an output. It writes to its own adjacent temporary directory, validates every frame,
and only publishes the directory when complete. Failed temporary conversions are
removed; originals remain untouched. Inputs are held one point file at a time, with
the comparatively small frame index in memory. The conversion is outside benchmark
timing/RSS processes.

Output contains native `sequence.avr`, `frames.csv`, canonical point CSVs and
`adapter_provenance.json`. The latter preserves the complete recording declaration,
ontology map, counts and SHA-256 of exact input metadata/index/point bytes in frame
order, length-prefixed as in M11A. That source digest is embedded in the native
manifest's source description, linking source provenance to the benchmark's native
sequence hash. Local absolute input paths and machine identifiers are not added.

Review converted pose direction, label mapping, bounds, retained order and provenance
against the actual source. Run the C++ loader/three-process comparison before any
experiment. Keep each repetition in a new output directory; never discard failed or
inconvenient runs. Publish release measurements only, alongside raw rows, frame and
repeat counts, exact commit/build, compiler/platform, hashes and warmup policy.
The harness verifies declared input identity; it cannot certify that a file actually
came from a sensor or that its labels/poses are accurate.

## Gate after a real recording arrives

Run representative release sequences/repeats and audit routing, invariant validity,
allocation/refinement/coarsening, raw latency distributions and whole-process RSS.
Only report elevation RMSE, hazard preservation or contamination metrics when valid
reference information/methods support them. Current general contamination count is
unavailable, not zero. Include failures rather than cherry-picking successful runs.

Then revisit the PRD/TRD and measured needs before prioritizing production semantics,
temporal tracking, traversability, multi-surface clearance or live transport. Their
ultimate prototype requirements do not make them part of this dataset-free adapter
milestone. Frontend remains frozen; a future bridge must use stable versioned DTOs
without exposing core pointers or changing canonical enums silently.

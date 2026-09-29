# Backend completion audit (M11)

Starting branch: main. Starting HEAD: fb28b86a28118d23e7ab3f5410ab81746e44e1e2.
The backend is unchanged from cc50675; the two subsequent commits contain frontend work.
The working tree and whitespace checks were clean. Frontend is frozen throughout M11.

Source precedence: current C++ and tests, frozen frontend contracts, architecture/TRD,
PRD, then older reports. Reviewed the supplied PRD, TRD, backend architecture,
frontend specification, implementation decisions (through ADR-25), root README,
frontend completion report and the relevant C++ contracts and tests.

Conflicts and limits found:

- The directive/specifications refer to RawPoint3D. The implemented input is
  PointXYZ (double x/y/z); source indices are original span ordinals. The offline
  envelope will retain intensity, timestamp and sensor Z without changing that ABI.
- The root README's Milestone 0 status is stale. Neural inference, ROS2 and several
  architecture diagrams describe intended deployment, not implemented runtime.
- FULL_AVR's controller supports uncertainty, but the current persistent pipeline
  supplies null uncertainty. The harness must record this limitation, not synthesize it.
- Dynamic clustering is standalone, after processFrame. There are no track IDs,
  velocities, predictions, multi-surface mapping or live transport implementations.
- No real recorded data is present in datasets/. M11B must stop awaiting input.
- Frontend analytical bytes and demonstration values are not backend measurements.

Baseline canonical builds and tests completed before any source edits:

| Preset | Registered | Passed | Failed | Skipped |
|---|---:|---:|---:|---:|
| dev | 695 | 695 | 0 | 0 |
| debug (ASan + UBSan) | 695 | 695 | 0 | 0 |
| release | 695 | 695 | 0 | 0 |

Commands: scripts/build_cpp.sh PRESET; ctest --test-dir backend/cpp/build/PRESET
--show-only=json-v1; scripts/test_cpp.sh PRESET. Local logs/discovery are preserved
under ignored results/m11a/baseline/. No suppressions were added.

Plan: M11A adds only a resolution-selection switch to the existing controller and a
separate offline package. Loader/provider/routing, replay, hysteresis and coarsening
remain common to all modes. Validate three fresh processes, schemas, topology,
static/dynamic separation, OS RSS and timing boundaries, then audit and commit.
Only afterward add M11B's recorded-input adapter and documentation. Without a real
sequence, stop before model integration, tracking, terrain extensions or transport.

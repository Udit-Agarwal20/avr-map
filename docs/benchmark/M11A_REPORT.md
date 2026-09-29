# M11A milestone report

- **Milestone:** Offline benchmark harness foundation.
- **Status:** COMPLETE; acceptance gates passed before commit.
- **Previous HEAD:** fb28b86a28118d23e7ab3f5410ab81746e44e1e2 (main).
- **New HEAD:** the commit containing this report, titled `feat: add offline AVR benchmark harness`; the resolved SHA is recorded in the final handoff.
- **Files changed:** new backend/cpp/avr_benchmark package (runner, loader/provider,
  SHA-256, artifacts, platform RSS, tests and tiny fixture), tools/benchmark_compare.py;
  backend/cpp/CMakeLists.txt; resolution_policy.hpp and resolution_controller.cpp;
  README.md, ADR-26, and docs/benchmark documentation.
- **Architecture change:** a default-preserving resolution-selection switch, plus a
  separate offline package. No mapping/replay/hysteresis/coarsening rewrite. No core
  semantic, reason, routing or status enum migration. No frontend changes.

| Canonical preset | Registered | Passed | Failed | Skipped |
|---|---:|---:|---:|---:|
| dev | 723 | 723 | 0 | 0 |
| debug, ASan + UBSan | 723 | 723 | 0 | 0 |
| release | 723 | 723 | 0 | 0 |

Baseline was 695 in each configuration. All 695 pre-existing tests remain; 28 harness
checks were added. Commands: scripts/build_cpp.sh PRESET; ctest --test-dir
backend/cpp/build/PRESET --show-only=json-v1; scripts/test_cpp.sh PRESET.
Builds succeeded without compiler warnings in the final acceptance logs. Debug
sanitizers are enabled, no diagnostics/suppressions. TSan is not applicable: no
multithreading was introduced. Native Linux execution is not validated on this Mac.

- **Build:** dev/RelWithDebInfo, debug and release succeeded on Darwin arm64 / AppleClang.
- **Data-truth check:** saved dev and release fixture comparisons both VALID, same
  sequence and routing hashes across three fresh processes. 20 frames, 280 points,
  160 static-fusable and 120 dynamic-excluded in each mode. Uniform observed owners
  reach L0; the hazard patch is finer in FULL_AVR than DISTANCE_ONLY; one direct
  coarsening merge is exercised after the semantic hazard disappears. These are
  **HARNESS VALIDATION FIXTURE ONLY / NOT REAL PERFORMANCE DATA**, not SIH evidence.
- **Artifacts:** ignored results/m11a/baseline/, acceptance/, dev-fixture/ and
  release-fixture/. Generated results, logs and build products are not committed.
- **Frontend diff:** zero against HEAD and fb28b86. Whitespace check clean.
- **Known limitations:** no real recording; no supplied uncertainty signal; no
  inference model, temporal tracking, terrain classification, multi-surface layer,
  ROS2 or transport. General dynamic contamination and accuracy metrics remain
  unavailable (null), not zero. RSS includes whole-process/harness overhead and
  peak includes preflight; build provenance is a CMake configuration snapshot.
- **P0:** no known blocker within the validated harness scope.
- **P1:** recorded-data validation and target Linux execution remain required before
  performance claims; obtain labeled data with timestamp/pose/license provenance.
- **P2:** tune defaults and prioritize PRD extensions only after real evidence.
- **Next milestone:** M11B adapter boundary; absent a real dataset, stop with
  AWAITING DATASET INPUT. Do not proceed automatically to advanced features.

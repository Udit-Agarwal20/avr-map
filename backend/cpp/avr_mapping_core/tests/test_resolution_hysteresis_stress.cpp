#include <gtest/gtest.h>

#include <cstdint>
#include <unordered_set>
#include <vector>

#include "avr_mapping/resolution_decision.hpp"
#include "avr_mapping/resolution_hysteresis.hpp"
#include "avr_mapping/resolution_hysteresis_config.hpp"
#include "avr_mapping/resolution_hysteresis_result.hpp"
#include "avr_mapping/resolution_hysteresis_state.hpp"
#include "avr_spatial/cell_key.hpp"
#include "avr_spatial/coordinate.hpp"

namespace avr::mapping {
struct ResolutionHysteresisTestAccess {
  static void setRevision(ResolutionHysteresisController& ctrl, uint64_t rev) noexcept {
    ctrl.state_revision_ = rev;
  }
};
}  // namespace avr::mapping

using namespace avr::mapping;

TEST(ResolutionHysteresisStressTest, HundredThousandObservationsDeterministicStressAndRepeatability) {
  constexpr size_t NUM_OBSERVATIONS = 120000;
  constexpr size_t NUM_REGIONS = 12000;  // Exceeds 10,000 distinct CellKeys
  constexpr uint32_t THRESHOLD = 5;

  // Generate distinct keys across negative and positive coordinates and all four levels
  std::vector<avr::spatial::CellKey> test_keys;
  test_keys.reserve(NUM_REGIONS);
  for (size_t i = 0; i < NUM_REGIONS; ++i) {
    const int32_t cx = static_cast<int32_t>(i % 200) - 100;
    const int32_t cy = static_cast<int32_t>(i / 200) - 100;
    const uint8_t level = static_cast<uint8_t>(i % 4);
    test_keys.push_back(*avr::spatial::CellKey::fromCellCoord(avr::spatial::CellCoord{.x = cx, .y = cy, .level = level}));
  }

  std::unordered_set<avr::spatial::CellKey> unique_keys(test_keys.begin(), test_keys.end());
  ASSERT_GE(unique_keys.size(), 10000U);

  const auto unknown_dummy_key = *avr::spatial::CellKey::fromCellCoord(avr::spatial::CellCoord{.x = 99999, .y = 99999, .level = 0});

  // Deterministic PRNG (Linear Congruential Generator)
  uint64_t rng = 0xDEADBEEF12345678ULL;
  auto next_random = [&rng]() -> uint64_t {
    rng = rng * 6364136223846793005ULL + 1442695040888963407ULL;
    return rng;
  };

  struct StreamResults {
    std::vector<HysteresisAction> actions{};
    std::vector<bool> acks{};
  };

  const auto run_stream = [&](ResolutionHysteresisController& ctrl) -> StreamResults {
    StreamResults sr;
    sr.actions.reserve(NUM_OBSERVATIONS);
    sr.acks.reserve(NUM_OBSERVATIONS / 4);

    // Track state in test driver
    std::vector<bool> region_initialized(NUM_REGIONS, false);
    std::vector<uint64_t> region_last_frame(NUM_REGIONS, 0);

    for (size_t step = 0; step < NUM_OBSERVATIONS; ++step) {
      const size_t region_idx = next_random() % NUM_REGIONS;
      const auto& region = test_keys[region_idx];
      const uint8_t cur_level = region.level;

      const uint64_t op_type = next_random() % 100;

      // 1. Invalid ack on unknown region periodically
      if (op_type == 0) {
        bool ack_un = ctrl.acknowledgeCoarsenResult(unknown_dummy_key, CoarsenAttemptOutcome::CANCELLED);
        EXPECT_FALSE(ack_un);
        sr.acks.push_back(ack_un);
      }

      // 2. First observation frame zero test or duplicate/out-of-order test
      uint64_t frame_seq = 0;
      if (!region_initialized[region_idx]) {
        // Some region lifecycles begin explicitly at frame_seq = 0
        if ((next_random() % 3) == 0) {
          frame_seq = 0;
        } else {
          frame_seq = next_random() % 20;
        }
        region_initialized[region_idx] = true;
        region_last_frame[region_idx] = frame_seq;
      } else {
        if (op_type == 1 && region_last_frame[region_idx] == 0) {
          // Attempt duplicate frame 0
          ResolutionDecision dec{.baseline_level = avr::spatial::L3, .requested_level = cur_level};
          auto res = ctrl.observe(region, 0, dec);
          EXPECT_EQ(res.action, HysteresisAction::INVALID_OBSERVATION);
          sr.actions.push_back(res.action);
          continue;
        }

        if (op_type == 2 && region_last_frame[region_idx] > 0) {
          // Attempt duplicate non-zero frame
          ResolutionDecision dec{.baseline_level = avr::spatial::L3, .requested_level = cur_level};
          auto res = ctrl.observe(region, region_last_frame[region_idx], dec);
          EXPECT_EQ(res.action, HysteresisAction::INVALID_OBSERVATION);
          sr.actions.push_back(res.action);
          continue;
        }

        if (op_type == 3 && region_last_frame[region_idx] > 10) {
          // Attempt out-of-order frame
          ResolutionDecision dec{.baseline_level = avr::spatial::L3, .requested_level = cur_level};
          auto res = ctrl.observe(region, region_last_frame[region_idx] - 5, dec);
          EXPECT_EQ(res.action, HysteresisAction::INVALID_OBSERVATION);
          sr.actions.push_back(res.action);
          continue;
        }

        // Strictly increasing valid frame sequence
        frame_seq = region_last_frame[region_idx] + 1 + (next_random() % 10);
        region_last_frame[region_idx] = frame_seq;
      }

      // 3. Invalid ack on known non-pending region periodically
      if (op_type == 4) {
        const auto* st = ctrl.findRegionState(region);
        if (st != nullptr && !st->coarsen_action_pending) {
          bool bad_ack = ctrl.acknowledgeCoarsenResult(region, CoarsenAttemptOutcome::BLOCKED_HOLD_FINE);
          EXPECT_FALSE(bad_ack);
          sr.acks.push_back(bad_ack);
        }
      }

      // Select requested level (finer, equal, or coarser)
      uint8_t req_level = static_cast<uint8_t>(next_random() % 4);

      ResolutionReasonBits reasons{};
      if ((next_random() % 2) != 0) reasons.set(ResolutionReason::UNCERTAINTY);
      if ((next_random() % 2) != 0) reasons.set(ResolutionReason::GEOMETRY_COMPLEXITY);

      ResolutionDecision dec{
          .baseline_level = avr::spatial::L3,
          .requested_level = req_level,
          .evidence_state = EvidenceState::SUFFICIENT,
          .reasons = reasons,
      };

      auto res = ctrl.observe(region, frame_seq, dec);
      sr.actions.push_back(res.action);

      // Verify immediate invariant after observation
      if (res.action == HysteresisAction::COARSEN_READY) {
        EXPECT_EQ(res.stable_coarsen_count, THRESHOLD);
        const uint64_t ack_choice = next_random() % 3;
        if (ack_choice == 0) {
          // APPLIED
          bool ok = ctrl.acknowledgeCoarsenResult(region, CoarsenAttemptOutcome::APPLIED);
          EXPECT_TRUE(ok);
          sr.acks.push_back(ok);
          region_initialized[region_idx] = false;
          region_last_frame[region_idx] = 0;
        } else if (ack_choice == 1) {
          // BLOCKED_HOLD_FINE (frame ordering preserved!)
          bool ok = ctrl.acknowledgeCoarsenResult(region, CoarsenAttemptOutcome::BLOCKED_HOLD_FINE);
          EXPECT_TRUE(ok);
          sr.acks.push_back(ok);
        } else {
          // CANCELLED (frame ordering preserved!)
          bool ok = ctrl.acknowledgeCoarsenResult(region, CoarsenAttemptOutcome::CANCELLED);
          EXPECT_TRUE(ok);
          sr.acks.push_back(ok);
        }
      } else if (res.action == HysteresisAction::REFINE_NOW || res.action == HysteresisAction::HOLD_CURRENT) {
        EXPECT_EQ(res.stable_coarsen_count, 0U);
      } else if (res.action == HysteresisAction::COARSEN_PENDING) {
        EXPECT_LT(res.stable_coarsen_count, THRESHOLD);
      }

      // Random eraseRegion lifecycle test
      if (op_type == 50) {
        ctrl.eraseRegion(region);
        region_initialized[region_idx] = false;
        region_last_frame[region_idx] = 0;
      }
    }

    return sr;
  };

  // Run controller 1
  uint64_t initial_seed = rng;
  auto ctrl1 = *ResolutionHysteresisController::create(ResolutionHysteresisConfig{.coarsen_stable_observations = THRESHOLD});
  rng = initial_seed;
  const auto results1 = run_stream(ctrl1);

  // Invariant verification on final state of ctrl1
  const auto snap1 = ctrl1.stateSnapshot();
  for (size_t i = 0; i < snap1.size(); ++i) {
    const auto& s = snap1[i];
    EXPECT_TRUE(s.has_last_frame);
    EXPECT_LE(s.stable_coarsen_count, THRESHOLD);
    if (s.coarsen_action_pending) {
      EXPECT_EQ(s.stable_coarsen_count, THRESHOLD);
      EXPECT_EQ(s.pending_target_level, s.region.level + 1);
    } else {
      EXPECT_EQ(s.pending_target_level, 0U);
    }
    if (i + 1 < snap1.size()) {
      EXPECT_TRUE(snap1[i].region < snap1[i + 1].region);
    }
  }

  // Run controller 2 with identical seed stream
  auto ctrl2 = *ResolutionHysteresisController::create(ResolutionHysteresisConfig{.coarsen_stable_observations = THRESHOLD});
  rng = initial_seed;
  const auto results2 = run_stream(ctrl2);

  // Repeatability verification: exact bit-for-bit equivalence
  EXPECT_EQ(results1.actions.size(), NUM_OBSERVATIONS);
  EXPECT_EQ(results1.actions, results2.actions);
  EXPECT_EQ(results1.acks, results2.acks);
  EXPECT_EQ(ctrl1.trackedRegionCount(), ctrl2.trackedRegionCount());
  EXPECT_EQ(ctrl1.stateSnapshot(), ctrl2.stateSnapshot());
}

TEST(ResolutionHysteresisStressTest, HundredThousandStagedPreviewAndCommitDeterministicStressAndRepeatability) {
  constexpr size_t NUM_PREVIEWS = 120000;
  constexpr size_t NUM_REGIONS = 12000;  // >= 10,000 CellKeys
  constexpr uint32_t THRESHOLD = 5;

  std::vector<avr::spatial::CellKey> test_keys;
  test_keys.reserve(NUM_REGIONS);
  for (size_t i = 0; i < NUM_REGIONS; ++i) {
    const int32_t cx = static_cast<int32_t>(i % 200) - 100;
    const int32_t cy = static_cast<int32_t>(i / 200) - 100;
    const uint8_t level = static_cast<uint8_t>(i % 4);
    test_keys.push_back(*avr::spatial::CellKey::fromCellCoord(avr::spatial::CellCoord{.x = cx, .y = cy, .level = level}));
  }

  uint64_t rng = 0xCAFEBABEDEADBEEFULL;
  auto next_random = [&rng]() -> uint64_t {
    rng = rng * 6364136223846793005ULL + 1442695040888963407ULL;
    return rng;
  };

  struct StagedStreamResults {
    std::vector<HysteresisAction> actions{};
    std::vector<PreparedCommitStatus> commit_statuses{};
    std::vector<PreparedBatchCommitStatus> batch_statuses{};
    std::vector<bool> acks{};
    uint64_t preview_count{0};
  };

  const auto run_staged_stream = [&](ResolutionHysteresisController& ctrl) -> StagedStreamResults {
    StagedStreamResults sr;
    sr.actions.reserve(NUM_PREVIEWS);
    sr.commit_statuses.reserve(NUM_PREVIEWS);

    std::vector<bool> region_initialized(NUM_REGIONS, false);
    std::vector<uint64_t> region_last_frame(NUM_REGIONS, 0);

    for (size_t step = 0; step < NUM_PREVIEWS; ++step) {
      const size_t region_idx = next_random() % NUM_REGIONS;
      const auto& region = test_keys[region_idx];
      const uint64_t op_type = next_random() % 100;

      // Frame sequence selection
      uint64_t frame_seq = 0;
      bool is_invalid_frame = false;
      if (!region_initialized[region_idx]) {
        if ((next_random() % 3) == 0) {
          frame_seq = 0;
        } else {
          frame_seq = next_random() % 20;
        }
      } else {
        if (op_type == 1) {
          // Duplicate frame
          frame_seq = region_last_frame[region_idx];
          is_invalid_frame = true;
        } else if (op_type == 2 && region_last_frame[region_idx] > 5) {
          // Out of order frame
          frame_seq = region_last_frame[region_idx] - 3;
          is_invalid_frame = true;
        } else {
          frame_seq = region_last_frame[region_idx] + 1 + (next_random() % 10);
        }
      }

      // Level and reasons
      uint8_t req_level = static_cast<uint8_t>(next_random() % 4);
      if (op_type == 3) {
        req_level = 99;  // Invalid requested level
      }

      ResolutionReasonBits reasons{};
      if ((next_random() % 2) != 0) reasons.set(ResolutionReason::UNCERTAINTY);
      if ((next_random() % 2) != 0) reasons.set(ResolutionReason::GEOMETRY_COMPLEXITY);

      ResolutionDecision dec{
          .baseline_level = avr::spatial::L3,
          .requested_level = req_level,
          .evidence_state = EvidenceState::SUFFICIENT,
          .reasons = reasons,
      };

      // 1. PREVIEW
      const auto prep = ctrl.previewObservation(region, frame_seq, dec);
      ++sr.preview_count;
      sr.actions.push_back(prep.result.action);

      if (is_invalid_frame || req_level > 3) {
        EXPECT_EQ(prep.result.action, HysteresisAction::INVALID_OBSERVATION);
        EXPECT_FALSE(prep.isCommittable());
        const auto status = ctrl.commitPreparedObservation(prep);
        EXPECT_EQ(status, PreparedCommitStatus::NOT_COMMITTABLE);
        sr.commit_statuses.push_back(status);
        continue;
      }

      // 2. Discarded preview branch (preview without commit)
      if (op_type == 4) {
        // Discarded! Region state and revision untouched.
        continue;
      }

      // 3. Stale commit branch (mutate another region, then attempt commit)
      if (op_type == 5) {
        const size_t other_idx = (region_idx + 1) % NUM_REGIONS;
        const auto& other_region = test_keys[other_idx];
        uint64_t other_seq = region_initialized[other_idx] ? (region_last_frame[other_idx] + 1) : 1;
        ctrl.observe(other_region, other_seq, ResolutionDecision{.baseline_level = avr::spatial::L3, .requested_level = other_region.level});
        region_initialized[other_idx] = true;
        region_last_frame[other_idx] = other_seq;

        // Attempt commit of stale prep
        const auto status = ctrl.commitPreparedObservation(prep);
        EXPECT_EQ(status, PreparedCommitStatus::STALE_CONTROLLER_REVISION);
        sr.commit_statuses.push_back(status);
        continue;
      }

      // 4. Batch commit branch
      if (op_type == 6) {
        // Build small batch of 3 distinct regions
        std::vector<HysteresisPreparedObservation> batch;
        batch.push_back(prep);

        const size_t r1 = (region_idx + 2) % NUM_REGIONS;
        const size_t r2 = (region_idx + 3) % NUM_REGIONS;
        uint64_t f1 = region_initialized[r1] ? (region_last_frame[r1] + 1) : 10;
        uint64_t f2 = region_initialized[r2] ? (region_last_frame[r2] + 1) : 10;

        batch.push_back(ctrl.previewObservation(test_keys[r1], f1, ResolutionDecision{.baseline_level = avr::spatial::L3, .requested_level = test_keys[r1].level}));
        batch.push_back(ctrl.previewObservation(test_keys[r2], f2, ResolutionDecision{.baseline_level = avr::spatial::L3, .requested_level = test_keys[r2].level}));
        sr.preview_count += 2;

        const auto b_res = ctrl.commitPreparedBatch(batch);
        sr.batch_statuses.push_back(b_res.status);
        if (b_res.isSuccess()) {
          region_initialized[region_idx] = true;
          region_last_frame[region_idx] = frame_seq;
          region_initialized[r1] = true;
          region_last_frame[r1] = f1;
          region_initialized[r2] = true;
          region_last_frame[r2] = f2;
        }
        continue;
      }

      // 5. Stale commit branch via clear
      if (op_type == 7) {
        ctrl.clear();
        std::fill(region_initialized.begin(), region_initialized.end(), false);
        std::fill(region_last_frame.begin(), region_last_frame.end(), 0);

        const auto status = ctrl.commitPreparedObservation(prep);
        EXPECT_EQ(status, PreparedCommitStatus::STALE_CONTROLLER_REVISION);
        sr.commit_statuses.push_back(status);
        continue;
      }

      // 6. Stale batch branch via unrelated ack
      if (op_type == 8) {
        std::vector<HysteresisPreparedObservation> batch;
        batch.push_back(prep);

        const size_t r1 = (region_idx + 2) % NUM_REGIONS;
        uint64_t f1 = region_initialized[r1] ? (region_last_frame[r1] + 1) : 10;
        batch.push_back(ctrl.previewObservation(test_keys[r1], f1, ResolutionDecision{.baseline_level = avr::spatial::L3, .requested_level = test_keys[r1].level}));
        sr.preview_count += 1;

        // Force a region to pending and acknowledge it to advance revision
        const size_t ack_idx = (region_idx + 4) % NUM_REGIONS;
        const auto& ack_region = test_keys[ack_idx];
        for (uint32_t t = 0; t < THRESHOLD; ++t) {
          uint64_t f = region_initialized[ack_idx] ? (region_last_frame[ack_idx] + 1) : (t + 1);
          ctrl.observe(ack_region, f, ResolutionDecision{.baseline_level = avr::spatial::L3, .requested_level = static_cast<uint8_t>(ack_region.level + 1)});
          region_initialized[ack_idx] = true;
          region_last_frame[ack_idx] = f;
        }
        EXPECT_TRUE(ctrl.acknowledgeCoarsenResult(ack_region, CoarsenAttemptOutcome::APPLIED));
        region_initialized[ack_idx] = false;
        region_last_frame[ack_idx] = 0;

        const auto b_res = ctrl.commitPreparedBatch(batch);
        EXPECT_EQ(b_res.status, PreparedBatchCommitStatus::STALE_CONTROLLER_REVISION);
        sr.batch_statuses.push_back(b_res.status);
        continue;
      }

      // 7. Normal single commit
      const auto status = ctrl.commitPreparedObservation(prep);
      sr.commit_statuses.push_back(status);
      EXPECT_EQ(status, PreparedCommitStatus::APPLIED);

      region_initialized[region_idx] = true;
      region_last_frame[region_idx] = frame_seq;

      // Verify immediate invariant after observation
      const auto* st = ctrl.findRegionState(region);
      EXPECT_NE(st, nullptr);
      if (st == nullptr) {
        continue;
      }
      EXPECT_TRUE(st->has_last_frame);
      EXPECT_LE(st->stable_coarsen_count, THRESHOLD);

      if (prep.result.action == HysteresisAction::COARSEN_READY) {
        EXPECT_EQ(st->stable_coarsen_count, THRESHOLD);
        EXPECT_TRUE(st->coarsen_action_pending);
        const uint64_t ack_choice = next_random() % 3;
        if (ack_choice == 0) {
          bool ok = ctrl.acknowledgeCoarsenResult(region, CoarsenAttemptOutcome::APPLIED);
          EXPECT_TRUE(ok);
          sr.acks.push_back(ok);
          region_initialized[region_idx] = false;
          region_last_frame[region_idx] = 0;
        } else if (ack_choice == 1) {
          bool ok = ctrl.acknowledgeCoarsenResult(region, CoarsenAttemptOutcome::BLOCKED_HOLD_FINE);
          EXPECT_TRUE(ok);
          sr.acks.push_back(ok);
        } else {
          bool ok = ctrl.acknowledgeCoarsenResult(region, CoarsenAttemptOutcome::CANCELLED);
          EXPECT_TRUE(ok);
          sr.acks.push_back(ok);
        }
      } else if (prep.result.action == HysteresisAction::REFINE_NOW || prep.result.action == HysteresisAction::HOLD_CURRENT) {
        EXPECT_EQ(st->stable_coarsen_count, 0U);
        EXPECT_FALSE(st->coarsen_action_pending);
      } else if (prep.result.action == HysteresisAction::COARSEN_PENDING) {
        EXPECT_LT(st->stable_coarsen_count, THRESHOLD);
        EXPECT_FALSE(st->coarsen_action_pending);
      }

      // Occasional erase
      if (op_type == 50) {
        ctrl.eraseRegion(region);
        region_initialized[region_idx] = false;
        region_last_frame[region_idx] = 0;
      }

      // Occasional clear
      if (op_type == 99 && (step % 5000) == 0) {
        ctrl.clear();
        std::fill(region_initialized.begin(), region_initialized.end(), false);
        std::fill(region_last_frame.begin(), region_last_frame.end(), 0);
      }
    }

    return sr;
  };

  // Run controller 1
  uint64_t initial_seed = rng;
  auto ctrl1 = *ResolutionHysteresisController::create(ResolutionHysteresisConfig{.coarsen_stable_observations = THRESHOLD});
  rng = initial_seed;
  const auto results1 = run_staged_stream(ctrl1);

  EXPECT_GE(results1.preview_count, 100000U);

  // Invariant verification on final state of ctrl1
  const auto snap1 = ctrl1.stateSnapshot();
  for (size_t i = 0; i < snap1.size(); ++i) {
    const auto& s = snap1[i];
    EXPECT_TRUE(s.has_last_frame);
    EXPECT_LE(s.stable_coarsen_count, THRESHOLD);
    if (s.coarsen_action_pending) {
      EXPECT_EQ(s.stable_coarsen_count, THRESHOLD);
      EXPECT_EQ(s.pending_target_level, s.region.level + 1);
    } else {
      EXPECT_EQ(s.pending_target_level, 0U);
    }
    if (i + 1 < snap1.size()) {
      EXPECT_TRUE(snap1[i].region < snap1[i + 1].region);
    }
  }

  // Run controller 2 with identical seed stream
  auto ctrl2 = *ResolutionHysteresisController::create(ResolutionHysteresisConfig{.coarsen_stable_observations = THRESHOLD});
  rng = initial_seed;
  const auto results2 = run_staged_stream(ctrl2);

  // Repeatability verification: exact bit-for-bit equivalence
  EXPECT_EQ(results1.actions, results2.actions);
  EXPECT_EQ(results1.commit_statuses, results2.commit_statuses);
  EXPECT_EQ(results1.batch_statuses, results2.batch_statuses);
  EXPECT_EQ(results1.acks, results2.acks);
  EXPECT_EQ(ctrl1.stateRevision(), ctrl2.stateRevision());
  EXPECT_EQ(ctrl1.trackedRegionCount(), ctrl2.trackedRegionCount());
  EXPECT_EQ(ctrl1.stateSnapshot(), ctrl2.stateSnapshot());
}

TEST(ResolutionHysteresisStressTest, NearMaxRevisionAndTerminalMutationFrozenStress) {
  auto ctrl = *ResolutionHysteresisController::create(ResolutionHysteresisConfig{.coarsen_stable_observations = 3});
  constexpr size_t NUM_REGIONS = 20;
  std::vector<avr::spatial::CellKey> test_keys;
  test_keys.reserve(NUM_REGIONS);
  for (size_t i = 0; i < NUM_REGIONS; ++i) {
    test_keys.push_back(*avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{static_cast<int32_t>(i), 0}, avr::spatial::L0));
  }

  // Populate regions
  for (size_t i = 0; i < 5; ++i) {
    ctrl.observe(test_keys[i], 10, ResolutionDecision{.baseline_level = avr::spatial::L3, .requested_level = avr::spatial::L1});
  }

  // Jump revision near UINT64_MAX
  constexpr uint64_t START_REV = UINT64_MAX - 10;
  ResolutionHysteresisTestAccess::setRevision(ctrl, START_REV);
  EXPECT_EQ(ctrl.stateRevision(), START_REV);

  // Perform 10 valid single-revision advances until UINT64_MAX
  for (uint64_t step = 0; step < 10; ++step) {
    const uint64_t expected_rev = START_REV + step;
    EXPECT_EQ(ctrl.stateRevision(), expected_rev);

    const auto prep = ctrl.previewObservation(test_keys[step % 5], 20 + step, ResolutionDecision{.baseline_level = avr::spatial::L3, .requested_level = avr::spatial::L1});
    EXPECT_TRUE(prep.isCommittable());
    EXPECT_EQ(prep.expected_controller_revision, expected_rev);

    const auto status = ctrl.commitPreparedObservation(prep);
    EXPECT_EQ(status, PreparedCommitStatus::APPLIED);
    EXPECT_EQ(ctrl.stateRevision(), expected_rev + 1);
  }

  // Controller is now at UINT64_MAX!
  EXPECT_EQ(ctrl.stateRevision(), UINT64_MAX);

  const auto snap_terminal = ctrl.stateSnapshot();
  const auto cfg_terminal = ctrl.config();
  const size_t tracked_terminal = ctrl.trackedRegionCount();

  // Test 100 random preview/commit/observe/ack/erase/clear attempts
  for (size_t step = 0; step < 100; ++step) {
    const auto& key = test_keys[step % NUM_REGIONS];

    // Preview at max revision is legal, produces valid result, but commit is REVISION_EXHAUSTED
    const auto prep = ctrl.previewObservation(key, 1000 + step, ResolutionDecision{.baseline_level = avr::spatial::L3, .requested_level = avr::spatial::L1});
    EXPECT_TRUE(prep.result.isSuccess());

    // Single commit rejected
    EXPECT_EQ(ctrl.commitPreparedObservation(prep), PreparedCommitStatus::REVISION_EXHAUSTED);

    // Batch commit rejected
    std::vector<HysteresisPreparedObservation> batch{prep};
    const auto b_res = ctrl.commitPreparedBatch(batch);
    EXPECT_EQ(b_res.status, PreparedBatchCommitStatus::REVISION_EXHAUSTED);
    EXPECT_EQ(b_res.committed_count, 0U);

    // Legacy observe fails closed
    const auto obs_res = ctrl.observe(key, 1000 + step, ResolutionDecision{.baseline_level = avr::spatial::L3, .requested_level = avr::spatial::L1});
    EXPECT_EQ(obs_res.action, HysteresisAction::INVALID_OBSERVATION);

    // Ack fails closed
    EXPECT_FALSE(ctrl.acknowledgeCoarsenResult(key, CoarsenAttemptOutcome::APPLIED));
    EXPECT_FALSE(ctrl.acknowledgeCoarsenResult(key, CoarsenAttemptOutcome::BLOCKED_HOLD_FINE));
    EXPECT_FALSE(ctrl.acknowledgeCoarsenResult(key, CoarsenAttemptOutcome::CANCELLED));

    // Erase fails closed
    EXPECT_FALSE(ctrl.eraseRegion(key));

    // Clear fails closed with zero mutation
    ctrl.clear();

    // Verify completely mutation-frozen state
    EXPECT_EQ(ctrl.stateRevision(), UINT64_MAX);  // Never wraps to 0!
    EXPECT_EQ(ctrl.trackedRegionCount(), tracked_terminal);
    EXPECT_EQ(ctrl.stateSnapshot(), snap_terminal);
    EXPECT_EQ(ctrl.config().coarsen_stable_observations, cfg_terminal.coarsen_stable_observations);
  }
}

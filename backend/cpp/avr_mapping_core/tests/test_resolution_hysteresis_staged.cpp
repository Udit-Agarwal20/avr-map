#include <gtest/gtest.h>

#include <cstdint>
#include <optional>
#include <span>
#include <vector>

#include "avr_mapping/resolution_decision.hpp"
#include "avr_mapping/resolution_hysteresis.hpp"
#include "avr_mapping/resolution_hysteresis_config.hpp"
#include "avr_mapping/resolution_hysteresis_result.hpp"
#include "avr_mapping/resolution_hysteresis_state.hpp"
#include "avr_spatial/cell_key.hpp"
#include "avr_spatial/constants.hpp"

namespace avr::mapping {

struct ResolutionHysteresisTestAccess {
  static void setRevision(ResolutionHysteresisController& ctrl, uint64_t rev) noexcept {
    ctrl.state_revision_ = rev;
  }
};

namespace {

ResolutionDecision makeDecision(
    uint8_t requested_level,
    ResolutionReasonBits reasons = {},
    uint8_t baseline_level = avr::spatial::L3) {
  ResolutionDecision dec{};
  dec.baseline_level = baseline_level;
  dec.requested_level = requested_level;
  dec.reasons = reasons;
  return dec;
}

// ============================================================================
// Section 1: Preview Non-Mutating & Zero Side-Effects
// ============================================================================

TEST(ResolutionHysteresisStagedTest, PreviewZeroMutationAndSnapshotImmutability) {
  auto ctrl = *ResolutionHysteresisController::create(ResolutionHysteresisConfig{.coarsen_stable_observations = 3});
  const auto key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L0);

  EXPECT_EQ(ctrl.stateRevision(), 0U);
  EXPECT_EQ(ctrl.trackedRegionCount(), 0U);
  EXPECT_FALSE(ctrl.containsRegion(key));

  // Preview on absent region
  const auto prep1 = ctrl.previewObservation(key, 10, makeDecision(avr::spatial::L1));
  EXPECT_TRUE(prep1.isCommittable());
  EXPECT_EQ(prep1.result.action, HysteresisAction::COARSEN_PENDING);
  EXPECT_EQ(prep1.result.target_level, avr::spatial::L1);
  EXPECT_EQ(prep1.expected_controller_revision, 0U);
  EXPECT_FALSE(prep1.expected_prior_present);
  EXPECT_TRUE(prep1.has_candidate_state);
  EXPECT_EQ(prep1.candidate_state.stable_coarsen_count, 1U);
  EXPECT_TRUE(prep1.candidate_state.has_last_frame);
  EXPECT_EQ(prep1.candidate_state.last_frame_seq, 10U);

  // Controller state remains COMPLETELY unchanged
  EXPECT_EQ(ctrl.stateRevision(), 0U);
  EXPECT_EQ(ctrl.trackedRegionCount(), 0U);
  EXPECT_FALSE(ctrl.containsRegion(key));
  EXPECT_EQ(ctrl.findRegionState(key), nullptr);
  EXPECT_FALSE(ctrl.getState(key).has_value());
  EXPECT_TRUE(ctrl.stateSnapshot().empty());

  // Discard prep1: zero side-effects
  // Repeat preview: deterministic identical result
  const auto prep2 = ctrl.previewObservation(key, 10, makeDecision(avr::spatial::L1));
  EXPECT_EQ(prep1, prep2);

  // Commit prep2
  const auto status = ctrl.commitPreparedObservation(prep2);
  EXPECT_EQ(status, PreparedCommitStatus::APPLIED);
  EXPECT_EQ(ctrl.stateRevision(), 1U);
  EXPECT_EQ(ctrl.trackedRegionCount(), 1U);
  EXPECT_TRUE(ctrl.containsRegion(key));

  // Snapshot after commit
  const auto snap_before = ctrl.stateSnapshot();
  ASSERT_EQ(snap_before.size(), 1U);

  // Preview on existing region
  const auto prep3 = ctrl.previewObservation(key, 20, makeDecision(avr::spatial::L1));
  EXPECT_TRUE(prep3.isCommittable());
  EXPECT_EQ(prep3.expected_controller_revision, 1U);
  EXPECT_TRUE(prep3.expected_prior_present);
  EXPECT_EQ(prep3.expected_prior_state.stable_coarsen_count, 1U);
  EXPECT_EQ(prep3.candidate_state.stable_coarsen_count, 2U);

  // Controller state again completely unchanged by preview
  EXPECT_EQ(ctrl.stateRevision(), 1U);
  EXPECT_EQ(ctrl.stateSnapshot(), snap_before);
}

// ============================================================================
// Section 2: Invalid Preview Non-Committable
// ============================================================================

TEST(ResolutionHysteresisStagedTest, InvalidPreviewNonCommittable) {
  auto ctrl = *ResolutionHysteresisController::create();
  const auto valid_key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L0);

  // Invalid region level (4)
  const avr::spatial::CellKey invalid_region{.morton = 0, .level = 4};
  const auto prep_bad_region = ctrl.previewObservation(invalid_region, 10, makeDecision(avr::spatial::L1));
  EXPECT_EQ(prep_bad_region.result.action, HysteresisAction::INVALID_OBSERVATION);
  EXPECT_FALSE(prep_bad_region.isCommittable());
  EXPECT_FALSE(prep_bad_region.has_candidate_state);
  EXPECT_EQ(ctrl.commitPreparedObservation(prep_bad_region), PreparedCommitStatus::NOT_COMMITTABLE);
  EXPECT_EQ(ctrl.stateRevision(), 0U);

  // Invalid requested level (99)
  const auto prep_bad_req = ctrl.previewObservation(valid_key, 10, makeDecision(99));
  EXPECT_EQ(prep_bad_req.result.action, HysteresisAction::INVALID_OBSERVATION);
  EXPECT_FALSE(prep_bad_req.isCommittable());
  EXPECT_EQ(ctrl.commitPreparedObservation(prep_bad_req), PreparedCommitStatus::NOT_COMMITTABLE);
  EXPECT_EQ(ctrl.stateRevision(), 0U);

  // Valid first observation
  ctrl.observe(valid_key, 10, makeDecision(avr::spatial::L1));
  EXPECT_EQ(ctrl.stateRevision(), 1U);

  // Duplicate frame sequence (10)
  const auto prep_dup = ctrl.previewObservation(valid_key, 10, makeDecision(avr::spatial::L1));
  EXPECT_EQ(prep_dup.result.action, HysteresisAction::INVALID_OBSERVATION);
  EXPECT_FALSE(prep_dup.isCommittable());
  EXPECT_EQ(ctrl.commitPreparedObservation(prep_dup), PreparedCommitStatus::NOT_COMMITTABLE);
  EXPECT_EQ(ctrl.stateRevision(), 1U);

  // Out-of-order frame sequence (5 < 10)
  const auto prep_ooo = ctrl.previewObservation(valid_key, 5, makeDecision(avr::spatial::L1));
  EXPECT_EQ(prep_ooo.result.action, HysteresisAction::INVALID_OBSERVATION);
  EXPECT_FALSE(prep_ooo.isCommittable());
  EXPECT_EQ(ctrl.commitPreparedObservation(prep_ooo), PreparedCommitStatus::NOT_COMMITTABLE);
  EXPECT_EQ(ctrl.stateRevision(), 1U);
}

// ============================================================================
// Section 3: Stale Precondition Detection (Ack, Erase, Clear, Other Mutation)
// ============================================================================

TEST(ResolutionHysteresisStagedTest, StaleAfterAck) {
  auto ctrl = *ResolutionHysteresisController::create(ResolutionHysteresisConfig{.coarsen_stable_observations = 2});
  const auto key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L0);

  ctrl.observe(key, 10, makeDecision(avr::spatial::L1));
  auto res = ctrl.observe(key, 20, makeDecision(avr::spatial::L1));
  EXPECT_EQ(res.action, HysteresisAction::COARSEN_READY);
  EXPECT_EQ(ctrl.stateRevision(), 2U);

  // Preview next observation while pending
  const auto prep = ctrl.previewObservation(key, 30, makeDecision(avr::spatial::L1));
  EXPECT_EQ(prep.result.action, HysteresisAction::COARSEN_READY_PENDING_ACK);
  EXPECT_EQ(prep.expected_controller_revision, 2U);

  // Intervening acknowledgement
  EXPECT_TRUE(ctrl.acknowledgeCoarsenResult(key, CoarsenAttemptOutcome::BLOCKED_HOLD_FINE));
  EXPECT_EQ(ctrl.stateRevision(), 3U);

  // Attempt commit of stale prepared observation
  const auto status = ctrl.commitPreparedObservation(prep);
  EXPECT_EQ(status, PreparedCommitStatus::STALE_CONTROLLER_REVISION);
  EXPECT_EQ(ctrl.stateRevision(), 3U);

  // Authoritative state remains BLOCKED_HOLD_FINE (streak 0, pending false)
  const auto st = ctrl.getState(key);
  ASSERT_TRUE(st.has_value());
  EXPECT_FALSE(st->coarsen_action_pending);
  EXPECT_EQ(st->stable_coarsen_count, 0U);
  EXPECT_EQ(st->last_frame_seq, 20U);
}

TEST(ResolutionHysteresisStagedTest, StaleAfterErase) {
  auto ctrl = *ResolutionHysteresisController::create();
  const auto key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L0);

  ctrl.observe(key, 10, makeDecision(avr::spatial::L1));
  EXPECT_EQ(ctrl.stateRevision(), 1U);

  const auto prep = ctrl.previewObservation(key, 20, makeDecision(avr::spatial::L1));
  EXPECT_EQ(prep.expected_controller_revision, 1U);

  // Erase region
  EXPECT_TRUE(ctrl.eraseRegion(key));
  EXPECT_EQ(ctrl.stateRevision(), 2U);

  // Stale commit rejected
  EXPECT_EQ(ctrl.commitPreparedObservation(prep), PreparedCommitStatus::STALE_CONTROLLER_REVISION);
  EXPECT_EQ(ctrl.stateRevision(), 2U);
  EXPECT_FALSE(ctrl.containsRegion(key));
}

TEST(ResolutionHysteresisStagedTest, StaleAfterClearExistingAndAbsent) {
  auto ctrl = *ResolutionHysteresisController::create();
  const auto key1 = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L0);
  const auto key2 = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{1, 1}, avr::spatial::L1);

  // Test 1: preview existing, then clear
  ctrl.observe(key1, 10, makeDecision(avr::spatial::L1));
  const auto prep_existing = ctrl.previewObservation(key1, 20, makeDecision(avr::spatial::L1));
  EXPECT_EQ(prep_existing.expected_controller_revision, 1U);

  ctrl.clear();
  EXPECT_EQ(ctrl.stateRevision(), 2U);  // Clear increments revision
  EXPECT_EQ(ctrl.commitPreparedObservation(prep_existing), PreparedCommitStatus::STALE_CONTROLLER_REVISION);

  // Test 2: preview absent on empty controller, then clear
  const auto prep_absent = ctrl.previewObservation(key2, 10, makeDecision(avr::spatial::L2));
  EXPECT_EQ(prep_absent.expected_controller_revision, 2U);

  ctrl.clear();  // Clear on already-empty controller acts as revision barrier!
  EXPECT_EQ(ctrl.stateRevision(), 3U);
  EXPECT_EQ(ctrl.commitPreparedObservation(prep_absent), PreparedCommitStatus::STALE_CONTROLLER_REVISION);
  EXPECT_FALSE(ctrl.containsRegion(key2));
}

TEST(ResolutionHysteresisStagedTest, StaleAfterOtherRegionMutation) {
  auto ctrl = *ResolutionHysteresisController::create();
  const auto key_a = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L0);
  const auto key_b = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{10, 10}, avr::spatial::L1);

  // Preview A
  const auto prep_a = ctrl.previewObservation(key_a, 10, makeDecision(avr::spatial::L1));
  EXPECT_EQ(prep_a.expected_controller_revision, 0U);

  // Mutate B
  ctrl.observe(key_b, 10, makeDecision(avr::spatial::L2));
  EXPECT_EQ(ctrl.stateRevision(), 1U);

  // Commit prepared A: rejected because revision changed
  EXPECT_EQ(ctrl.commitPreparedObservation(prep_a), PreparedCommitStatus::STALE_CONTROLLER_REVISION);
  EXPECT_EQ(ctrl.stateRevision(), 1U);
  EXPECT_FALSE(ctrl.containsRegion(key_a));
  EXPECT_TRUE(ctrl.containsRegion(key_b));
}

TEST(ResolutionHysteresisStagedTest, DoubleCommitRejectedStale) {
  auto ctrl = *ResolutionHysteresisController::create();
  const auto key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L0);

  const auto prep = ctrl.previewObservation(key, 10, makeDecision(avr::spatial::L1));
  EXPECT_TRUE(prep.isCommittable());
  EXPECT_EQ(prep.expected_controller_revision, 0U);

  // First commit succeeds: APPLIED
  const auto status1 = ctrl.commitPreparedObservation(prep);
  EXPECT_EQ(status1, PreparedCommitStatus::APPLIED);
  EXPECT_EQ(ctrl.stateRevision(), 1U);
  EXPECT_TRUE(ctrl.containsRegion(key));

  // Snapshot after first commit
  const auto snap1 = ctrl.stateSnapshot();

  // Attempt duplicate commit of the EXACT SAME prepared observation
  const auto status2 = ctrl.commitPreparedObservation(prep);
  EXPECT_EQ(status2, PreparedCommitStatus::STALE_CONTROLLER_REVISION);

  // Zero second mutation: revision and snapshot completely unchanged
  EXPECT_EQ(ctrl.stateRevision(), 1U);
  EXPECT_EQ(ctrl.stateSnapshot(), snap1);
}

// ============================================================================
// Section 4: Preview Does Not Affect Ack Behavior
// ============================================================================

TEST(ResolutionHysteresisStagedTest, PreviewDoesNotAffectAckBehavior) {
  auto ctrl = *ResolutionHysteresisController::create(ResolutionHysteresisConfig{.coarsen_stable_observations = 2});
  const auto key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L0);

  ctrl.observe(key, 10, makeDecision(avr::spatial::L1));  // streak 1 < 2, not pending

  // Preview second observation that would reach COARSEN_READY
  const auto prep = ctrl.previewObservation(key, 20, makeDecision(avr::spatial::L1));
  EXPECT_EQ(prep.result.action, HysteresisAction::COARSEN_READY);
  EXPECT_TRUE(prep.candidate_state.coarsen_action_pending);

  // Controller state is NOT pending! Calling acknowledge must fail cleanly
  EXPECT_FALSE(ctrl.acknowledgeCoarsenResult(key, CoarsenAttemptOutcome::APPLIED));
  EXPECT_FALSE(ctrl.acknowledgeCoarsenResult(key, CoarsenAttemptOutcome::BLOCKED_HOLD_FINE));
  EXPECT_FALSE(ctrl.acknowledgeCoarsenResult(key, CoarsenAttemptOutcome::CANCELLED));
  EXPECT_TRUE(ctrl.containsRegion(key));
  EXPECT_EQ(ctrl.getState(key)->stable_coarsen_count, 1U);
}

// ============================================================================
// Section 5: Batch Commit Operations and Invariants
// ============================================================================

TEST(ResolutionHysteresisStagedTest, BatchCommitAppliedAndSingleRevisionAdvance) {
  auto ctrl = *ResolutionHysteresisController::create(ResolutionHysteresisConfig{.coarsen_stable_observations = 3});
  const auto k0 = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L0);
  const auto k1 = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{1, 1}, avr::spatial::L1);
  const auto k2 = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{2, 2}, avr::spatial::L2);
  const auto k3 = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{-5, -5}, avr::spatial::L3);

  // Preview all 4 from revision 0
  std::vector<HysteresisPreparedObservation> batch;
  batch.push_back(ctrl.previewObservation(k0, 10, makeDecision(avr::spatial::L1)));
  batch.push_back(ctrl.previewObservation(k1, 10, makeDecision(avr::spatial::L2)));
  batch.push_back(ctrl.previewObservation(k2, 10, makeDecision(avr::spatial::L3)));
  batch.push_back(ctrl.previewObservation(k3, 10, makeDecision(avr::spatial::L0)));  // refine

  EXPECT_EQ(ctrl.stateRevision(), 0U);

  // Commit batch
  const auto res = ctrl.commitPreparedBatch(batch);
  EXPECT_EQ(res.status, PreparedBatchCommitStatus::APPLIED);
  EXPECT_TRUE(res.isSuccess());
  EXPECT_EQ(res.committed_count, 4U);

  // stateRevision_ increments EXACTLY ONCE for the whole batch
  EXPECT_EQ(ctrl.stateRevision(), 1U);
  EXPECT_EQ(ctrl.trackedRegionCount(), 4U);

  EXPECT_EQ(ctrl.getState(k0)->stable_coarsen_count, 1U);
  EXPECT_EQ(ctrl.getState(k1)->stable_coarsen_count, 1U);
  EXPECT_EQ(ctrl.getState(k2)->stable_coarsen_count, 1U);
  EXPECT_EQ(ctrl.getState(k3)->stable_coarsen_count, 0U);
}

TEST(ResolutionHysteresisStagedTest, BatchSubsetSurvivingCommit) {
  auto ctrl = *ResolutionHysteresisController::create(ResolutionHysteresisConfig{.coarsen_stable_observations = 3});
  const auto kA = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L0);
  const auto kB = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{1, 1}, avr::spatial::L1);
  const auto kC = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{2, 2}, avr::spatial::L2);
  const auto kD = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{3, 3}, avr::spatial::L3);

  // Preview A, B, C, D at revision 0
  const auto prepA = ctrl.previewObservation(kA, 10, makeDecision(avr::spatial::L1));
  const auto prepB = ctrl.previewObservation(kB, 10, makeDecision(avr::spatial::L2));
  const auto prepC = ctrl.previewObservation(kC, 10, makeDecision(avr::spatial::L3));
  const auto prepD = ctrl.previewObservation(kD, 10, makeDecision(avr::spatial::L0));

  EXPECT_EQ(prepA.expected_controller_revision, 0U);
  EXPECT_EQ(prepB.expected_controller_revision, 0U);
  EXPECT_EQ(prepC.expected_controller_revision, 0U);
  EXPECT_EQ(prepD.expected_controller_revision, 0U);

  // Future pipeline simulation: structural refinement converts B and D into internal nodes before fusion.
  // After successful fusion, only surviving leaf observations A and C are submitted as a batch.
  std::vector<HysteresisPreparedObservation> surviving_batch{prepA, prepC};

  const auto res = ctrl.commitPreparedBatch(surviving_batch);
  EXPECT_EQ(res.status, PreparedBatchCommitStatus::APPLIED);
  EXPECT_TRUE(res.isSuccess());
  EXPECT_EQ(res.committed_count, 2U);

  // Exactly one revision advance
  EXPECT_EQ(ctrl.stateRevision(), 1U);
  EXPECT_EQ(ctrl.trackedRegionCount(), 2U);
  EXPECT_TRUE(ctrl.containsRegion(kA));
  EXPECT_FALSE(ctrl.containsRegion(kB));
  EXPECT_TRUE(ctrl.containsRegion(kC));
  EXPECT_FALSE(ctrl.containsRegion(kD));
}

TEST(ResolutionHysteresisStagedTest, BatchEmptyIsSuccessfulNoOp) {
  auto ctrl = *ResolutionHysteresisController::create();
  EXPECT_EQ(ctrl.stateRevision(), 0U);

  const auto res = ctrl.commitPreparedBatch({});
  EXPECT_EQ(res.status, PreparedBatchCommitStatus::APPLIED);
  EXPECT_TRUE(res.isSuccess());
  EXPECT_EQ(res.committed_count, 0U);
  EXPECT_EQ(ctrl.stateRevision(), 0U);  // No revision advance
}

TEST(ResolutionHysteresisStagedTest, BatchStaleRevisionRejectionZeroMutation) {
  auto ctrl = *ResolutionHysteresisController::create();
  const auto k0 = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L0);
  const auto k1 = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{1, 1}, avr::spatial::L1);
  const auto k_other = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{99, 99}, avr::spatial::L2);

  std::vector<HysteresisPreparedObservation> batch;
  batch.push_back(ctrl.previewObservation(k0, 10, makeDecision(avr::spatial::L1)));
  batch.push_back(ctrl.previewObservation(k1, 10, makeDecision(avr::spatial::L2)));

  // Intervening mutation on unrelated key
  ctrl.observe(k_other, 10, makeDecision(avr::spatial::L3));
  EXPECT_EQ(ctrl.stateRevision(), 1U);

  // Attempt batch
  const auto res = ctrl.commitPreparedBatch(batch);
  EXPECT_EQ(res.status, PreparedBatchCommitStatus::STALE_CONTROLLER_REVISION);
  EXPECT_FALSE(res.isSuccess());
  EXPECT_EQ(res.committed_count, 0U);
  EXPECT_EQ(ctrl.stateRevision(), 1U);
  EXPECT_FALSE(ctrl.containsRegion(k0));
  EXPECT_FALSE(ctrl.containsRegion(k1));
}

TEST(ResolutionHysteresisStagedTest, BatchDuplicateRegionRejectionZeroMutation) {
  auto ctrl = *ResolutionHysteresisController::create();
  const auto key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L0);

  std::vector<HysteresisPreparedObservation> batch;
  batch.push_back(ctrl.previewObservation(key, 10, makeDecision(avr::spatial::L1)));
  batch.push_back(ctrl.previewObservation(key, 20, makeDecision(avr::spatial::L1)));

  const auto res = ctrl.commitPreparedBatch(batch);
  EXPECT_EQ(res.status, PreparedBatchCommitStatus::DUPLICATE_REGION);
  EXPECT_FALSE(res.isSuccess());
  EXPECT_EQ(res.committed_count, 0U);
  EXPECT_EQ(ctrl.stateRevision(), 0U);
  EXPECT_FALSE(ctrl.containsRegion(key));
}

TEST(ResolutionHysteresisStagedTest, BatchNonCommittableRejectionZeroMutation) {
  auto ctrl = *ResolutionHysteresisController::create();
  const auto k_valid = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L0);
  const avr::spatial::CellKey k_invalid{.morton = 0, .level = 4};

  std::vector<HysteresisPreparedObservation> batch;
  batch.push_back(ctrl.previewObservation(k_valid, 10, makeDecision(avr::spatial::L1)));
  batch.push_back(ctrl.previewObservation(k_invalid, 10, makeDecision(avr::spatial::L1)));  // invalid!

  const auto res = ctrl.commitPreparedBatch(batch);
  EXPECT_EQ(res.status, PreparedBatchCommitStatus::NOT_COMMITTABLE);
  EXPECT_FALSE(res.isSuccess());
  EXPECT_EQ(res.committed_count, 0U);
  EXPECT_EQ(ctrl.stateRevision(), 0U);
  EXPECT_FALSE(ctrl.containsRegion(k_valid));
}

TEST(ResolutionHysteresisStagedTest, BatchPriorStateMismatchZeroMutation) {
  auto ctrl = *ResolutionHysteresisController::create();
  const auto k0 = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L0);
  const auto k1 = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{1, 1}, avr::spatial::L1);

  ctrl.observe(k0, 10, makeDecision(avr::spatial::L1));

  auto prep0 = ctrl.previewObservation(k0, 20, makeDecision(avr::spatial::L1));
  auto prep1 = ctrl.previewObservation(k1, 20, makeDecision(avr::spatial::L2));

  // Artificially tamper with prep0's expected prior state without changing expected revision
  prep0.expected_prior_state.stable_coarsen_count = 999;

  std::vector<HysteresisPreparedObservation> batch{prep0, prep1};
  const auto res = ctrl.commitPreparedBatch(batch);
  EXPECT_EQ(res.status, PreparedBatchCommitStatus::PRIOR_STATE_MISMATCH);
  EXPECT_FALSE(res.isSuccess());
  EXPECT_EQ(res.committed_count, 0U);
  EXPECT_FALSE(ctrl.containsRegion(k1));
  EXPECT_EQ(ctrl.getState(k0)->stable_coarsen_count, 1U);
}

// ============================================================================
// Section 6: Frame Boundary and Golden Value Tests
// ============================================================================

TEST(ResolutionHysteresisStagedTest, FrameZeroGolden) {
  auto ctrl = *ResolutionHysteresisController::create();
  const auto key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L0);

  const auto prep0 = ctrl.previewObservation(key, 0, makeDecision(avr::spatial::L1));
  EXPECT_TRUE(prep0.isCommittable());
  EXPECT_EQ(prep0.candidate_state.last_frame_seq, 0U);
  EXPECT_TRUE(prep0.candidate_state.has_last_frame);

  EXPECT_EQ(ctrl.commitPreparedObservation(prep0), PreparedCommitStatus::APPLIED);
  EXPECT_EQ(ctrl.getState(key)->last_frame_seq, 0U);

  // Duplicate frame 0 preview rejected
  const auto prep_dup = ctrl.previewObservation(key, 0, makeDecision(avr::spatial::L1));
  EXPECT_EQ(prep_dup.result.action, HysteresisAction::INVALID_OBSERVATION);
  EXPECT_FALSE(prep_dup.isCommittable());
}

TEST(ResolutionHysteresisStagedTest, NonConsecutiveFrameGolden) {
  auto ctrl = *ResolutionHysteresisController::create();
  const auto key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L0);

  const std::vector<uint64_t> frames = {0, 10, 42, 105, 1000};
  for (uint64_t f : frames) {
    const auto prep = ctrl.previewObservation(key, f, makeDecision(avr::spatial::L1));
    EXPECT_TRUE(prep.isCommittable());
    EXPECT_EQ(ctrl.commitPreparedObservation(prep), PreparedCommitStatus::APPLIED);
    EXPECT_EQ(ctrl.getState(key)->last_frame_seq, f);
  }
}

TEST(ResolutionHysteresisStagedTest, Uint64FrameBoundaryGolden) {
  auto ctrl = *ResolutionHysteresisController::create();
  const auto key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L0);

  constexpr uint64_t kHigh1 = UINT64_MAX - 1;
  constexpr uint64_t kHigh2 = UINT64_MAX;

  const auto prep1 = ctrl.previewObservation(key, kHigh1, makeDecision(avr::spatial::L1));
  EXPECT_TRUE(prep1.isCommittable());
  EXPECT_EQ(ctrl.commitPreparedObservation(prep1), PreparedCommitStatus::APPLIED);

  const auto prep2 = ctrl.previewObservation(key, kHigh2, makeDecision(avr::spatial::L1));
  EXPECT_TRUE(prep2.isCommittable());
  EXPECT_EQ(ctrl.commitPreparedObservation(prep2), PreparedCommitStatus::APPLIED);

  const auto prep_dup = ctrl.previewObservation(key, kHigh2, makeDecision(avr::spatial::L1));
  EXPECT_EQ(prep_dup.result.action, HysteresisAction::INVALID_OBSERVATION);
}

// ============================================================================
// Section 7: Transitions and Veto Rules Golden
// ============================================================================

TEST(ResolutionHysteresisStagedTest, RefineAndEqualAndCoarsenGolden) {
  auto ctrl = *ResolutionHysteresisController::create(ResolutionHysteresisConfig{.coarsen_stable_observations = 3});
  const auto key_l3 = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);

  // Refine preview
  const auto prep_refine = ctrl.previewObservation(key_l3, 10, makeDecision(avr::spatial::L0));
  EXPECT_EQ(prep_refine.result.action, HysteresisAction::REFINE_NOW);
  EXPECT_EQ(prep_refine.result.target_level, avr::spatial::L0);
  EXPECT_EQ(ctrl.commitPreparedObservation(prep_refine), PreparedCommitStatus::APPLIED);

  // Equal preview
  const auto prep_equal = ctrl.previewObservation(key_l3, 20, makeDecision(avr::spatial::L3));
  EXPECT_EQ(prep_equal.result.action, HysteresisAction::HOLD_CURRENT);
  EXPECT_EQ(prep_equal.result.target_level, avr::spatial::L3);
  EXPECT_EQ(ctrl.commitPreparedObservation(prep_equal), PreparedCommitStatus::APPLIED);

  // Coarsen threshold on L0
  const auto key_l0 = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L0);
  ctrl.observe(key_l0, 10, makeDecision(avr::spatial::L1));  // 1
  ctrl.observe(key_l0, 20, makeDecision(avr::spatial::L1));  // 2

  // 3rd coarse preview reaches COARSEN_READY
  const auto prep_coarsen = ctrl.previewObservation(key_l0, 30, makeDecision(avr::spatial::L1));
  EXPECT_EQ(prep_coarsen.result.action, HysteresisAction::COARSEN_READY);
  EXPECT_EQ(prep_coarsen.result.stable_coarsen_count, 3U);
  EXPECT_TRUE(prep_coarsen.candidate_state.coarsen_action_pending);

  // Controller state before commit still count 2, pending false
  EXPECT_EQ(ctrl.getState(key_l0)->stable_coarsen_count, 2U);
  EXPECT_FALSE(ctrl.getState(key_l0)->coarsen_action_pending);

  // Commit
  EXPECT_EQ(ctrl.commitPreparedObservation(prep_coarsen), PreparedCommitStatus::APPLIED);
  EXPECT_EQ(ctrl.getState(key_l0)->stable_coarsen_count, 3U);
  EXPECT_TRUE(ctrl.getState(key_l0)->coarsen_action_pending);

  // 4th coarse preview returns COARSEN_READY_PENDING_ACK
  const auto prep_pending = ctrl.previewObservation(key_l0, 40, makeDecision(avr::spatial::L1));
  EXPECT_EQ(prep_pending.result.action, HysteresisAction::COARSEN_READY_PENDING_ACK);
  EXPECT_TRUE(prep_pending.result.already_pending);
}

TEST(ResolutionHysteresisStagedTest, PendingVetoFinerAndEqual) {
  auto ctrl = *ResolutionHysteresisController::create(ResolutionHysteresisConfig{.coarsen_stable_observations = 2});
  const auto key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L1);

  ctrl.observe(key, 10, makeDecision(avr::spatial::L2));
  ctrl.observe(key, 20, makeDecision(avr::spatial::L2));
  EXPECT_TRUE(ctrl.getState(key)->coarsen_action_pending);

  // Finer veto preview (L0 < L1)
  const auto prep_finer = ctrl.previewObservation(key, 30, makeDecision(avr::spatial::L0));
  EXPECT_EQ(prep_finer.result.action, HysteresisAction::REFINE_NOW);
  EXPECT_FALSE(prep_finer.candidate_state.coarsen_action_pending);
  EXPECT_EQ(prep_finer.candidate_state.stable_coarsen_count, 0U);

  // Controller state unchanged before commit
  EXPECT_TRUE(ctrl.getState(key)->coarsen_action_pending);

  EXPECT_EQ(ctrl.commitPreparedObservation(prep_finer), PreparedCommitStatus::APPLIED);
  EXPECT_FALSE(ctrl.getState(key)->coarsen_action_pending);
  EXPECT_EQ(ctrl.getState(key)->stable_coarsen_count, 0U);

  // Build back to pending
  ctrl.observe(key, 40, makeDecision(avr::spatial::L2));
  ctrl.observe(key, 50, makeDecision(avr::spatial::L2));
  EXPECT_TRUE(ctrl.getState(key)->coarsen_action_pending);

  // Equal veto preview (L1 == L1)
  const auto prep_equal = ctrl.previewObservation(key, 60, makeDecision(avr::spatial::L1));
  EXPECT_EQ(prep_equal.result.action, HysteresisAction::HOLD_CURRENT);
  EXPECT_FALSE(prep_equal.candidate_state.coarsen_action_pending);
  EXPECT_EQ(prep_equal.candidate_state.stable_coarsen_count, 0U);

  EXPECT_EQ(ctrl.commitPreparedObservation(prep_equal), PreparedCommitStatus::APPLIED);
  EXPECT_FALSE(ctrl.getState(key)->coarsen_action_pending);
  EXPECT_EQ(ctrl.getState(key)->stable_coarsen_count, 0U);
}

// ============================================================================
// Section 8: Legacy vs Staged Equivalence
// ============================================================================

TEST(ResolutionHysteresisStagedTest, LegacyVsStagedStreamEquivalence) {
  auto ctrl_legacy = *ResolutionHysteresisController::create(ResolutionHysteresisConfig{.coarsen_stable_observations = 3});
  auto ctrl_staged = *ResolutionHysteresisController::create(ResolutionHysteresisConfig{.coarsen_stable_observations = 3});

  const auto k0 = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L0);
  const auto k1 = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{-10, 20}, avr::spatial::L1);
  const auto k2 = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{5, -5}, avr::spatial::L2);

  struct Step {
    avr::spatial::CellKey key;
    uint64_t seq;
    uint8_t req_level;
  };

  const std::vector<Step> steps = {
      {k0, 0, avr::spatial::L1},    // streak 1
      {k1, 5, avr::spatial::L2},    // streak 1
      {k2, 10, avr::spatial::L0},   // refine
      {k0, 20, avr::spatial::L1},   // streak 2
      {k0, 30, avr::spatial::L1},   // streak 3 -> ready
      {k1, 25, avr::spatial::L1},   // equal -> reset
      {k2, 20, avr::spatial::L2},   // equal
      {k0, 40, avr::spatial::L1},   // ready pending ack
  };

  for (const auto& step : steps) {
    const auto dec = makeDecision(step.req_level);
    const auto res_leg = ctrl_legacy.observe(step.key, step.seq, dec);

    const auto prep = ctrl_staged.previewObservation(step.key, step.seq, dec);
    EXPECT_EQ(prep.result, res_leg);

    const auto status = ctrl_staged.commitPreparedObservation(prep);
    EXPECT_EQ(status, PreparedCommitStatus::APPLIED);

    EXPECT_EQ(ctrl_legacy.stateRevision(), ctrl_staged.stateRevision());
    EXPECT_EQ(ctrl_legacy.stateSnapshot(), ctrl_staged.stateSnapshot());
  }

  // Test ack equivalence
  EXPECT_TRUE(ctrl_legacy.acknowledgeCoarsenResult(k0, CoarsenAttemptOutcome::APPLIED));
  EXPECT_TRUE(ctrl_staged.acknowledgeCoarsenResult(k0, CoarsenAttemptOutcome::APPLIED));
  EXPECT_EQ(ctrl_legacy.stateRevision(), ctrl_staged.stateRevision());
  EXPECT_EQ(ctrl_legacy.stateSnapshot(), ctrl_staged.stateSnapshot());

  // Test erase equivalence
  EXPECT_TRUE(ctrl_legacy.eraseRegion(k1));
  EXPECT_TRUE(ctrl_staged.eraseRegion(k1));
  EXPECT_EQ(ctrl_legacy.stateRevision(), ctrl_staged.stateRevision());
  EXPECT_EQ(ctrl_legacy.stateSnapshot(), ctrl_staged.stateSnapshot());

  // Test clear equivalence
  ctrl_legacy.clear();
  ctrl_staged.clear();
  EXPECT_EQ(ctrl_legacy.stateRevision(), ctrl_staged.stateRevision());
  EXPECT_EQ(ctrl_legacy.stateSnapshot(), ctrl_staged.stateSnapshot());
}

// ============================================================================
// Section 9: Batch vs Sequential Semantic Equivalence
// ============================================================================

TEST(ResolutionHysteresisStagedTest, BatchVsSequentialSemanticEquivalence) {
  auto ctrl_batch = *ResolutionHysteresisController::create(ResolutionHysteresisConfig{.coarsen_stable_observations = 3});
  auto ctrl_seq = *ResolutionHysteresisController::create(ResolutionHysteresisConfig{.coarsen_stable_observations = 3});

  constexpr size_t N = 10;
  std::vector<avr::spatial::CellKey> keys;
  keys.reserve(N);
  for (size_t i = 0; i < N; ++i) {
    const int32_t c = static_cast<int32_t>(i) * 10 - 50;
    const uint8_t level = static_cast<uint8_t>(i % 4);
    keys.push_back(*avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{c, c}, level));
  }

  std::vector<HysteresisPreparedObservation> batch;
  batch.reserve(N);
  for (size_t i = 0; i < N; ++i) {
    const uint8_t req = static_cast<uint8_t>((keys[i].level + 1) % 4);
    batch.push_back(ctrl_batch.previewObservation(keys[i], 100, makeDecision(req)));
  }

  const auto res = ctrl_batch.commitPreparedBatch(batch);
  EXPECT_EQ(res.status, PreparedBatchCommitStatus::APPLIED);
  EXPECT_EQ(res.committed_count, N);
  EXPECT_EQ(ctrl_batch.stateRevision(), 1U);  // Advanced once

  // Sequential controller applies same observations
  for (size_t i = 0; i < N; ++i) {
    const uint8_t req = static_cast<uint8_t>((keys[i].level + 1) % 4);
    ctrl_seq.observe(keys[i], 100, makeDecision(req));
  }
  EXPECT_EQ(ctrl_seq.stateRevision(), static_cast<uint64_t>(N));  // Advanced N times

  // But final state snapshots must be identical!
  EXPECT_EQ(ctrl_batch.stateSnapshot(), ctrl_seq.stateSnapshot());
}

// ============================================================================
// Section 10: Revision Mutation Rules & String Helpers
// ============================================================================

TEST(ResolutionHysteresisStagedTest, RevisionMutationRulesAndStrings) {
  auto ctrl = *ResolutionHysteresisController::create();
  const auto unknown = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{99, 99}, avr::spatial::L0);

  EXPECT_EQ(ctrl.stateRevision(), 0U);

  // Unsuccessful ack does NOT advance revision
  EXPECT_FALSE(ctrl.acknowledgeCoarsenResult(unknown, CoarsenAttemptOutcome::APPLIED));
  EXPECT_EQ(ctrl.stateRevision(), 0U);

  // Unsuccessful erase does NOT advance revision
  EXPECT_FALSE(ctrl.eraseRegion(unknown));
  EXPECT_EQ(ctrl.stateRevision(), 0U);

  // Clear ALWAYS advances revision
  ctrl.clear();
  EXPECT_EQ(ctrl.stateRevision(), 1U);
  ctrl.clear();
  EXPECT_EQ(ctrl.stateRevision(), 2U);

  // String helpers
  EXPECT_EQ(preparedCommitStatusToString(PreparedCommitStatus::APPLIED), "APPLIED");
  EXPECT_EQ(preparedCommitStatusToString(PreparedCommitStatus::NOT_COMMITTABLE), "NOT_COMMITTABLE");
  EXPECT_EQ(preparedCommitStatusToString(PreparedCommitStatus::STALE_CONTROLLER_REVISION), "STALE_CONTROLLER_REVISION");
  EXPECT_EQ(preparedCommitStatusToString(PreparedCommitStatus::PRIOR_STATE_MISMATCH), "PRIOR_STATE_MISMATCH");
  EXPECT_EQ(preparedCommitStatusToString(PreparedCommitStatus::REVISION_EXHAUSTED), "REVISION_EXHAUSTED");

  EXPECT_EQ(preparedBatchCommitStatusToString(PreparedBatchCommitStatus::APPLIED), "APPLIED");
  EXPECT_EQ(preparedBatchCommitStatusToString(PreparedBatchCommitStatus::NOT_COMMITTABLE), "NOT_COMMITTABLE");
  EXPECT_EQ(preparedBatchCommitStatusToString(PreparedBatchCommitStatus::STALE_CONTROLLER_REVISION), "STALE_CONTROLLER_REVISION");
  EXPECT_EQ(preparedBatchCommitStatusToString(PreparedBatchCommitStatus::PRIOR_STATE_MISMATCH), "PRIOR_STATE_MISMATCH");
  EXPECT_EQ(preparedBatchCommitStatusToString(PreparedBatchCommitStatus::DUPLICATE_REGION), "DUPLICATE_REGION");
  EXPECT_EQ(preparedBatchCommitStatusToString(PreparedBatchCommitStatus::REVISION_EXHAUSTED), "REVISION_EXHAUSTED");
}

TEST(ResolutionHysteresisStagedTest, RevisionExhaustionFailClose) {
  auto ctrl = *ResolutionHysteresisController::create();
  const auto key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L0);

  ResolutionHysteresisTestAccess::setRevision(ctrl, UINT64_MAX);
  EXPECT_EQ(ctrl.stateRevision(), UINT64_MAX);

  const auto prep = ctrl.previewObservation(key, 10, makeDecision(avr::spatial::L1));
  EXPECT_TRUE(prep.isCommittable());
  EXPECT_EQ(prep.expected_controller_revision, UINT64_MAX);

  EXPECT_EQ(ctrl.commitPreparedObservation(prep), PreparedCommitStatus::REVISION_EXHAUSTED);
  EXPECT_FALSE(ctrl.containsRegion(key));

  std::vector<HysteresisPreparedObservation> batch{prep};
  const auto batch_res = ctrl.commitPreparedBatch(batch);
  EXPECT_EQ(batch_res.status, PreparedBatchCommitStatus::REVISION_EXHAUSTED);
  EXPECT_FALSE(ctrl.containsRegion(key));

  const auto obs_res = ctrl.observe(key, 10, makeDecision(avr::spatial::L1));
  EXPECT_EQ(obs_res.action, HysteresisAction::INVALID_OBSERVATION);
  EXPECT_FALSE(ctrl.containsRegion(key));

  EXPECT_FALSE(ctrl.acknowledgeCoarsenResult(key, CoarsenAttemptOutcome::APPLIED));
  EXPECT_FALSE(ctrl.eraseRegion(key));

  ctrl.clear();
  EXPECT_EQ(ctrl.stateRevision(), UINT64_MAX);
}

TEST(ResolutionHysteresisStagedTest, ClearAtRevisionExhaustionZeroMutation) {
  auto ctrl = *ResolutionHysteresisController::create(ResolutionHysteresisConfig{.coarsen_stable_observations = 3});
  const auto k0 = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L0);
  const auto k1 = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{1, 1}, avr::spatial::L1);

  // Populate at least one tracked region
  ctrl.observe(k0, 10, makeDecision(avr::spatial::L1));
  ctrl.observe(k1, 10, makeDecision(avr::spatial::L2));
  EXPECT_EQ(ctrl.trackedRegionCount(), 2U);
  EXPECT_EQ(ctrl.stateRevision(), 2U);

  // Freeze controller at revision exhaustion
  ResolutionHysteresisTestAccess::setRevision(ctrl, UINT64_MAX);
  EXPECT_EQ(ctrl.stateRevision(), UINT64_MAX);

  const auto snap_before = ctrl.stateSnapshot();
  const auto cfg_before = ctrl.config();
  EXPECT_EQ(snap_before.size(), 2U);

  // Call clear() on exhausted controller
  ctrl.clear();

  // MUST perform ZERO mutation: tracked state unchanged, revision unchanged, config unchanged
  EXPECT_EQ(ctrl.stateRevision(), UINT64_MAX);
  EXPECT_EQ(ctrl.trackedRegionCount(), 2U);
  EXPECT_TRUE(ctrl.containsRegion(k0));
  EXPECT_TRUE(ctrl.containsRegion(k1));
  EXPECT_EQ(ctrl.stateSnapshot(), snap_before);
  EXPECT_EQ(ctrl.config().coarsen_stable_observations, cfg_before.coarsen_stable_observations);
}

}  // namespace
}  // namespace avr::mapping

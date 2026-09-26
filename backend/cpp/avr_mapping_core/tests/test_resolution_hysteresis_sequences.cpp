#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "avr_mapping/resolution_decision.hpp"
#include "avr_mapping/resolution_hysteresis.hpp"
#include "avr_mapping/resolution_hysteresis_config.hpp"
#include "avr_mapping/resolution_hysteresis_result.hpp"
#include "avr_spatial/cell_key.hpp"
#include "avr_spatial/constants.hpp"
#include "avr_spatial/coordinate.hpp"

using namespace avr::mapping;

static ResolutionDecision makeDecision(uint8_t requested_level, ResolutionReasonBits reasons = {}) {
  return ResolutionDecision{
      .baseline_level = avr::spatial::L3,
      .requested_level = requested_level,
      .evidence_state = EvidenceState::SUFFICIENT,
      .reasons = reasons,
  };
}

// ============================================================================
// Section 1: Default-10 Stable Golden Sequence
// ============================================================================

TEST(ResolutionHysteresisSequenceTest, Default10StableSequence) {
  auto ctrl = *ResolutionHysteresisController::create();
  const auto key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L0);

  // Observations 1..9: must be COARSEN_PENDING
  for (uint32_t i = 1; i <= 9; ++i) {
    const uint64_t seq = static_cast<uint64_t>(i * 10);
    const auto res = ctrl.observe(key, seq, makeDecision(avr::spatial::L1));
    EXPECT_EQ(res.action, HysteresisAction::COARSEN_PENDING) << "Failed at observation " << i;
    EXPECT_EQ(res.stable_coarsen_count, i);
    EXPECT_EQ(res.target_level, avr::spatial::L1);
    EXPECT_FALSE(res.already_pending);
  }

  // Observation 10: must be COARSEN_READY
  const auto res10 = ctrl.observe(key, 100, makeDecision(avr::spatial::L1));
  EXPECT_EQ(res10.action, HysteresisAction::COARSEN_READY);
  EXPECT_EQ(res10.stable_coarsen_count, 10U);
  EXPECT_EQ(res10.target_level, avr::spatial::L1);
  EXPECT_FALSE(res10.already_pending);

  // Subsequent observations 11..20 without ack: must return COARSEN_READY_PENDING_ACK
  for (uint32_t i = 11; i <= 20; ++i) {
    const uint64_t seq = static_cast<uint64_t>(i * 10);
    const auto res = ctrl.observe(key, seq, makeDecision(avr::spatial::L1));
    EXPECT_EQ(res.action, HysteresisAction::COARSEN_READY_PENDING_ACK) << "Failed at observation " << i;
    EXPECT_EQ(res.stable_coarsen_count, 10U);  // Saturated
    EXPECT_TRUE(res.already_pending);
  }
}

// ============================================================================
// Section 2: Default-10 Oscillation Sequence
// ============================================================================

TEST(ResolutionHysteresisSequenceTest, Default10OscillationNeverCoarsens) {
  auto ctrl = *ResolutionHysteresisController::create();
  const auto key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L1);

  // Alternates: coarser (L2) then equal (L1) for 50 cycles
  for (uint64_t c = 1; c <= 50; ++c) {
    // 1. Coarser request
    const auto res_coarse = ctrl.observe(key, c * 20 - 10, makeDecision(avr::spatial::L2));
    EXPECT_EQ(res_coarse.action, HysteresisAction::COARSEN_PENDING);
    EXPECT_EQ(res_coarse.stable_coarsen_count, 1U);

    // 2. Equal request (resets streak!)
    const auto res_equal = ctrl.observe(key, c * 20, makeDecision(avr::spatial::L1));
    EXPECT_EQ(res_equal.action, HysteresisAction::HOLD_CURRENT);
    EXPECT_EQ(res_equal.stable_coarsen_count, 0U);
  }

  // Confirm state count is 0 and never reached ready
  const auto state = ctrl.getState(key);
  ASSERT_TRUE(state.has_value());
  EXPECT_EQ(state->stable_coarsen_count, 0U);
  EXPECT_FALSE(state->coarsen_action_pending);
}

// ============================================================================
// Section 3: Refine Reset Golden Sequence
// ============================================================================

TEST(ResolutionHysteresisSequenceTest, RefineResetGolden) {
  auto ctrl = *ResolutionHysteresisController::create(ResolutionHysteresisConfig{.coarsen_stable_observations = 5});
  const auto key_l1 = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L1);

  // 3 coarser observations -> count 3
  EXPECT_EQ(ctrl.observe(key_l1, 10, makeDecision(avr::spatial::L2)).action, HysteresisAction::COARSEN_PENDING);
  EXPECT_EQ(ctrl.observe(key_l1, 20, makeDecision(avr::spatial::L2)).action, HysteresisAction::COARSEN_PENDING);
  EXPECT_EQ(ctrl.observe(key_l1, 30, makeDecision(avr::spatial::L3)).action, HysteresisAction::COARSEN_PENDING);
  EXPECT_EQ(ctrl.getState(key_l1)->stable_coarsen_count, 3U);

  // Decision requests L0 (finer than L1) -> REFINE_NOW L0
  const auto res = ctrl.observe(key_l1, 40, makeDecision(avr::spatial::L0));
  EXPECT_EQ(res.action, HysteresisAction::REFINE_NOW);
  EXPECT_EQ(res.target_level, avr::spatial::L0);
  EXPECT_EQ(res.stable_coarsen_count, 0U);

  // State has 0 streak and no pending state
  const auto state = ctrl.getState(key_l1);
  ASSERT_TRUE(state.has_value());
  EXPECT_EQ(state->stable_coarsen_count, 0U);
  EXPECT_FALSE(state->coarsen_action_pending);
}

// ============================================================================
// Section 4: Hold-Fine Retry Sequence
// ============================================================================

TEST(ResolutionHysteresisSequenceTest, HoldFineRetrySequence) {
  auto ctrl = *ResolutionHysteresisController::create(ResolutionHysteresisConfig{.coarsen_stable_observations = 3});
  const auto key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L1);

  // 1. Accumulate 3 coarser observations to reach COARSEN_READY
  ctrl.observe(key, 10, makeDecision(avr::spatial::L2));
  ctrl.observe(key, 20, makeDecision(avr::spatial::L2));
  auto ready1 = ctrl.observe(key, 30, makeDecision(avr::spatial::L2));
  EXPECT_EQ(ready1.action, HysteresisAction::COARSEN_READY);

  // 2. Caller reports merge blocked due to e.g. cache eviction (BLOCKED_HOLD_FINE)
  EXPECT_TRUE(ctrl.acknowledgeCoarsenResult(key, CoarsenAttemptOutcome::BLOCKED_HOLD_FINE));
  EXPECT_EQ(ctrl.getState(key)->stable_coarsen_count, 0U);
  EXPECT_FALSE(ctrl.getState(key)->coarsen_action_pending);

  // 3. Region accumulates 2 new coarser observations -> still pending
  EXPECT_EQ(ctrl.observe(key, 40, makeDecision(avr::spatial::L2)).action, HysteresisAction::COARSEN_PENDING);
  EXPECT_EQ(ctrl.observe(key, 50, makeDecision(avr::spatial::L2)).action, HysteresisAction::COARSEN_PENDING);

  // 4. 3rd new observation -> reaches COARSEN_READY again
  auto ready2 = ctrl.observe(key, 60, makeDecision(avr::spatial::L2));
  EXPECT_EQ(ready2.action, HysteresisAction::COARSEN_READY);
  EXPECT_EQ(ready2.stable_coarsen_count, 3U);
}

// ============================================================================
// Section 5: Non-Consecutive Frame Sequence
// ============================================================================

TEST(ResolutionHysteresisSequenceTest, NonConsecutiveFrameSequence) {
  auto ctrl = *ResolutionHysteresisController::create(ResolutionHysteresisConfig{.coarsen_stable_observations = 3});
  const auto key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L0);

  // Non-consecutive frame IDs: 10, 100, 500
  EXPECT_EQ(ctrl.observe(key, 10, makeDecision(avr::spatial::L1)).action, HysteresisAction::COARSEN_PENDING);
  EXPECT_EQ(ctrl.observe(key, 100, makeDecision(avr::spatial::L1)).action, HysteresisAction::COARSEN_PENDING);
  auto res3 = ctrl.observe(key, 500, makeDecision(avr::spatial::L1));
  EXPECT_EQ(res3.action, HysteresisAction::COARSEN_READY);
  EXPECT_EQ(res3.stable_coarsen_count, 3U);
}

// ============================================================================
// Section 6: Duplicate and Out-of-Order Frame Rejection
// ============================================================================

TEST(ResolutionHysteresisSequenceTest, DuplicateFrameRejectionPreservesState) {
  auto ctrl = *ResolutionHysteresisController::create(ResolutionHysteresisConfig{.coarsen_stable_observations = 5});
  const auto key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L0);

  ctrl.observe(key, 10, makeDecision(avr::spatial::L1));
  ctrl.observe(key, 20, makeDecision(avr::spatial::L1));

  const auto snap_before = ctrl.stateSnapshot();

  // Duplicate frame 20 submitted
  const auto res_dup = ctrl.observe(key, 20, makeDecision(avr::spatial::L1));
  EXPECT_EQ(res_dup.action, HysteresisAction::INVALID_OBSERVATION);
  EXPECT_FALSE(res_dup.isSuccess());

  // Snapshot must be 100% identical (zero mutation)
  EXPECT_EQ(ctrl.stateSnapshot(), snap_before);

  // Frame 30 continues correctly
  const auto res30 = ctrl.observe(key, 30, makeDecision(avr::spatial::L1));
  EXPECT_EQ(res30.action, HysteresisAction::COARSEN_PENDING);
  EXPECT_EQ(res30.stable_coarsen_count, 3U);
}

TEST(ResolutionHysteresisSequenceTest, OutOfOrderFrameRejectionPreservesState) {
  auto ctrl = *ResolutionHysteresisController::create(ResolutionHysteresisConfig{.coarsen_stable_observations = 5});
  const auto key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L0);

  ctrl.observe(key, 10, makeDecision(avr::spatial::L1));
  ctrl.observe(key, 42, makeDecision(avr::spatial::L1));
  ctrl.observe(key, 105, makeDecision(avr::spatial::L1));

  const auto snap_before = ctrl.stateSnapshot();

  // Out-of-order frame 50 submitted (when latest is 105)
  const auto res_ooo = ctrl.observe(key, 50, makeDecision(avr::spatial::L1));
  EXPECT_EQ(res_ooo.action, HysteresisAction::INVALID_OBSERVATION);
  EXPECT_FALSE(res_ooo.isSuccess());

  // Zero mutation
  EXPECT_EQ(ctrl.stateSnapshot(), snap_before);

  // Frame 106 accepted
  const auto res106 = ctrl.observe(key, 106, makeDecision(avr::spatial::L1));
  EXPECT_EQ(res106.action, HysteresisAction::COARSEN_PENDING);
  EXPECT_EQ(res106.stable_coarsen_count, 4U);
}

// ============================================================================
// Section 7: Multi-Region Independence
// ============================================================================

TEST(ResolutionHysteresisSequenceTest, MultiRegionIndependence) {
  auto ctrl = *ResolutionHysteresisController::create(ResolutionHysteresisConfig{.coarsen_stable_observations = 3});

  const auto key_a = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L0);
  const auto key_b = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{10, 10}, avr::spatial::L1);
  const auto key_c = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{20, 20}, avr::spatial::L2);

  // Frame 10:
  ctrl.observe(key_a, 10, makeDecision(avr::spatial::L1));  // coarse
  ctrl.observe(key_b, 10, makeDecision(avr::spatial::L2));  // coarse
  ctrl.observe(key_c, 10, makeDecision(avr::spatial::L0));  // refine

  // Frame 20:
  ctrl.observe(key_a, 20, makeDecision(avr::spatial::L1));  // coarse
  ctrl.observe(key_b, 20, makeDecision(avr::spatial::L1));  // equal (resets b!)
  ctrl.observe(key_c, 20, makeDecision(avr::spatial::L2));  // equal

  // Frame 30:
  auto res_a30 = ctrl.observe(key_a, 30, makeDecision(avr::spatial::L1));
  auto res_b30 = ctrl.observe(key_b, 30, makeDecision(avr::spatial::L2));

  // Region A has reached threshold 3 -> COARSEN_READY
  EXPECT_EQ(res_a30.action, HysteresisAction::COARSEN_READY);
  EXPECT_EQ(res_a30.stable_coarsen_count, 3U);

  // Region B reset at frame 20, so at frame 30 count is 1 -> COARSEN_PENDING
  EXPECT_EQ(res_b30.action, HysteresisAction::COARSEN_PENDING);
  EXPECT_EQ(res_b30.stable_coarsen_count, 1U);

  // Region C state
  EXPECT_EQ(ctrl.getState(key_c)->stable_coarsen_count, 0U);
}

// ============================================================================
// Section 8: Fast Multi-Level Refine vs Gradual Multi-Level Coarsen
// ============================================================================

TEST(ResolutionHysteresisSequenceTest, FastMultiLevelRefineImmediate) {
  auto ctrl = *ResolutionHysteresisController::create();
  const auto root_l3 = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);

  // Immediate jump from L3 to L0 without any observation wait
  const auto res = ctrl.observe(root_l3, 10, makeDecision(avr::spatial::L0));
  EXPECT_EQ(res.action, HysteresisAction::REFINE_NOW);
  EXPECT_EQ(res.current_level, avr::spatial::L3);
  EXPECT_EQ(res.target_level, avr::spatial::L0);
  EXPECT_EQ(res.stable_coarsen_count, 0U);
}

TEST(ResolutionHysteresisSequenceTest, GradualMultiLevelCoarsen) {
  auto ctrl = *ResolutionHysteresisController::create(ResolutionHysteresisConfig{.coarsen_stable_observations = 10});

  // Step 1: L0 requests L3 for 10 observations -> reaches COARSEN_READY for L1 ONLY!
  const auto key_l0 = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L0);
  for (uint32_t i = 1; i <= 9; ++i) {
    EXPECT_EQ(ctrl.observe(key_l0, i * 10, makeDecision(avr::spatial::L3)).action, HysteresisAction::COARSEN_PENDING);
  }
  auto res_l0_ready = ctrl.observe(key_l0, 100, makeDecision(avr::spatial::L3));
  EXPECT_EQ(res_l0_ready.action, HysteresisAction::COARSEN_READY);
  EXPECT_EQ(res_l0_ready.target_level, avr::spatial::L1);  // Target is strictly L1, NOT L3!

  // Simulate M9A coarsening applied: erase child L0
  EXPECT_TRUE(ctrl.acknowledgeCoarsenResult(key_l0, CoarsenAttemptOutcome::APPLIED));

  // Step 2: Now parent L1 starts fresh. Requires ANOTHER 10 observations to coarsen L1 -> L2
  const auto key_l1 = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L1);
  for (uint32_t i = 1; i <= 9; ++i) {
    EXPECT_EQ(ctrl.observe(key_l1, 100 + i * 10, makeDecision(avr::spatial::L3)).action, HysteresisAction::COARSEN_PENDING);
  }
  auto res_l1_ready = ctrl.observe(key_l1, 200, makeDecision(avr::spatial::L3));
  EXPECT_EQ(res_l1_ready.action, HysteresisAction::COARSEN_READY);
  EXPECT_EQ(res_l1_ready.target_level, avr::spatial::L2);  // Target is L2!

  EXPECT_TRUE(ctrl.acknowledgeCoarsenResult(key_l1, CoarsenAttemptOutcome::APPLIED));

  // Step 3: Now parent L2 starts fresh. Requires ANOTHER 10 observations to coarsen L2 -> L3
  const auto key_l2 = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L2);
  for (uint32_t i = 1; i <= 9; ++i) {
    EXPECT_EQ(ctrl.observe(key_l2, 200 + i * 10, makeDecision(avr::spatial::L3)).action, HysteresisAction::COARSEN_PENDING);
  }
  auto res_l2_ready = ctrl.observe(key_l2, 300, makeDecision(avr::spatial::L3));
  EXPECT_EQ(res_l2_ready.action, HysteresisAction::COARSEN_READY);
  EXPECT_EQ(res_l2_ready.target_level, avr::spatial::L3);
}

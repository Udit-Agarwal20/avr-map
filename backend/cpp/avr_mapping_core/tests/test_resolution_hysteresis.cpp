#include "avr_mapping/resolution_hysteresis.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <optional>
#include <vector>

#include "avr_mapping/resolution_decision.hpp"
#include "avr_mapping/resolution_hysteresis_config.hpp"
#include "avr_mapping/resolution_hysteresis_result.hpp"
#include "avr_mapping/resolution_hysteresis_state.hpp"
#include "avr_spatial/cell_key.hpp"
#include "avr_spatial/constants.hpp"

namespace avr::mapping {
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
// Section 1: Configuration Validation Contract
// ============================================================================

TEST(ResolutionHysteresisTest, ConfigValidationContract) {
  // Valid configurations
  EXPECT_TRUE(ResolutionHysteresisConfig{.coarsen_stable_observations = 10}.isValid());
  EXPECT_TRUE(ResolutionHysteresisConfig{.coarsen_stable_observations = 1}.isValid());
  EXPECT_TRUE(ResolutionHysteresisConfig{.coarsen_stable_observations = 100}.isValid());

  // Invalid: 0 must be explicitly rejected, never silently clamped
  EXPECT_FALSE(ResolutionHysteresisConfig{.coarsen_stable_observations = 0}.isValid());
}

TEST(ResolutionHysteresisTest, FactoryConstruction) {
  // Valid config creates controller
  auto ctrl_opt = ResolutionHysteresisController::create(ResolutionHysteresisConfig{.coarsen_stable_observations = 5});
  ASSERT_TRUE(ctrl_opt.has_value());
  EXPECT_EQ(ctrl_opt->config().coarsen_stable_observations, 5U);
  EXPECT_EQ(ctrl_opt->trackedRegionCount(), 0U);

  // Default config
  auto ctrl_default = ResolutionHysteresisController::create();
  ASSERT_TRUE(ctrl_default.has_value());
  EXPECT_EQ(ctrl_default->config().coarsen_stable_observations, 10U);

  // Invalid config returns nullopt (no normal-looking controller produced)
  auto invalid_opt = ResolutionHysteresisController::create(ResolutionHysteresisConfig{.coarsen_stable_observations = 0});
  EXPECT_FALSE(invalid_opt.has_value());
}

// ============================================================================
// Section 2: Fast Multi-Level Refinement Rule
// ============================================================================

TEST(ResolutionHysteresisTest, FastRefinementSingleAndMultiLevel) {
  auto ctrl = *ResolutionHysteresisController::create(ResolutionHysteresisConfig{.coarsen_stable_observations = 10});
  const auto key_l3 = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);

  // Immediate multi-level refinement: L3 -> L0
  const auto res1 = ctrl.observe(key_l3, 10, makeDecision(avr::spatial::L0));
  EXPECT_EQ(res1.action, HysteresisAction::REFINE_NOW);
  EXPECT_EQ(res1.current_level, avr::spatial::L3);
  EXPECT_EQ(res1.requested_level, avr::spatial::L0);
  EXPECT_EQ(res1.target_level, avr::spatial::L0);
  EXPECT_EQ(res1.stable_coarsen_count, 0U);
  EXPECT_FALSE(res1.already_pending);

  // Single-level refinement: L2 -> L1
  const auto key_l2 = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L2);
  const auto res2 = ctrl.observe(key_l2, 10, makeDecision(avr::spatial::L1));
  EXPECT_EQ(res2.action, HysteresisAction::REFINE_NOW);
  EXPECT_EQ(res2.target_level, avr::spatial::L1);
}

// ============================================================================
// Section 3: Equal-Level Hold Rule
// ============================================================================

TEST(ResolutionHysteresisTest, EqualLevelHoldCurrent) {
  auto ctrl = *ResolutionHysteresisController::create(ResolutionHysteresisConfig{.coarsen_stable_observations = 5});
  const auto key_l1 = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L1);

  // First observation requesting equal level
  const auto res1 = ctrl.observe(key_l1, 10, makeDecision(avr::spatial::L1));
  EXPECT_EQ(res1.action, HysteresisAction::HOLD_CURRENT);
  EXPECT_EQ(res1.current_level, avr::spatial::L1);
  EXPECT_EQ(res1.requested_level, avr::spatial::L1);
  EXPECT_EQ(res1.target_level, avr::spatial::L1);
  EXPECT_EQ(res1.stable_coarsen_count, 0U);
  EXPECT_FALSE(res1.already_pending);
}

// ============================================================================
// Section 4: One-Level Coarsening Only & Coarser Target Variation
// ============================================================================

TEST(ResolutionHysteresisTest, OneLevelCoarseningOnlyAndTargetVariation) {
  auto ctrl = *ResolutionHysteresisController::create(ResolutionHysteresisConfig{.coarsen_stable_observations = 3});
  const auto key_l0 = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L0);

  // Observation 1: requested L2 (coarser than L0) -> targets L1 only
  auto res1 = ctrl.observe(key_l0, 10, makeDecision(avr::spatial::L2));
  EXPECT_EQ(res1.action, HysteresisAction::COARSEN_PENDING);
  EXPECT_EQ(res1.target_level, avr::spatial::L1);
  EXPECT_EQ(res1.stable_coarsen_count, 1U);

  // Observation 2: requested L3 (farther coarse variation) -> contributes to same L0->L1 streak
  auto res2 = ctrl.observe(key_l0, 20, makeDecision(avr::spatial::L3));
  EXPECT_EQ(res2.action, HysteresisAction::COARSEN_PENDING);
  EXPECT_EQ(res2.target_level, avr::spatial::L1);
  EXPECT_EQ(res2.stable_coarsen_count, 2U);

  // Observation 3: requested L1 -> reaches threshold -> COARSEN_READY target L1 only!
  auto res3 = ctrl.observe(key_l0, 30, makeDecision(avr::spatial::L1));
  EXPECT_EQ(res3.action, HysteresisAction::COARSEN_READY);
  EXPECT_EQ(res3.target_level, avr::spatial::L1);
  EXPECT_EQ(res3.stable_coarsen_count, 3U);
  EXPECT_FALSE(res3.already_pending);

  // Subsequent coarser observation while pending ack returns COARSEN_READY_PENDING_ACK
  auto res4 = ctrl.observe(key_l0, 40, makeDecision(avr::spatial::L3));
  EXPECT_EQ(res4.action, HysteresisAction::COARSEN_READY_PENDING_ACK);
  EXPECT_EQ(res4.target_level, avr::spatial::L1);
  EXPECT_EQ(res4.stable_coarsen_count, 3U);  // Saturated
  EXPECT_TRUE(res4.already_pending);
}

// ============================================================================
// Section 5: Acknowledgement API Contracts (APPLIED, BLOCKED, CANCELLED)
// ============================================================================

TEST(ResolutionHysteresisTest, AcknowledgeAppliedErasesChildState) {
  auto ctrl = *ResolutionHysteresisController::create(ResolutionHysteresisConfig{.coarsen_stable_observations = 2});
  const auto child_key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L0);

  ctrl.observe(child_key, 10, makeDecision(avr::spatial::L1));
  auto res = ctrl.observe(child_key, 20, makeDecision(avr::spatial::L1));
  EXPECT_EQ(res.action, HysteresisAction::COARSEN_READY);
  EXPECT_TRUE(ctrl.containsRegion(child_key));

  // Acknowledge APPLIED
  EXPECT_TRUE(ctrl.acknowledgeCoarsenResult(child_key, CoarsenAttemptOutcome::APPLIED));
  EXPECT_FALSE(ctrl.containsRegion(child_key));

  // Future parent starts completely fresh
  const auto parent_key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L1);
  EXPECT_FALSE(ctrl.containsRegion(parent_key));
}

TEST(ResolutionHysteresisTest, AcknowledgeBlockedHoldFineResetsStreak) {
  auto ctrl = *ResolutionHysteresisController::create(ResolutionHysteresisConfig{.coarsen_stable_observations = 3});
  const auto key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L1);

  ctrl.observe(key, 10, makeDecision(avr::spatial::L2));
  ctrl.observe(key, 20, makeDecision(avr::spatial::L2));
  auto res = ctrl.observe(key, 30, makeDecision(avr::spatial::L2));
  EXPECT_EQ(res.action, HysteresisAction::COARSEN_READY);

  // Acknowledge BLOCKED_HOLD_FINE
  EXPECT_TRUE(ctrl.acknowledgeCoarsenResult(key, CoarsenAttemptOutcome::BLOCKED_HOLD_FINE));

  auto state = ctrl.getState(key);
  ASSERT_TRUE(state.has_value());
  EXPECT_FALSE(state->coarsen_action_pending);
  EXPECT_EQ(state->stable_coarsen_count, 0U);
  EXPECT_TRUE(state->has_last_frame);
  EXPECT_EQ(state->last_frame_seq, 30U);

  // Duplicate frame 30 must still reject!
  EXPECT_EQ(ctrl.observe(key, 30, makeDecision(avr::spatial::L2)).action, HysteresisAction::INVALID_OBSERVATION);

  // Requires 3 brand new stable observations before emitting COARSEN_READY again
  EXPECT_EQ(ctrl.observe(key, 40, makeDecision(avr::spatial::L2)).action, HysteresisAction::COARSEN_PENDING);
  EXPECT_EQ(ctrl.observe(key, 50, makeDecision(avr::spatial::L2)).action, HysteresisAction::COARSEN_PENDING);
  EXPECT_EQ(ctrl.observe(key, 60, makeDecision(avr::spatial::L2)).action, HysteresisAction::COARSEN_READY);
}

TEST(ResolutionHysteresisTest, AcknowledgeCancelledResetsStreak) {
  auto ctrl = *ResolutionHysteresisController::create(ResolutionHysteresisConfig{.coarsen_stable_observations = 2});
  const auto key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L0);

  ctrl.observe(key, 10, makeDecision(avr::spatial::L1));
  ctrl.observe(key, 20, makeDecision(avr::spatial::L1));
  EXPECT_TRUE(ctrl.getState(key)->coarsen_action_pending);

  EXPECT_TRUE(ctrl.acknowledgeCoarsenResult(key, CoarsenAttemptOutcome::CANCELLED));
  auto state = ctrl.getState(key);
  ASSERT_TRUE(state.has_value());
  EXPECT_FALSE(state->coarsen_action_pending);
  EXPECT_EQ(state->stable_coarsen_count, 0U);
  EXPECT_TRUE(state->has_last_frame);
  EXPECT_EQ(state->last_frame_seq, 20U);

  // Duplicate frame 20 rejected
  EXPECT_EQ(ctrl.observe(key, 20, makeDecision(avr::spatial::L1)).action, HysteresisAction::INVALID_OBSERVATION);
}

TEST(ResolutionHysteresisTest, AcknowledgementFallibilityAndNonPendingRejection) {
  auto ctrl = *ResolutionHysteresisController::create(ResolutionHysteresisConfig{.coarsen_stable_observations = 5});
  const auto key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L0);
  const auto unknown_key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{100, 100}, avr::spatial::L0);

  // Unknown region returns false, zero insertion
  EXPECT_FALSE(ctrl.acknowledgeCoarsenResult(unknown_key, CoarsenAttemptOutcome::APPLIED));
  EXPECT_FALSE(ctrl.acknowledgeCoarsenResult(unknown_key, CoarsenAttemptOutcome::BLOCKED_HOLD_FINE));
  EXPECT_FALSE(ctrl.acknowledgeCoarsenResult(unknown_key, CoarsenAttemptOutcome::CANCELLED));
  EXPECT_EQ(ctrl.trackedRegionCount(), 0U);

  // Region in HOLD_CURRENT (non-pending)
  ctrl.observe(key, 10, makeDecision(avr::spatial::L0));
  EXPECT_FALSE(ctrl.acknowledgeCoarsenResult(key, CoarsenAttemptOutcome::APPLIED));
  EXPECT_FALSE(ctrl.acknowledgeCoarsenResult(key, CoarsenAttemptOutcome::BLOCKED_HOLD_FINE));
  EXPECT_FALSE(ctrl.acknowledgeCoarsenResult(key, CoarsenAttemptOutcome::CANCELLED));
  EXPECT_TRUE(ctrl.containsRegion(key));
  EXPECT_EQ(ctrl.getState(key)->stable_coarsen_count, 0U);

  // Region in COARSEN_PENDING below threshold
  ctrl.observe(key, 20, makeDecision(avr::spatial::L1));  // count = 1 < 5
  EXPECT_FALSE(ctrl.getState(key)->coarsen_action_pending);
  EXPECT_FALSE(ctrl.acknowledgeCoarsenResult(key, CoarsenAttemptOutcome::APPLIED));
  EXPECT_FALSE(ctrl.acknowledgeCoarsenResult(key, CoarsenAttemptOutcome::BLOCKED_HOLD_FINE));
  EXPECT_FALSE(ctrl.acknowledgeCoarsenResult(key, CoarsenAttemptOutcome::CANCELLED));
  EXPECT_EQ(ctrl.getState(key)->stable_coarsen_count, 1U);
}

// ============================================================================
// Section 6: Pending Action Veto Rules (Finer and Equal)
// ============================================================================

TEST(ResolutionHysteresisTest, PendingFinerVeto) {
  auto ctrl = *ResolutionHysteresisController::create(ResolutionHysteresisConfig{.coarsen_stable_observations = 2});
  const auto key_l1 = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L1);

  ctrl.observe(key_l1, 10, makeDecision(avr::spatial::L2));
  auto res_ready = ctrl.observe(key_l1, 20, makeDecision(avr::spatial::L2));
  EXPECT_EQ(res_ready.action, HysteresisAction::COARSEN_READY);

  // Before acknowledgement, a finer request arrives (L0)
  auto res_finer = ctrl.observe(key_l1, 30, makeDecision(avr::spatial::L0));
  EXPECT_EQ(res_finer.action, HysteresisAction::REFINE_NOW);
  EXPECT_EQ(res_finer.target_level, avr::spatial::L0);
  EXPECT_EQ(res_finer.stable_coarsen_count, 0U);
  EXPECT_FALSE(res_finer.already_pending);

  auto state = ctrl.getState(key_l1);
  ASSERT_TRUE(state.has_value());
  EXPECT_FALSE(state->coarsen_action_pending);
  EXPECT_EQ(state->stable_coarsen_count, 0U);
  EXPECT_EQ(state->last_frame_seq, 30U);
}

TEST(ResolutionHysteresisTest, PendingEqualVeto) {
  auto ctrl = *ResolutionHysteresisController::create(ResolutionHysteresisConfig{.coarsen_stable_observations = 2});
  const auto key_l1 = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L1);

  ctrl.observe(key_l1, 10, makeDecision(avr::spatial::L2));
  ctrl.observe(key_l1, 20, makeDecision(avr::spatial::L2));

  // Before acknowledgement, an equal request arrives (L1)
  auto res_equal = ctrl.observe(key_l1, 30, makeDecision(avr::spatial::L1));
  EXPECT_EQ(res_equal.action, HysteresisAction::HOLD_CURRENT);
  EXPECT_EQ(res_equal.target_level, avr::spatial::L1);
  EXPECT_EQ(res_equal.stable_coarsen_count, 0U);
  EXPECT_FALSE(res_equal.already_pending);

  auto state = ctrl.getState(key_l1);
  ASSERT_TRUE(state.has_value());
  EXPECT_FALSE(state->coarsen_action_pending);
  EXPECT_EQ(state->stable_coarsen_count, 0U);
  EXPECT_EQ(state->last_frame_seq, 30U);
}

// ============================================================================
// Section 7: Reason Policy
// ============================================================================

TEST(ResolutionHysteresisTest, ReasonPolicyLatestContextOnly) {
  auto ctrl = *ResolutionHysteresisController::create(ResolutionHysteresisConfig{.coarsen_stable_observations = 3});
  const auto key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L0);

  ResolutionReasonBits r1{};
  r1.set(ResolutionReason::DISTANCE_BASELINE);

  ResolutionReasonBits r2{};
  r2.set(ResolutionReason::UNCERTAINTY);

  ResolutionReasonBits r3{};
  r3.set(ResolutionReason::GEOMETRY_COMPLEXITY);

  ctrl.observe(key, 10, makeDecision(avr::spatial::L1, r1));
  ctrl.observe(key, 20, makeDecision(avr::spatial::L1, r2));
  auto res = ctrl.observe(key, 30, makeDecision(avr::spatial::L1, r3));

  // Returned trigger reasons must match MOST RECENT decision (r3), NOT union of r1, r2, r3!
  EXPECT_EQ(res.trigger_reasons, r3);
  EXPECT_FALSE(res.trigger_reasons.has(ResolutionReason::DISTANCE_BASELINE));
  EXPECT_FALSE(res.trigger_reasons.has(ResolutionReason::UNCERTAINTY));
  EXPECT_TRUE(res.trigger_reasons.has(ResolutionReason::GEOMETRY_COMPLEXITY));
}

// ============================================================================
// Section 8: State Lifecycle APIs & Deterministic Snapshot
// ============================================================================

TEST(ResolutionHysteresisTest, LifecycleEraseAndClear) {
  auto ctrl = *ResolutionHysteresisController::create();
  const auto k1 = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L0);
  const auto k2 = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{10, 10}, avr::spatial::L1);

  ctrl.observe(k1, 10, makeDecision(avr::spatial::L1));
  ctrl.observe(k2, 10, makeDecision(avr::spatial::L2));
  EXPECT_EQ(ctrl.trackedRegionCount(), 2U);
  EXPECT_TRUE(ctrl.containsRegion(k1));
  EXPECT_TRUE(ctrl.containsRegion(k2));

  // Erase k1
  EXPECT_TRUE(ctrl.eraseRegion(k1));
  EXPECT_FALSE(ctrl.containsRegion(k1));
  EXPECT_TRUE(ctrl.containsRegion(k2));
  EXPECT_EQ(ctrl.trackedRegionCount(), 1U);
  EXPECT_FALSE(ctrl.eraseRegion(k1));  // Already erased

  // Clear all
  ctrl.clear();
  EXPECT_EQ(ctrl.trackedRegionCount(), 0U);
  EXPECT_FALSE(ctrl.containsRegion(k2));
}

TEST(ResolutionHysteresisTest, ClearConfigPreservation) {
  auto ctrl = *ResolutionHysteresisController::create(ResolutionHysteresisConfig{.coarsen_stable_observations = 7});
  const auto k1 = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L0);

  ctrl.observe(k1, 10, makeDecision(avr::spatial::L1));
  EXPECT_EQ(ctrl.trackedRegionCount(), 1U);

  ctrl.clear();
  EXPECT_EQ(ctrl.trackedRegionCount(), 0U);
  EXPECT_TRUE(ctrl.stateSnapshot().empty());
  EXPECT_EQ(ctrl.config().coarsen_stable_observations, 7U);  // Config preserved!

  // New observations obey preserved config threshold 7
  for (uint64_t f = 1; f < 7; ++f) {
    auto res = ctrl.observe(k1, f * 10, makeDecision(avr::spatial::L1));
    EXPECT_EQ(res.action, HysteresisAction::COARSEN_PENDING);
    EXPECT_EQ(res.stable_coarsen_count, static_cast<uint32_t>(f));
  }
  auto res_ready = ctrl.observe(k1, 70, makeDecision(avr::spatial::L1));
  EXPECT_EQ(res_ready.action, HysteresisAction::COARSEN_READY);
  EXPECT_EQ(res_ready.stable_coarsen_count, 7U);
}

TEST(ResolutionHysteresisTest, DeterministicSortedSnapshot) {
  auto ctrl = *ResolutionHysteresisController::create();
  const auto k_high = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{50, 50}, avr::spatial::L2);
  const auto k_mid = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{10, 10}, avr::spatial::L1);
  const auto k_low = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{-10, -10}, avr::spatial::L0);

  // Insert in arbitrary order
  ctrl.observe(k_high, 10, makeDecision(avr::spatial::L3));
  ctrl.observe(k_low, 10, makeDecision(avr::spatial::L1));
  ctrl.observe(k_mid, 10, makeDecision(avr::spatial::L2));

  const auto snap = ctrl.stateSnapshot();
  ASSERT_EQ(snap.size(), 3U);

  // Must be strictly sorted by CellKey
  EXPECT_TRUE(snap[0].region < snap[1].region);
  EXPECT_TRUE(snap[1].region < snap[2].region);

  // Snapshot exposes debug state
  EXPECT_TRUE(snap[0].has_last_frame);
  EXPECT_EQ(snap[0].last_frame_seq, 10U);
}

// ============================================================================
// Section 9: Frame Zero & High Value Boundary Tests
// ============================================================================

TEST(ResolutionHysteresisTest, FrameZeroAcceptedAsLegalFirstFrame) {
  auto ctrl = *ResolutionHysteresisController::create(ResolutionHysteresisConfig{.coarsen_stable_observations = 2});
  const auto key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L0);

  // Fresh region submitting frame_seq = 0
  const auto res1 = ctrl.observe(key, 0, makeDecision(avr::spatial::L1));
  EXPECT_EQ(res1.action, HysteresisAction::COARSEN_PENDING);
  EXPECT_EQ(res1.stable_coarsen_count, 1U);

  const auto state1 = ctrl.getState(key);
  ASSERT_TRUE(state1.has_value());
  EXPECT_TRUE(state1->has_last_frame);
  EXPECT_EQ(state1->last_frame_seq, 0U);

  // Duplicate frame 0 rejected with zero state mutation
  const auto res_dup = ctrl.observe(key, 0, makeDecision(avr::spatial::L1));
  EXPECT_EQ(res_dup.action, HysteresisAction::INVALID_OBSERVATION);
  EXPECT_EQ(ctrl.getState(key)->stable_coarsen_count, 1U);

  // Next frame 1 accepted
  const auto res2 = ctrl.observe(key, 1, makeDecision(avr::spatial::L1));
  EXPECT_EQ(res2.action, HysteresisAction::COARSEN_READY);
  EXPECT_EQ(res2.stable_coarsen_count, 2U);
  EXPECT_EQ(ctrl.getState(key)->last_frame_seq, 1U);
}

TEST(ResolutionHysteresisTest, Uint64FrameBoundaryTest) {
  auto ctrl = *ResolutionHysteresisController::create(ResolutionHysteresisConfig{.coarsen_stable_observations = 2});
  const auto key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L0);

  // UINT64_MAX - 1
  constexpr uint64_t kHigh1 = UINT64_MAX - 1;
  const auto res1 = ctrl.observe(key, kHigh1, makeDecision(avr::spatial::L1));
  EXPECT_EQ(res1.action, HysteresisAction::COARSEN_PENDING);
  EXPECT_EQ(ctrl.getState(key)->last_frame_seq, kHigh1);

  // UINT64_MAX
  constexpr uint64_t kHigh2 = UINT64_MAX;
  const auto res2 = ctrl.observe(key, kHigh2, makeDecision(avr::spatial::L1));
  EXPECT_EQ(res2.action, HysteresisAction::COARSEN_READY);
  EXPECT_EQ(ctrl.getState(key)->last_frame_seq, kHigh2);

  // Duplicate UINT64_MAX rejected
  const auto res_dup = ctrl.observe(key, kHigh2, makeDecision(avr::spatial::L1));
  EXPECT_EQ(res_dup.action, HysteresisAction::INVALID_OBSERVATION);
  EXPECT_EQ(ctrl.getState(key)->last_frame_seq, kHigh2);
}

// ============================================================================
// Section 10: Read-Only Lookups and Result Target Contract
// ============================================================================

TEST(ResolutionHysteresisTest, ReadOnlyRegionStateLookup) {
  auto ctrl = *ResolutionHysteresisController::create(ResolutionHysteresisConfig{.coarsen_stable_observations = 2});
  const auto key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L1);
  const auto unknown_key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{99, 99}, avr::spatial::L1);

  // Unknown region lookup
  EXPECT_EQ(ctrl.findRegionState(unknown_key), nullptr);
  EXPECT_FALSE(ctrl.regionState(unknown_key).has_value());
  EXPECT_FALSE(ctrl.getState(unknown_key).has_value());
  EXPECT_EQ(ctrl.trackedRegionCount(), 0U);

  // Observe region
  ctrl.observe(key, 10, makeDecision(avr::spatial::L2));

  // Known region pointer lookup
  const auto* ptr = ctrl.findRegionState(key);
  ASSERT_NE(ptr, nullptr);
  EXPECT_TRUE(ptr->has_last_frame);
  EXPECT_EQ(ptr->last_frame_seq, 10U);
  EXPECT_EQ(ptr->stable_coarsen_count, 1U);
  EXPECT_FALSE(ptr->coarsen_action_pending);

  // Known region snapshot lookup
  const auto snap_opt = ctrl.regionState(key);
  ASSERT_TRUE(snap_opt.has_value());
  EXPECT_EQ(snap_opt->region, key);
  EXPECT_EQ(snap_opt->last_frame_seq, 10U);
  EXPECT_EQ(snap_opt->stable_coarsen_count, 1U);
}

TEST(ResolutionHysteresisTest, ResultTargetContract) {
  auto ctrl = *ResolutionHysteresisController::create(ResolutionHysteresisConfig{.coarsen_stable_observations = 2});
  const auto key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L1);

  // HOLD_CURRENT: target == current region level (L1)
  const auto res_hold = ctrl.observe(key, 10, makeDecision(avr::spatial::L1));
  EXPECT_EQ(res_hold.action, HysteresisAction::HOLD_CURRENT);
  EXPECT_EQ(res_hold.target_level, avr::spatial::L1);

  // REFINE_NOW: target == requested level (L0)
  const auto res_refine = ctrl.observe(key, 20, makeDecision(avr::spatial::L0));
  EXPECT_EQ(res_refine.action, HysteresisAction::REFINE_NOW);
  EXPECT_EQ(res_refine.target_level, avr::spatial::L0);

  // COARSEN_PENDING: target == current + 1 (L2)
  const auto res_pend = ctrl.observe(key, 30, makeDecision(avr::spatial::L3));
  EXPECT_EQ(res_pend.action, HysteresisAction::COARSEN_PENDING);
  EXPECT_EQ(res_pend.target_level, avr::spatial::L2);

  // COARSEN_READY: target == current + 1 (L2)
  const auto res_ready = ctrl.observe(key, 40, makeDecision(avr::spatial::L3));
  EXPECT_EQ(res_ready.action, HysteresisAction::COARSEN_READY);
  EXPECT_EQ(res_ready.target_level, avr::spatial::L2);

  // COARSEN_READY_PENDING_ACK: target == existing pending target (L2)
  const auto res_pend_ack = ctrl.observe(key, 50, makeDecision(avr::spatial::L3));
  EXPECT_EQ(res_pend_ack.action, HysteresisAction::COARSEN_READY_PENDING_ACK);
  EXPECT_EQ(res_pend_ack.target_level, avr::spatial::L2);

  // INVALID_OBSERVATION: target is documented non-actionable (0)
  const auto res_inv = ctrl.observe(key, 50, makeDecision(avr::spatial::L3));  // duplicate frame 50
  EXPECT_EQ(res_inv.action, HysteresisAction::INVALID_OBSERVATION);
  EXPECT_EQ(res_inv.target_level, 0U);
}

// ============================================================================
// Section 11: Threshold = 1 Generality Test
// ============================================================================

TEST(ResolutionHysteresisTest, ThresholdOneImmediateCoarsenReady) {
  auto ctrl = *ResolutionHysteresisController::create(ResolutionHysteresisConfig{.coarsen_stable_observations = 1});
  const auto key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L1);

  // First coarser observation immediately reaches COARSEN_READY
  const auto res = ctrl.observe(key, 10, makeDecision(avr::spatial::L2));
  EXPECT_EQ(res.action, HysteresisAction::COARSEN_READY);
  EXPECT_EQ(res.stable_coarsen_count, 1U);
  EXPECT_EQ(res.target_level, avr::spatial::L2);
  EXPECT_FALSE(res.already_pending);
}

// ============================================================================
// Section 12: Negative Coordinates and Same Morton Different Level
// ============================================================================

TEST(ResolutionHysteresisTest, NegativeCoordinateKeys) {
  auto ctrl = *ResolutionHysteresisController::create(ResolutionHysteresisConfig{.coarsen_stable_observations = 2});
  const auto neg_key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{-25, -40}, avr::spatial::L1);

  auto res1 = ctrl.observe(neg_key, 10, makeDecision(avr::spatial::L2));
  EXPECT_EQ(res1.action, HysteresisAction::COARSEN_PENDING);

  auto res2 = ctrl.observe(neg_key, 20, makeDecision(avr::spatial::L2));
  EXPECT_EQ(res2.action, HysteresisAction::COARSEN_READY);
  EXPECT_TRUE(ctrl.containsRegion(neg_key));
}

TEST(ResolutionHysteresisTest, SameMortonDifferentLevelIndependent) {
  auto ctrl = *ResolutionHysteresisController::create(ResolutionHysteresisConfig{.coarsen_stable_observations = 2});

  const auto key_l0 = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L0);
  const auto key_l1 = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L1);

  // Observe key_l0 with coarser requests (L1)
  ctrl.observe(key_l0, 10, makeDecision(avr::spatial::L1));
  ctrl.observe(key_l0, 20, makeDecision(avr::spatial::L1));

  // Observe key_l1 with finer request (L0)
  ctrl.observe(key_l1, 10, makeDecision(avr::spatial::L0));

  EXPECT_EQ(ctrl.trackedRegionCount(), 2U);
  EXPECT_TRUE(ctrl.getState(key_l0)->coarsen_action_pending);
  EXPECT_FALSE(ctrl.getState(key_l1)->coarsen_action_pending);
  EXPECT_EQ(ctrl.getState(key_l1)->stable_coarsen_count, 0U);
}

// ============================================================================
// Section 13: Invalid Level Protection
// ============================================================================

TEST(ResolutionHysteresisTest, InvalidLevelRejectedWithZeroMutation) {
  auto ctrl = *ResolutionHysteresisController::create();

  // Invalid region level (4 is out of [0, 3])
  const avr::spatial::CellKey invalid_region{.morton = 0, .level = 4};
  const auto res1 = ctrl.observe(invalid_region, 10, makeDecision(avr::spatial::L2));
  EXPECT_EQ(res1.action, HysteresisAction::INVALID_OBSERVATION);
  EXPECT_FALSE(res1.isSuccess());
  EXPECT_EQ(ctrl.trackedRegionCount(), 0U);
  EXPECT_TRUE(ctrl.stateSnapshot().empty());

  // Valid region but invalid requested level (99)
  const auto valid_region = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L0);
  const auto res2 = ctrl.observe(valid_region, 10, makeDecision(99));
  EXPECT_EQ(res2.action, HysteresisAction::INVALID_OBSERVATION);
  EXPECT_FALSE(res2.isSuccess());
  EXPECT_EQ(ctrl.trackedRegionCount(), 0U);
  EXPECT_FALSE(ctrl.containsRegion(valid_region));
  EXPECT_TRUE(ctrl.stateSnapshot().empty());

  // Valid region, valid requested, but invalid baseline level (88)
  ResolutionDecision malformed_dec{};
  malformed_dec.requested_level = avr::spatial::L1;
  malformed_dec.baseline_level = 88;
  const auto res3 = ctrl.observe(valid_region, 10, malformed_dec);
  EXPECT_EQ(res3.action, HysteresisAction::INVALID_OBSERVATION);
  EXPECT_FALSE(res3.isSuccess());
  EXPECT_EQ(ctrl.trackedRegionCount(), 0U);
  EXPECT_FALSE(ctrl.containsRegion(valid_region));
}

// ============================================================================
// Section 14: Memory Layout and Sizes
// ============================================================================

TEST(ResolutionHysteresisTest, MemoryLayoutAndSizes) {
  EXPECT_EQ(sizeof(ResolutionHysteresisConfig), 4U);
  EXPECT_EQ(sizeof(RegionHysteresisState), 24U);
  EXPECT_EQ(alignof(RegionHysteresisState), 8U);
  EXPECT_EQ(sizeof(HysteresisResult), 32U);
}

}  // namespace
}  // namespace avr::mapping

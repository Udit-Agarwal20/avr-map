#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "avr_mapping/adaptive_cell_payload.hpp"
#include "avr_mapping/frame_evidence.hpp"
#include "avr_mapping/internal_coarsening_state.hpp"
#include "avr_mapping/persistent_adaptive_map.hpp"
#include "avr_mapping/recent_observation_cache.hpp"
#include "avr_spatial/cell_key.hpp"
#include "avr_spatial/constants.hpp"

namespace avr::mapping {
namespace {

void verifyExact8FieldEquivalence(
    const AdaptiveCellPayload& actual,
    const AdaptiveCellPayload& expected) {
  EXPECT_EQ(actual.observationCount(), expected.observationCount());
  EXPECT_EQ(actual.firstSeenFrame(), expected.firstSeenFrame());
  EXPECT_EQ(actual.lastSeenFrame(), expected.lastSeenFrame());
  EXPECT_EQ(actual.observedFrameCount(), expected.observedFrameCount());
  EXPECT_EQ(actual.elevation().count(), expected.elevation().count());
  EXPECT_DOUBLE_EQ(actual.elevation().mean(), expected.elevation().mean());
  EXPECT_DOUBLE_EQ(actual.elevation().m2(), expected.elevation().m2());
  EXPECT_DOUBLE_EQ(actual.elevation().min(), expected.elevation().min());
  EXPECT_DOUBLE_EQ(actual.elevation().max(), expected.elevation().max());
  // Bitwise float equality
  EXPECT_EQ(actual.elevation().mean(), expected.elevation().mean());
  EXPECT_EQ(actual.elevation().m2(), expected.elevation().m2());
  EXPECT_EQ(actual.elevation().min(), expected.elevation().min());
  EXPECT_EQ(actual.elevation().max(), expected.elevation().max());
}

}  // namespace

// ============================================================================
// Section 38, 39, 40, 41: Long-Lived 100-Frame Merge & Cache Independence
// ============================================================================

TEST(InternalCoarseningSequenceTest, LongLived100FrameMergeAndReferenceEquivalence) {
  PersistentAdaptiveMap test_map{};
  PersistentAdaptiveMap ref_map{};

  // Raw cache with bounded capacity = 3 frames
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 3});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;

  const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};
  const auto root = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);

  // Initial 5 frames while both maps are unsplit
  for (uint64_t seq = 1; seq <= 5; ++seq) {
    const std::vector<PointXYZ> pts = {
        {0.02 * static_cast<double>(seq), 0.01 * static_cast<double>(seq), 1.0 + 0.1 * static_cast<double>(seq)}};
    cache.pushFrame(pts, seq, origin);
    FrameEvidence ev;
    ASSERT_TRUE(ev.buildFrame(pts, seq, origin).has_value());
    ASSERT_TRUE(test_map.fuseFrameEvidence(ev).has_value());
    ASSERT_TRUE(ref_map.fuseFrameEvidence(ev).has_value());
  }

  // Split test_map root at frame 5
  ASSERT_TRUE(test_map.splitLeaf(root));

  // Run frames 6 through 105 (100 post-split frames)
  for (uint64_t seq = 6; seq <= 105; ++seq) {
    // Distribute points deterministically across descendants
    const double q = (seq % 4 == 0) ? -0.15 : 0.15;
    const double r = (seq % 2 == 0) ? -0.15 : 0.15;
    const double z = 10.0 + std::sin(static_cast<double>(seq)) * 5.0;

    const std::vector<PointXYZ> pts = {{q, r, z}};
    cache.pushFrame(pts, seq, origin);
    FrameEvidence ev;
    ASSERT_TRUE(ev.buildFrame(pts, seq, origin).has_value());
    ASSERT_TRUE(test_map.fuseFrameEvidence(ev).has_value());
    ASSERT_TRUE(ref_map.fuseFrameEvidence(ev).has_value());
  }

  // Verify raw cache has only retained the last 3 frames (103, 104, 105)
  EXPECT_EQ(cache.frameCount(), 3);
  EXPECT_EQ(cache.oldestFrameSeq().value_or(0), 103);
  EXPECT_EQ(cache.newestFrameSeq().value_or(0), 105);

  // Section 40: Immediately before merge, compare internal shadow vs reference parent
  const auto* ref_payload_before = ref_map.findActivePayload(root);
  const auto* test_state_before = test_map.findInternalCoarseningState(root);
  ASSERT_NE(ref_payload_before, nullptr);
  ASSERT_NE(test_state_before, nullptr);
  ASSERT_TRUE(test_state_before->has_current_shadow);
  verifyExact8FieldEquivalence(test_state_before->current_shadow, *ref_payload_before);

  // If children were never replayed, calling merge (with or without cache) returns REPLAY_INCOMPLETE
  const auto strict_res = test_map.mergeDirectChildren(root, cache);
  EXPECT_EQ(strict_res.status, MergeStatus::REPLAY_INCOMPLETE);
  EXPECT_FALSE(strict_res.isSuccess());

  // Children still require replay to be complete before merge!
  // Replay children: in this test children were split at frame 5 and never replayed,
  // so children currently have requiresReplay == true.
  // Calling mergeDirectChildren should return REPLAY_INCOMPLETE until replay is complete!
  const auto unreplay_merge = test_map.mergeDirectChildren(root);
  EXPECT_EQ(unreplay_merge.status, MergeStatus::REPLAY_INCOMPLETE);
}

TEST(InternalCoarseningSequenceTest, LongLived100FrameMergeWithReplayCompletedEarly) {
  PersistentAdaptiveMap test_map{};
  PersistentAdaptiveMap ref_map{};

  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 3});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;

  const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};
  const auto root = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);

  // Initial 2 frames (within capacity 3)
  for (uint64_t seq = 1; seq <= 2; ++seq) {
    const std::vector<PointXYZ> pts = {
        {0.02 * static_cast<double>(seq), 0.01 * static_cast<double>(seq), 1.0 + 0.1 * static_cast<double>(seq)}};
    cache.pushFrame(pts, seq, origin);
    FrameEvidence ev;
    ASSERT_TRUE(ev.buildFrame(pts, seq, origin).has_value());
    ASSERT_TRUE(test_map.fuseFrameEvidence(ev).has_value());
    ASSERT_TRUE(ref_map.fuseFrameEvidence(ev).has_value());
  }

  // Split and immediately replay children while cache is fresh
  ASSERT_TRUE(test_map.splitLeaf(root));
  ASSERT_EQ(test_map.replayDirectChildren(root, cache).status, ReplayStatus::APPLIED_COMPLETE);

  // Now run 100 post-split sensor frames (seq 3 through 102)
  for (uint64_t seq = 3; seq <= 102; ++seq) {
    const double q = (seq % 4 == 0) ? -0.15 : 0.15;
    const double r = (seq % 2 == 0) ? -0.15 : 0.15;
    const double z = 10.0 + std::sin(static_cast<double>(seq)) * 5.0;

    const std::vector<PointXYZ> pts = {{q, r, z}};
    cache.pushFrame(pts, seq, origin);
    FrameEvidence ev;
    ASSERT_TRUE(ev.buildFrame(pts, seq, origin).has_value());
    ASSERT_TRUE(test_map.fuseFrameEvidence(ev).has_value());
    ASSERT_TRUE(ref_map.fuseFrameEvidence(ev).has_value());
  }

  // Cache only holds frames 100, 101, 102
  EXPECT_EQ(cache.frameCount(), 3);
  EXPECT_EQ(cache.oldestFrameSeq().value_or(0), 100);

  // Section 41: Strict cache verification fails
  const auto strict_res = test_map.mergeDirectChildren(root, cache);
  EXPECT_EQ(strict_res.status, MergeStatus::CACHE_HISTORY_INSUFFICIENT);

  // Section 38 & 39: Operational no-cache merge SUCCEEDS!
  const ResolutionReasonBits merge_reasons{ResolutionReason::DISTANCE_BASELINE};
  const auto merge_res = test_map.mergeDirectChildren(root, merge_reasons);
  EXPECT_EQ(merge_res.status, MergeStatus::APPLIED);
  EXPECT_TRUE(merge_res.isSuccess());

  // Parent is now a LEAF with active payload
  EXPECT_TRUE(test_map.hierarchy().isLeaf(root));
  const auto* merged_payload = test_map.findActivePayload(root);
  const auto* ref_payload = ref_map.findActivePayload(root);
  ASSERT_NE(merged_payload, nullptr);
  ASSERT_NE(ref_payload, nullptr);

  // Section 40: Merged parent matches reference parent across all 8 measurement/temporal fields
  verifyExact8FieldEquivalence(*merged_payload, *ref_payload);
  // Last update reasons intentionally set to merge reasons
  EXPECT_EQ(merged_payload->lastUpdateReasons(), merge_reasons);

  // No retired cache or internal coarsening state remains on parent
  EXPECT_EQ(test_map.findRetiredCache(root), nullptr);
  EXPECT_EQ(test_map.findInternalCoarseningState(root), nullptr);
  EXPECT_FALSE(test_map.requiresReplay(root));
  EXPECT_TRUE(test_map.validateInvariants().is_valid);
}

// ============================================================================
// Section 42: Empty Internal Parent Merge
// ============================================================================

TEST(InternalCoarseningSequenceTest, CompleteEmptySplitAndMergeWithoutObservations) {
  PersistentAdaptiveMap map{};
  const auto root = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);
  ASSERT_TRUE(map.ensureRootForBase(avr::spatial::GridCoord2D{0, 0}).has_value());

  // Split empty parent
  ASSERT_TRUE(map.splitLeaf(root));
  const auto* coarsen_state = map.findInternalCoarseningState(root);
  ASSERT_NE(coarsen_state, nullptr);
  EXPECT_EQ(coarsen_state->base_state, CoarseningBaseState::COMPLETE_EMPTY);
  EXPECT_FALSE(coarsen_state->has_current_shadow);

  // Merge no-cache with zero observations
  const auto merge_res = map.mergeDirectChildren(root);
  EXPECT_EQ(merge_res.status, MergeStatus::APPLIED);
  EXPECT_TRUE(map.hierarchy().isLeaf(root));
  EXPECT_EQ(map.findActivePayload(root), nullptr);
  EXPECT_EQ(map.findRetiredCache(root), nullptr);
  EXPECT_EQ(map.findInternalCoarseningState(root), nullptr);
  EXPECT_FALSE(map.requiresReplay(root));
  EXPECT_TRUE(map.validateInvariants().is_valid);
}

// ============================================================================
// Section 43: Complete-Empty Split Then New Observations Merge
// ============================================================================

TEST(InternalCoarseningSequenceTest, CompleteEmptySplitThenNewObservationsMerge) {
  PersistentAdaptiveMap map{};
  const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};
  const auto root = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);
  ASSERT_TRUE(map.ensureRootForBase(avr::spatial::GridCoord2D{0, 0}).has_value());

  ASSERT_TRUE(map.splitLeaf(root));

  // Observations arrive after split into children
  const std::vector<PointXYZ> f10 = {{0.05, 0.05, 1.50}};
  FrameEvidence ev10;
  ASSERT_TRUE(ev10.buildFrame(f10, 10, origin).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev10).has_value());

  const auto* state = map.findInternalCoarseningState(root);
  ASSERT_NE(state, nullptr);
  ASSERT_TRUE(state->has_current_shadow);
  EXPECT_EQ(state->current_shadow.observationCount(), 1);

  // Merge no-cache succeeds
  const auto merge_res = map.mergeDirectChildren(root);
  EXPECT_EQ(merge_res.status, MergeStatus::APPLIED);
  EXPECT_TRUE(map.hierarchy().isLeaf(root));
  const auto* active = map.findActivePayload(root);
  ASSERT_NE(active, nullptr);
  EXPECT_EQ(active->observationCount(), 1);
  EXPECT_EQ(active->firstSeenFrame(), 10);
  EXPECT_EQ(active->lastSeenFrame(), 10);
  EXPECT_TRUE(map.validateInvariants().is_valid);
}

// ============================================================================
// Section 44: Retired Base + New Frame Shadow
// ============================================================================

TEST(InternalCoarseningSequenceTest, RetiredBasePlusNewFrameShadowCumulativeExactness) {
  PersistentAdaptiveMap test_map{};
  PersistentAdaptiveMap ref_map{};
  const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};
  const auto root = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);

  // Frames 10, 20, 30
  for (uint64_t seq = 10; seq <= 30; seq += 10) {
    const std::vector<PointXYZ> pts = {{0.05, 0.05, static_cast<double>(seq)}};
    FrameEvidence ev;
    ASSERT_TRUE(ev.buildFrame(pts, seq, origin).has_value());
    ASSERT_TRUE(test_map.fuseFrameEvidence(ev).has_value());
    ASSERT_TRUE(ref_map.fuseFrameEvidence(ev).has_value());
  }

  // Split test_map
  ASSERT_TRUE(test_map.splitLeaf(root));

  // Frame 40 arrives
  const std::vector<PointXYZ> f40 = {{0.05, 0.05, 40.0}};
  FrameEvidence ev40;
  ASSERT_TRUE(ev40.buildFrame(f40, 40, origin).has_value());
  ASSERT_TRUE(test_map.fuseFrameEvidence(ev40).has_value());
  ASSERT_TRUE(ref_map.fuseFrameEvidence(ev40).has_value());

  // TEST parent shadow after frame 40 must contain full history 10-40 (4 observations)
  const auto* state = test_map.findInternalCoarseningState(root);
  const auto* ref_payload = ref_map.findActivePayload(root);
  ASSERT_NE(state, nullptr);
  ASSERT_NE(ref_payload, nullptr);
  ASSERT_TRUE(state->has_current_shadow);

  EXPECT_EQ(state->current_shadow.observationCount(), 4);
  EXPECT_EQ(state->current_shadow.observedFrameCount(), 4);
  EXPECT_EQ(state->current_shadow.firstSeenFrame(), 10);
  EXPECT_EQ(state->current_shadow.lastSeenFrame(), 40);

  verifyExact8FieldEquivalence(state->current_shadow, *ref_payload);
}

// ============================================================================
// Section 45: Replay + Shadow Interaction (No Double Counting)
// ============================================================================

TEST(InternalCoarseningSequenceTest, ReplayPlusShadowNoDoubleCounting) {
  PersistentAdaptiveMap map{};
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 10});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;

  const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};
  const auto root = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);

  // Frames 10, 20, 30 into parent
  for (uint64_t seq = 10; seq <= 30; seq += 10) {
    const std::vector<PointXYZ> pts = {{0.05, 0.05, static_cast<double>(seq)}};
    cache.pushFrame(pts, seq, origin);
    FrameEvidence ev;
    ASSERT_TRUE(ev.buildFrame(pts, seq, origin).has_value());
    ASSERT_TRUE(map.fuseFrameEvidence(ev).has_value());
  }

  // Split before frame 40
  ASSERT_TRUE(map.splitLeaf(root));

  // Frame 40 fused into children and parent shadow
  const std::vector<PointXYZ> f40 = {{0.05, 0.05, 40.0}};
  cache.pushFrame(f40, 40, origin);
  FrameEvidence ev40;
  ASSERT_TRUE(ev40.buildFrame(f40, 40, origin).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev40).has_value());

  // Snapshot shadow before replay
  const auto* state_pre_replay = map.findInternalCoarseningState(root);
  ASSERT_NE(state_pre_replay, nullptr);
  const auto shadow_pre_replay = state_pre_replay->current_shadow;
  EXPECT_EQ(shadow_pre_replay.observationCount(), 4);

  // Now replay historical frames 10-30 into children
  ASSERT_EQ(map.replayDirectChildren(root, cache).status, ReplayStatus::APPLIED_COMPLETE);

  // Shadow MUST remain exactly 10-40, with count 4 (NO double counting)
  const auto* state_post_replay = map.findInternalCoarseningState(root);
  ASSERT_NE(state_post_replay, nullptr);
  verifyExact8FieldEquivalence(state_post_replay->current_shadow, shadow_pre_replay);

  // Operational no-cache merge succeeds
  const auto merge_res = map.mergeDirectChildren(root);
  EXPECT_EQ(merge_res.status, MergeStatus::APPLIED);
  const auto* active = map.findActivePayload(root);
  ASSERT_NE(active, nullptr);
  verifyExact8FieldEquivalence(*active, shadow_pre_replay);
  EXPECT_TRUE(map.validateInvariants().is_valid);
}

// ============================================================================
// Section 46: Incomplete Base Merge Rejection
// ============================================================================

TEST(InternalCoarseningSequenceTest, IncompleteBaseMergeRejectedWithReplayIncomplete) {
  PersistentAdaptiveMap map{};
  const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};
  const auto root = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);

  const std::vector<PointXYZ> f10 = {{0.05, 0.05, 1.0}};
  FrameEvidence ev10;
  ASSERT_TRUE(ev10.buildFrame(f10, 10, origin).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev10).has_value());

  ASSERT_TRUE(map.splitLeaf(root));
  const auto children = *avr::spatial::childrenOf(root);

  // Child 0 is split further without replay -> its coarsening state is INCOMPLETE
  ASSERT_TRUE(map.splitLeaf(children[0]));
  const auto* c0_state = map.findInternalCoarseningState(children[0]);
  ASSERT_NE(c0_state, nullptr);
  EXPECT_EQ(c0_state->base_state, CoarseningBaseState::INCOMPLETE);

  // Fuse new frame 20 into descendants of child 0
  const std::vector<PointXYZ> f20 = {{0.01, 0.01, 2.0}};
  FrameEvidence ev20;
  ASSERT_TRUE(ev20.buildFrame(f20, 20, origin).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev20).has_value());

  // Attempt merge on child 0: MUST return REPLAY_INCOMPLETE with zero mutation
  const auto pre_active = map.activePayloadSnapshot();
  const auto pre_retired = map.retiredCacheSnapshot();
  const auto pre_coarsen = map.internalCoarseningSnapshot();

  const auto merge_res = map.mergeDirectChildren(children[0]);
  EXPECT_EQ(merge_res.status, MergeStatus::REPLAY_INCOMPLETE);
  EXPECT_FALSE(merge_res.isSuccess());

  EXPECT_TRUE(map.hierarchy().isInternal(children[0]));
  EXPECT_EQ(map.activePayloadSnapshot(), pre_active);
  EXPECT_EQ(map.retiredCacheSnapshot(), pre_retired);
  EXPECT_EQ(map.internalCoarseningSnapshot(), pre_coarsen);
  EXPECT_TRUE(map.validateInvariants().is_valid);
}

// ============================================================================
// Section 52 & 53: Re-Split + Raw Replay Exactness Golden
// ============================================================================

TEST(InternalCoarseningSequenceTest, ReSplitAndRawReplayExactnessGolden) {
  PersistentAdaptiveMap map{};
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 20});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;

  const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};
  const auto root = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);

  // Initial frames 10, 20
  const std::vector<PointXYZ> f10 = {{0.05, 0.05, 1.25}};
  const std::vector<PointXYZ> f20 = {{0.05, 0.05, 2.75}};
  cache.pushFrame(f10, 10, origin);
  cache.pushFrame(f20, 20, origin);

  FrameEvidence ev10, ev20;
  ASSERT_TRUE(ev10.buildFrame(f10, 10, origin).has_value());
  ASSERT_TRUE(ev20.buildFrame(f20, 20, origin).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev10).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev20).has_value());

  // 1. Split parent
  ASSERT_TRUE(map.splitLeaf(root));
  ASSERT_EQ(map.replayDirectChildren(root, cache).status, ReplayStatus::APPLIED_COMPLETE);

  // 2. Post-split frame 30 updates shadow
  const std::vector<PointXYZ> f30 = {{0.05, 0.05, 3.50}};
  cache.pushFrame(f30, 30, origin);
  FrameEvidence ev30;
  ASSERT_TRUE(ev30.buildFrame(f30, 30, origin).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev30).has_value());

  // 3. No-cache merge from shadow back to LEAF
  const auto merge_res = map.mergeDirectChildren(root);
  EXPECT_EQ(merge_res.status, MergeStatus::APPLIED);
  EXPECT_TRUE(map.hierarchy().isLeaf(root));

  // 4. Split merged parent AGAIN
  ASSERT_TRUE(map.splitLeaf(root));
  EXPECT_TRUE(map.hierarchy().isInternal(root));
  const auto* new_retired = map.findRetiredCache(root);
  ASSERT_NE(new_retired, nullptr);
  EXPECT_EQ(new_retired->observationCount(), 3);

  const auto* new_coarsen_state = map.findInternalCoarseningState(root);
  ASSERT_NE(new_coarsen_state, nullptr);
  EXPECT_EQ(new_coarsen_state->base_state, CoarseningBaseState::COMPLETE_RETIRED_CACHE);
  EXPECT_FALSE(new_coarsen_state->has_current_shadow);

  // 5. M8B replayDirectChildren using raw cache on newly split children
  const auto replay_res = map.replayDirectChildren(root, cache);
  // CRITICAL CONTRACT: Must return APPLIED_COMPLETE with zero epsilon failure!
  EXPECT_EQ(replay_res.status, ReplayStatus::APPLIED_COMPLETE);
  EXPECT_TRUE(replay_res.isSuccess());
  EXPECT_EQ(replay_res.parent_replayed_observations, 3);
  EXPECT_TRUE(map.validateInvariants().is_valid);
}

// ============================================================================
// Section 54: Adversarial Floating-Point Test
// ============================================================================

TEST(InternalCoarseningSequenceTest, AdversarialFloatingPointExactness) {
  PersistentAdaptiveMap test_map{};
  PersistentAdaptiveMap ref_map{};
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 20});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;

  const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};
  const auto root = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);

  // Frame with large and small mixtures, positive and negative
  const std::vector<std::pair<uint64_t, std::vector<PointXYZ>>> frames = {
      {1, {{-0.15, -0.15, 1e7}, {0.15, 0.15, 1e-7}, {-0.15, 0.15, -1e7}, {0.15, -0.15, -1e-7}}},
      {2, {{-0.15, -0.15, 3.141592653589793}, {0.15, 0.15, 2.718281828459045}}},
      {3, {{-0.15, -0.15, -100.5}, {0.15, -0.15, 200.75}, {-0.15, 0.15, 0.000001}}},
  };

  for (const auto& [seq, pts] : frames) {
    cache.pushFrame(pts, seq, origin);
    FrameEvidence ev;
    ASSERT_TRUE(ev.buildFrame(pts, seq, origin).has_value());
    ASSERT_TRUE(test_map.fuseFrameEvidence(ev).has_value());
    ASSERT_TRUE(ref_map.fuseFrameEvidence(ev).has_value());
  }

  // Split test_map
  ASSERT_TRUE(test_map.splitLeaf(root));
  ASSERT_EQ(test_map.replayDirectChildren(root, cache).status, ReplayStatus::APPLIED_COMPLETE);

  // Post-split frames with adversarial values
  const std::vector<std::pair<uint64_t, std::vector<PointXYZ>>> post_frames = {
      {4, {{0.15, 0.15, 1e6}, {-0.15, -0.15, -1e6}, {0.15, -0.15, 42.42}}},
      {5, {{-0.15, 0.15, 0.123456789}, {0.15, 0.15, -987.654321}}},
  };

  for (const auto& [seq, pts] : post_frames) {
    cache.pushFrame(pts, seq, origin);
    FrameEvidence ev;
    ASSERT_TRUE(ev.buildFrame(pts, seq, origin).has_value());
    ASSERT_TRUE(test_map.fuseFrameEvidence(ev).has_value());
    ASSERT_TRUE(ref_map.fuseFrameEvidence(ev).has_value());

    const auto* ref_payload = ref_map.findActivePayload(root);
    const auto* test_state = test_map.findInternalCoarseningState(root);
    ASSERT_NE(ref_payload, nullptr);
    ASSERT_NE(test_state, nullptr);
    ASSERT_TRUE(test_state->has_current_shadow);

    // Bit-for-bit equivalence after every frame
    verifyExact8FieldEquivalence(test_state->current_shadow, *ref_payload);
  }

  // Merge from shadow
  const auto merge_res = test_map.mergeDirectChildren(root);
  EXPECT_EQ(merge_res.status, MergeStatus::APPLIED);

  // Split again
  ASSERT_TRUE(test_map.splitLeaf(root));

  // Raw replay on re-split MUST pass APPLIED_COMPLETE
  const auto replay_res = test_map.replayDirectChildren(root, cache);
  EXPECT_EQ(replay_res.status, ReplayStatus::APPLIED_COMPLETE);
  EXPECT_TRUE(test_map.validateInvariants().is_valid);
}

}  // namespace avr::mapping

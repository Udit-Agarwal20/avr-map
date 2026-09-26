#include <gtest/gtest.h>

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

struct PersistentAdaptiveMapTestAccess {
  static AdaptivePayloadStore& payloadStore(PersistentAdaptiveMap& map) noexcept {
    return map.payload_store_;
  }
  static std::unordered_map<avr::spatial::CellKey, InternalCoarseningState>& internalCoarseningStates(
      PersistentAdaptiveMap& map) noexcept {
    return map.internal_coarsening_states_;
  }
};

namespace {

// Helper to compare the 8 canonical measurement/temporal fields for bit-for-bit equality
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
  // Bitwise representation equality
  EXPECT_EQ(actual.elevation().mean(), expected.elevation().mean());
  EXPECT_EQ(actual.elevation().m2(), expected.elevation().m2());
  EXPECT_EQ(actual.elevation().min(), expected.elevation().min());
  EXPECT_EQ(actual.elevation().max(), expected.elevation().max());
}

}  // namespace

// ============================================================================
// Section 6, 12, 13, 14: Base State Initialization
// ============================================================================

TEST(InternalCoarseningShadowTest, SplitInitializesCorrectBaseStates) {
  // 1. Complete retired cache split
  {
    PersistentAdaptiveMap map{};
    const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};
    const auto root = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);
    const std::vector<PointXYZ> pts = {{0.05, 0.05, 1.0}};
    FrameEvidence ev;
    ASSERT_TRUE(ev.buildFrame(pts, 10, origin).has_value());
    ASSERT_TRUE(map.fuseFrameEvidence(ev).has_value());

    ASSERT_TRUE(map.splitLeaf(root));
    const auto* state = map.findInternalCoarseningState(root);
    ASSERT_NE(state, nullptr);
    EXPECT_EQ(state->base_state, CoarseningBaseState::COMPLETE_RETIRED_CACHE);
    EXPECT_FALSE(state->has_current_shadow);
    EXPECT_TRUE(map.validateInvariants().is_valid);
  }

  // 2. Complete empty split
  {
    PersistentAdaptiveMap map{};
    const auto root = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);
    ASSERT_TRUE(map.ensureRootForBase(avr::spatial::GridCoord2D{0, 0}).has_value());

    ASSERT_TRUE(map.splitLeaf(root));
    const auto* state = map.findInternalCoarseningState(root);
    ASSERT_NE(state, nullptr);
    EXPECT_EQ(state->base_state, CoarseningBaseState::COMPLETE_EMPTY);
    EXPECT_FALSE(state->has_current_shadow);
    EXPECT_TRUE(map.validateInvariants().is_valid);
  }

  // 3. Incomplete split (parent leaf was already REPLAY_REQUIRED)
  {
    PersistentAdaptiveMap map{};
    const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};
    const auto root = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);
    const std::vector<PointXYZ> pts = {{0.05, 0.05, 1.0}};
    FrameEvidence ev;
    ASSERT_TRUE(ev.buildFrame(pts, 10, origin).has_value());
    ASSERT_TRUE(map.fuseFrameEvidence(ev).has_value());

    ASSERT_TRUE(map.splitLeaf(root));
    const auto children = *avr::spatial::childrenOf(root);
    // child is LEAF and has requiresReplay == true
    ASSERT_TRUE(map.requiresReplay(children[0]));

    // Split child further: child's internal coarsening state must be INCOMPLETE
    ASSERT_TRUE(map.splitLeaf(children[0]));
    const auto* child_state = map.findInternalCoarseningState(children[0]);
    ASSERT_NE(child_state, nullptr);
    EXPECT_EQ(child_state->base_state, CoarseningBaseState::INCOMPLETE);
    EXPECT_FALSE(child_state->has_current_shadow);
    EXPECT_TRUE(map.validateInvariants().is_valid);
  }
}

// ============================================================================
// Section 15: Invariant - Every Internal Node Owns Exactly One Coarsening State
// ============================================================================

TEST(InternalCoarseningShadowTest, EveryInternalNodeOwnsStateAndLeafOwnsNone) {
  PersistentAdaptiveMap map{};
  const auto root = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);
  ASSERT_TRUE(map.ensureRootForBase(avr::spatial::GridCoord2D{0, 0}).has_value());
  EXPECT_TRUE(map.hierarchy().isLeaf(root));
  EXPECT_EQ(map.findInternalCoarseningState(root), nullptr);

  ASSERT_TRUE(map.splitLeaf(root));
  EXPECT_TRUE(map.hierarchy().isInternal(root));
  EXPECT_NE(map.findInternalCoarseningState(root), nullptr);

  const auto children = *avr::spatial::childrenOf(root);
  for (const auto& child : children) {
    EXPECT_TRUE(map.hierarchy().isLeaf(child));
    EXPECT_EQ(map.findInternalCoarseningState(child), nullptr);
  }

  // Refine child[0] down to L1
  ASSERT_TRUE(map.splitLeaf(children[0]));
  EXPECT_TRUE(map.hierarchy().isInternal(children[0]));
  EXPECT_NE(map.findInternalCoarseningState(children[0]), nullptr);

  const auto stats = map.coarseningStats();
  EXPECT_EQ(stats.internal_coarsening_state_count, 2);
  EXPECT_EQ(stats.internal_coarsening_state_count, map.hierarchy().internalCount());
  EXPECT_TRUE(map.validateInvariants().is_valid);
}

// ============================================================================
// Section 22: CRITICAL EXACTNESS GOLDEN (Bit-for-Bit Reference Equivalence)
// ============================================================================

TEST(InternalCoarseningShadowTest, CriticalExactnessGoldenUnsplitReferenceEquivalence) {
  // REFERENCE map: root kept unsplit as active LEAF for all frames
  PersistentAdaptiveMap ref_map{};
  // TEST map: root split before frame 40, receives post-split frames into children
  PersistentAdaptiveMap test_map{};

  const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};
  const auto root = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);

  // Frames 10, 20, 30: both maps unsplit
  const std::vector<std::pair<uint64_t, std::vector<PointXYZ>>> initial_frames = {
      {10, {{0.05, 0.05, 1.25}, {0.15, 0.25, 2.50}}},
      {20, {{-0.10, -0.10, -0.50}, {0.30, 0.10, 3.10}}},
      {30, {{0.02, -0.05, 0.80}, {-0.20, 0.20, 1.90}}},
  };

  for (const auto& [seq, pts] : initial_frames) {
    FrameEvidence ev;
    ASSERT_TRUE(ev.buildFrame(pts, seq, origin).has_value());
    ASSERT_TRUE(ref_map.fuseFrameEvidence(ev).has_value());
    ASSERT_TRUE(test_map.fuseFrameEvidence(ev).has_value());
  }

  // Verify identical active payloads prior to split
  const auto* ref_payload_pre = ref_map.findActivePayload(root);
  const auto* test_payload_pre = test_map.findActivePayload(root);
  ASSERT_NE(ref_payload_pre, nullptr);
  ASSERT_NE(test_payload_pre, nullptr);
  verifyExact8FieldEquivalence(*test_payload_pre, *ref_payload_pre);

  // Split TEST map root before frame 40
  ASSERT_TRUE(test_map.splitLeaf(root));
  const auto* state_pre_40 = test_map.findInternalCoarseningState(root);
  ASSERT_NE(state_pre_40, nullptr);
  EXPECT_FALSE(state_pre_40->has_current_shadow);

  // Post-split frames 40, 50, 60, 70 with varied point counts and child distributions
  const std::vector<std::pair<uint64_t, std::vector<PointXYZ>>> post_split_frames = {
      {40, {{0.05, 0.05, 1.10}, {-0.15, 0.15, 2.20}, {0.20, -0.20, -0.40}}},
      {50, {{-0.05, -0.05, -0.85}, {0.10, 0.10, 4.30}}},
      {60, {{0.35, 0.35, 5.00}, {-0.35, -0.35, -2.10}, {0.01, 0.02, 0.00}}},
      {70, {{-0.25, 0.10, 1.75}, {0.12, -0.15, 0.55}}},
  };

  for (const auto& [seq, pts] : post_split_frames) {
    FrameEvidence ev;
    ASSERT_TRUE(ev.buildFrame(pts, seq, origin).has_value());
    ASSERT_TRUE(ref_map.fuseFrameEvidence(ev).has_value());
    ASSERT_TRUE(test_map.fuseFrameEvidence(ev).has_value());

    // Compare TEST parent current shadow against REFERENCE parent active payload after EVERY frame
    const auto* ref_payload = ref_map.findActivePayload(root);
    const auto* test_state = test_map.findInternalCoarseningState(root);

    ASSERT_NE(ref_payload, nullptr);
    ASSERT_NE(test_state, nullptr);
    ASSERT_TRUE(test_state->has_current_shadow);

    verifyExact8FieldEquivalence(test_state->current_shadow, *ref_payload);
    EXPECT_TRUE(test_map.validateInvariants().is_valid);
    EXPECT_TRUE(ref_map.validateInvariants().is_valid);
  }
}

// ============================================================================
// Section 23: Multi-Child Same-Frame Shadow Test
// ============================================================================

TEST(InternalCoarseningShadowTest, MultiChildSameFrameUpdatesShadowExactlyOnce) {
  PersistentAdaptiveMap map{};
  const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};
  const auto root = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);

  ASSERT_TRUE(map.ensureRootForBase(avr::spatial::GridCoord2D{0, 0}).has_value());
  ASSERT_TRUE(map.splitLeaf(root));

  // One frame containing 4 points, each falling into a different quadrant/child of root
  // root at (0,0) L3 spans [0.0, 0.40) in world coordinates with 4 L2 children
  const std::vector<PointXYZ> f10 = {
      {0.10, 0.10, 1.0},  // Child 0: [0.0, 0.20) x [0.0, 0.20)
      {0.30, 0.10, 2.0},  // Child 1: [0.20, 0.40) x [0.0, 0.20)
      {0.10, 0.30, 3.0},  // Child 2: [0.0, 0.20) x [0.20, 0.40)
      {0.30, 0.30, 4.0},  // Child 3: [0.20, 0.40) x [0.20, 0.40)
  };

  FrameEvidence ev10;
  ASSERT_TRUE(ev10.buildFrame(f10, 10, origin).has_value());
  const auto stats_opt = map.fuseFrameEvidence(ev10);
  ASSERT_TRUE(stats_opt.has_value());
  const auto& stats = *stats_opt;

  // 4 leaf payloads updated
  EXPECT_EQ(stats.unique_leaves_fused, 4);
  EXPECT_EQ(stats.internal_ancestors_considered, 1);
  EXPECT_EQ(stats.internal_shadows_created, 1);
  EXPECT_EQ(stats.internal_shadows_updated, 0);

  const auto* state = map.findInternalCoarseningState(root);
  ASSERT_NE(state, nullptr);
  ASSERT_TRUE(state->has_current_shadow);

  // Parent observed_frame_count MUST increment by 1, NOT 4!
  EXPECT_EQ(state->current_shadow.observedFrameCount(), 1);
  // Elevation count equals sum of all accepted points (4)
  EXPECT_EQ(state->current_shadow.observationCount(), 4);
  EXPECT_EQ(state->current_shadow.firstSeenFrame(), 10);
  EXPECT_EQ(state->current_shadow.lastSeenFrame(), 10);
  EXPECT_TRUE(map.validateInvariants().is_valid);
}

// ============================================================================
// Section 24: Deep Ancestor Test (L3 -> L2 -> L1 -> L0)
// ============================================================================

TEST(InternalCoarseningShadowTest, DeepAncestorSingleUpdatePerAncestorPerFrame) {
  PersistentAdaptiveMap test_map{};
  PersistentAdaptiveMap ref_l1_map{};
  const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};
  const avr::spatial::GridCoord2D base_coord{0, 0};

  // Refine test_map to L0
  ASSERT_TRUE(test_map.refineToLevel(base_coord, avr::spatial::L0));
  const auto l0_leaf = *test_map.hierarchy().findLeafContainingBase(base_coord);
  const auto l1_parent = *avr::spatial::parentOf(l0_leaf);
  const auto l2_parent = *avr::spatial::parentOf(l1_parent);
  const auto l3_root = *avr::spatial::parentOf(l2_parent);

  EXPECT_TRUE(test_map.hierarchy().isInternal(l3_root));
  EXPECT_TRUE(test_map.hierarchy().isInternal(l2_parent));
  EXPECT_TRUE(test_map.hierarchy().isInternal(l1_parent));
  EXPECT_TRUE(test_map.hierarchy().isLeaf(l0_leaf));

  // Prepare reference map for L1 parent region
  ASSERT_TRUE(ref_l1_map.refineToLevel(base_coord, avr::spatial::L1));

  // Fuse a frame with points inside L0
  const std::vector<PointXYZ> f10 = {{0.01, 0.01, 2.50}, {0.02, 0.02, 3.50}};
  FrameEvidence ev10;
  ASSERT_TRUE(ev10.buildFrame(f10, 10, origin).has_value());

  ASSERT_TRUE(ref_l1_map.fuseFrameEvidence(ev10).has_value());
  const auto stats_opt = test_map.fuseFrameEvidence(ev10);
  ASSERT_TRUE(stats_opt.has_value());
  const auto& stats = *stats_opt;

  // Ancestor discovery must find 3 internal ancestors: L1, L2, L3
  EXPECT_EQ(stats.internal_ancestors_considered, 3);
  EXPECT_EQ(stats.internal_shadows_created, 3);
  EXPECT_EQ(stats.internal_shadows_updated, 0);

  // Each internal ancestor shadow must have observed_frame_count == 1
  const auto* s1 = test_map.findInternalCoarseningState(l1_parent);
  const auto* s2 = test_map.findInternalCoarseningState(l2_parent);
  const auto* s3 = test_map.findInternalCoarseningState(l3_root);

  ASSERT_NE(s1, nullptr);
  ASSERT_NE(s2, nullptr);
  ASSERT_NE(s3, nullptr);

  EXPECT_EQ(s1->current_shadow.observedFrameCount(), 1);
  EXPECT_EQ(s2->current_shadow.observedFrameCount(), 1);
  EXPECT_EQ(s3->current_shadow.observedFrameCount(), 1);

  EXPECT_EQ(s1->current_shadow.observationCount(), 2);
  EXPECT_EQ(s2->current_shadow.observationCount(), 2);
  EXPECT_EQ(s3->current_shadow.observationCount(), 2);

  // L1 shadow in test_map matches reference L1 leaf active payload bit-for-bit
  const auto* ref_l1_payload = ref_l1_map.findActivePayload(l1_parent);
  ASSERT_NE(ref_l1_payload, nullptr);
  verifyExact8FieldEquivalence(s1->current_shadow, *ref_l1_payload);
  EXPECT_TRUE(test_map.validateInvariants().is_valid);
}

// ============================================================================
// Section 25: Multiple Leaves Same Ancestor Deduplication Test
// ============================================================================

TEST(InternalCoarseningShadowTest, MultipleLeavesSameAncestorDeduplication) {
  PersistentAdaptiveMap map{};
  const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};
  const avr::spatial::GridCoord2D base_coord{0, 0};

  ASSERT_TRUE(map.refineToLevel(base_coord, avr::spatial::L1));
  const auto root = *avr::spatial::CellKey::fromGridCoord(base_coord, avr::spatial::L3);

  // Root is L3 (INTERNAL), has children at L2. L2 child is also INTERNAL, has children at L1.
  // Fuse frame touching multiple leaves
  const std::vector<PointXYZ> f10 = {
      {0.01, 0.01, 1.0},
      {0.05, 0.01, 2.0},
      {0.01, 0.05, 3.0},
      {0.05, 0.05, 4.0},
  };
  FrameEvidence ev10;
  ASSERT_TRUE(ev10.buildFrame(f10, 10, origin).has_value());
  const auto stats = map.fuseFrameEvidence(ev10);
  ASSERT_TRUE(stats.has_value());

  // Root L3 shadow must have observed_frame_count == 1, NOT 4
  const auto* root_state = map.findInternalCoarseningState(root);
  ASSERT_NE(root_state, nullptr);
  EXPECT_EQ(root_state->current_shadow.observedFrameCount(), 1);
  EXPECT_EQ(root_state->current_shadow.observationCount(), 4);
}

// ============================================================================
// Section 26, 27, 63: Fusion Prepare/Commit Failure Rollback
// ============================================================================

TEST(InternalCoarseningShadowTest, FusionNumericFailureInShadowRollsBackCompletely) {
  PersistentAdaptiveMap map{};
  const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};
  const auto root = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);

  // Initialize parent with a frame
  const std::vector<PointXYZ> f10 = {{0.05, 0.05, 1.0}};
  FrameEvidence ev10;
  ASSERT_TRUE(ev10.buildFrame(f10, 10, origin).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev10).has_value());

  ASSERT_TRUE(map.splitLeaf(root));

  // Force candidate shadow to fail on next update by injecting an out-of-order frame sequence into shadow
  // e.g. shadow last_seen_frame = 100
  auto& shadow_map = PersistentAdaptiveMapTestAccess::internalCoarseningStates(map);
  shadow_map[root].has_current_shadow = true;
  shadow_map[root].current_shadow = AdaptiveCellPayload(
      ElevationStats(1, 1.0, 0.0, 1.0, 1.0),
      /*first_seen=*/10,
      /*last_seen=*/100,  // Far ahead
      /*observed_count=*/1,
      ResolutionReasonBits{});

  const auto pre_active = map.activePayloadSnapshot();
  const auto pre_retired = map.retiredCacheSnapshot();
  const auto pre_replay = map.replayRequiredSnapshot();
  const auto pre_shadow = map.internalCoarseningSnapshot();
  const auto pre_stats = map.statistics();

  // Next frame is sequence 20 (< 100).
  // Leaf fusion for sequence 20 is numerically valid (leaves were at seq 10).
  // But shadow candidate tryFuse(..., 20) will fail because frame_seq 20 <= last_seen 100!
  const std::vector<PointXYZ> f20 = {{0.05, 0.05, 2.0}};
  FrameEvidence ev20;
  ASSERT_TRUE(ev20.buildFrame(f20, 20, origin).has_value());

  const auto res = map.fuseFrameEvidence(ev20);
  EXPECT_FALSE(res.has_value());

  // Strict rollback verification: zero leaf mutation, zero shadow mutation, zero sequencing advance
  EXPECT_EQ(map.activePayloadSnapshot(), pre_active);
  EXPECT_EQ(map.retiredCacheSnapshot(), pre_retired);
  EXPECT_EQ(map.replayRequiredSnapshot(), pre_replay);
  EXPECT_EQ(map.internalCoarseningSnapshot(), pre_shadow);
  EXPECT_EQ(map.statistics(), pre_stats);
  EXPECT_EQ(map.lastFusedFrameSeq(), 10);
  EXPECT_EQ(map.totalFramesFused(), 1);
  EXPECT_EQ(map.totalObservationsFused(), 1);
}

// ============================================================================
// Section 28, 29, 59: Shadow Updates Do NOT Increment Sensor Observation Totals
// ============================================================================

TEST(InternalCoarseningShadowTest, ShadowCountsDoNotIncrementGlobalSensorTotals) {
  PersistentAdaptiveMap map{};
  const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};
  const avr::spatial::GridCoord2D base{0, 0};

  ASSERT_TRUE(map.refineToLevel(base, avr::spatial::L0));

  // Frame with 10 points all falling inside base cell (0, 0)
  std::vector<PointXYZ> pts;
  for (int i = 0; i < 10; ++i) {
    pts.push_back({0.005 * (i + 1), 0.005 * (i + 1), 1.0});
  }

  FrameEvidence ev10;
  ASSERT_TRUE(ev10.buildFrame(pts, 10, origin).has_value());
  const auto stats = map.fuseFrameEvidence(ev10);
  ASSERT_TRUE(stats.has_value());

  EXPECT_EQ(stats->observations_fused, 10);
  EXPECT_EQ(stats->accepted_points, 10);
  // Global cumulative sensor observations must equal EXACTLY 10, not 10 + 3 * 10
  EXPECT_EQ(map.totalObservationsFused(), 10);
  // Summary duplicate count across ancestors is 3 * 10 = 30
  EXPECT_EQ(stats->internal_shadow_observations_summarized, 30);
  EXPECT_EQ(map.statistics().total_observations_fused, 10);
}

// ============================================================================
// Section 30: Read-Only Inspection API & Deterministic Snapshot
// ============================================================================

TEST(InternalCoarseningShadowTest, ReadOnlyInspectionAndCanonicalSortedSnapshot) {
  PersistentAdaptiveMap map{};
  const avr::spatial::GridCoord2D coord_a{0, 0};
  const avr::spatial::GridCoord2D coord_b{10, 10};

  ASSERT_TRUE(map.ensureRootForBase(coord_a).has_value());
  ASSERT_TRUE(map.ensureRootForBase(coord_b).has_value());

  const auto root_a = *avr::spatial::CellKey::fromGridCoord(coord_a, avr::spatial::L3);
  const auto root_b = *avr::spatial::CellKey::fromGridCoord(coord_b, avr::spatial::L3);

  ASSERT_TRUE(map.splitLeaf(root_a));
  ASSERT_TRUE(map.splitLeaf(root_b));

  const auto snapshot = map.internalCoarseningSnapshot();
  ASSERT_EQ(snapshot.size(), 2);

  // Must be sorted strictly in ascending canonical CellKey order
  EXPECT_LT(snapshot[0].first, snapshot[1].first);

  const auto* state_a = map.findInternalCoarseningState(root_a);
  const auto* state_b = map.findInternalCoarseningState(root_b);
  ASSERT_NE(state_a, nullptr);
  ASSERT_NE(state_b, nullptr);

  // Absent cell lookup returns nullptr
  const auto missing_key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{50, 50}, avr::spatial::L3);
  EXPECT_EQ(map.findInternalCoarseningState(missing_key), nullptr);
}

// ============================================================================
// Section 31: Replay Does NOT Update Internal Parent Shadow
// ============================================================================

TEST(InternalCoarseningShadowTest, ReplayDoesNotUpdateParentShadow) {
  PersistentAdaptiveMap map{};
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 10});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;

  const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};
  const auto root = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);

  // Frames 10, 20
  const std::vector<PointXYZ> f10 = {{0.05, 0.05, 1.0}};
  const std::vector<PointXYZ> f20 = {{0.05, 0.05, 2.0}};
  cache.pushFrame(f10, 10, origin);
  cache.pushFrame(f20, 20, origin);

  FrameEvidence ev10, ev20;
  ASSERT_TRUE(ev10.buildFrame(f10, 10, origin).has_value());
  ASSERT_TRUE(ev20.buildFrame(f20, 20, origin).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev10).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev20).has_value());

  // Split root
  ASSERT_TRUE(map.splitLeaf(root));

  // Post-split frame 30 arrives into children and updates internal shadow
  const std::vector<PointXYZ> f30 = {{0.05, 0.05, 3.0}};
  FrameEvidence ev30;
  ASSERT_TRUE(ev30.buildFrame(f30, 30, origin).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev30).has_value());

  // Snapshot shadow before replay
  const auto pre_replay_shadow = map.internalCoarseningSnapshot();
  const auto* pre_state = map.findInternalCoarseningState(root);
  ASSERT_NE(pre_state, nullptr);
  ASSERT_TRUE(pre_state->has_current_shadow);
  const auto pre_payload = pre_state->current_shadow;

  // Run M8B direct-parent replay
  const auto replay_res = map.replayDirectChildren(root, cache);
  EXPECT_EQ(replay_res.status, ReplayStatus::APPLIED_COMPLETE);

  // Shadow MUST remain completely bit-for-bit identical after replay!
  const auto* post_state = map.findInternalCoarseningState(root);
  ASSERT_NE(post_state, nullptr);
  ASSERT_TRUE(post_state->has_current_shadow);
  verifyExact8FieldEquivalence(post_state->current_shadow, pre_payload);
  EXPECT_EQ(map.internalCoarseningSnapshot(), pre_replay_shadow);
  EXPECT_TRUE(map.validateInvariants().is_valid);
}

// ============================================================================
// Section 32 & 51: Structural Merge Does NOT Update Ancestor Shadow
// ============================================================================

TEST(InternalCoarseningShadowTest, StructuralMergeDoesNotUpdateGrandparentShadow) {
  PersistentAdaptiveMap map{};
  const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};
  const avr::spatial::GridCoord2D base{0, 0};

  // Refine down to L1: root (L3) -> L2 parent -> L1 parent
  ASSERT_TRUE(map.refineToLevel(base, avr::spatial::L1));
  const auto l1_leaf = *map.hierarchy().findLeafContainingBase(base);
  const auto l2_parent = *avr::spatial::parentOf(l1_leaf);
  const auto l3_grandparent = *avr::spatial::parentOf(l2_parent);

  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 10});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;

  // Fuse observations into L1 leaf
  const std::vector<PointXYZ> f10 = {{0.01, 0.01, 1.50}};
  cache.pushFrame(f10, 10, origin);
  FrameEvidence ev10;
  ASSERT_TRUE(ev10.buildFrame(f10, 10, origin).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev10).has_value());

  // Further split L1 leaf into L0 children
  ASSERT_TRUE(map.splitLeaf(l1_leaf));
  // Replay direct children to complete their history before future merge
  ASSERT_EQ(map.replayDirectChildren(l1_leaf, cache).status, ReplayStatus::APPLIED_COMPLETE);

  // Fuse frame 20 into L0 children
  const std::vector<PointXYZ> f20 = {{0.01, 0.01, 2.50}};
  FrameEvidence ev20;
  ASSERT_TRUE(ev20.buildFrame(f20, 20, origin).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev20).has_value());

  // Capture L3 and L2 shadows prior to merging L0 children into L1
  const auto* l3_state_pre = map.findInternalCoarseningState(l3_grandparent);
  const auto* l2_state_pre = map.findInternalCoarseningState(l2_parent);
  ASSERT_NE(l3_state_pre, nullptr);
  ASSERT_NE(l2_state_pre, nullptr);
  ASSERT_TRUE(l3_state_pre->has_current_shadow);
  ASSERT_TRUE(l2_state_pre->has_current_shadow);
  const auto l3_shadow_pre = l3_state_pre->current_shadow;
  const auto l2_shadow_pre = l2_state_pre->current_shadow;

  // Merge L0 children back into L1 leaf (operational no-cache merge)
  const auto merge_res = map.mergeDirectChildren(l1_leaf);
  EXPECT_EQ(merge_res.status, MergeStatus::APPLIED);

  // Grandparent L3 and parent L2 shadows MUST remain bit-for-bit unchanged!
  const auto* l3_state_post = map.findInternalCoarseningState(l3_grandparent);
  const auto* l2_state_post = map.findInternalCoarseningState(l2_parent);
  ASSERT_NE(l3_state_post, nullptr);
  ASSERT_NE(l2_state_post, nullptr);
  verifyExact8FieldEquivalence(l3_state_post->current_shadow, l3_shadow_pre);
  verifyExact8FieldEquivalence(l2_state_post->current_shadow, l2_shadow_pre);
  EXPECT_TRUE(map.validateInvariants().is_valid);
}

// ============================================================================
// Section 47: Validator Invariant Coverage
// ============================================================================

TEST(InternalCoarseningShadowTest, ValidatorInvariantChecks) {
  PersistentAdaptiveMap map{};
  const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};
  const auto root = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);

  const std::vector<PointXYZ> f10 = {{0.05, 0.05, 1.0}};
  FrameEvidence ev10;
  ASSERT_TRUE(ev10.buildFrame(f10, 10, origin).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev10).has_value());
  ASSERT_TRUE(map.splitLeaf(root));
  EXPECT_TRUE(map.validateInvariants().is_valid);

  auto& coarsening_states = PersistentAdaptiveMapTestAccess::internalCoarseningStates(map);

  // Test Rule A: Missing coarsening state on internal node
  {
    auto saved_state = coarsening_states[root];
    coarsening_states.erase(root);
    auto val = map.validateInvariants();
    EXPECT_FALSE(val.is_valid);
    EXPECT_EQ(val.code, PersistentMapValidationCode::INTERNAL_NODE_MISSING_COARSENING_STATE);
    coarsening_states[root] = saved_state;
  }

  // Test Rule B: Coarsening state on non-internal node (e.g. child leaf)
  {
    const auto child = (*avr::spatial::childrenOf(root))[0];
    coarsening_states[child] = InternalCoarseningState{
        .base_state = CoarseningBaseState::COMPLETE_EMPTY,
        .has_current_shadow = false,
    };
    auto val = map.validateInvariants();
    EXPECT_FALSE(val.is_valid);
    EXPECT_EQ(val.code, PersistentMapValidationCode::COARSENING_STATE_ON_NON_INTERNAL);
    coarsening_states.erase(child);
  }

  // Test Rule D: COMPLETE_RETIRED_CACHE requires retired cache
  {
    auto saved_base = coarsening_states[root].base_state;
    // Set base state to COMPLETE_EMPTY while retired cache exists
    coarsening_states[root].base_state = CoarseningBaseState::COMPLETE_EMPTY;
    auto val = map.validateInvariants();
    EXPECT_FALSE(val.is_valid);
    EXPECT_EQ(val.code, PersistentMapValidationCode::COARSENING_BASE_EMPTY_HAS_RETIRED_CACHE);
    coarsening_states[root].base_state = saved_base;
  }

  // Test Rule G: Shadow invalid frame order (first_seen > last_seen)
  {
    coarsening_states[root].has_current_shadow = true;
    coarsening_states[root].current_shadow = AdaptiveCellPayload(
        ElevationStats(1, 1.0, 0.0, 1.0, 1.0),
        /*first_seen=*/50,
        /*last_seen=*/20,  // invalid
        /*observed_count=*/1,
        ResolutionReasonBits{});
    auto val = map.validateInvariants();
    EXPECT_FALSE(val.is_valid);
    EXPECT_EQ(val.code, PersistentMapValidationCode::COARSENING_SHADOW_FRAME_ORDER_INVALID);
    coarsening_states[root].has_current_shadow = false;
  }
}

// ============================================================================
// Section 48: Retired Parent Cache Immutability
// ============================================================================

TEST(InternalCoarseningShadowTest, RetiredParentCacheImmutability) {
  PersistentAdaptiveMap map{};
  const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};
  const auto root = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);

  const std::vector<PointXYZ> f10 = {{0.05, 0.05, 1.0}};
  FrameEvidence ev10;
  ASSERT_TRUE(ev10.buildFrame(f10, 10, origin).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev10).has_value());
  ASSERT_TRUE(map.splitLeaf(root));

  const auto* retired_init = map.findRetiredCache(root);
  ASSERT_NE(retired_init, nullptr);
  const auto retired_copy = *retired_init;

  // Fuse 10 subsequent frames into descendants
  for (uint64_t seq = 20; seq <= 110; seq += 10) {
    const std::vector<PointXYZ> pts = {{0.05, 0.05, static_cast<double>(seq)}};
    FrameEvidence ev;
    ASSERT_TRUE(ev.buildFrame(pts, seq, origin).has_value());
    ASSERT_TRUE(map.fuseFrameEvidence(ev).has_value());

    // Retired parent cache MUST remain strictly bit-for-bit immutable!
    const auto* retired_curr = map.findRetiredCache(root);
    ASSERT_NE(retired_curr, nullptr);
    verifyExact8FieldEquivalence(*retired_curr, retired_copy);
  }
}

// ============================================================================
// Section 49: Reset Clears Internal Coarsening States
// ============================================================================

TEST(InternalCoarseningShadowTest, ResetClearsCoarseningStates) {
  PersistentAdaptiveMap map{};
  const auto root = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);
  ASSERT_TRUE(map.ensureRootForBase(avr::spatial::GridCoord2D{0, 0}).has_value());
  ASSERT_TRUE(map.splitLeaf(root));

  EXPECT_EQ(map.coarseningStats().internal_coarsening_state_count, 1);
  EXPECT_FALSE(map.internalCoarseningSnapshot().empty());

  map.reset();

  EXPECT_EQ(map.coarseningStats().internal_coarsening_state_count, 0);
  EXPECT_TRUE(map.internalCoarseningSnapshot().empty());
  EXPECT_EQ(map.findInternalCoarseningState(root), nullptr);
  EXPECT_TRUE(map.validateInvariants().is_valid);
}

// ============================================================================
// Section 58 & 60: Memory Layout & Sizing Report
// ============================================================================

TEST(InternalCoarseningShadowTest, MemoryLayoutAndSizing) {
  EXPECT_EQ(sizeof(CoarseningBaseState), 1);
  EXPECT_EQ(alignof(InternalCoarseningState), alignof(AdaptiveCellPayload));
  EXPECT_GE(sizeof(InternalCoarseningState), sizeof(AdaptiveCellPayload) + sizeof(CoarseningBaseState));

  std::cout << "[M9B2A Memory Report] sizeof(CoarseningBaseState) = "
            << sizeof(CoarseningBaseState) << " bytes" << std::endl;
  std::cout << "[M9B2A Memory Report] sizeof(InternalCoarseningState) = "
            << sizeof(InternalCoarseningState) << " bytes" << std::endl;
  std::cout << "[M9B2A Memory Report] alignof(InternalCoarseningState) = "
            << alignof(InternalCoarseningState) << " bytes" << std::endl;

  PersistentAdaptiveMap map{};
  const auto base = avr::spatial::GridCoord2D{0, 0};
  ASSERT_TRUE(map.refineToLevel(base, avr::spatial::L0));
  const auto stats = map.coarseningStats();

  std::cout << "[M9B2A Memory Report] internal state count = "
            << stats.internal_coarsening_state_count
            << ", map bucket_count = " << stats.map_bucket_count << std::endl;
}

// ============================================================================
// Section 29: Structural Split Grandparent Immutability
// ============================================================================

TEST(InternalCoarseningShadowTest, StructuralSplitGrandparentImmutability) {
  PersistentAdaptiveMap map{};
  const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};
  const avr::spatial::GridCoord2D base{0, 0};

  ASSERT_TRUE(map.refineToLevel(base, avr::spatial::L1));
  const auto l1_leaf = *map.hierarchy().findLeafContainingBase(base);
  const auto l2_parent = *avr::spatial::parentOf(l1_leaf);
  const auto l3_grandparent = *avr::spatial::parentOf(l2_parent);

  // Fuse observations into L1 leaf
  const std::vector<PointXYZ> f10 = {{0.01, 0.01, 1.50}};
  FrameEvidence ev10;
  ASSERT_TRUE(ev10.buildFrame(f10, 10, origin).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev10).has_value());

  const auto* l3_state_pre = map.findInternalCoarseningState(l3_grandparent);
  const auto* l2_state_pre = map.findInternalCoarseningState(l2_parent);
  ASSERT_NE(l3_state_pre, nullptr);
  ASSERT_NE(l2_state_pre, nullptr);
  const auto l3_shadow_pre = l3_state_pre->current_shadow;
  const auto l2_shadow_pre = l2_state_pre->current_shadow;

  // Split L1 leaf into L0: this is a structural operation
  ASSERT_TRUE(map.splitLeaf(l1_leaf));

  // Ancestor shadows MUST remain bit-for-bit unchanged
  const auto* l3_state_post = map.findInternalCoarseningState(l3_grandparent);
  const auto* l2_state_post = map.findInternalCoarseningState(l2_parent);
  ASSERT_NE(l3_state_post, nullptr);
  ASSERT_NE(l2_state_post, nullptr);
  verifyExact8FieldEquivalence(l3_state_post->current_shadow, l3_shadow_pre);
  verifyExact8FieldEquivalence(l2_state_post->current_shadow, l2_shadow_pre);
  EXPECT_TRUE(map.validateInvariants().is_valid);
}

// ============================================================================
// Section 32: Split State Collision Rejection
// ============================================================================

TEST(InternalCoarseningShadowTest, SplitStateCollisionRejection) {
  PersistentAdaptiveMap map{};
  const auto root = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);
  ASSERT_TRUE(map.ensureRootForBase(avr::spatial::GridCoord2D{0, 0}).has_value());
  EXPECT_TRUE(map.hierarchy().isLeaf(root));

  // Artificially inject an existing state for the leaf parent
  auto& states = PersistentAdaptiveMapTestAccess::internalCoarseningStates(map);
  states[root] = InternalCoarseningState{
      .base_state = CoarseningBaseState::COMPLETE_EMPTY,
      .has_current_shadow = false,
      .current_shadow = AdaptiveCellPayload{},
  };

  // splitLeaf must detect collision and reject fail-close
  EXPECT_FALSE(map.splitLeaf(root));
  EXPECT_TRUE(map.hierarchy().isLeaf(root));
}

// ============================================================================
// Section 38 & 39: Planned Root + Shadow Transaction Atomic Commit & Rollback
// ============================================================================

TEST(InternalCoarseningShadowTest, PlannedRootAndShadowTransactionAtomicCommitAndRollback) {
  PersistentAdaptiveMap map{};
  const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};
  const auto root_a = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);
  ASSERT_TRUE(map.ensureRootForBase(avr::spatial::GridCoord2D{0, 0}).has_value());
  ASSERT_TRUE(map.splitLeaf(root_a));

  // 1. Valid frame creating planned root B while updating shadow A
  const std::vector<PointXYZ> f10 = {
      {0.10, 0.10, 1.0},                                        // In existing leaf under root A
      {10.0 * avr::spatial::BASE_RESOLUTION_METERS + 0.01,       // In unseen root B at (10, 10)
       10.0 * avr::spatial::BASE_RESOLUTION_METERS + 0.01, 2.0},
  };
  FrameEvidence ev10;
  ASSERT_TRUE(ev10.buildFrame(f10, 10, origin).has_value());
  const auto stats10 = map.fuseFrameEvidence(ev10);
  ASSERT_TRUE(stats10.has_value());
  EXPECT_EQ(stats10->roots_created, 1);
  EXPECT_TRUE(map.validateInvariants().is_valid);

  // 2. Failure rollback: corrupt shadow A sequence ordering, frame contains point in unseen root C
  auto& states = PersistentAdaptiveMapTestAccess::internalCoarseningStates(map);
  states[root_a].current_shadow = AdaptiveCellPayload(
      ElevationStats(1, 1.0, 0.0, 1.0, 1.0),
      /*first_seen=*/10,
      /*last_seen=*/100,  // Sequence 100
      /*observed_count=*/1,
      ResolutionReasonBits{});

  const auto pre_topo = map.hierarchy().topologySnapshot();
  const auto pre_seq = map.lastFusedFrameSeq();

  // Incoming frame 20 (< 100) touches existing leaf in root A and unseen root C at (20, 20)
  const std::vector<PointXYZ> f20 = {
      {0.10, 0.10, 3.0},
      {20.0 * avr::spatial::BASE_RESOLUTION_METERS + 0.01,
       20.0 * avr::spatial::BASE_RESOLUTION_METERS + 0.01, 4.0},
  };
  FrameEvidence ev20;
  ASSERT_TRUE(ev20.buildFrame(f20, 20, origin).has_value());
  EXPECT_FALSE(map.fuseFrameEvidence(ev20).has_value());

  // Strict rollback: root C was NOT created, topology and time unchanged
  EXPECT_EQ(map.hierarchy().topologySnapshot(), pre_topo);
  EXPECT_EQ(map.lastFusedFrameSeq(), pre_seq);
}

// ============================================================================
// Section 22: Shadow Frame-Order Exactness Non-Consecutive & Frame Zero
// ============================================================================

TEST(InternalCoarseningShadowTest, ShadowFrameOrderExactnessNonConsecutiveAndZero) {
  PersistentAdaptiveMap test_map{};
  PersistentAdaptiveMap ref_map{};
  const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};
  const auto root = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);

  const std::vector<std::pair<uint64_t, std::vector<PointXYZ>>> frames = {
      {0, {{0.10, 0.10, 1.0}}},
      {10, {{0.10, 0.10, 2.0}}},
      {42, {{0.10, 0.10, 3.0}}},
      {105, {{0.10, 0.10, 4.0}}},
      {1000, {{0.10, 0.10, 5.0}}},
  };

  // Frames 0 and 10 unsplit
  for (size_t i = 0; i < 2; ++i) {
    FrameEvidence ev;
    ASSERT_TRUE(ev.buildFrame(frames[i].second, frames[i].first, origin).has_value());
    ASSERT_TRUE(test_map.fuseFrameEvidence(ev).has_value());
    ASSERT_TRUE(ref_map.fuseFrameEvidence(ev).has_value());
  }

  // Split test map after frame 10
  ASSERT_TRUE(test_map.splitLeaf(root));

  // Frames 42, 105, 1000 into children and shadow
  for (size_t i = 2; i < frames.size(); ++i) {
    FrameEvidence ev;
    ASSERT_TRUE(ev.buildFrame(frames[i].second, frames[i].first, origin).has_value());
    ASSERT_TRUE(test_map.fuseFrameEvidence(ev).has_value());
    ASSERT_TRUE(ref_map.fuseFrameEvidence(ev).has_value());

    const auto* ref_payload = ref_map.findActivePayload(root);
    const auto* test_state = test_map.findInternalCoarseningState(root);
    ASSERT_NE(ref_payload, nullptr);
    ASSERT_NE(test_state, nullptr);
    ASSERT_TRUE(test_state->has_current_shadow);
    verifyExact8FieldEquivalence(test_state->current_shadow, *ref_payload);
  }
}

// ============================================================================
// Section 23: UINT64 Frame-Boundary Shadow Exactness
// ============================================================================

TEST(InternalCoarseningShadowTest, Uint64FrameBoundaryShadowExactness) {
  PersistentAdaptiveMap map{};
  const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};
  const auto root = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);
  ASSERT_TRUE(map.ensureRootForBase(avr::spatial::GridCoord2D{0, 0}).has_value());
  ASSERT_TRUE(map.splitLeaf(root));

  constexpr uint64_t SEQ_A = UINT64_MAX - 1;
  constexpr uint64_t SEQ_B = UINT64_MAX;

  const std::vector<PointXYZ> pt = {{0.10, 0.10, 1.50}};
  FrameEvidence ev_a, ev_b;
  ASSERT_TRUE(ev_a.buildFrame(pt, SEQ_A, origin).has_value());
  ASSERT_TRUE(ev_b.buildFrame(pt, SEQ_B, origin).has_value());

  ASSERT_TRUE(map.fuseFrameEvidence(ev_a).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev_b).has_value());

  const auto* state = map.findInternalCoarseningState(root);
  ASSERT_NE(state, nullptr);
  ASSERT_TRUE(state->has_current_shadow);
  EXPECT_EQ(state->current_shadow.lastSeenFrame(), SEQ_B);
  EXPECT_EQ(state->current_shadow.firstSeenFrame(), SEQ_A);
  EXPECT_EQ(state->current_shadow.observedFrameCount(), 2);
  EXPECT_TRUE(map.validateInvariants().is_valid);
}

// ============================================================================
// Section 13: Corrupted Shadow Strict Verification Failure
// ============================================================================

TEST(InternalCoarseningShadowTest, CorruptedShadowStrictVerificationFailure) {
  PersistentAdaptiveMap map{};
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 10});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;

  const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};
  const auto root = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);

  // Initial frames 10, 20
  const std::vector<PointXYZ> f10 = {{0.10, 0.10, 1.0}};
  const std::vector<PointXYZ> f20 = {{0.10, 0.10, 2.0}};
  cache.pushFrame(f10, 10, origin);
  cache.pushFrame(f20, 20, origin);
  FrameEvidence ev10, ev20;
  ASSERT_TRUE(ev10.buildFrame(f10, 10, origin).has_value());
  ASSERT_TRUE(ev20.buildFrame(f20, 20, origin).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev10).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev20).has_value());

  // Split and replay children
  ASSERT_TRUE(map.splitLeaf(root));
  ASSERT_EQ(map.replayDirectChildren(root, cache).status, ReplayStatus::APPLIED_COMPLETE);

  // Post-split frame 30
  const std::vector<PointXYZ> f30 = {{0.10, 0.10, 3.0}};
  cache.pushFrame(f30, 30, origin);
  FrameEvidence ev30;
  ASSERT_TRUE(ev30.buildFrame(f30, 30, origin).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev30).has_value());

  // Corrupt parent current_shadow mean slightly while keeping count, min, max, times identical
  auto& states = PersistentAdaptiveMapTestAccess::internalCoarseningStates(map);
  const auto current = states[root].current_shadow;
  states[root].current_shadow = AdaptiveCellPayload(
      ElevationStats(current.elevation().count(),
                     current.elevation().mean() + 1.0,  // Corrupted mean
                     current.elevation().m2(),
                     current.elevation().min(),
                     current.elevation().max()),
      current.firstSeenFrame(),
      current.lastSeenFrame(),
      current.observedFrameCount(),
      current.lastUpdateReasons());

  // Strict cache overload must detect parent reconstruction mismatch!
  const auto strict_res = map.mergeDirectChildren(root, cache);
  EXPECT_EQ(strict_res.status, MergeStatus::PARENT_RECONSTRUCTION_MISMATCH);
  EXPECT_FALSE(strict_res.isSuccess());
  EXPECT_TRUE(map.hierarchy().isInternal(root));
}

// ============================================================================
// Section 5, 8, 42: Validator Incomplete Matrix & Empty Shadow Checks
// ============================================================================

TEST(InternalCoarseningShadowTest, ValidatorIncompleteMatrixAndEmptyShadow) {
  PersistentAdaptiveMap map{};
  const auto root = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);
  ASSERT_TRUE(map.ensureRootForBase(avr::spatial::GridCoord2D{0, 0}).has_value());
  ASSERT_TRUE(map.splitLeaf(root));

  auto& states = PersistentAdaptiveMapTestAccess::internalCoarseningStates(map);

  // 1. Non-materialized shadow (!has_current_shadow) with non-empty payload fails
  states[root].has_current_shadow = false;
  states[root].current_shadow = AdaptiveCellPayload(
      ElevationStats(1, 1.0, 0.0, 1.0, 1.0), 10, 10, 1, ResolutionReasonBits{});
  EXPECT_FALSE(map.validateInvariants().is_valid);
  EXPECT_EQ(map.validateInvariants().code, PersistentMapValidationCode::COARSENING_SHADOW_OBSERVATIONS_INVALID);

  // Restore clean empty shadow
  states[root].current_shadow = AdaptiveCellPayload{};
  EXPECT_TRUE(map.validateInvariants().is_valid);

  // 2. INCOMPLETE base with no retired cache is valid
  states[root].base_state = CoarseningBaseState::INCOMPLETE;
  EXPECT_TRUE(map.validateInvariants().is_valid);
}

}  // namespace avr::mapping

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <random>
#include <type_traits>
#include <vector>

#include "avr_mapping/adaptive_cell_payload.hpp"
#include "avr_mapping/frame_evidence.hpp"
#include "avr_mapping/internal_coarsening_state.hpp"
#include "avr_mapping/persistent_adaptive_map.hpp"
#include "avr_mapping/recent_observation_cache.hpp"
#include "avr_mapping/subtree_collapse_result.hpp"
#include "avr_spatial/cell_key.hpp"
#include "avr_spatial/constants.hpp"
#include "avr_spatial/coordinate.hpp"

namespace avr::mapping {

struct PersistentAdaptiveMapTestAccess {
  static AdaptivePayloadStore& payloadStore(PersistentAdaptiveMap& map) noexcept {
    return map.payload_store_;
  }
  static std::unordered_map<avr::spatial::CellKey, InternalCoarseningState>& internalCoarseningStates(
      PersistentAdaptiveMap& map) noexcept {
    return map.internal_coarsening_states_;
  }
  static AdaptiveHierarchy& hierarchy(PersistentAdaptiveMap& map) noexcept {
    return map.hierarchy_;
  }
};

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
  // Bitwise representation equality
  EXPECT_EQ(actual.elevation().mean(), expected.elevation().mean());
  EXPECT_EQ(actual.elevation().m2(), expected.elevation().m2());
  EXPECT_EQ(actual.elevation().min(), expected.elevation().min());
  EXPECT_EQ(actual.elevation().max(), expected.elevation().max());
}

struct MapSnapshot {
  std::vector<std::pair<avr::spatial::CellKey, AdaptiveCellPayload>> active_payloads;
  std::vector<std::pair<avr::spatial::CellKey, AdaptiveCellPayload>> retired_caches;
  std::vector<avr::spatial::CellKey> replay_required;
  std::vector<avr::spatial::CellKey> retired_incomplete;
  std::vector<std::pair<avr::spatial::CellKey, InternalCoarseningState>> coarsening_states;
  PersistentMapStats stats;

  static MapSnapshot capture(const PersistentAdaptiveMap& map) {
    return MapSnapshot{
        .active_payloads = map.activePayloadSnapshot(),
        .retired_caches = map.retiredCacheSnapshot(),
        .replay_required = map.replayRequiredSnapshot(),
        .retired_incomplete = map.retiredCacheIncompleteSnapshot(),
        .coarsening_states = map.internalCoarseningSnapshot(),
        .stats = map.statistics(),
    };
  }

  void verifyIdentical(const MapSnapshot& other) const {
    EXPECT_EQ(active_payloads, other.active_payloads);
    EXPECT_EQ(retired_caches, other.retired_caches);
    EXPECT_EQ(replay_required, other.replay_required);
    EXPECT_EQ(retired_incomplete, other.retired_incomplete);
    EXPECT_EQ(coarsening_states, other.coarsening_states);
    EXPECT_TRUE(stats == other.stats);
  }
};

}  // namespace

// ============================================================================
// Section 33: Direct One-Level Gap Closure
// ============================================================================

TEST(PersistentSubtreeCollapseTest, DirectOneLevelGapClosure) {
  PersistentAdaptiveMap map{};
  const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};
  const avr::spatial::GridCoord2D base_coord{0, 0};
  const auto root_key = *avr::spatial::CellKey::fromGridCoord(base_coord, avr::spatial::L3);

  // Fuse frame 10 into L3 root
  const std::vector<PointXYZ> f10 = {{0.05, 0.05, 1.25}, {0.10, 0.10, 1.75}};
  FrameEvidence ev10;
  ASSERT_TRUE(ev10.buildFrame(f10, 10, origin).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev10).has_value());

  // Split root to L2 (4 leaves with requiresReplay == true)
  ASSERT_TRUE(map.splitLeaf(root_key));
  EXPECT_TRUE(map.hierarchy().isInternal(root_key));
  const auto children = *avr::spatial::childrenOf(root_key);
  for (const auto& child : children) {
    EXPECT_TRUE(map.requiresReplay(child));
    EXPECT_TRUE(map.hierarchy().isLeaf(child));
  }

  // Fuse frame 20 into children (shadow updates on root)
  const std::vector<PointXYZ> f20 = {{0.06, 0.06, 2.00}};
  FrameEvidence ev20;
  ASSERT_TRUE(ev20.buildFrame(f20, 20, origin).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev20).has_value());

  // Children still have requiresReplay == true (reconstruction not performed)
  EXPECT_TRUE(map.requiresReplay(children[0]));

  // Explicit recovery collapse
  const auto result = map.collapseSubtreeFromCompleteShadow(root_key);
  EXPECT_EQ(result.status, SubtreeCollapseStatus::APPLIED_GAP_CLOSED);
  EXPECT_TRUE(result.isSuccess());
  EXPECT_EQ(result.ancestor, root_key);
  EXPECT_EQ(result.ancestor_level, avr::spatial::L3);
  EXPECT_EQ(result.finest_removed_level, avr::spatial::L2);
  EXPECT_EQ(result.levels_collapsed, 1);
  EXPECT_EQ(result.descendant_nodes_removed, 4);
  EXPECT_EQ(result.descendant_internal_nodes_removed, 0);
  EXPECT_EQ(result.descendant_leaves_removed, 4);
  EXPECT_EQ(result.replay_required_leaves_closed, 4);
  EXPECT_EQ(result.candidate_parent_observations, 3);
  EXPECT_TRUE(result.had_materialized_shadow);
  EXPECT_TRUE(result.had_retired_cache);

  // Removed descendant keys verification
  EXPECT_EQ(result.removed_descendant_keys.size(), 4);
  EXPECT_EQ(result.removed_descendant_keys.size(), result.descendant_nodes_removed);
  EXPECT_TRUE(std::is_sorted(result.removed_descendant_keys.begin(), result.removed_descendant_keys.end()));
  EXPECT_EQ(std::adjacent_find(result.removed_descendant_keys.begin(), result.removed_descendant_keys.end()),
            result.removed_descendant_keys.end());
  EXPECT_EQ(std::find(result.removed_descendant_keys.begin(), result.removed_descendant_keys.end(), root_key),
            result.removed_descendant_keys.end());
  for (const auto& child : children) {
    EXPECT_NE(std::find(result.removed_descendant_keys.begin(), result.removed_descendant_keys.end(), child),
              result.removed_descendant_keys.end());
  }

  // Structural verification
  EXPECT_TRUE(map.hierarchy().isLeaf(root_key));
  EXPECT_FALSE(map.requiresReplay(root_key));
  EXPECT_EQ(map.findRetiredCache(root_key), nullptr);
  EXPECT_EQ(map.findInternalCoarseningState(root_key), nullptr);

  const auto* active = map.findActivePayload(root_key);
  ASSERT_NE(active, nullptr);
  EXPECT_EQ(active->observationCount(), 3);
  EXPECT_EQ(active->firstSeenFrame(), 10);
  EXPECT_EQ(active->lastSeenFrame(), 20);
  EXPECT_EQ(active->observedFrameCount(), 2);

  // All 4 children removed
  for (const auto& child : children) {
    EXPECT_EQ(map.hierarchy().nodeState(child), NodeState::ABSENT);
    EXPECT_EQ(map.findActivePayload(child), nullptr);
    EXPECT_EQ(map.findRetiredCache(child), nullptr);
    EXPECT_FALSE(map.requiresReplay(child));
  }

  EXPECT_TRUE(map.validateInvariants().is_valid);
}

// ============================================================================
// Section 34 & 35: Deep L3 -> L0 Gap Recovery Golden & Unsplit Equivalence
// ============================================================================

TEST(PersistentSubtreeCollapseTest, DeepL3ToL0GapRecoveryGoldenAndReferenceEquivalence) {
  PersistentAdaptiveMap test_map{};
  PersistentAdaptiveMap ref_map{};
  const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};
  const avr::spatial::GridCoord2D base_coord{0, 0};
  const auto l3_root = *avr::spatial::CellKey::fromGridCoord(base_coord, avr::spatial::L3);

  // 1. Build initial L3 history (frames 10, 11)
  const std::vector<PointXYZ> f10 = {{0.01, 0.01, 1.0}, {0.02, 0.02, 2.0}};
  const std::vector<PointXYZ> f11 = {{0.03, 0.03, 3.0}};
  FrameEvidence ev10, ev11;
  ASSERT_TRUE(ev10.buildFrame(f10, 10, origin).has_value());
  ASSERT_TRUE(ev11.buildFrame(f11, 11, origin).has_value());

  ASSERT_TRUE(test_map.fuseFrameEvidence(ev10).has_value());
  ASSERT_TRUE(test_map.fuseFrameEvidence(ev11).has_value());

  ASSERT_TRUE(ref_map.fuseFrameEvidence(ev10).has_value());
  ASSERT_TRUE(ref_map.fuseFrameEvidence(ev11).has_value());

  // 2. Refine a branch all the way: L3 -> L2 -> L1 -> L0 without replay
  ASSERT_TRUE(test_map.refineToLevel(base_coord, avr::spatial::L0));

  const auto l0_leaf = *test_map.hierarchy().findLeafContainingBase(base_coord);
  const auto l1_parent = *avr::spatial::parentOf(l0_leaf);
  const auto l2_parent = *avr::spatial::parentOf(l1_parent);

  EXPECT_TRUE(test_map.hierarchy().isInternal(l3_root));
  EXPECT_TRUE(test_map.hierarchy().isInternal(l2_parent));
  EXPECT_TRUE(test_map.hierarchy().isInternal(l1_parent));
  EXPECT_TRUE(test_map.hierarchy().isLeaf(l0_leaf));

  // Verify intermediate states: L3 complete base, L2 incomplete, L1 incomplete, deep leaves replay-required
  const auto* s3 = test_map.findInternalCoarseningState(l3_root);
  const auto* s2 = test_map.findInternalCoarseningState(l2_parent);
  const auto* s1 = test_map.findInternalCoarseningState(l1_parent);

  ASSERT_NE(s3, nullptr);
  ASSERT_NE(s2, nullptr);
  ASSERT_NE(s1, nullptr);

  EXPECT_EQ(s3->base_state, CoarseningBaseState::COMPLETE_RETIRED_CACHE);
  EXPECT_EQ(s2->base_state, CoarseningBaseState::INCOMPLETE);
  EXPECT_EQ(s1->base_state, CoarseningBaseState::INCOMPLETE);
  EXPECT_TRUE(test_map.requiresReplay(l0_leaf));

  // 3. Fuse additional genuine sensor frames (frames 20, 25, 30)
  const std::vector<PointXYZ> f20 = {{0.012, 0.012, 1.5}};
  const std::vector<PointXYZ> f25 = {{0.015, 0.015, 2.5}};
  const std::vector<PointXYZ> f30 = {{0.018, 0.018, 3.5}};
  FrameEvidence ev20, ev25, ev30;
  ASSERT_TRUE(ev20.buildFrame(f20, 20, origin).has_value());
  ASSERT_TRUE(ev25.buildFrame(f25, 25, origin).has_value());
  ASSERT_TRUE(ev30.buildFrame(f30, 30, origin).has_value());

  ASSERT_TRUE(test_map.fuseFrameEvidence(ev20).has_value());
  ASSERT_TRUE(test_map.fuseFrameEvidence(ev25).has_value());
  ASSERT_TRUE(test_map.fuseFrameEvidence(ev30).has_value());

  ASSERT_TRUE(ref_map.fuseFrameEvidence(ev20).has_value());
  ASSERT_TRUE(ref_map.fuseFrameEvidence(ev25).has_value());
  ASSERT_TRUE(ref_map.fuseFrameEvidence(ev30).has_value());

  // Immediately before collapse: L3 current_shadow == ref_map active L3 payload across exact 8 fields
  const auto* s3_before = test_map.findInternalCoarseningState(l3_root);
  ASSERT_NE(s3_before, nullptr);
  ASSERT_TRUE(s3_before->has_current_shadow);
  const auto* ref_payload = ref_map.findActivePayload(l3_root);
  ASSERT_NE(ref_payload, nullptr);
  verifyExact8FieldEquivalence(s3_before->current_shadow, *ref_payload);

  // 4. Collapse subtree from complete L3
  const ResolutionReasonBits collapse_reasons{ResolutionReason::DISTANCE_BASELINE};
  const auto result = test_map.collapseSubtreeFromCompleteShadow(l3_root, collapse_reasons);

  EXPECT_EQ(result.status, SubtreeCollapseStatus::APPLIED_GAP_CLOSED);
  EXPECT_TRUE(result.isSuccess());
  EXPECT_EQ(result.ancestor, l3_root);
  EXPECT_EQ(result.ancestor_level, avr::spatial::L3);
  EXPECT_EQ(result.finest_removed_level, avr::spatial::L0);
  EXPECT_EQ(result.levels_collapsed, 3);
  EXPECT_GT(result.descendant_nodes_removed, 0);
  EXPECT_GT(result.descendant_internal_nodes_removed, 0);
  EXPECT_GT(result.descendant_leaves_removed, 0);
  EXPECT_GT(result.replay_required_leaves_closed, 0);

  // Removed descendant keys verification
  EXPECT_EQ(result.removed_descendant_keys.size(), result.descendant_nodes_removed);
  EXPECT_TRUE(std::is_sorted(result.removed_descendant_keys.begin(), result.removed_descendant_keys.end()));
  EXPECT_EQ(std::adjacent_find(result.removed_descendant_keys.begin(), result.removed_descendant_keys.end()),
            result.removed_descendant_keys.end());
  EXPECT_EQ(std::find(result.removed_descendant_keys.begin(), result.removed_descendant_keys.end(), l3_root),
            result.removed_descendant_keys.end());
  for (const auto& k : result.removed_descendant_keys) {
    EXPECT_EQ(test_map.hierarchy().nodeState(k), NodeState::ABSENT);
    EXPECT_EQ(test_map.findActivePayload(k), nullptr);
    EXPECT_EQ(test_map.findRetiredCache(k), nullptr);
    EXPECT_FALSE(test_map.requiresReplay(k));
    EXPECT_EQ(test_map.findInternalCoarseningState(k), nullptr);
  }

  // 5. Verify final map state
  EXPECT_TRUE(test_map.hierarchy().isLeaf(l3_root));
  EXPECT_FALSE(test_map.requiresReplay(l3_root));
  EXPECT_EQ(test_map.findRetiredCache(l3_root), nullptr);
  EXPECT_EQ(test_map.findInternalCoarseningState(l3_root), nullptr);

  // 6. Section 35: Exact 8-field equivalence with unsplit reference
  const auto* restored = test_map.findActivePayload(l3_root);
  ASSERT_NE(restored, nullptr);
  verifyExact8FieldEquivalence(*restored, *ref_payload);
  EXPECT_EQ(restored->lastUpdateReasons(), collapse_reasons);

  // Invariant validator valid
  EXPECT_TRUE(test_map.validateInvariants().is_valid);
}

// ============================================================================
// Section 36 & 59: Long-Lived Raw-Cache-Independent Gap Recovery
// ============================================================================

TEST(PersistentSubtreeCollapseTest, LongLivedRawCacheIndependentGapRecovery) {
  PersistentAdaptiveMap test_map{};
  PersistentAdaptiveMap ref_map{};
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 3});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;

  const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};
  const avr::spatial::GridCoord2D base_coord{0, 0};
  const auto l3_root = *avr::spatial::CellKey::fromGridCoord(base_coord, avr::spatial::L3);

  // Fuse early frames 1 and 2
  for (uint64_t seq = 1; seq <= 2; ++seq) {
    const std::vector<PointXYZ> pts = {{0.01 * static_cast<double>(seq), 0.01 * static_cast<double>(seq), 1.0 + 0.1 * static_cast<double>(seq)}};
    FrameEvidence ev;
    ASSERT_TRUE(ev.buildFrame(pts, seq, origin).has_value());
    ASSERT_TRUE(test_map.fuseFrameEvidence(ev).has_value());
    ASSERT_TRUE(ref_map.fuseFrameEvidence(ev).has_value());
    ASSERT_TRUE(cache.pushFrame(pts, seq, origin).has_value());
  }

  // Deep refine to L0
  ASSERT_TRUE(test_map.refineToLevel(base_coord, avr::spatial::L0));

  // Fuse 100 subsequent real frames (seq 3 to 102)
  for (uint64_t seq = 3; seq <= 102; ++seq) {
    const std::vector<PointXYZ> pts = {{0.01, 0.01, 2.0 + 0.01 * static_cast<double>(seq)}};
    FrameEvidence ev;
    ASSERT_TRUE(ev.buildFrame(pts, seq, origin).has_value());
    ASSERT_TRUE(test_map.fuseFrameEvidence(ev).has_value());
    ASSERT_TRUE(ref_map.fuseFrameEvidence(ev).has_value());
    ASSERT_TRUE(cache.pushFrame(pts, seq, origin).has_value());
  }

  // Early raw history (seq 1, 2) is completely evicted from cache
  EXPECT_FALSE(cache.containsFrame(1));
  EXPECT_FALSE(cache.containsFrame(2));
  EXPECT_EQ(cache.frameCount(), 3);
  EXPECT_EQ(*cache.oldestFrameSeq(), 100);
  EXPECT_EQ(*cache.newestFrameSeq(), 102);

  // Collapse subtree on L3
  const auto result = test_map.collapseSubtreeFromCompleteShadow(l3_root);
  EXPECT_EQ(result.status, SubtreeCollapseStatus::APPLIED_GAP_CLOSED);
  EXPECT_TRUE(result.isSuccess());

  // Restored active payload matches unsplit reference across exact 8 fields
  const auto* restored = test_map.findActivePayload(l3_root);
  const auto* ref_payload = ref_map.findActivePayload(l3_root);
  ASSERT_NE(restored, nullptr);
  ASSERT_NE(ref_payload, nullptr);
  verifyExact8FieldEquivalence(*restored, *ref_payload);
  EXPECT_TRUE(test_map.validateInvariants().is_valid);
}

// ============================================================================
// Section 37: Intermediate Incomplete Collapse Failure
// ============================================================================

TEST(PersistentSubtreeCollapseTest, IntermediateIncompleteCollapseFailure) {
  PersistentAdaptiveMap map{};
  const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};
  const avr::spatial::GridCoord2D base_coord{0, 0};
  const auto l3_root = *avr::spatial::CellKey::fromGridCoord(base_coord, avr::spatial::L3);

  const std::vector<PointXYZ> pts = {{0.05, 0.05, 1.0}};
  FrameEvidence ev;
  ASSERT_TRUE(ev.buildFrame(pts, 10, origin).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev).has_value());

  ASSERT_TRUE(map.refineToLevel(base_coord, avr::spatial::L0));

  const auto l0_leaf = *map.hierarchy().findLeafContainingBase(base_coord);
  const auto l1_parent = *avr::spatial::parentOf(l0_leaf);
  const auto l2_parent = *avr::spatial::parentOf(l1_parent);

  const auto snap_before = MapSnapshot::capture(map);

  // Attempt collapse on intermediate incomplete L2
  const auto res_l2 = map.collapseSubtreeFromCompleteShadow(l2_parent);
  EXPECT_EQ(res_l2.status, SubtreeCollapseStatus::ANCESTOR_BASE_INCOMPLETE);
  EXPECT_FALSE(res_l2.isSuccess());

  // Zero mutation on map
  const auto snap_after_l2 = MapSnapshot::capture(map);
  snap_before.verifyIdentical(snap_after_l2);

  // Attempt collapse on intermediate incomplete L1
  const auto res_l1 = map.collapseSubtreeFromCompleteShadow(l1_parent);
  EXPECT_EQ(res_l1.status, SubtreeCollapseStatus::ANCESTOR_BASE_INCOMPLETE);
  EXPECT_FALSE(res_l1.isSuccess());

  // Zero mutation on map
  const auto snap_after_l1 = MapSnapshot::capture(map);
  snap_before.verifyIdentical(snap_after_l1);

  // Now collapse complete L3 root: succeeds!
  const auto res_l3 = map.collapseSubtreeFromCompleteShadow(l3_root);
  EXPECT_EQ(res_l3.status, SubtreeCollapseStatus::APPLIED_GAP_CLOSED);
  EXPECT_TRUE(res_l3.isSuccess());
  EXPECT_TRUE(map.hierarchy().isLeaf(l3_root));
  EXPECT_TRUE(map.validateInvariants().is_valid);
}

// ============================================================================
// Section 38: Complete-Retired Without Materialized Shadow
// ============================================================================

TEST(PersistentSubtreeCollapseTest, CompleteRetiredWithoutMaterializedShadow) {
  PersistentAdaptiveMap map{};
  const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};
  const avr::spatial::GridCoord2D base_coord{0, 0};
  const auto root_key = *avr::spatial::CellKey::fromGridCoord(base_coord, avr::spatial::L3);

  const std::vector<PointXYZ> f10 = {{0.05, 0.05, 1.5}, {0.08, 0.08, 2.5}};
  FrameEvidence ev10;
  ASSERT_TRUE(ev10.buildFrame(f10, 10, origin).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev10).has_value());

  const auto original_l3_payload = *map.findActivePayload(root_key);

  // Split root to L2, no frames fused afterward
  ASSERT_TRUE(map.splitLeaf(root_key));
  const auto* state = map.findInternalCoarseningState(root_key);
  ASSERT_NE(state, nullptr);
  EXPECT_FALSE(state->has_current_shadow);
  EXPECT_EQ(state->base_state, CoarseningBaseState::COMPLETE_RETIRED_CACHE);

  // Collapse
  const auto result = map.collapseSubtreeFromCompleteShadow(root_key);
  EXPECT_EQ(result.status, SubtreeCollapseStatus::APPLIED_GAP_CLOSED);
  EXPECT_TRUE(result.isSuccess());
  EXPECT_FALSE(result.had_materialized_shadow);
  EXPECT_TRUE(result.had_retired_cache);

  // Restored active payload equals original retired base
  const auto* restored = map.findActivePayload(root_key);
  ASSERT_NE(restored, nullptr);
  verifyExact8FieldEquivalence(*restored, original_l3_payload);
  EXPECT_TRUE(map.validateInvariants().is_valid);
}

// ============================================================================
// Section 39: Complete-Empty Without Shadow
// ============================================================================

TEST(PersistentSubtreeCollapseTest, CompleteEmptyWithoutShadow) {
  PersistentAdaptiveMap map{};
  const avr::spatial::GridCoord2D base_coord{0, 0};
  const auto root_key = *avr::spatial::CellKey::fromGridCoord(base_coord, avr::spatial::L3);

  // Allocate empty root and refine to L1
  ASSERT_TRUE(map.refineToLevel(base_coord, avr::spatial::L1));
  EXPECT_TRUE(map.hierarchy().isInternal(root_key));

  const auto* state = map.findInternalCoarseningState(root_key);
  ASSERT_NE(state, nullptr);
  EXPECT_EQ(state->base_state, CoarseningBaseState::COMPLETE_EMPTY);
  EXPECT_FALSE(state->has_current_shadow);

  // Collapse
  const auto result = map.collapseSubtreeFromCompleteShadow(root_key);
  EXPECT_EQ(result.status, SubtreeCollapseStatus::APPLIED_GAP_CLOSED);
  EXPECT_TRUE(result.isSuccess());
  EXPECT_FALSE(result.had_materialized_shadow);
  EXPECT_FALSE(result.had_retired_cache);
  EXPECT_EQ(result.candidate_parent_observations, 0);

  // Ancestor restored as LEAF with no active payload and replay false
  EXPECT_TRUE(map.hierarchy().isLeaf(root_key));
  EXPECT_FALSE(map.requiresReplay(root_key));
  EXPECT_EQ(map.findActivePayload(root_key), nullptr);
  EXPECT_EQ(map.findRetiredCache(root_key), nullptr);
  EXPECT_EQ(map.findInternalCoarseningState(root_key), nullptr);
  EXPECT_TRUE(map.validateInvariants().is_valid);
}

// ============================================================================
// Section 40: Complete-Empty With New Fine Observations
// ============================================================================

TEST(PersistentSubtreeCollapseTest, CompleteEmptyWithNewFineObservations) {
  PersistentAdaptiveMap test_map{};
  PersistentAdaptiveMap ref_map{};
  const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};
  const avr::spatial::GridCoord2D base_coord{0, 0};
  const auto root_key = *avr::spatial::CellKey::fromGridCoord(base_coord, avr::spatial::L3);

  // Test map: start empty, refine to L1
  ASSERT_TRUE(test_map.refineToLevel(base_coord, avr::spatial::L1));

  // Fuse frame 10 with observations in test_map and ref_map
  const std::vector<PointXYZ> f10 = {{0.05, 0.05, 3.0}, {0.06, 0.06, 4.0}};
  FrameEvidence ev10;
  ASSERT_TRUE(ev10.buildFrame(f10, 10, origin).has_value());
  ASSERT_TRUE(test_map.fuseFrameEvidence(ev10).has_value());
  ASSERT_TRUE(ref_map.fuseFrameEvidence(ev10).has_value());

  const auto* state = test_map.findInternalCoarseningState(root_key);
  ASSERT_NE(state, nullptr);
  EXPECT_EQ(state->base_state, CoarseningBaseState::COMPLETE_EMPTY);
  EXPECT_TRUE(state->has_current_shadow);

  // Collapse
  const auto result = test_map.collapseSubtreeFromCompleteShadow(root_key);
  EXPECT_EQ(result.status, SubtreeCollapseStatus::APPLIED_GAP_CLOSED);
  EXPECT_TRUE(result.isSuccess());

  const auto* restored = test_map.findActivePayload(root_key);
  const auto* ref_payload = ref_map.findActivePayload(root_key);
  ASSERT_NE(restored, nullptr);
  ASSERT_NE(ref_payload, nullptr);
  verifyExact8FieldEquivalence(*restored, *ref_payload);
  EXPECT_TRUE(test_map.validateInvariants().is_valid);
}

// ============================================================================
// Section 41: Descendant Count Greater Than Shadow Failure
// ============================================================================

TEST(PersistentSubtreeCollapseTest, DescendantCountGreaterThanShadowFailure) {
  PersistentAdaptiveMap map{};
  const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};
  const avr::spatial::GridCoord2D base_coord{0, 0};
  const auto root_key = *avr::spatial::CellKey::fromGridCoord(base_coord, avr::spatial::L3);

  const std::vector<PointXYZ> f10 = {{0.05, 0.05, 1.0}};
  FrameEvidence ev10;
  ASSERT_TRUE(ev10.buildFrame(f10, 10, origin).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev10).has_value());
  ASSERT_TRUE(map.splitLeaf(root_key));

  const auto children = *avr::spatial::childrenOf(root_key);
  // Inject descendant payload with count > candidate count (candidate has count 1)
  ElevationStats corrupt_elevation;
  corrupt_elevation.update(1.0);
  corrupt_elevation.update(1.5);  // count = 2
  AdaptiveCellPayload corrupt_payload(corrupt_elevation, 10, 10, 1, {});

  PersistentAdaptiveMapTestAccess::payloadStore(map).setActivePayload(children[0], corrupt_payload);
  const auto snap_before = MapSnapshot::capture(map);

  const auto result = map.collapseSubtreeFromCompleteShadow(root_key);
  EXPECT_EQ(result.status, SubtreeCollapseStatus::DESCENDANT_COVERAGE_INCONSISTENT);
  EXPECT_FALSE(result.isSuccess());
  EXPECT_TRUE(result.removed_descendant_keys.empty());

  const auto snap_after = MapSnapshot::capture(map);
  snap_before.verifyIdentical(snap_after);
}

// ============================================================================
// Section 42: Descendant Newer Than Shadow Failure
// ============================================================================

TEST(PersistentSubtreeCollapseTest, DescendantNewerThanShadowFailure) {
  PersistentAdaptiveMap map{};
  const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};
  const avr::spatial::GridCoord2D base_coord{0, 0};
  const auto root_key = *avr::spatial::CellKey::fromGridCoord(base_coord, avr::spatial::L3);

  const std::vector<PointXYZ> f10 = {{0.05, 0.05, 1.0}};
  FrameEvidence ev10;
  ASSERT_TRUE(ev10.buildFrame(f10, 10, origin).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev10).has_value());
  ASSERT_TRUE(map.splitLeaf(root_key));

  const auto children = *avr::spatial::childrenOf(root_key);

  // Candidate last_seen_frame is 10. Inject descendant with last_seen_frame = 20.
  ElevationStats corrupt_elevation;
  corrupt_elevation.update(1.0);
  AdaptiveCellPayload corrupt_payload(corrupt_elevation, 20, 20, 1, {});

  PersistentAdaptiveMapTestAccess::payloadStore(map).setActivePayload(children[0], corrupt_payload);
  const auto snap_before = MapSnapshot::capture(map);

  const auto result = map.collapseSubtreeFromCompleteShadow(root_key);
  EXPECT_EQ(result.status, SubtreeCollapseStatus::DESCENDANT_COVERAGE_INCONSISTENT);
  EXPECT_FALSE(result.isSuccess());
  EXPECT_TRUE(result.removed_descendant_keys.empty());

  const auto snap_after = MapSnapshot::capture(map);
  snap_before.verifyIdentical(snap_after);
}

// ============================================================================
// Section 43: Descendant Range Outside Shadow Failure
// ============================================================================

TEST(PersistentSubtreeCollapseTest, DescendantRangeOutsideShadowFailure) {
  PersistentAdaptiveMap map{};
  const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};
  const avr::spatial::GridCoord2D base_coord{0, 0};
  const auto root_key = *avr::spatial::CellKey::fromGridCoord(base_coord, avr::spatial::L3);

  // Candidate z range: [1.0, 2.0]
  const std::vector<PointXYZ> f10 = {{0.05, 0.05, 1.0}, {0.06, 0.06, 2.0}};
  FrameEvidence ev10;
  ASSERT_TRUE(ev10.buildFrame(f10, 10, origin).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev10).has_value());
  ASSERT_TRUE(map.splitLeaf(root_key));

  const auto children = *avr::spatial::childrenOf(root_key);

  // 1. Descendant min_z < candidate min_z (0.5 < 1.0)
  {
    ElevationStats corrupt_elevation;
    corrupt_elevation.update(0.5);
    AdaptiveCellPayload corrupt_payload(corrupt_elevation, 10, 10, 1, {});
    PersistentAdaptiveMapTestAccess::payloadStore(map).setActivePayload(children[0], corrupt_payload);

    const auto result = map.collapseSubtreeFromCompleteShadow(root_key);
    EXPECT_EQ(result.status, SubtreeCollapseStatus::DESCENDANT_COVERAGE_INCONSISTENT);
    EXPECT_FALSE(result.isSuccess());
    EXPECT_TRUE(result.removed_descendant_keys.empty());
    PersistentAdaptiveMapTestAccess::payloadStore(map).clearCell(children[0]);
  }

  // 2. Descendant max_z > candidate max_z (2.5 > 2.0)
  {
    ElevationStats corrupt_elevation;
    corrupt_elevation.update(2.5);
    AdaptiveCellPayload corrupt_payload(corrupt_elevation, 10, 10, 1, {});
    PersistentAdaptiveMapTestAccess::payloadStore(map).setActivePayload(children[0], corrupt_payload);

    const auto result = map.collapseSubtreeFromCompleteShadow(root_key);
    EXPECT_EQ(result.status, SubtreeCollapseStatus::DESCENDANT_COVERAGE_INCONSISTENT);
    EXPECT_FALSE(result.isSuccess());
    EXPECT_TRUE(result.removed_descendant_keys.empty());
    PersistentAdaptiveMapTestAccess::payloadStore(map).clearCell(children[0]);
  }
}

// ============================================================================
// Section 44: Valid Partial Descendant History
// ============================================================================

TEST(PersistentSubtreeCollapseTest, ValidPartialDescendantHistory) {
  PersistentAdaptiveMap map{};
  const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};
  const avr::spatial::GridCoord2D base_coord{0, 0};
  const auto root_key = *avr::spatial::CellKey::fromGridCoord(base_coord, avr::spatial::L3);

  // Candidate parent has 10 observations across frames 10 to 12
  for (uint64_t seq = 10; seq <= 12; ++seq) {
    const std::vector<PointXYZ> f = {
        {0.05, 0.05, 1.0 + 0.1 * static_cast<double>(seq)},
        {0.06, 0.06, 1.1 + 0.1 * static_cast<double>(seq)},
        {0.07, 0.07, 1.2 + 0.1 * static_cast<double>(seq)}};
    FrameEvidence ev;
    ASSERT_TRUE(ev.buildFrame(f, seq, origin).has_value());
    ASSERT_TRUE(map.fuseFrameEvidence(ev).has_value());
  }

  ASSERT_TRUE(map.splitLeaf(root_key));

  // Fuse frame 13 into one child (adds 1 point)
  const std::vector<PointXYZ> f13 = {{0.05, 0.05, 1.5}};
  FrameEvidence ev13;
  ASSERT_TRUE(ev13.buildFrame(f13, 13, origin).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev13).has_value());

  // Active descendants have strictly less observations (only 1 point) than parent candidate (10 points)
  const auto result = map.collapseSubtreeFromCompleteShadow(root_key);
  EXPECT_EQ(result.status, SubtreeCollapseStatus::APPLIED_GAP_CLOSED);
  EXPECT_TRUE(result.isSuccess());
  EXPECT_TRUE(map.hierarchy().isLeaf(root_key));
  EXPECT_EQ(map.findActivePayload(root_key)->observationCount(), 10);
  EXPECT_TRUE(map.validateInvariants().is_valid);
}

// ============================================================================
// Section 45: Replay-Complete Descendants Also Work
// ============================================================================

TEST(PersistentSubtreeCollapseTest, ReplayCompleteDescendantsAlsoWork) {
  PersistentAdaptiveMap map{};
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 10});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;
  const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};
  const avr::spatial::GridCoord2D base_coord{0, 0};
  const auto root_key = *avr::spatial::CellKey::fromGridCoord(base_coord, avr::spatial::L3);

  const std::vector<PointXYZ> f10 = {{0.05, 0.05, 1.5}};
  FrameEvidence ev10;
  ASSERT_TRUE(ev10.buildFrame(f10, 10, origin).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev10).has_value());
  ASSERT_TRUE(cache.pushFrame(f10, 10, origin).has_value());

  ASSERT_TRUE(map.splitLeaf(root_key));

  // Replay children to completion
  const auto replay_res = map.replayDirectChildren(root_key, cache);
  EXPECT_EQ(replay_res.status, ReplayStatus::APPLIED_COMPLETE);

  const auto children = *avr::spatial::childrenOf(root_key);
  for (const auto& child : children) {
    EXPECT_FALSE(map.requiresReplay(child));
  }

  // Generic collapse primitive still works on replay-complete descendants
  const auto result = map.collapseSubtreeFromCompleteShadow(root_key);
  EXPECT_EQ(result.status, SubtreeCollapseStatus::APPLIED_GAP_CLOSED);
  EXPECT_TRUE(result.isSuccess());
  EXPECT_EQ(result.replay_required_leaves_closed, 0);
  EXPECT_TRUE(map.hierarchy().isLeaf(root_key));
  EXPECT_TRUE(map.validateInvariants().is_valid);
}

// ============================================================================
// Section 46: Structural Depth Variants (L3->L2, L3->L1, L3->L0, L2->L0)
// ============================================================================

TEST(PersistentSubtreeCollapseTest, StructuralDepthVariants) {
  const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};

  // Variant 1: L3 -> L2 (1 level)
  {
    PersistentAdaptiveMap map{};
    const avr::spatial::GridCoord2D coord{0, 0};
    const auto root = *avr::spatial::CellKey::fromGridCoord(coord, avr::spatial::L3);
    const std::vector<PointXYZ> pts = {{0.05, 0.05, 1.0}};
    FrameEvidence ev;
    ASSERT_TRUE(ev.buildFrame(pts, 10, origin).has_value());
    ASSERT_TRUE(map.fuseFrameEvidence(ev).has_value());
    ASSERT_TRUE(map.refineToLevel(coord, avr::spatial::L2));

    const auto res = map.collapseSubtreeFromCompleteShadow(root);
    EXPECT_EQ(res.status, SubtreeCollapseStatus::APPLIED_GAP_CLOSED);
    EXPECT_EQ(res.levels_collapsed, 1);
    EXPECT_TRUE(map.hierarchy().isLeaf(root));
    EXPECT_TRUE(map.validateInvariants().is_valid);
  }

  // Variant 2: L3 -> L1 (2 levels)
  {
    PersistentAdaptiveMap map{};
    const avr::spatial::GridCoord2D coord{0, 0};
    const auto root = *avr::spatial::CellKey::fromGridCoord(coord, avr::spatial::L3);
    const std::vector<PointXYZ> pts = {{0.05, 0.05, 1.0}};
    FrameEvidence ev;
    ASSERT_TRUE(ev.buildFrame(pts, 10, origin).has_value());
    ASSERT_TRUE(map.fuseFrameEvidence(ev).has_value());
    ASSERT_TRUE(map.refineToLevel(coord, avr::spatial::L1));

    const auto res = map.collapseSubtreeFromCompleteShadow(root);
    EXPECT_EQ(res.status, SubtreeCollapseStatus::APPLIED_GAP_CLOSED);
    EXPECT_EQ(res.levels_collapsed, 2);
    EXPECT_TRUE(map.hierarchy().isLeaf(root));
    EXPECT_TRUE(map.validateInvariants().is_valid);
  }

  // Variant 3: L3 -> L0 (3 levels)
  {
    PersistentAdaptiveMap map{};
    const avr::spatial::GridCoord2D coord{0, 0};
    const auto root = *avr::spatial::CellKey::fromGridCoord(coord, avr::spatial::L3);
    const std::vector<PointXYZ> pts = {{0.05, 0.05, 1.0}};
    FrameEvidence ev;
    ASSERT_TRUE(ev.buildFrame(pts, 10, origin).has_value());
    ASSERT_TRUE(map.fuseFrameEvidence(ev).has_value());
    ASSERT_TRUE(map.refineToLevel(coord, avr::spatial::L0));

    const auto res = map.collapseSubtreeFromCompleteShadow(root);
    EXPECT_EQ(res.status, SubtreeCollapseStatus::APPLIED_GAP_CLOSED);
    EXPECT_EQ(res.levels_collapsed, 3);
    EXPECT_TRUE(map.hierarchy().isLeaf(root));
    EXPECT_TRUE(map.validateInvariants().is_valid);
  }

  // Variant 4: L2 -> L0 (2 levels, where L2 has COMPLETE base)
  {
    PersistentAdaptiveMap map{};
    auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 10});
    ASSERT_TRUE(cache_opt.has_value());
    auto& cache = *cache_opt;
    const avr::spatial::GridCoord2D coord{0, 0};
    const auto l3_root = *avr::spatial::CellKey::fromGridCoord(coord, avr::spatial::L3);
    const std::vector<PointXYZ> pts10 = {{0.05, 0.05, 1.0}};
    FrameEvidence ev10;
    ASSERT_TRUE(ev10.buildFrame(pts10, 10, origin).has_value());
    ASSERT_TRUE(map.fuseFrameEvidence(ev10).has_value());
    ASSERT_TRUE(cache.pushFrame(pts10, 10, origin).has_value());

    // Split L3 -> L2 and replay L2 to completion so L2 has COMPLETE active payload
    ASSERT_TRUE(map.splitLeaf(l3_root));
    ASSERT_EQ(map.replayDirectChildren(l3_root, cache).status, ReplayStatus::APPLIED_COMPLETE);

    const auto l2_leaf = *map.hierarchy().findLeafContainingBase(coord);
    EXPECT_FALSE(map.requiresReplay(l2_leaf));
    EXPECT_NE(map.findActivePayload(l2_leaf), nullptr);

    // Refine L2 to L0
    ASSERT_TRUE(map.refineToLevel(coord, avr::spatial::L0));
    EXPECT_TRUE(map.hierarchy().isInternal(l2_leaf));
    const auto* l2_coarsen = map.findInternalCoarseningState(l2_leaf);
    ASSERT_NE(l2_coarsen, nullptr);
    EXPECT_EQ(l2_coarsen->base_state, CoarseningBaseState::COMPLETE_RETIRED_CACHE);

    // Collapse L2 subtree
    const auto res = map.collapseSubtreeFromCompleteShadow(l2_leaf);
    EXPECT_EQ(res.status, SubtreeCollapseStatus::APPLIED_GAP_CLOSED);
    EXPECT_EQ(res.levels_collapsed, 2);
    EXPECT_TRUE(map.hierarchy().isLeaf(l2_leaf));
    EXPECT_TRUE(map.hierarchy().isInternal(l3_root));
    EXPECT_TRUE(map.validateInvariants().is_valid);
  }
}

// ============================================================================
// Section 47: Negative Coordinates
// ============================================================================

TEST(PersistentSubtreeCollapseTest, NegativeCoordinates) {
  PersistentAdaptiveMap test_map{};
  PersistentAdaptiveMap ref_map{};
  const auto origin = avr::spatial::WorldPoint2D{-50.0, -50.0};
  const avr::spatial::GridCoord2D base_coord{-500, -500};
  const auto root_key = *avr::spatial::CellKey::fromGridCoord(base_coord, avr::spatial::L3);

  // Continuous coordinates corresponding to base_coord
  const double wx = static_cast<double>(base_coord.x) * avr::spatial::BASE_RESOLUTION_METERS;
  const double wy = static_cast<double>(base_coord.y) * avr::spatial::BASE_RESOLUTION_METERS;
  const std::vector<PointXYZ> f10 = {{wx + 0.01, wy + 0.01, -2.5}};
  FrameEvidence ev10;
  ASSERT_TRUE(ev10.buildFrame(f10, 10, origin).has_value());
  ASSERT_TRUE(test_map.fuseFrameEvidence(ev10).has_value());
  ASSERT_TRUE(ref_map.fuseFrameEvidence(ev10).has_value());

  // Deep refine in negative space
  ASSERT_TRUE(test_map.refineToLevel(base_coord, avr::spatial::L0));

  // Fuse new frame
  const std::vector<PointXYZ> f20 = {{wx + 0.02, wy + 0.02, -1.5}};
  FrameEvidence ev20;
  ASSERT_TRUE(ev20.buildFrame(f20, 20, origin).has_value());
  ASSERT_TRUE(test_map.fuseFrameEvidence(ev20).has_value());
  ASSERT_TRUE(ref_map.fuseFrameEvidence(ev20).has_value());

  // Collapse
  const auto result = test_map.collapseSubtreeFromCompleteShadow(root_key);
  EXPECT_EQ(result.status, SubtreeCollapseStatus::APPLIED_GAP_CLOSED);
  EXPECT_TRUE(result.isSuccess());

  const auto* restored = test_map.findActivePayload(root_key);
  const auto* ref_payload = ref_map.findActivePayload(root_key);
  ASSERT_NE(restored, nullptr);
  ASSERT_NE(ref_payload, nullptr);
  verifyExact8FieldEquivalence(*restored, *ref_payload);
  EXPECT_TRUE(test_map.validateInvariants().is_valid);
}

// ============================================================================
// Section 48: Grandparent Shadow Immutability
// ============================================================================

TEST(PersistentSubtreeCollapseTest, GrandparentShadowImmutability) {
  PersistentAdaptiveMap map{};
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 10});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;
  const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};
  const avr::spatial::GridCoord2D coord{0, 0};
  const auto l3_root = *avr::spatial::CellKey::fromGridCoord(coord, avr::spatial::L3);

  const std::vector<PointXYZ> pts10 = {{0.05, 0.05, 1.0}};
  FrameEvidence ev10;
  ASSERT_TRUE(ev10.buildFrame(pts10, 10, origin).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev10).has_value());
  ASSERT_TRUE(cache.pushFrame(pts10, 10, origin).has_value());

  // Split L3 -> L2 and replay
  ASSERT_TRUE(map.splitLeaf(l3_root));
  ASSERT_EQ(map.replayDirectChildren(l3_root, cache).status, ReplayStatus::APPLIED_COMPLETE);

  const auto l2_leaf = *map.hierarchy().findLeafContainingBase(coord);
  // Refine L2 -> L0
  ASSERT_TRUE(map.refineToLevel(coord, avr::spatial::L0));

  // Fuse new frame 20 into descendants (both L2 and L3 shadows materialize)
  const std::vector<PointXYZ> pts20 = {{0.06, 0.06, 2.0}};
  FrameEvidence ev20;
  ASSERT_TRUE(ev20.buildFrame(pts20, 20, origin).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev20).has_value());

  // Capture L3 state before collapsing L2
  const auto* l3_state_before = map.findInternalCoarseningState(l3_root);
  ASSERT_NE(l3_state_before, nullptr);
  const auto l3_shadow_before = l3_state_before->current_shadow;
  const auto l3_base_before = l3_state_before->base_state;
  const bool l3_has_shadow_before = l3_state_before->has_current_shadow;

  // Collapse L2 subtree
  const auto res = map.collapseSubtreeFromCompleteShadow(l2_leaf);
  EXPECT_EQ(res.status, SubtreeCollapseStatus::APPLIED_GAP_CLOSED);
  EXPECT_TRUE(res.isSuccess());

  // L3 shadow and coarsening state must be bit-for-bit identical!
  const auto* l3_state_after = map.findInternalCoarseningState(l3_root);
  ASSERT_NE(l3_state_after, nullptr);
  EXPECT_EQ(l3_state_after->base_state, l3_base_before);
  EXPECT_EQ(l3_state_after->has_current_shadow, l3_has_shadow_before);
  verifyExact8FieldEquivalence(l3_state_after->current_shadow, l3_shadow_before);

  EXPECT_TRUE(map.hierarchy().isInternal(l3_root));
  EXPECT_TRUE(map.hierarchy().isLeaf(l2_leaf));
  EXPECT_TRUE(map.validateInvariants().is_valid);
}

// ============================================================================
// Section 49 & 50: Re-Split After Gap Closure & Short-History M8B Replay
// ============================================================================

TEST(PersistentSubtreeCollapseTest, ReSplitAfterGapClosureAndShortHistoryReplay) {
  PersistentAdaptiveMap map{};
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 10});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;
  const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};
  const avr::spatial::GridCoord2D coord{0, 0};
  const auto l3_root = *avr::spatial::CellKey::fromGridCoord(coord, avr::spatial::L3);

  // Short history in cache (frame 10, 20)
  const std::vector<PointXYZ> f10 = {{0.05, 0.05, 1.0}};
  FrameEvidence ev10;
  ASSERT_TRUE(ev10.buildFrame(f10, 10, origin).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev10).has_value());
  ASSERT_TRUE(cache.pushFrame(f10, 10, origin).has_value());

  // Deep refine without replay
  ASSERT_TRUE(map.refineToLevel(coord, avr::spatial::L0));

  const std::vector<PointXYZ> f20 = {{0.06, 0.06, 2.0}};
  FrameEvidence ev20;
  ASSERT_TRUE(ev20.buildFrame(f20, 20, origin).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev20).has_value());
  ASSERT_TRUE(cache.pushFrame(f20, 20, origin).has_value());

  // 1. Collapse subtree
  const auto collapse_res = map.collapseSubtreeFromCompleteShadow(l3_root);
  EXPECT_EQ(collapse_res.status, SubtreeCollapseStatus::APPLIED_GAP_CLOSED);
  EXPECT_TRUE(map.hierarchy().isLeaf(l3_root));

  // 2. Re-split restored ancestor
  ASSERT_TRUE(map.splitLeaf(l3_root));
  EXPECT_TRUE(map.hierarchy().isInternal(l3_root));

  const auto* new_coarsen = map.findInternalCoarseningState(l3_root);
  ASSERT_NE(new_coarsen, nullptr);
  EXPECT_EQ(new_coarsen->base_state, CoarseningBaseState::COMPLETE_RETIRED_CACHE);
  EXPECT_FALSE(new_coarsen->has_current_shadow);

  const auto* retired = map.findRetiredCache(l3_root);
  ASSERT_NE(retired, nullptr);
  EXPECT_EQ(retired->observationCount(), 2);

  const auto children = *avr::spatial::childrenOf(l3_root);
  for (const auto& child : children) {
    EXPECT_TRUE(map.requiresReplay(child));
  }

  // 3. Section 50: Short-history M8B replay succeeds cleanly
  const auto replay_res = map.replayDirectChildren(l3_root, cache);
  EXPECT_EQ(replay_res.status, ReplayStatus::APPLIED_COMPLETE);
  EXPECT_TRUE(replay_res.isSuccess());

  for (const auto& child : children) {
    EXPECT_FALSE(map.requiresReplay(child));
  }
  EXPECT_TRUE(map.validateInvariants().is_valid);
}

// ============================================================================
// Section 27 & 54: Ingestion Counters & Structural Stats
// ============================================================================

TEST(PersistentSubtreeCollapseTest, IngestionCountersUnchangedAndStructuralStatsReported) {
  PersistentAdaptiveMap map{};
  const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};
  const avr::spatial::GridCoord2D coord{0, 0};
  const auto root_key = *avr::spatial::CellKey::fromGridCoord(coord, avr::spatial::L3);

  const std::vector<PointXYZ> f10 = {{0.05, 0.05, 1.0}, {0.06, 0.06, 2.0}};
  FrameEvidence ev10;
  ASSERT_TRUE(ev10.buildFrame(f10, 10, origin).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev10).has_value());

  ASSERT_TRUE(map.refineToLevel(coord, avr::spatial::L1));

  const uint64_t frames_before = map.totalFramesFused();
  const uint64_t obs_before = map.totalObservationsFused();
  const uint64_t last_seq_before = map.lastFusedFrameSeq();
  const bool has_fused_before = map.hasFusedFrames();

  const auto result = map.collapseSubtreeFromCompleteShadow(root_key);
  EXPECT_EQ(result.status, SubtreeCollapseStatus::APPLIED_GAP_CLOSED);

  // Ingestion counters strictly unchanged
  EXPECT_EQ(map.totalFramesFused(), frames_before);
  EXPECT_EQ(map.totalObservationsFused(), obs_before);
  EXPECT_EQ(map.lastFusedFrameSeq(), last_seq_before);
  EXPECT_EQ(map.hasFusedFrames(), has_fused_before);

  // Structural diagnostics verified
  EXPECT_EQ(result.ancestor, root_key);
  EXPECT_EQ(result.ancestor_level, avr::spatial::L3);
  EXPECT_EQ(result.finest_removed_level, avr::spatial::L1);
  EXPECT_EQ(result.levels_collapsed, 2);
  EXPECT_EQ(result.descendant_nodes_removed, 8); // 3 L2 leaves + 1 L2 internal + 4 L1 leaves
  EXPECT_EQ(result.descendant_internal_nodes_removed, 1); // 1 L2 internal
  EXPECT_EQ(result.descendant_leaves_removed, 7); // 3 L2 leaves + 4 L1 leaves
  EXPECT_EQ(result.candidate_parent_observations, 2);
  EXPECT_FALSE(result.had_materialized_shadow);
  EXPECT_TRUE(result.had_retired_cache);
}

// ============================================================================
// Section 56: Transaction Failure Matrix (Zero Mutation)
// ============================================================================

TEST(PersistentSubtreeCollapseTest, TransactionFailureMatrixZeroMutation) {
  PersistentAdaptiveMap map{};
  const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};
  const avr::spatial::GridCoord2D coord{0, 0};
  const auto root_key = *avr::spatial::CellKey::fromGridCoord(coord, avr::spatial::L3);

  const std::vector<PointXYZ> f10 = {{0.05, 0.05, 1.0}};
  FrameEvidence ev10;
  ASSERT_TRUE(ev10.buildFrame(f10, 10, origin).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev10).has_value());
  ASSERT_TRUE(map.refineToLevel(coord, avr::spatial::L1));

  const auto snap_baseline = MapSnapshot::capture(map);

  // 1. Invalid ancestor level (L0 cannot be an internal ancestor)
  {
    const auto l0_key = *avr::spatial::CellKey::fromGridCoord(coord, avr::spatial::L0);
    const auto res = map.collapseSubtreeFromCompleteShadow(l0_key);
    EXPECT_EQ(res.status, SubtreeCollapseStatus::INVALID_ANCESTOR);
    EXPECT_TRUE(res.removed_descendant_keys.empty());
    snap_baseline.verifyIdentical(MapSnapshot::capture(map));
  }

  // 2. Ancestor absent
  {
    const auto absent_key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{5000, 5000}, avr::spatial::L3);
    const auto res = map.collapseSubtreeFromCompleteShadow(absent_key);
    EXPECT_EQ(res.status, SubtreeCollapseStatus::INVALID_ANCESTOR);
    EXPECT_TRUE(res.removed_descendant_keys.empty());
    snap_baseline.verifyIdentical(MapSnapshot::capture(map));
  }

  // 3. Ancestor is a LEAF
  {
    const auto leaf_opt = map.hierarchy().findLeafContainingBase(coord);
    ASSERT_TRUE(leaf_opt.has_value());
    const auto res = map.collapseSubtreeFromCompleteShadow(*leaf_opt);
    EXPECT_EQ(res.status, SubtreeCollapseStatus::ANCESTOR_NOT_INTERNAL);
    EXPECT_TRUE(res.removed_descendant_keys.empty());
    snap_baseline.verifyIdentical(MapSnapshot::capture(map));
  }

  // 4. Ancestor INCOMPLETE
  {
    const auto l2_internal = *avr::spatial::parentOf(*map.hierarchy().findLeafContainingBase(coord));
    const auto res = map.collapseSubtreeFromCompleteShadow(l2_internal);
    EXPECT_EQ(res.status, SubtreeCollapseStatus::ANCESTOR_BASE_INCOMPLETE);
    EXPECT_TRUE(res.removed_descendant_keys.empty());
    snap_baseline.verifyIdentical(MapSnapshot::capture(map));
  }

  // 5. Active payload on internal ancestor
  {
    PersistentAdaptiveMap test_map{};
    FrameEvidence ev;
    ASSERT_TRUE(ev.buildFrame(f10, 10, origin).has_value());
    ASSERT_TRUE(test_map.fuseFrameEvidence(ev).has_value());
    ASSERT_TRUE(test_map.refineToLevel(coord, avr::spatial::L1));

    ElevationStats elev;
    elev.update(1.0);
    AdaptiveCellPayload bogus(elev, 10, 10, 1, {});
    PersistentAdaptiveMapTestAccess::payloadStore(test_map).setActivePayload(root_key, bogus);

    const auto snap_before = MapSnapshot::capture(test_map);
    const auto res = test_map.collapseSubtreeFromCompleteShadow(root_key);
    EXPECT_EQ(res.status, SubtreeCollapseStatus::PAYLOAD_STATE_INVALID);
    EXPECT_TRUE(res.removed_descendant_keys.empty());
    snap_before.verifyIdentical(MapSnapshot::capture(test_map));
  }

  // 6. Complete empty with retired cache
  {
    PersistentAdaptiveMap empty_map{};
    ASSERT_TRUE(empty_map.refineToLevel(coord, avr::spatial::L1));

    ElevationStats elev;
    elev.update(1.0);
    AdaptiveCellPayload bogus(elev, 10, 10, 1, {});
    PersistentAdaptiveMapTestAccess::payloadStore(empty_map).setRetiredCache(root_key, bogus);

    const auto empty_snap = MapSnapshot::capture(empty_map);
    const auto res = empty_map.collapseSubtreeFromCompleteShadow(root_key);
    EXPECT_EQ(res.status, SubtreeCollapseStatus::PAYLOAD_STATE_INVALID);
    EXPECT_TRUE(res.removed_descendant_keys.empty());
    empty_snap.verifyIdentical(MapSnapshot::capture(empty_map));
  }

  // 7. Active payload on internal descendant
  {
    PersistentAdaptiveMap test_map{};
    FrameEvidence ev;
    ASSERT_TRUE(ev.buildFrame(f10, 10, origin).has_value());
    ASSERT_TRUE(test_map.fuseFrameEvidence(ev).has_value());
    ASSERT_TRUE(test_map.refineToLevel(coord, avr::spatial::L1));

    const auto l2_internal = *avr::spatial::parentOf(*test_map.hierarchy().findLeafContainingBase(coord));
    ElevationStats elev;
    elev.update(1.0);
    AdaptiveCellPayload bogus(elev, 10, 10, 1, {});
    PersistentAdaptiveMapTestAccess::payloadStore(test_map).setActivePayload(l2_internal, bogus);

    const auto snap_before = MapSnapshot::capture(test_map);
    const auto res = test_map.collapseSubtreeFromCompleteShadow(root_key);
    EXPECT_EQ(res.status, SubtreeCollapseStatus::PAYLOAD_STATE_INVALID);
    EXPECT_TRUE(res.removed_descendant_keys.empty());
    snap_before.verifyIdentical(MapSnapshot::capture(test_map));
  }

  // 8. Retired cache on leaf descendant
  {
    PersistentAdaptiveMap test_map{};
    FrameEvidence ev;
    ASSERT_TRUE(ev.buildFrame(f10, 10, origin).has_value());
    ASSERT_TRUE(test_map.fuseFrameEvidence(ev).has_value());
    ASSERT_TRUE(test_map.refineToLevel(coord, avr::spatial::L1));

    const auto leaf = *test_map.hierarchy().findLeafContainingBase(coord);
    ElevationStats elev;
    elev.update(1.0);
    AdaptiveCellPayload bogus(elev, 10, 10, 1, {});
    PersistentAdaptiveMapTestAccess::payloadStore(test_map).setRetiredCache(leaf, bogus);

    const auto snap_before = MapSnapshot::capture(test_map);
    const auto res = test_map.collapseSubtreeFromCompleteShadow(root_key);
    EXPECT_EQ(res.status, SubtreeCollapseStatus::PAYLOAD_STATE_INVALID);
    EXPECT_TRUE(res.removed_descendant_keys.empty());
    snap_before.verifyIdentical(MapSnapshot::capture(test_map));
  }

  EXPECT_TRUE(map.validateInvariants().is_valid);
}

// ============================================================================
// Section 57: Determinism
// ============================================================================

TEST(PersistentSubtreeCollapseTest, Determinism) {
  auto run_sequence = []() -> std::pair<SubtreeCollapseResult, MapSnapshot> {
    PersistentAdaptiveMap map{};
    const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};
    const avr::spatial::GridCoord2D coord{10, -20};
    const auto root_key = *avr::spatial::CellKey::fromGridCoord(coord, avr::spatial::L3);

    const std::vector<PointXYZ> f10 = {{1.05, -2.05, 1.25}, {1.08, -2.08, 2.50}};
    FrameEvidence ev10;
    EXPECT_TRUE(ev10.buildFrame(f10, 10, origin).has_value());
    EXPECT_TRUE(map.fuseFrameEvidence(ev10).has_value());

    EXPECT_TRUE(map.refineToLevel(coord, avr::spatial::L0));

    const std::vector<PointXYZ> f20 = {{1.06, -2.06, 1.75}};
    FrameEvidence ev20;
    EXPECT_TRUE(ev20.buildFrame(f20, 20, origin).has_value());
    EXPECT_TRUE(map.fuseFrameEvidence(ev20).has_value());

    const auto res = map.collapseSubtreeFromCompleteShadow(root_key);
    return {res, MapSnapshot::capture(map)};
  };

  const auto [res1, snap1] = run_sequence();
  const auto [res2, snap2] = run_sequence();

  EXPECT_EQ(res1.status, res2.status);
  EXPECT_EQ(res1.ancestor, res2.ancestor);
  EXPECT_EQ(res1.ancestor_level, res2.ancestor_level);
  EXPECT_EQ(res1.finest_removed_level, res2.finest_removed_level);
  EXPECT_EQ(res1.levels_collapsed, res2.levels_collapsed);
  EXPECT_EQ(res1.descendant_nodes_removed, res2.descendant_nodes_removed);
  EXPECT_EQ(res1.descendant_internal_nodes_removed, res2.descendant_internal_nodes_removed);
  EXPECT_EQ(res1.descendant_leaves_removed, res2.descendant_leaves_removed);
  EXPECT_EQ(res1.replay_required_leaves_closed, res2.replay_required_leaves_closed);
  EXPECT_EQ(res1.active_descendant_payloads_removed, res2.active_descendant_payloads_removed);
  EXPECT_EQ(res1.active_descendant_observations_removed, res2.active_descendant_observations_removed);
  EXPECT_EQ(res1.candidate_parent_observations, res2.candidate_parent_observations);
  EXPECT_EQ(res1.had_materialized_shadow, res2.had_materialized_shadow);
  EXPECT_EQ(res1.had_retired_cache, res2.had_retired_cache);

  snap1.verifyIdentical(snap2);
}

// ============================================================================
// Section 58: Stress Suite Over 100 Scenarios
// ============================================================================

TEST(PersistentSubtreeCollapseTest, StressSuiteOverHundredScenarios) {
  std::mt19937_64 rng(0x9B3C9B3CULL);
  std::uniform_int_distribution<int32_t> coord_dist(-200, 200);
  std::uniform_int_distribution<uint32_t> target_level_dist(0, 2);  // L0, L1, or L2
  std::uniform_int_distribution<uint32_t> bool_dist(0, 1);
  std::uniform_real_distribution<double> z_dist(-50.0, 50.0);

  size_t scenarios_executed = 0;

  for (size_t scenario = 0; scenario < 105; ++scenario) {
    PersistentAdaptiveMap map{};
    const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};
    const int32_t gx = coord_dist(rng);
    const int32_t gy = coord_dist(rng);
    const avr::spatial::GridCoord2D coord{gx, gy};
    const auto root_key = *avr::spatial::CellKey::fromGridCoord(coord, avr::spatial::L3);

    const bool has_initial_payload = (bool_dist(rng) == 1);
    uint64_t seq = 1;

    const double wx = static_cast<double>(coord.x) * avr::spatial::BASE_RESOLUTION_METERS;
    const double wy = static_cast<double>(coord.y) * avr::spatial::BASE_RESOLUTION_METERS;

    if (has_initial_payload) {
      const std::vector<PointXYZ> pts = {
          {wx + 0.01, wy + 0.01, z_dist(rng)},
          {wx + 0.02, wy + 0.02, z_dist(rng)}};
      FrameEvidence ev;
      ASSERT_TRUE(ev.buildFrame(pts, seq++, origin).has_value());
      ASSERT_TRUE(map.fuseFrameEvidence(ev).has_value());
    }

    const uint8_t target_level = static_cast<uint8_t>(target_level_dist(rng));
    ASSERT_TRUE(map.refineToLevel(coord, target_level));

    const bool fuse_after_split = (bool_dist(rng) == 1);
    if (fuse_after_split) {
      const std::vector<PointXYZ> pts = {{wx + 0.015, wy + 0.015, z_dist(rng)}};
      FrameEvidence ev;
      ASSERT_TRUE(ev.buildFrame(pts, seq++, origin).has_value());
      ASSERT_TRUE(map.fuseFrameEvidence(ev).has_value());
    }

    // Collapse
    const auto res = map.collapseSubtreeFromCompleteShadow(root_key);
    EXPECT_EQ(res.status, SubtreeCollapseStatus::APPLIED_GAP_CLOSED);
    EXPECT_TRUE(res.isSuccess());
    EXPECT_TRUE(map.hierarchy().isLeaf(root_key));
    EXPECT_FALSE(map.requiresReplay(root_key));
    EXPECT_EQ(map.findRetiredCache(root_key), nullptr);
    EXPECT_EQ(map.findInternalCoarseningState(root_key), nullptr);
    EXPECT_TRUE(map.validateInvariants().is_valid);

    ++scenarios_executed;
  }

  EXPECT_GE(scenarios_executed, 100);
}

// ============================================================================
// Section 39 & 40: Result Copy/Move Traits and Memory Bounds
// ============================================================================

TEST(PersistentSubtreeCollapseTest, ResultCopyMoveTraitsAndBounds) {
  static_assert(std::is_copy_constructible_v<SubtreeCollapseResult>);
  static_assert(std::is_move_constructible_v<SubtreeCollapseResult>);
  static_assert(std::is_copy_assignable_v<SubtreeCollapseResult>);
  static_assert(std::is_move_assignable_v<SubtreeCollapseResult>);

  SubtreeCollapseResult res1{};
  res1.status = SubtreeCollapseStatus::APPLIED_GAP_CLOSED;
  res1.descendant_nodes_removed = 2;
  const auto k1 = *avr::spatial::CellKey::fromGridCoord({0, 0}, avr::spatial::L2);
  const auto k2 = *avr::spatial::CellKey::fromGridCoord({0, 1}, avr::spatial::L2);
  res1.removed_descendant_keys = {k1, k2};

  // Test copy construction and assignment
  SubtreeCollapseResult res2 = res1;
  EXPECT_EQ(res2.status, res1.status);
  EXPECT_EQ(res2.descendant_nodes_removed, res1.descendant_nodes_removed);
  EXPECT_EQ(res2.removed_descendant_keys, res1.removed_descendant_keys);

  SubtreeCollapseResult res3{};
  res3 = res1;
  EXPECT_EQ(res3.status, res1.status);
  EXPECT_EQ(res3.removed_descendant_keys, res1.removed_descendant_keys);

  // Test move construction and assignment
  SubtreeCollapseResult res4 = std::move(res2);
  EXPECT_EQ(res4.status, res1.status);
  EXPECT_EQ(res4.removed_descendant_keys.size(), 2);

  SubtreeCollapseResult res5{};
  res5 = std::move(res3);
  EXPECT_EQ(res5.status, res1.status);
  EXPECT_EQ(res5.removed_descendant_keys.size(), 2);
}

// ============================================================================
// Section 10, 11, 12, 13: Success Key List Golden & Metadata Cleanup Matrix
// ============================================================================

TEST(PersistentSubtreeCollapseTest, SuccessKeyListGoldenAndMetadataCleanupMatrix) {
  PersistentAdaptiveMap map{};
  const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};
  const avr::spatial::GridCoord2D base_coord{0, 0};
  const auto root_key = *avr::spatial::CellKey::fromGridCoord(base_coord, avr::spatial::L3);

  // 1. Fuse frame 10 into L3
  const std::vector<PointXYZ> f10 = {{0.05, 0.05, 1.0}, {0.10, 0.10, 2.0}};
  FrameEvidence ev10;
  ASSERT_TRUE(ev10.buildFrame(f10, 10, origin).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev10).has_value());

  // 2. Split root to L2 (children have requiresReplay == true)
  ASSERT_TRUE(map.splitLeaf(root_key));
  const auto l2_children = *avr::spatial::childrenOf(root_key);
  const auto l2_c0 = l2_children[0];

  // Fuse frame 12 into l2_c0 so it acquires an active payload while still replay-required
  const std::vector<PointXYZ> f12 = {{0.05, 0.05, 1.2}};
  FrameEvidence ev12;
  ASSERT_TRUE(ev12.buildFrame(f12, 12, origin).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev12).has_value());

  // 3. Split L2 child 0 to L1 -> child 0 now has active payload + replay-required, so retired_cache_incomplete == true
  ASSERT_TRUE(map.splitLeaf(l2_c0));
  EXPECT_TRUE(PersistentAdaptiveMapTestAccess::payloadStore(map).isRetiredCacheIncomplete(l2_c0));
  const auto l1_children = *avr::spatial::childrenOf(l2_c0);
  const auto l1_c0 = l1_children[0];

  // Fuse frame 14 into l1_c0 so it acquires an active payload while still replay-required
  const std::vector<PointXYZ> f14 = {{0.05, 0.05, 1.4}};
  FrameEvidence ev14;
  ASSERT_TRUE(ev14.buildFrame(f14, 14, origin).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev14).has_value());

  // 4. Split L1 child 0 to L0 -> child 0 now has active payload + replay-required, so retired_cache_incomplete == true
  ASSERT_TRUE(map.splitLeaf(l1_c0));
  EXPECT_TRUE(PersistentAdaptiveMapTestAccess::payloadStore(map).isRetiredCacheIncomplete(l1_c0));

  // 5. Fuse frame 20 into a leaf under l1_c0
  const std::vector<PointXYZ> f20 = {{0.01, 0.01, 1.5}};
  FrameEvidence ev20;
  ASSERT_TRUE(ev20.buildFrame(f20, 20, origin).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev20).has_value());

  // 6. Enumerate all expected descendant keys prior to collapse
  std::vector<avr::spatial::CellKey> expected_descendants;
  std::vector<avr::spatial::CellKey> to_visit = {root_key};
  while (!to_visit.empty()) {
    const auto cur = to_visit.back();
    to_visit.pop_back();
    const auto ch_opt = avr::spatial::childrenOf(cur);
    if (ch_opt.has_value()) {
      for (const auto& ch : *ch_opt) {
        if (map.hierarchy().isAllocated(ch)) {
          expected_descendants.push_back(ch);
          if (map.hierarchy().isInternal(ch)) {
            to_visit.push_back(ch);
          }
        }
      }
    }
  }
  std::sort(expected_descendants.begin(), expected_descendants.end());
  ASSERT_FALSE(expected_descendants.empty());

  // 7. Collapse subtree from complete L3
  const auto result = map.collapseSubtreeFromCompleteShadow(root_key);
  EXPECT_EQ(result.status, SubtreeCollapseStatus::APPLIED_GAP_CLOSED);
  EXPECT_TRUE(result.isSuccess());

  // 8. Verify removed_descendant_keys golden contract
  EXPECT_EQ(result.removed_descendant_keys, expected_descendants);
  EXPECT_EQ(result.descendant_nodes_removed, expected_descendants.size());
  EXPECT_EQ(std::find(result.removed_descendant_keys.begin(), result.removed_descendant_keys.end(), root_key),
            result.removed_descendant_keys.end());

  // 9. Section 11 & 12: Verify complete metadata cleanup for EVERY removed descendant
  for (const auto& k : result.removed_descendant_keys) {
    EXPECT_EQ(map.hierarchy().nodeState(k), NodeState::ABSENT);
    EXPECT_EQ(map.findActivePayload(k), nullptr);
    EXPECT_EQ(map.findRetiredCache(k), nullptr);
    EXPECT_FALSE(map.requiresReplay(k));
    EXPECT_FALSE(PersistentAdaptiveMapTestAccess::payloadStore(map).isRetiredCacheIncomplete(k));
    EXPECT_EQ(map.findInternalCoarseningState(k), nullptr);
  }

  // 10. Section 13: Verify restored ancestor metadata matrix
  EXPECT_EQ(map.hierarchy().nodeState(root_key), NodeState::LEAF);
  EXPECT_EQ(map.findRetiredCache(root_key), nullptr);
  EXPECT_FALSE(PersistentAdaptiveMapTestAccess::payloadStore(map).isRetiredCacheIncomplete(root_key));
  EXPECT_FALSE(map.requiresReplay(root_key));
  EXPECT_EQ(map.findInternalCoarseningState(root_key), nullptr);
  const auto* active = map.findActivePayload(root_key);
  ASSERT_NE(active, nullptr);

  EXPECT_TRUE(map.validateInvariants().is_valid);
}

// ============================================================================
// Section 14 & 15: Unrelated Sibling & Grandparent Metadata Immutability
// ============================================================================

TEST(PersistentSubtreeCollapseTest, UnrelatedSiblingAndGrandparentImmutability) {
  PersistentAdaptiveMap map{};
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 10});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;
  const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};

  // Tree A
  const auto root_A = *avr::spatial::CellKey::fromWorldPoint({0.05, 0.05}, avr::spatial::L3);

  // Tree B (unrelated sibling tree)
  const auto root_B = *avr::spatial::CellKey::fromWorldPoint({10.05, 10.05}, avr::spatial::L3);
  ASSERT_NE(root_A, root_B);

  // 1. Fuse frame 10 into both trees
  const std::vector<PointXYZ> f10 = {{0.05, 0.05, 1.0}, {10.05, 10.05, 2.0}};
  FrameEvidence ev10;
  ASSERT_TRUE(ev10.buildFrame(f10, 10, origin).has_value());
  ASSERT_TRUE(cache.pushFrame(f10, 10, origin));
  ASSERT_TRUE(map.fuseFrameEvidence(ev10).has_value());

  // Split root_A to L2 and replay so L2 children have COMPLETE base
  ASSERT_TRUE(map.splitLeaf(root_A));
  const auto res_replay = map.replayDirectChildren(root_A, cache);
  ASSERT_EQ(res_replay.status, ReplayStatus::APPLIED_COMPLETE);
  const auto l2_children_A = *avr::spatial::childrenOf(root_A);
  const auto parent_L2 = l2_children_A[0];

  // Split parent_L2 to L1 (now root_A is grandparent G, parent_L2 is parent P)
  ASSERT_TRUE(map.splitLeaf(parent_L2));
  const auto* p_state = map.findInternalCoarseningState(parent_L2);
  ASSERT_NE(p_state, nullptr);
  EXPECT_EQ(p_state->base_state, CoarseningBaseState::COMPLETE_RETIRED_CACHE);

  // Split root_B to L2
  ASSERT_TRUE(map.splitLeaf(root_B));

  // 2. Fuse frame 20 into parent_L2 children and root_B
  const std::vector<PointXYZ> f20 = {{0.01, 0.01, 1.5}, {10.05, 10.05, 2.5}};
  FrameEvidence ev20;
  ASSERT_TRUE(ev20.buildFrame(f20, 20, origin).has_value());
  ASSERT_TRUE(cache.pushFrame(f20, 20, origin));
  ASSERT_TRUE(map.fuseFrameEvidence(ev20).has_value());

  // Snapshot unrelated tree B before collapse
  const auto snap_B_before = MapSnapshot::capture(map);

  // Snapshot grandparent root_A before collapse
  const auto* g_state_before = map.findInternalCoarseningState(root_A);
  ASSERT_NE(g_state_before, nullptr);
  ASSERT_TRUE(g_state_before->has_current_shadow);
  const auto g_shadow_before = g_state_before->current_shadow;
  const auto g_base_state_before = g_state_before->base_state;
  const auto g_node_state_before = map.hierarchy().nodeState(root_A);

  // 3. Collapse parent_L2 subtree
  const auto result = map.collapseSubtreeFromCompleteShadow(parent_L2);
  EXPECT_EQ(result.status, SubtreeCollapseStatus::APPLIED_GAP_CLOSED);
  EXPECT_TRUE(result.isSuccess());

  // 4. Verify grandparent root_A is bit-for-bit unchanged
  const auto* g_state_after = map.findInternalCoarseningState(root_A);
  ASSERT_NE(g_state_after, nullptr);
  EXPECT_EQ(g_state_after->base_state, g_base_state_before);
  ASSERT_TRUE(g_state_after->has_current_shadow);
  verifyExact8FieldEquivalence(g_state_after->current_shadow, g_shadow_before);
  EXPECT_EQ(map.hierarchy().nodeState(root_A), g_node_state_before);

  // 5. Verify unrelated root_B nodes remain unchanged
  EXPECT_TRUE(map.hierarchy().isInternal(root_B));
  const auto* b_state = map.findInternalCoarseningState(root_B);
  ASSERT_NE(b_state, nullptr);
  const auto* b_retired = map.findRetiredCache(root_B);
  ASSERT_NE(b_retired, nullptr);
  EXPECT_EQ(b_retired->observationCount(), 1);

  EXPECT_TRUE(map.validateInvariants().is_valid);
}

// ============================================================================
// Section 26 & 27: Full-Depth & Mixed-Depth Plan Ordering
// ============================================================================

TEST(PersistentSubtreeCollapseTest, FullDepthAndMixedDepthPlanOrdering) {
  // Full depth: L3 -> 4 L2 -> 16 L1 -> 64 L0 (84 descendant nodes total)
  {
    PersistentAdaptiveMap map{};
    const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};
    const avr::spatial::GridCoord2D base_coord{0, 0};
    const auto root_key = *avr::spatial::CellKey::fromGridCoord(base_coord, avr::spatial::L3);

    const std::vector<PointXYZ> f10 = {{0.05, 0.05, 1.0}};
    FrameEvidence ev10;
    ASSERT_TRUE(ev10.buildFrame(f10, 10, origin).has_value());
    ASSERT_TRUE(map.fuseFrameEvidence(ev10).has_value());

    // Split L3 -> 4 L2
    ASSERT_TRUE(map.splitLeaf(root_key));
    const auto l2_nodes = *avr::spatial::childrenOf(root_key);

    // Split all 4 L2 -> 16 L1
    for (const auto& l2 : l2_nodes) {
      ASSERT_TRUE(map.splitLeaf(l2));
      const auto l1_nodes = *avr::spatial::childrenOf(l2);
      // Split all 4 L1 -> 16 L0
      for (const auto& l1 : l1_nodes) {
        ASSERT_TRUE(map.splitLeaf(l1));
      }
    }

    // Fuse frame 20 into fine leaves
    const std::vector<PointXYZ> f20 = {{0.05, 0.05, 2.0}};
    FrameEvidence ev20;
    ASSERT_TRUE(ev20.buildFrame(f20, 20, origin).has_value());
    ASSERT_TRUE(map.fuseFrameEvidence(ev20).has_value());

    const auto result = map.collapseSubtreeFromCompleteShadow(root_key);
    EXPECT_EQ(result.status, SubtreeCollapseStatus::APPLIED_GAP_CLOSED);
    EXPECT_TRUE(result.isSuccess());
    EXPECT_EQ(result.descendant_nodes_removed, 84);              // 4 + 16 + 64
    EXPECT_EQ(result.descendant_internal_nodes_removed, 20);     // 4 + 16
    EXPECT_EQ(result.descendant_leaves_removed, 64);             // 64
    EXPECT_EQ(result.levels_collapsed, 3);
    EXPECT_EQ(result.finest_removed_level, avr::spatial::L0);
    EXPECT_EQ(result.removed_descendant_keys.size(), 84);
    EXPECT_TRUE(std::is_sorted(result.removed_descendant_keys.begin(), result.removed_descendant_keys.end()));

    EXPECT_TRUE(map.hierarchy().isLeaf(root_key));
    EXPECT_TRUE(map.validateInvariants().is_valid);
  }

  // Mixed depth: L3 -> 4 L2, only child 0 split to L1, only child 0-0 split to L0
  {
    PersistentAdaptiveMap map{};
    const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};
    const avr::spatial::GridCoord2D base_coord{0, 0};
    const auto root_key = *avr::spatial::CellKey::fromGridCoord(base_coord, avr::spatial::L3);

    const std::vector<PointXYZ> f10 = {{0.05, 0.05, 1.0}};
    FrameEvidence ev10;
    ASSERT_TRUE(ev10.buildFrame(f10, 10, origin).has_value());
    ASSERT_TRUE(map.fuseFrameEvidence(ev10).has_value());

    ASSERT_TRUE(map.splitLeaf(root_key));
    const auto l2_nodes = *avr::spatial::childrenOf(root_key);
    ASSERT_TRUE(map.splitLeaf(l2_nodes[0]));
    const auto l1_nodes = *avr::spatial::childrenOf(l2_nodes[0]);
    ASSERT_TRUE(map.splitLeaf(l1_nodes[0]));

    const auto result = map.collapseSubtreeFromCompleteShadow(root_key);
    EXPECT_EQ(result.status, SubtreeCollapseStatus::APPLIED_GAP_CLOSED);
    EXPECT_TRUE(result.isSuccess());
    EXPECT_EQ(result.descendant_nodes_removed, 4 + 4 + 4);       // 12 descendants
    EXPECT_EQ(result.descendant_internal_nodes_removed, 2);     // l2_nodes[0] and l1_nodes[0]
    EXPECT_EQ(result.descendant_leaves_removed, 10);            // 3 + 3 + 4
    EXPECT_EQ(result.levels_collapsed, 3);
    EXPECT_EQ(result.finest_removed_level, avr::spatial::L0);
    EXPECT_EQ(result.removed_descendant_keys.size(), 12);
    EXPECT_TRUE(std::is_sorted(result.removed_descendant_keys.begin(), result.removed_descendant_keys.end()));

    EXPECT_TRUE(map.hierarchy().isLeaf(root_key));
    EXPECT_TRUE(map.validateInvariants().is_valid);
  }
}

// ============================================================================
// Section 30, 31, 32: Complete Cumulative Ingestion Counters Comparison
// ============================================================================

TEST(PersistentSubtreeCollapseTest, CompleteCumulativeIngestionCountersComparison) {
  PersistentAdaptiveMap map{};
  const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};
  const avr::spatial::GridCoord2D base_coord{0, 0};
  const auto root_key = *avr::spatial::CellKey::fromGridCoord(base_coord, avr::spatial::L3);

  // Fuse 3 frames
  for (uint64_t seq = 1; seq <= 3; ++seq) {
    const std::vector<PointXYZ> f = {{0.05, 0.05, static_cast<double>(seq)}};
    FrameEvidence ev;
    ASSERT_TRUE(ev.buildFrame(f, seq, origin).has_value());
    ASSERT_TRUE(map.fuseFrameEvidence(ev).has_value());
  }

  // Refine to L1
  ASSERT_TRUE(map.refineToLevel(base_coord, avr::spatial::L1));

  // Fuse 2 more frames
  for (uint64_t seq = 4; seq <= 5; ++seq) {
    const std::vector<PointXYZ> f = {{0.05, 0.05, static_cast<double>(seq)}};
    FrameEvidence ev;
    ASSERT_TRUE(ev.buildFrame(f, seq, origin).has_value());
    ASSERT_TRUE(map.fuseFrameEvidence(ev).has_value());
  }

  const auto stats_before = map.statistics();
  EXPECT_EQ(stats_before.last_fused_frame_seq, 5);
  EXPECT_TRUE(stats_before.has_fused_frames);
  EXPECT_EQ(stats_before.total_frames_fused, 5);
  EXPECT_EQ(stats_before.total_observations_fused, 5);

  // Execute subtree collapse
  const auto result = map.collapseSubtreeFromCompleteShadow(root_key);
  EXPECT_EQ(result.status, SubtreeCollapseStatus::APPLIED_GAP_CLOSED);
  EXPECT_TRUE(result.isSuccess());

  const auto stats_after = map.statistics();

  // All cumulative ingestion counters must remain 100% unchanged!
  EXPECT_EQ(stats_after.last_fused_frame_seq, stats_before.last_fused_frame_seq);
  EXPECT_EQ(stats_after.has_fused_frames, stats_before.has_fused_frames);
  EXPECT_EQ(stats_after.total_frames_fused, stats_before.total_frames_fused);
  EXPECT_EQ(stats_after.total_observations_fused, stats_before.total_observations_fused);

  EXPECT_TRUE(map.validateInvariants().is_valid);
}

}  // namespace avr::mapping


#include <gtest/gtest.h>

#include <vector>

#include "avr_mapping/frame_evidence.hpp"
#include "avr_mapping/persistent_adaptive_map.hpp"
#include "avr_spatial/cell_key.hpp"
#include "avr_spatial/constants.hpp"

namespace avr::mapping {

// Test access helper for failure injection
struct PersistentAdaptiveMapTestAccess {
  static AdaptivePayloadStore& payloadStore(PersistentAdaptiveMap& map) noexcept {
    return map.payload_store_;
  }
};

namespace {

TEST(PersistentPayloadSplitTest, SplitWithoutHistory) {
  PersistentAdaptiveMap map{};
  const avr::spatial::GridCoord2D base{0, 0};

  const auto root_key = *map.ensureRootForBase(base);
  EXPECT_TRUE(map.hierarchy().isLeaf(root_key));
  EXPECT_FALSE(map.requiresReplay(root_key));
  EXPECT_EQ(map.findActivePayload(root_key), nullptr);

  // Split root without any historical payload
  EXPECT_TRUE(map.splitLeaf(root_key));

  EXPECT_TRUE(map.hierarchy().isInternal(root_key));
  EXPECT_EQ(map.findRetiredCache(root_key), nullptr);

  const auto children = *avr::spatial::childrenOf(root_key);
  for (const auto& child : children) {
    EXPECT_TRUE(map.hierarchy().isLeaf(child));
    EXPECT_FALSE(map.requiresReplay(child));
    EXPECT_EQ(map.findActivePayload(child), nullptr);
  }

  EXPECT_TRUE(map.validateInvariants().is_valid);
}

TEST(PersistentPayloadSplitTest, SplitAfterHistory) {
  PersistentAdaptiveMap map{};
  FrameEvidence evidence{};

  const std::vector<PointXYZ> pts = {
      {0.05, 0.05, 2.0},
      {0.15, 0.15, 4.0},
  };
  ASSERT_TRUE(evidence.buildFrame(pts, 10, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(evidence).has_value());

  const auto root_key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{1, 1}, avr::spatial::L3);
  const auto* parent_payload_before = map.findActivePayload(root_key);
  ASSERT_NE(parent_payload_before, nullptr);
  const AdaptiveCellPayload saved_parent_payload = *parent_payload_before;

  // Split L3 -> L2
  EXPECT_TRUE(map.splitLeaf(root_key));

  // Parent is now INTERNAL
  EXPECT_TRUE(map.hierarchy().isInternal(root_key));
  EXPECT_EQ(map.findActivePayload(root_key), nullptr);

  // Parent active payload moved to immutable RETIRED_PARENT_CACHE
  const auto* retired_cache = map.findRetiredCache(root_key);
  ASSERT_NE(retired_cache, nullptr);
  EXPECT_EQ(*retired_cache, saved_parent_payload);
  EXPECT_FALSE(map.isRetiredCacheIncomplete(root_key));

  // All 4 children must be LEAF, have NO fabricated historical payload, and be REPLAY_REQUIRED
  const auto children = *avr::spatial::childrenOf(root_key);
  for (const auto& child : children) {
    EXPECT_TRUE(map.hierarchy().isLeaf(child));
    EXPECT_TRUE(map.requiresReplay(child));
    EXPECT_EQ(map.findActivePayload(child), nullptr);
  }

  EXPECT_TRUE(map.validateInvariants().is_valid);
}

TEST(PersistentPayloadSplitTest, CacheImmutability) {
  PersistentAdaptiveMap map{};
  FrameEvidence ev1{};

  const std::vector<PointXYZ> pts1 = {
      {0.05, 0.05, 10.0},
      {0.05, 0.05, 20.0},
  };
  ASSERT_TRUE(ev1.buildFrame(pts1, 1, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev1).has_value());

  const auto root_key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{1, 1}, avr::spatial::L3);
  ASSERT_TRUE(map.splitLeaf(root_key));

  const auto* cached_parent_before = map.findRetiredCache(root_key);
  ASSERT_NE(cached_parent_before, nullptr);
  const AdaptiveCellPayload expected_cache = *cached_parent_before;

  // Fuse later frame 2 into one child leaf
  FrameEvidence ev2{};
  const std::vector<PointXYZ> pts2 = {{0.05, 0.05, 50.0}};
  ASSERT_TRUE(ev2.buildFrame(pts2, 2, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev2).has_value());

  // Parent retired cache must remain bit-for-bit identical!
  const auto* cached_parent_after = map.findRetiredCache(root_key);
  ASSERT_NE(cached_parent_after, nullptr);
  EXPECT_EQ(*cached_parent_after, expected_cache);
  EXPECT_EQ(cached_parent_after->observationCount(), 2ULL);
  EXPECT_DOUBLE_EQ(cached_parent_after->elevation().mean(), 15.0);

  EXPECT_TRUE(map.validateInvariants().is_valid);
}

TEST(PersistentPayloadSplitTest, ReplayGapPropagation) {
  PersistentAdaptiveMap map{};
  FrameEvidence ev{};

  const std::vector<PointXYZ> pts = {{0.05, 0.05, 1.0}};
  ASSERT_TRUE(ev.buildFrame(pts, 1, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev).has_value());

  const auto root_key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{1, 1}, avr::spatial::L3);
  ASSERT_TRUE(map.splitLeaf(root_key));

  const auto children = *avr::spatial::childrenOf(root_key);
  const auto replay_child = children[0];
  EXPECT_TRUE(map.requiresReplay(replay_child));

  // Split replay-required L2 child down to L1 before replay exists
  EXPECT_TRUE(map.splitLeaf(replay_child));

  EXPECT_TRUE(map.hierarchy().isInternal(replay_child));
  EXPECT_FALSE(map.requiresReplay(replay_child));

  const auto grandchildren = *avr::spatial::childrenOf(replay_child);
  for (const auto& grandchild : grandchildren) {
    EXPECT_TRUE(map.hierarchy().isLeaf(grandchild));
    EXPECT_TRUE(map.requiresReplay(grandchild));
    EXPECT_EQ(map.findActivePayload(grandchild), nullptr);
  }

  EXPECT_TRUE(map.validateInvariants().is_valid);
}

TEST(PersistentPayloadSplitTest, ActivePartialPayloadSplit) {
  PersistentAdaptiveMap map{};

  // Frame 1: root
  FrameEvidence ev1{};
  const std::vector<PointXYZ> pts1 = {{0.05, 0.05, 1.0}};
  ASSERT_TRUE(ev1.buildFrame(pts1, 1, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev1).has_value());

  // Split root to L2: child is replay-required
  const auto root_key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{1, 1}, avr::spatial::L3);
  ASSERT_TRUE(map.splitLeaf(root_key));

  const auto child_key = *map.hierarchy().findLeafContainingBase(avr::spatial::GridCoord2D{1, 1});
  EXPECT_TRUE(map.requiresReplay(child_key));

  // Fuse frame 2 into child: now child has ACTIVE_PAYLOAD and REPLAY_REQUIRED == true
  FrameEvidence ev2{};
  const std::vector<PointXYZ> pts2 = {{0.05, 0.05, 5.0}};
  ASSERT_TRUE(ev2.buildFrame(pts2, 2, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev2).has_value());

  EXPECT_NE(map.findActivePayload(child_key), nullptr);
  EXPECT_TRUE(map.requiresReplay(child_key));

  // Now split this child to L1
  EXPECT_TRUE(map.splitLeaf(child_key));

  // Retired cache must exist AND be marked as incomplete!
  const auto* cache = map.findRetiredCache(child_key);
  ASSERT_NE(cache, nullptr);
  EXPECT_TRUE(map.isRetiredCacheIncomplete(child_key));

  // All 4 grandchildren must be REPLAY_REQUIRED
  const auto grandchildren = *avr::spatial::childrenOf(child_key);
  for (const auto& gc : grandchildren) {
    EXPECT_TRUE(map.hierarchy().isLeaf(gc));
    EXPECT_TRUE(map.requiresReplay(gc));
    EXPECT_EQ(map.findActivePayload(gc), nullptr);
  }

  EXPECT_TRUE(map.validateInvariants().is_valid);
}

TEST(PersistentPayloadSplitTest, MultipleChildFusion) {
  PersistentAdaptiveMap map{};
  const auto root_key = *map.ensureRootForBase(avr::spatial::GridCoord2D{0, 0});
  ASSERT_TRUE(map.splitLeaf(root_key));

  const auto children = *avr::spatial::childrenOf(root_key);

  // Fuse frame with points in child[0] and child[1]
  FrameEvidence ev{};
  const std::vector<PointXYZ> pts = {
      {0.05, 0.05, 10.0},
      {0.25, 0.05, 20.0},
  };
  ASSERT_TRUE(ev.buildFrame(pts, 1, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());
  const auto stats = map.fuseFrameEvidence(ev);
  ASSERT_TRUE(stats.has_value());
  EXPECT_EQ(stats->unique_leaves_fused, 2ULL);
  EXPECT_EQ(stats->observations_fused, 2ULL);

  // Only those 2 leaves have active payloads
  size_t active_leaf_count = 0;
  for (const auto& child : children) {
    if (map.findActivePayload(child) != nullptr) {
      ++active_leaf_count;
    }
  }
  EXPECT_EQ(active_leaf_count, 2ULL);
  EXPECT_TRUE(map.validateInvariants().is_valid);
}

TEST(PersistentPayloadSplitTest, RetiredCacheStateMatrix) {
  // Case A: clean leaf + complete active payload -> split
  {
    PersistentAdaptiveMap map{};
    const auto r_key = *map.ensureRootForBase(avr::spatial::GridCoord2D{0, 0});
    FrameEvidence ev{};
    const std::vector<PointXYZ> pts = {{0.05, 0.05, 2.0}};
    ASSERT_TRUE(ev.buildFrame(pts, 1, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());
    ASSERT_TRUE(map.fuseFrameEvidence(ev).has_value());
    EXPECT_FALSE(map.requiresReplay(r_key));

    EXPECT_TRUE(map.splitLeaf(r_key));
    EXPECT_NE(map.findRetiredCache(r_key), nullptr);
    EXPECT_FALSE(map.isRetiredCacheIncomplete(r_key));
    const auto children_a = *avr::spatial::childrenOf(r_key);
    for (const auto& c : children_a) {
      EXPECT_TRUE(map.requiresReplay(c));
    }
    EXPECT_TRUE(map.validateInvariants().is_valid);
  }

  // Case B: replay-required leaf + no active payload -> split
  {
    PersistentAdaptiveMap map{};
    const auto r_key = *map.ensureRootForBase(avr::spatial::GridCoord2D{0, 0});
    FrameEvidence ev{};
    const std::vector<PointXYZ> pts = {{0.05, 0.05, 2.0}};
    ASSERT_TRUE(ev.buildFrame(pts, 1, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());
    ASSERT_TRUE(map.fuseFrameEvidence(ev).has_value());
    ASSERT_TRUE(map.splitLeaf(r_key));

    const auto child_without_payload = (*avr::spatial::childrenOf(r_key))[3];
    EXPECT_TRUE(map.requiresReplay(child_without_payload));
    EXPECT_EQ(map.findActivePayload(child_without_payload), nullptr);

    EXPECT_TRUE(map.splitLeaf(child_without_payload));
    EXPECT_EQ(map.findRetiredCache(child_without_payload), nullptr);
    const auto children_b = *avr::spatial::childrenOf(child_without_payload);
    for (const auto& gc : children_b) {
      EXPECT_TRUE(map.requiresReplay(gc));
    }
    EXPECT_TRUE(map.validateInvariants().is_valid);
  }

  // Case C: replay-required leaf + active payload -> split
  {
    PersistentAdaptiveMap map{};
    const auto r_key = *map.ensureRootForBase(avr::spatial::GridCoord2D{0, 0});
    FrameEvidence ev1{};
    const std::vector<PointXYZ> pts1 = {{0.05, 0.05, 2.0}};
    ASSERT_TRUE(ev1.buildFrame(pts1, 1, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());
    ASSERT_TRUE(map.fuseFrameEvidence(ev1).has_value());
    ASSERT_TRUE(map.splitLeaf(r_key));

    const auto target_child = *map.hierarchy().findLeafContainingBase(avr::spatial::GridCoord2D{1, 1});
    EXPECT_TRUE(map.requiresReplay(target_child));

    // Fuse frame 2 into replay-required child
    FrameEvidence ev2{};
    const std::vector<PointXYZ> pts2 = {{0.05, 0.05, 5.0}};
    ASSERT_TRUE(ev2.buildFrame(pts2, 2, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());
    ASSERT_TRUE(map.fuseFrameEvidence(ev2).has_value());
    EXPECT_NE(map.findActivePayload(target_child), nullptr);
    EXPECT_TRUE(map.requiresReplay(target_child));

    EXPECT_TRUE(map.splitLeaf(target_child));
    EXPECT_NE(map.findRetiredCache(target_child), nullptr);
    EXPECT_TRUE(map.isRetiredCacheIncomplete(target_child));
    const auto children_c = *avr::spatial::childrenOf(target_child);
    for (const auto& gc : children_c) {
      EXPECT_TRUE(map.requiresReplay(gc));
    }
    EXPECT_TRUE(map.validateInvariants().is_valid);
  }

  // Case D: clean leaf + no active payload -> split
  {
    PersistentAdaptiveMap map{};
    const auto r_key = *map.ensureRootForBase(avr::spatial::GridCoord2D{0, 0});
    EXPECT_FALSE(map.requiresReplay(r_key));
    EXPECT_EQ(map.findActivePayload(r_key), nullptr);

    EXPECT_TRUE(map.splitLeaf(r_key));
    EXPECT_EQ(map.findRetiredCache(r_key), nullptr);
    const auto children_d = *avr::spatial::childrenOf(r_key);
    for (const auto& c : children_d) {
      EXPECT_FALSE(map.requiresReplay(c));
    }
    EXPECT_TRUE(map.validateInvariants().is_valid);
  }
}

TEST(PersistentPayloadSplitTest, RefineWithHistoryL3ToL0) {
  PersistentAdaptiveMap map{};
  const avr::spatial::GridCoord2D base{1, 1};

  FrameEvidence ev{};
  const std::vector<PointXYZ> pts = {{0.05, 0.05, 3.0}};
  ASSERT_TRUE(ev.buildFrame(pts, 10, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev).has_value());

  const auto root_key = *avr::spatial::CellKey::fromGridCoord(base, avr::spatial::L3);
  ASSERT_NE(map.findActivePayload(root_key), nullptr);

  // Refine from L3 all the way to L0
  EXPECT_TRUE(map.refineToLevel(base, avr::spatial::L0));

  // Verify structural path
  const auto l0_leaf = *map.hierarchy().findLeafContainingBase(base);
  EXPECT_EQ(l0_leaf.level, avr::spatial::L0);
  EXPECT_TRUE(map.hierarchy().isLeaf(l0_leaf));
  EXPECT_TRUE(map.requiresReplay(l0_leaf));
  EXPECT_EQ(map.findActivePayload(l0_leaf), nullptr);

  // L3 root must be INTERNAL and have retired cache
  EXPECT_TRUE(map.hierarchy().isInternal(root_key));
  ASSERT_NE(map.findRetiredCache(root_key), nullptr);
  EXPECT_FALSE(map.isRetiredCacheIncomplete(root_key));

  // The intermediate ancestor nodes along the path (L2, L1) had no active payload when split
  // and must NOT have fabricated any retired caches
  const auto l2_ancestor = *avr::spatial::CellKey::fromGridCoord(base, avr::spatial::L2);
  const auto l1_ancestor = *avr::spatial::CellKey::fromGridCoord(base, avr::spatial::L1);
  EXPECT_TRUE(map.hierarchy().isInternal(l2_ancestor));
  EXPECT_TRUE(map.hierarchy().isInternal(l1_ancestor));
  EXPECT_EQ(map.findRetiredCache(l2_ancestor), nullptr);
  EXPECT_EQ(map.findRetiredCache(l1_ancestor), nullptr);

  EXPECT_TRUE(map.validateInvariants().is_valid);
}

TEST(PersistentPayloadSplitTest, NoHalfSplitLogicalFailure) {
  PersistentAdaptiveMap map{};
  const auto root_key = *map.ensureRootForBase(avr::spatial::GridCoord2D{0, 0});

  FrameEvidence ev{};
  const std::vector<PointXYZ> pts = {{0.05, 0.05, 2.0}};
  ASSERT_TRUE(ev.buildFrame(pts, 1, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev).has_value());

  // Record baseline snapshots before any rejected attempts
  const auto base_active_snap = map.activePayloadSnapshot();
  const auto base_cache_snap = map.retiredCacheSnapshot();
  const auto base_replay_snap = map.replayRequiredSnapshot();
  const auto base_stats = map.statistics();

  // 1. Missing node
  const auto missing_key = *avr::spatial::CellKey::fromGridCoord(
      avr::spatial::GridCoord2D{999, 999}, avr::spatial::L3);
  EXPECT_FALSE(map.splitLeaf(missing_key));
  EXPECT_EQ(map.activePayloadSnapshot(), base_active_snap);
  EXPECT_EQ(map.retiredCacheSnapshot(), base_cache_snap);
  EXPECT_EQ(map.replayRequiredSnapshot(), base_replay_snap);
  EXPECT_EQ(map.statistics(), base_stats);

  // 2. Parent already INTERNAL (split once, then split parent again)
  ASSERT_TRUE(map.splitLeaf(root_key));
  const auto after_split_active = map.activePayloadSnapshot();
  const auto after_split_cache = map.retiredCacheSnapshot();
  const auto after_split_replay = map.replayRequiredSnapshot();
  const auto after_split_stats = map.statistics();

  EXPECT_FALSE(map.splitLeaf(root_key));
  EXPECT_EQ(map.activePayloadSnapshot(), after_split_active);
  EXPECT_EQ(map.retiredCacheSnapshot(), after_split_cache);
  EXPECT_EQ(map.replayRequiredSnapshot(), after_split_replay);
  EXPECT_EQ(map.statistics(), after_split_stats);

  // 3. L0 parent cannot be split
  const auto base_l0 = avr::spatial::GridCoord2D{0, 0};
  ASSERT_TRUE(map.refineToLevel(base_l0, avr::spatial::L0));
  const auto l0_leaf = *map.hierarchy().findLeafContainingBase(base_l0);
  EXPECT_EQ(l0_leaf.level, avr::spatial::L0);
  EXPECT_FALSE(map.splitLeaf(l0_leaf));

  // 4. Unexpected stale retired cache on a leaf: prevalidation rejects with zero mutation
  {
    PersistentAdaptiveMap fresh_map{};
    const auto r = *fresh_map.ensureRootForBase(avr::spatial::GridCoord2D{10, 10});
    ElevationStats stale_elev{};
    stale_elev.update(9.0);
    // Inject stale cache on leaf using test access
    PersistentAdaptiveMapTestAccess::payloadStore(fresh_map).setRetiredCache(
        r, AdaptiveCellPayload(stale_elev, 1, 1, 1, ResolutionReasonBits{}));

    const auto snap_before = fresh_map.activePayloadSnapshot();
    EXPECT_FALSE(fresh_map.splitLeaf(r));
    EXPECT_EQ(fresh_map.activePayloadSnapshot(), snap_before);
    EXPECT_TRUE(fresh_map.hierarchy().isLeaf(r));
  }

  // 5. Unexpected child payload metadata on un-split leaf: prevalidation rejects
  {
    PersistentAdaptiveMap fresh_map{};
    const auto r = *fresh_map.ensureRootForBase(avr::spatial::GridCoord2D{20, 20});
    const auto children = *avr::spatial::childrenOf(r);
    // Inject metadata on child[0] before parent split
    PersistentAdaptiveMapTestAccess::payloadStore(fresh_map).setReplayRequired(children[0], true);

    const auto snap_before = fresh_map.activePayloadSnapshot();
    EXPECT_FALSE(fresh_map.splitLeaf(r));
    EXPECT_EQ(fresh_map.activePayloadSnapshot(), snap_before);
    EXPECT_TRUE(fresh_map.hierarchy().isLeaf(r));
  }
}

TEST(PersistentPayloadSplitTest, RetiredCacheImmutabilityMultipleDescendantFusions) {
  PersistentAdaptiveMap map{};

  const std::vector<PointXYZ> pts1 = {{0.05, 0.05, 10.0}, {0.05, 0.05, 20.0}};
  FrameEvidence ev1{};
  ASSERT_TRUE(ev1.buildFrame(pts1, 1, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());
  const ResolutionReasonBits reasons_a(ResolutionReason::DISTANCE_BASELINE);
  ASSERT_TRUE(map.fuseFrameEvidence(ev1, reasons_a).has_value());

  const auto root_key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{1, 1}, avr::spatial::L3);
  ASSERT_TRUE(map.splitLeaf(root_key));

  const auto* parent_cache_before = map.findRetiredCache(root_key);
  ASSERT_NE(parent_cache_before, nullptr);
  const AdaptiveCellPayload expected_cache = *parent_cache_before;
  EXPECT_FALSE(map.isRetiredCacheIncomplete(root_key));

  // Fuse frame 2 into child[0]
  FrameEvidence ev2{};
  const std::vector<PointXYZ> pts2 = {{0.05, 0.05, 30.0}};
  ASSERT_TRUE(ev2.buildFrame(pts2, 2, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev2, ResolutionReasonBits(ResolutionReason::UNCERTAINTY)).has_value());

  // Fuse frame 3 into child[1]
  FrameEvidence ev3{};
  const std::vector<PointXYZ> pts3 = {{0.25, 0.05, 40.0}};
  ASSERT_TRUE(ev3.buildFrame(pts3, 3, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev3, ResolutionReasonBits(ResolutionReason::GEOMETRY_COMPLEXITY)).has_value());

  // Fuse frame 4 into child[0] again
  FrameEvidence ev4{};
  const std::vector<PointXYZ> pts4 = {{0.05, 0.05, 50.0}};
  ASSERT_TRUE(ev4.buildFrame(pts4, 4, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev4).has_value());

  // Parent retired cache must remain bit-for-bit identical across all fields!
  const auto* parent_cache_after = map.findRetiredCache(root_key);
  ASSERT_NE(parent_cache_after, nullptr);
  EXPECT_EQ(*parent_cache_after, expected_cache);
  EXPECT_EQ(parent_cache_after->observationCount(), 2ULL);
  EXPECT_DOUBLE_EQ(parent_cache_after->elevation().mean(), 15.0);
  EXPECT_DOUBLE_EQ(parent_cache_after->elevation().min(), 10.0);
  EXPECT_DOUBLE_EQ(parent_cache_after->elevation().max(), 20.0);
  EXPECT_EQ(parent_cache_after->firstSeenFrame(), 1ULL);
  EXPECT_EQ(parent_cache_after->lastSeenFrame(), 1ULL);
  EXPECT_EQ(parent_cache_after->observedFrameCount(), 1ULL);
  EXPECT_TRUE(parent_cache_after->lastUpdateReasons().has(ResolutionReason::DISTANCE_BASELINE));
  EXPECT_FALSE(parent_cache_after->lastUpdateReasons().has(ResolutionReason::UNCERTAINTY));
  EXPECT_FALSE(map.isRetiredCacheIncomplete(root_key));

  EXPECT_TRUE(map.validateInvariants().is_valid);
}

}  // namespace
}  // namespace avr::mapping

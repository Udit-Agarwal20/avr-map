#include <gtest/gtest.h>

#include <vector>

#include "avr_mapping/adaptive_hierarchy.hpp"
#include "avr_spatial/constants.hpp"

namespace avr::mapping {

// Section 31: Basic root test
TEST(AdaptiveHierarchyTest, BasicRootCreationAndFootprintOwnership) {
  AdaptiveHierarchy hierarchy;

  const avr::spatial::GridCoord2D base_coord{.x = 2, .y = 3};
  const auto root_key_opt = hierarchy.ensureRootForBase(base_coord);
  ASSERT_TRUE(root_key_opt.has_value());
  const auto root_key = *root_key_opt;

  EXPECT_EQ(root_key.level, avr::spatial::L3);
  EXPECT_EQ(hierarchy.nodeCount(), 1u);
  EXPECT_EQ(hierarchy.leafCount(), 1u);
  EXPECT_EQ(hierarchy.internalCount(), 0u);

  const auto stats = hierarchy.statistics();
  EXPECT_EQ(stats.leaf_count_by_level[avr::spatial::L3], 1u);
  EXPECT_EQ(stats.leaf_count_by_level[avr::spatial::L2], 0u);
  EXPECT_EQ(stats.leaf_count_by_level[avr::spatial::L1], 0u);
  EXPECT_EQ(stats.leaf_count_by_level[avr::spatial::L0], 0u);

  // An L3 cell covers 8x8 = 64 base cells
  const auto root_cell_coord = root_key.decode();
  const int32_t start_bx = root_cell_coord.x * 8;
  const int32_t start_by = root_cell_coord.y * 8;

  for (int32_t dx = 0; dx < 8; ++dx) {
    for (int32_t dy = 0; dy < 8; ++dy) {
      const avr::spatial::GridCoord2D pt{.x = start_bx + dx, .y = start_by + dy};
      const auto leaf = hierarchy.findLeafContainingBase(pt);
      ASSERT_TRUE(leaf.has_value());
      EXPECT_EQ(*leaf, root_key);
    }
  }

  // Outside root footprint: no leaf
  const avr::spatial::GridCoord2D outside_pt{.x = start_bx + 10, .y = start_by};
  EXPECT_FALSE(hierarchy.findLeafContainingBase(outside_pt).has_value());

  EXPECT_TRUE(hierarchy.validateInvariants());
}

// Section 32: One split test
TEST(AdaptiveHierarchyTest, SingleSplitFromL3ToL2) {
  AdaptiveHierarchy hierarchy;
  const avr::spatial::GridCoord2D base{.x = 0, .y = 0};
  const auto root_key = *hierarchy.ensureRootForBase(base);

  EXPECT_EQ(hierarchy.nodeCount(), 1u);
  EXPECT_EQ(hierarchy.leafCount(), 1u);
  EXPECT_EQ(hierarchy.internalCount(), 0u);

  // Split root
  EXPECT_TRUE(hierarchy.splitLeaf(root_key));

  EXPECT_EQ(hierarchy.nodeCount(), 5u);
  EXPECT_EQ(hierarchy.leafCount(), 4u);
  EXPECT_EQ(hierarchy.internalCount(), 1u);
  EXPECT_EQ(hierarchy.nodeState(root_key), NodeState::INTERNAL);

  const auto stats = hierarchy.statistics();
  EXPECT_EQ(stats.internal_count_by_level[avr::spatial::L3], 1u);
  EXPECT_EQ(stats.leaf_count_by_level[avr::spatial::L2], 4u);

  // All 64 base cells must have exactly one owning L2 leaf
  for (int32_t bx = 0; bx < 8; ++bx) {
    for (int32_t by = 0; by < 8; ++by) {
      const auto leaf = hierarchy.findLeafContainingBase(avr::spatial::GridCoord2D{.x = bx, .y = by});
      ASSERT_TRUE(leaf.has_value());
      EXPECT_EQ(leaf->level, avr::spatial::L2);
    }
  }

  EXPECT_TRUE(hierarchy.validateInvariants());
}

// Section 33: Mixed-resolution test
TEST(AdaptiveHierarchyTest, MixedResolutionHierarchyOwnership) {
  AdaptiveHierarchy hierarchy;
  const auto root_key = *hierarchy.ensureRootForBase(avr::spatial::GridCoord2D{.x = 0, .y = 0});
  ASSERT_TRUE(hierarchy.splitLeaf(root_key));

  // Find L2 child owning (0, 0) and split to L1
  const auto l2_leaf = *hierarchy.findLeafContainingBase(avr::spatial::GridCoord2D{.x = 0, .y = 0});
  EXPECT_EQ(l2_leaf.level, avr::spatial::L2);
  ASSERT_TRUE(hierarchy.splitLeaf(l2_leaf));

  // Active leaves: 3 L2 + 4 L1 = 7 leaves
  EXPECT_EQ(hierarchy.leafCount(), 7u);
  auto stats = hierarchy.statistics();
  EXPECT_EQ(stats.leaf_count_by_level[avr::spatial::L2], 3u);
  EXPECT_EQ(stats.leaf_count_by_level[avr::spatial::L1], 4u);

  // Split one L1 child owning (0, 0) to L0
  const auto l1_leaf = *hierarchy.findLeafContainingBase(avr::spatial::GridCoord2D{.x = 0, .y = 0});
  EXPECT_EQ(l1_leaf.level, avr::spatial::L1);
  ASSERT_TRUE(hierarchy.splitLeaf(l1_leaf));

  // Active leaves: 3 L2 + 3 L1 + 4 L0 = 10 leaves
  EXPECT_EQ(hierarchy.leafCount(), 10u);
  stats = hierarchy.statistics();
  EXPECT_EQ(stats.leaf_count_by_level[avr::spatial::L2], 3u);
  EXPECT_EQ(stats.leaf_count_by_level[avr::spatial::L1], 3u);
  EXPECT_EQ(stats.leaf_count_by_level[avr::spatial::L0], 4u);

  // Exhaustively verify every one of the 64 base cells has exactly one owner
  for (int32_t bx = 0; bx < 8; ++bx) {
    for (int32_t by = 0; by < 8; ++by) {
      const auto leaf = hierarchy.findLeafContainingBase(avr::spatial::GridCoord2D{.x = bx, .y = by});
      ASSERT_TRUE(leaf.has_value());
      EXPECT_TRUE(leaf->level <= avr::spatial::L2);
    }
  }

  EXPECT_TRUE(hierarchy.validateInvariants());
}

// Section 34: Full root refinement test
TEST(AdaptiveHierarchyTest, FullRootRefinementToL0Counts) {
  AdaptiveHierarchy hierarchy;
  const auto root_key = *hierarchy.ensureRootForBase(avr::spatial::GridCoord2D{.x = 0, .y = 0});
  EXPECT_EQ(root_key.level, avr::spatial::L3);

  // Refine all 64 base cells to L0
  for (int32_t bx = 0; bx < 8; ++bx) {
    for (int32_t by = 0; by < 8; ++by) {
      const auto leaf = hierarchy.refineToLevel(avr::spatial::GridCoord2D{.x = bx, .y = by}, avr::spatial::L0);
      ASSERT_TRUE(leaf.has_value());
      EXPECT_EQ(leaf->level, avr::spatial::L0);
    }
  }

  // Expected quadtree node counts:
  // L3: 1 (INTERNAL)
  // L2: 4 (INTERNAL)
  // L1: 16 (INTERNAL)
  // L0: 64 (LEAF)
  // Total nodes: 1 + 4 + 16 + 64 = 85
  // Internal nodes: 1 + 4 + 16 = 21
  // Leaves: 64
  EXPECT_EQ(hierarchy.nodeCount(), 85u);
  EXPECT_EQ(hierarchy.internalCount(), 21u);
  EXPECT_EQ(hierarchy.leafCount(), 64u);

  const auto stats = hierarchy.statistics();
  EXPECT_EQ(stats.internal_count_by_level[avr::spatial::L3], 1u);
  EXPECT_EQ(stats.internal_count_by_level[avr::spatial::L2], 4u);
  EXPECT_EQ(stats.internal_count_by_level[avr::spatial::L1], 16u);
  EXPECT_EQ(stats.leaf_count_by_level[avr::spatial::L0], 64u);

  // Every base cell must be owned by an L0 leaf
  for (int32_t bx = 0; bx < 8; ++bx) {
    for (int32_t by = 0; by < 8; ++by) {
      const auto leaf = hierarchy.findLeafContainingBase(avr::spatial::GridCoord2D{.x = bx, .y = by});
      ASSERT_TRUE(leaf.has_value());
      EXPECT_EQ(leaf->level, avr::spatial::L0);
    }
  }

  EXPECT_TRUE(hierarchy.validateInvariants());
}

// Section 35: Full merge back test
TEST(AdaptiveHierarchyTest, FullMergeBackToL3Root) {
  AdaptiveHierarchy hierarchy;
  const auto root_key = *hierarchy.ensureRootForBase(avr::spatial::GridCoord2D{.x = 0, .y = 0});

  // Fully refine to L0
  for (int32_t bx = 0; bx < 8; ++bx) {
    for (int32_t by = 0; by < 8; ++by) {
      hierarchy.refineToLevel(avr::spatial::GridCoord2D{.x = bx, .y = by}, avr::spatial::L0);
    }
  }
  EXPECT_EQ(hierarchy.nodeCount(), 85u);

  // Bottom-up merge: 16 L1 parents merge their 64 L0 children
  for (int32_t cx = 0; cx < 4; ++cx) {
    for (int32_t cy = 0; cy < 4; ++cy) {
      const auto l1_coord = avr::spatial::CellCoord{.x = cx, .y = cy, .level = avr::spatial::L1};
      const auto l1_key = *avr::spatial::CellKey::fromCellCoord(l1_coord);
      ASSERT_TRUE(hierarchy.mergeChildren(l1_key));
    }
  }
  EXPECT_EQ(hierarchy.nodeCount(), 21u);
  EXPECT_EQ(hierarchy.leafCount(), 16u);
  EXPECT_EQ(hierarchy.internalCount(), 5u);
  EXPECT_TRUE(hierarchy.validateInvariants());

  // 4 L2 parents merge their 16 L1 children
  for (int32_t cx = 0; cx < 2; ++cx) {
    for (int32_t cy = 0; cy < 2; ++cy) {
      const auto l2_coord = avr::spatial::CellCoord{.x = cx, .y = cy, .level = avr::spatial::L2};
      const auto l2_key = *avr::spatial::CellKey::fromCellCoord(l2_coord);
      ASSERT_TRUE(hierarchy.mergeChildren(l2_key));
    }
  }
  EXPECT_EQ(hierarchy.nodeCount(), 5u);
  EXPECT_EQ(hierarchy.leafCount(), 4u);
  EXPECT_EQ(hierarchy.internalCount(), 1u);
  EXPECT_TRUE(hierarchy.validateInvariants());

  // 1 L3 root merges its 4 L2 children
  ASSERT_TRUE(hierarchy.mergeChildren(root_key));
  EXPECT_EQ(hierarchy.nodeCount(), 1u);
  EXPECT_EQ(hierarchy.leafCount(), 1u);
  EXPECT_EQ(hierarchy.internalCount(), 0u);
  EXPECT_EQ(hierarchy.nodeState(root_key), NodeState::LEAF);
  EXPECT_TRUE(hierarchy.validateInvariants());
}

// Section 36: Illegal merge test
TEST(AdaptiveHierarchyTest, IllegalMergeRejectedWithoutMutation) {
  AdaptiveHierarchy hierarchy;
  const auto root_key = *hierarchy.ensureRootForBase(avr::spatial::GridCoord2D{.x = 0, .y = 0});
  hierarchy.splitLeaf(root_key);

  // Split one of the L2 children to L1
  const auto l2_child = *hierarchy.findLeafContainingBase(avr::spatial::GridCoord2D{.x = 0, .y = 0});
  hierarchy.splitLeaf(l2_child);

  const auto snap_before = hierarchy.topologySnapshot();
  const auto stats_before = hierarchy.statistics();

  // Attempt to merge root while one child is INTERNAL
  EXPECT_FALSE(hierarchy.mergeChildren(root_key));

  const auto snap_after = hierarchy.topologySnapshot();
  const auto stats_after = hierarchy.statistics();

  EXPECT_EQ(snap_before, snap_after);
  EXPECT_EQ(stats_before.node_count, stats_after.node_count);
  EXPECT_EQ(stats_before.leaf_count, stats_after.leaf_count);
  EXPECT_EQ(stats_before.internal_count, stats_after.internal_count);
  EXPECT_TRUE(hierarchy.validateInvariants());
}

// Section 37: Illegal split test
TEST(AdaptiveHierarchyTest, IllegalSplitRejections) {
  AdaptiveHierarchy hierarchy;
  const auto root_key = *hierarchy.ensureRootForBase(avr::spatial::GridCoord2D{.x = 0, .y = 0});

  // Refine to L0
  const auto l0_leaf = *hierarchy.refineToLevel(avr::spatial::GridCoord2D{.x = 0, .y = 0}, avr::spatial::L0);
  EXPECT_EQ(l0_leaf.level, avr::spatial::L0);

  // 1. Cannot split L0 leaf
  EXPECT_FALSE(hierarchy.splitLeaf(l0_leaf));

  // 2. Cannot split missing key
  const auto fake_key = *avr::spatial::CellKey::fromCellCoord(avr::spatial::CellCoord{.x = 999, .y = 999, .level = 2});
  EXPECT_FALSE(hierarchy.splitLeaf(fake_key));

  // 3. Cannot split already INTERNAL node
  EXPECT_FALSE(hierarchy.splitLeaf(root_key));

  EXPECT_TRUE(hierarchy.validateInvariants());
}

// Section 40: Multiple root test
TEST(AdaptiveHierarchyTest, MultipleAdjacentRootsAcrossQuadrants) {
  AdaptiveHierarchy hierarchy;

  // Allocate 4 roots touching the origin in all 4 quadrants:
  // Q1: (0, 0), Q2: (-1, 0), Q3: (-1, -1), Q4: (0, -1)
  const std::vector<avr::spatial::GridCoord2D> root_points = {
      {.x = 2, .y = 2},
      {.x = -2, .y = 2},
      {.x = -2, .y = -2},
      {.x = 2, .y = -2},
  };

  std::vector<avr::spatial::CellKey> root_keys;
  for (const auto& pt : root_points) {
    const auto rk = hierarchy.ensureRootForBase(pt);
    ASSERT_TRUE(rk.has_value());
    root_keys.push_back(*rk);
  }
  EXPECT_EQ(hierarchy.leafCount(), 4u);

  // Refine root 0 to L0, root 1 to L1, root 2 to L2, leave root 3 at L3
  hierarchy.refineToLevel(root_points[0], avr::spatial::L0);
  hierarchy.refineToLevel(root_points[1], avr::spatial::L1);
  hierarchy.refineToLevel(root_points[2], avr::spatial::L2);

  EXPECT_EQ(hierarchy.findLeafContainingBase(root_points[0])->level, avr::spatial::L0);
  EXPECT_EQ(hierarchy.findLeafContainingBase(root_points[1])->level, avr::spatial::L1);
  EXPECT_EQ(hierarchy.findLeafContainingBase(root_points[2])->level, avr::spatial::L2);
  EXPECT_EQ(hierarchy.findLeafContainingBase(root_points[3])->level, avr::spatial::L3);

  EXPECT_TRUE(hierarchy.validateInvariants());
}

// Section 41 & 42: Refine-to-level and refine already finer
TEST(AdaptiveHierarchyTest, RefineToLevelBehavior) {
  AdaptiveHierarchy hierarchy;
  const avr::spatial::GridCoord2D base{.x = 10, .y = 10};

  // Refine from fresh root to L0
  const auto l0_key = hierarchy.refineToLevel(base, avr::spatial::L0);
  ASSERT_TRUE(l0_key.has_value());
  EXPECT_EQ(l0_key->level, avr::spatial::L0);

  // Sibling cells remain coarser (10 leaves total)
  EXPECT_EQ(hierarchy.leafCount(), 10u);

  // Section 42: If location already owns L0 and caller requests L2, do not coarsen
  const auto same_key = hierarchy.refineToLevel(base, avr::spatial::L2);
  ASSERT_TRUE(same_key.has_value());
  EXPECT_EQ(same_key->level, avr::spatial::L0);
  EXPECT_EQ(hierarchy.leafCount(), 10u);
  EXPECT_TRUE(hierarchy.validateInvariants());
}

// Section 44: Tile reuse across split and merge cycles
TEST(AdaptiveHierarchyTest, TilePoolReuseAcrossSplitMergeCycles) {
  AdaptiveHierarchy hierarchy(32);
  const avr::spatial::GridCoord2D base{.x = 0, .y = 0};

  for (int cycle = 0; cycle < 5; ++cycle) {
    const auto root_key = *hierarchy.ensureRootForBase(base);
    hierarchy.splitLeaf(root_key);

    const size_t tiles_split = hierarchy.tileCount();
    const size_t pool_in_use_split = hierarchy.statistics().pool_in_use;

    hierarchy.mergeChildren(root_key);

    const size_t tiles_merged = hierarchy.tileCount();
    const size_t pool_in_use_merged = hierarchy.statistics().pool_in_use;

    EXPECT_LT(tiles_merged, tiles_split);
    EXPECT_LT(pool_in_use_merged, pool_in_use_split);
  }

  // Pool capacity must not have ballooned
  EXPECT_LE(hierarchy.statistics().pool_block_count, 2u);
  EXPECT_TRUE(hierarchy.validateInvariants());
}

TEST(AdaptiveHierarchyTest, ResetClearsAllTopologyAndRetainsPoolCapacity) {
  AdaptiveHierarchy hierarchy;
  hierarchy.refineToLevel(avr::spatial::GridCoord2D{.x = 0, .y = 0}, avr::spatial::L0);
  EXPECT_GT(hierarchy.nodeCount(), 0u);
  EXPECT_GT(hierarchy.tileCount(), 0u);

  const size_t initial_pool_capacity = hierarchy.statistics().pool_capacity;

  hierarchy.reset();
  EXPECT_EQ(hierarchy.nodeCount(), 0u);
  EXPECT_EQ(hierarchy.leafCount(), 0u);
  EXPECT_EQ(hierarchy.internalCount(), 0u);
  EXPECT_EQ(hierarchy.tileCount(), 0u);
  EXPECT_TRUE(hierarchy.leafSnapshot().empty());
  EXPECT_TRUE(hierarchy.topologySnapshot().empty());

  const auto stats = hierarchy.statistics();
  EXPECT_EQ(stats.pool_in_use, 0u);
  EXPECT_EQ(stats.pool_capacity, initial_pool_capacity);
  EXPECT_EQ(stats.pool_free, initial_pool_capacity);
  EXPECT_TRUE(hierarchy.validateInvariants());
}

// Section 3: Refine invalid-input tests
TEST(AdaptiveHierarchyTest, RefineInvalidLevelLeavesTopologyUnmutated) {
  AdaptiveHierarchy hierarchy;

  // 1. On empty hierarchy: invalid requested levels reject with zero mutation
  EXPECT_FALSE(hierarchy.refineToLevel(avr::spatial::GridCoord2D{.x = 0, .y = 0}, 4).has_value());
  EXPECT_FALSE(hierarchy.refineToLevel(avr::spatial::GridCoord2D{.x = 0, .y = 0}, 255).has_value());
  EXPECT_EQ(hierarchy.nodeCount(), 0u);
  EXPECT_EQ(hierarchy.tileCount(), 0u);
  EXPECT_TRUE(hierarchy.leafSnapshot().empty());
  EXPECT_TRUE(hierarchy.topologySnapshot().empty());

  // Populate hierarchy with mixed-resolution nodes
  hierarchy.refineToLevel(avr::spatial::GridCoord2D{.x = 0, .y = 0}, avr::spatial::L1);
  hierarchy.ensureRootForBase(avr::spatial::GridCoord2D{.x = 20, .y = 20});

  const auto snap_before = hierarchy.topologySnapshot();
  const auto leaves_before = hierarchy.leafSnapshot();
  const auto stats_before = hierarchy.statistics();

  // Test invalid requested levels: 4, 5, 255
  EXPECT_FALSE(hierarchy.refineToLevel(avr::spatial::GridCoord2D{.x = 0, .y = 0}, 4).has_value());
  EXPECT_FALSE(hierarchy.refineToLevel(avr::spatial::GridCoord2D{.x = 0, .y = 0}, 255).has_value());
  EXPECT_FALSE(hierarchy.refineToLevel(avr::spatial::GridCoord2D{.x = 100, .y = 100}, 4).has_value());
  EXPECT_FALSE(hierarchy.refineToLevel(avr::spatial::GridCoord2D{.x = 100, .y = 100}, 255).has_value());

  const auto snap_after = hierarchy.topologySnapshot();
  const auto leaves_after = hierarchy.leafSnapshot();
  const auto stats_after = hierarchy.statistics();

  EXPECT_EQ(snap_before, snap_after);
  EXPECT_EQ(leaves_before, leaves_after);
  EXPECT_EQ(stats_before.node_count, stats_after.node_count);
  EXPECT_EQ(stats_before.leaf_count, stats_after.leaf_count);
  EXPECT_EQ(stats_before.internal_count, stats_after.internal_count);
  EXPECT_EQ(stats_before.tile_count, stats_after.tile_count);
  EXPECT_EQ(stats_before.leaf_count_by_level, stats_after.leaf_count_by_level);
  EXPECT_EQ(stats_before.internal_count_by_level, stats_after.internal_count_by_level);
  EXPECT_TRUE(hierarchy.validateInvariants());
}

// Section 8: Non-copyable, non-movable contract
TEST(AdaptiveHierarchyTest, NonCopyableNonMovableContract) {
  static_assert(!std::is_copy_constructible_v<AdaptiveHierarchy>);
  static_assert(!std::is_copy_assignable_v<AdaptiveHierarchy>);
  static_assert(!std::is_move_constructible_v<AdaptiveHierarchy>);
  static_assert(!std::is_move_assignable_v<AdaptiveHierarchy>);
  SUCCEED();
}

// Section 11: Multi-root shared child-tile lifecycle
TEST(AdaptiveHierarchyTest, MultiRootSharedChildTileLifecycle) {
  AdaptiveHierarchy hierarchy;

  // Base coordinates for two adjacent L3 roots in tile (0, 0, L3):
  // Root A covers base [0..7, 0..7] -> CellCoord (0, 0, L3)
  // Root B covers base [8..15, 0..7] -> CellCoord (1, 0, L3)
  const auto root_a_opt = hierarchy.ensureRootForBase(avr::spatial::GridCoord2D{.x = 2, .y = 2});
  const auto root_b_opt = hierarchy.ensureRootForBase(avr::spatial::GridCoord2D{.x = 10, .y = 2});
  ASSERT_TRUE(root_a_opt.has_value());
  ASSERT_TRUE(root_b_opt.has_value());
  const auto root_a = *root_a_opt;
  const auto root_b = *root_b_opt;

  // Both roots reside in the same L3 tile (0, 0, L3)
  EXPECT_EQ(hierarchy.tileCount(), 1u);

  // Refine Root A to L2 -> 4 children in L2 tile (0, 0, L2)
  ASSERT_TRUE(hierarchy.splitLeaf(root_a));
  EXPECT_EQ(hierarchy.tileCount(), 2u);

  // Refine Root B to L2 -> 4 children also in L2 tile (0, 0, L2)
  ASSERT_TRUE(hierarchy.splitLeaf(root_b));
  // Tile count remains 2 because both sets of L2 children share the same L2 tile!
  EXPECT_EQ(hierarchy.tileCount(), 2u);

  // Merge Branch A back to L3
  ASSERT_TRUE(hierarchy.mergeChildren(root_a));
  EXPECT_EQ(hierarchy.nodeState(root_a), NodeState::LEAF);

  // CRITICAL: The shared child-level L2 tile must REMAIN in TileStore
  // because Branch B still uses its 4 child slots!
  EXPECT_EQ(hierarchy.tileCount(), 2u);
  EXPECT_TRUE(hierarchy.validateInvariants());

  // Merge Branch B back to L3
  ASSERT_TRUE(hierarchy.mergeChildren(root_b));
  EXPECT_EQ(hierarchy.nodeState(root_b), NodeState::LEAF);

  // CRITICAL: Now that the last child in the shared L2 tile disappeared,
  // the tile must be removed from TileStore and released to TilePool!
  EXPECT_EQ(hierarchy.tileCount(), 1u);
  EXPECT_TRUE(hierarchy.validateInvariants());
}

// Section 12: Reset and reuse hierarchy
TEST(AdaptiveHierarchyTest, ResetAndReuseHierarchy) {
  AdaptiveHierarchy hierarchy;

  // Build complex mixed-resolution hierarchy across quadrants
  hierarchy.refineToLevel(avr::spatial::GridCoord2D{.x = 5, .y = 5}, avr::spatial::L0);
  hierarchy.refineToLevel(avr::spatial::GridCoord2D{.x = -15, .y = 25}, avr::spatial::L1);
  hierarchy.refineToLevel(avr::spatial::GridCoord2D{.x = -30, .y = -30}, avr::spatial::L2);

  const auto stats_initial = hierarchy.statistics();
  EXPECT_GT(stats_initial.node_count, 0u);
  EXPECT_GT(stats_initial.tile_count, 0u);
  const size_t recorded_capacity = stats_initial.pool_capacity;
  const size_t recorded_blocks = stats_initial.pool_block_count;

  hierarchy.reset();

  EXPECT_EQ(hierarchy.tileCount(), 0u);
  EXPECT_EQ(hierarchy.nodeCount(), 0u);
  EXPECT_EQ(hierarchy.leafCount(), 0u);
  EXPECT_EQ(hierarchy.internalCount(), 0u);
  EXPECT_TRUE(hierarchy.leafSnapshot().empty());
  EXPECT_TRUE(hierarchy.topologySnapshot().empty());

  const auto stats_after_reset = hierarchy.statistics();
  EXPECT_EQ(stats_after_reset.pool_in_use, 0u);
  EXPECT_EQ(stats_after_reset.pool_free, recorded_capacity);
  EXPECT_EQ(stats_after_reset.pool_capacity, recorded_capacity);
  EXPECT_EQ(stats_after_reset.pool_block_count, recorded_blocks);

  // Build a new hierarchy at completely different coordinates
  hierarchy.refineToLevel(avr::spatial::GridCoord2D{.x = 100, .y = 100}, avr::spatial::L1);

  // Verify pool reuses existing capacity without allocating an extra block
  const auto stats_new = hierarchy.statistics();
  EXPECT_EQ(stats_new.pool_block_count, recorded_blocks);
  EXPECT_EQ(stats_new.pool_capacity, recorded_capacity);
  EXPECT_GT(stats_new.pool_in_use, 0u);

  // Verify no stale topology from the previous hierarchy reappears
  EXPECT_FALSE(hierarchy.findLeafContainingBase(avr::spatial::GridCoord2D{.x = 5, .y = 5}).has_value());
  EXPECT_FALSE(hierarchy.findLeafContainingBase(avr::spatial::GridCoord2D{.x = -15, .y = 25}).has_value());
  EXPECT_FALSE(hierarchy.findLeafContainingBase(avr::spatial::GridCoord2D{.x = -30, .y = -30}).has_value());
  EXPECT_TRUE(hierarchy.findLeafContainingBase(avr::spatial::GridCoord2D{.x = 100, .y = 100}).has_value());
  EXPECT_TRUE(hierarchy.validateInvariants());
}

// Section 14: HierarchyStats consistency invariants
TEST(AdaptiveHierarchyTest, HierarchyStatsConsistencyInvariants) {
  AdaptiveHierarchy hierarchy;

  auto verify_invariants = [&hierarchy]() {
    const auto s = hierarchy.statistics();
    EXPECT_EQ(s.node_count, s.leaf_count + s.internal_count);

    size_t sum_leaves = 0;
    for (size_t c : s.leaf_count_by_level) {
      sum_leaves += c;
    }
    EXPECT_EQ(sum_leaves, s.leaf_count);

    size_t sum_internals = 0;
    for (size_t c : s.internal_count_by_level) {
      sum_internals += c;
    }
    EXPECT_EQ(sum_internals, s.internal_count);

    EXPECT_EQ(s.pool_capacity, s.pool_in_use + s.pool_free);
    EXPECT_EQ(s.tile_count, s.pool_in_use);
    EXPECT_EQ(s.sizeof_adaptive_tile, sizeof(AdaptiveTile));
    EXPECT_EQ(s.tile_payload_bytes, s.tile_count * sizeof(AdaptiveTile));
    EXPECT_EQ(s.pool_reserved_payload_bytes, s.pool_capacity * sizeof(AdaptiveTile));
    EXPECT_TRUE(hierarchy.validateInvariants());
  };

  // State 1: Fresh empty
  verify_invariants();

  // State 2: Single root
  hierarchy.ensureRootForBase(avr::spatial::GridCoord2D{.x = 0, .y = 0});
  verify_invariants();

  // State 3: Refined to L0
  hierarchy.refineToLevel(avr::spatial::GridCoord2D{.x = 0, .y = 0}, avr::spatial::L0);
  verify_invariants();

  // State 4: Multiple roots and mixed resolutions
  hierarchy.refineToLevel(avr::spatial::GridCoord2D{.x = 50, .y = -20}, avr::spatial::L2);
  hierarchy.refineToLevel(avr::spatial::GridCoord2D{.x = -50, .y = 20}, avr::spatial::L1);
  verify_invariants();

  // State 5: Merged partial
  const auto root_opt = hierarchy.rootKeyForBase(avr::spatial::GridCoord2D{.x = 50, .y = -20});
  ASSERT_TRUE(root_opt.has_value());
  EXPECT_TRUE(hierarchy.mergeChildren(*root_opt));
  verify_invariants();

  // State 6: Reset
  hierarchy.reset();
  verify_invariants();
}

}  // namespace avr::mapping


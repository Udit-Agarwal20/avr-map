#include <gtest/gtest.h>

#include <vector>

#include "avr_mapping/adaptive_hierarchy.hpp"
#include "avr_spatial/coordinate.hpp"
#include "avr_spatial/tile_key.hpp"

namespace avr::mapping {

// Section 38: Negative coordinate test
TEST(HierarchyBoundaryTest, NegativeCoordinatesNearCriticalBoundaries) {
  AdaptiveHierarchy hierarchy;

  const std::vector<int32_t> critical_coords = {-1, -8, -9, -16, -17, -32, -33};

  for (int32_t cx : critical_coords) {
    for (int32_t cy : critical_coords) {
      hierarchy.reset();
      const avr::spatial::GridCoord2D base{.x = cx, .y = cy};

      // 1. Ensure root exists
      const auto root_key = hierarchy.ensureRootForBase(base);
      ASSERT_TRUE(root_key.has_value());

      // Verify spatial consistency with avr_spatial
      const auto expected_l3_coord = avr::spatial::gridToCellCoord(base, avr::spatial::L3);
      const auto expected_root_key = avr::spatial::CellKey::fromCellCoord(expected_l3_coord);
      ASSERT_TRUE(expected_root_key.has_value());
      EXPECT_EQ(*root_key, *expected_root_key);

      // Verify tile spatial mapping
      const auto tile_coord = avr::spatial::cellToTileCoord(expected_l3_coord);
      const auto expected_tile_key = avr::spatial::TileKey::fromTileCoord(tile_coord);
      ASSERT_TRUE(expected_tile_key.has_value());

      // 2. Ownership lookup
      const auto leaf = hierarchy.findLeafContainingBase(base);
      ASSERT_TRUE(leaf.has_value());
      EXPECT_EQ(*leaf, *root_key);

      // 3. Refine to L0
      const auto l0_leaf = hierarchy.refineToLevel(base, avr::spatial::L0);
      ASSERT_TRUE(l0_leaf.has_value());
      EXPECT_EQ(l0_leaf->level, avr::spatial::L0);

      const auto expected_l0_coord = avr::spatial::gridToCellCoord(base, avr::spatial::L0);
      const auto expected_l0_key = avr::spatial::CellKey::fromCellCoord(expected_l0_coord);
      ASSERT_TRUE(expected_l0_key.has_value());
      EXPECT_EQ(*l0_leaf, *expected_l0_key);

      // Verify leaf lookup after refinement
      const auto refined_leaf = hierarchy.findLeafContainingBase(base);
      ASSERT_TRUE(refined_leaf.has_value());
      EXPECT_EQ(*refined_leaf, *expected_l0_key);
    }
  }

  EXPECT_TRUE(hierarchy.validateInvariants());
}

// Section 39: Tile boundary test across multiple levels
TEST(HierarchyBoundaryTest, AdaptiveTileBoundariesCrossing) {
  AdaptiveHierarchy hierarchy;

  // Coordinates around 16-cell tile transitions: -17, -16, -15, -1, 0, 15, 16, 17
  const std::vector<int32_t> test_indices = {-17, -16, -15, -1, 0, 15, 16, 17};

  for (int32_t x : test_indices) {
    for (int32_t y : test_indices) {
      // Scale to base-grid coordinates
      const avr::spatial::GridCoord2D base{.x = x * 8, .y = y * 8};
      const auto root = hierarchy.ensureRootForBase(base);
      ASSERT_TRUE(root.has_value());

      // Check leaf before refinement
      const auto leaf_l3 = hierarchy.findLeafContainingBase(base);
      ASSERT_TRUE(leaf_l3.has_value());
      EXPECT_EQ(*leaf_l3, *root);

      // Refine across boundary
      const auto leaf_l1 = hierarchy.refineToLevel(base, avr::spatial::L1);
      ASSERT_TRUE(leaf_l1.has_value());
      EXPECT_EQ(leaf_l1->level, avr::spatial::L1);

      const auto found = hierarchy.findLeafContainingBase(base);
      ASSERT_TRUE(found.has_value());
      EXPECT_EQ(*found, *leaf_l1);
    }
  }

  EXPECT_TRUE(hierarchy.validateInvariants());
}

}  // namespace avr::mapping

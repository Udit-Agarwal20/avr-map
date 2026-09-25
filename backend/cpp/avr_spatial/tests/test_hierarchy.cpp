#include <gtest/gtest.h>

#include <limits>
#include <optional>

#include "avr_spatial/bounds.hpp"
#include "avr_spatial/cell_key.hpp"

namespace avr::spatial {

TEST(HierarchyTest, ParentRelationship) {
  // Cell at level 1: (3, -2) -> parent at level 2: (floorDiv(3,2), floorDiv(-2,2)) = (1, -1)
  const auto child_opt = CellKey::fromCellCoord({.x = 3, .y = -2, .level = L1});
  ASSERT_TRUE(child_opt.has_value());
  const auto parent_opt = parentOf(*child_opt);

  ASSERT_TRUE(parent_opt.has_value());
  EXPECT_EQ(parent_opt->level, L2);

  const CellCoord p_coord = parent_opt->decode();
  EXPECT_EQ(p_coord.x, 1);
  EXPECT_EQ(p_coord.y, -1);
}

TEST(HierarchyTest, Level3HasNoParent) {
  const auto k3 = CellKey::fromCellCoord({.x = 0, .y = 0, .level = L3});
  ASSERT_TRUE(k3.has_value());
  EXPECT_EQ(parentOf(*k3), std::nullopt);
}

TEST(HierarchyTest, Level0HasNoChildren) {
  const auto k0 = CellKey::fromCellCoord({.x = 0, .y = 0, .level = L0});
  ASSERT_TRUE(k0.has_value());
  EXPECT_EQ(childrenOf(*k0), std::nullopt);
}

TEST(HierarchyTest, InvalidLevelsReturnNullopt) {
  // Direct creation with invalid levels returns nullopt
  EXPECT_EQ(CellKey::fromCellCoord({.x = 0, .y = 0, .level = 4}), std::nullopt);
  EXPECT_EQ(CellKey::fromCellCoord({.x = 0, .y = 0, .level = 5}), std::nullopt);
  EXPECT_EQ(CellKey::fromCellCoord({.x = 0, .y = 0, .level = 255}), std::nullopt);

  // CellKey manually constructed with invalid level
  const CellKey invalid_key{.morton = 0, .level = 4};
  EXPECT_EQ(parentOf(invalid_key), std::nullopt);
  EXPECT_EQ(childrenOf(invalid_key), std::nullopt);
}

TEST(HierarchyTest, ParentChildOverflowHardening) {
  // Audit 2*cx and 2*cx+1 near INT32_MIN and INT32_MAX
  // Representable boundary: cx in [-1073741824, 1073741823]

  // 1. Extreme boundary values that ARE representable
  const auto max_repr_key = CellKey::fromCellCoord({.x = 1073741823, .y = 1073741823, .level = L1});
  ASSERT_TRUE(max_repr_key.has_value());
  const auto max_children = childrenOf(*max_repr_key);
  ASSERT_TRUE(max_children.has_value());
  EXPECT_EQ((*max_children)[3].decode().x, std::numeric_limits<int32_t>::max());

  const auto min_repr_key = CellKey::fromCellCoord({.x = -1073741824, .y = -1073741824, .level = L1});
  ASSERT_TRUE(min_repr_key.has_value());
  const auto min_children = childrenOf(*min_repr_key);
  ASSERT_TRUE(min_children.has_value());
  EXPECT_EQ((*min_children)[0].decode().x, std::numeric_limits<int32_t>::min());

  // 2. Values that would overflow int32_t must fail safely by returning std::nullopt
  // cx = INT32_MAX -> 2*cx overflows
  const auto overflow_pos = CellKey::fromCellCoord({.x = std::numeric_limits<int32_t>::max(), .y = 0, .level = L1});
  ASSERT_TRUE(overflow_pos.has_value());
  EXPECT_EQ(childrenOf(*overflow_pos), std::nullopt);

  // cx = INT32_MIN -> 2*cx overflows
  const auto overflow_neg = CellKey::fromCellCoord({.x = std::numeric_limits<int32_t>::min(), .y = 0, .level = L1});
  ASSERT_TRUE(overflow_neg.has_value());
  EXPECT_EQ(childrenOf(*overflow_neg), std::nullopt);

  // cx = 1073741824 (just 1 above max representable) -> 2*cx + 1 overflows
  const auto overflow_just_above = CellKey::fromCellCoord({.x = 1073741824, .y = 0, .level = L1});
  ASSERT_TRUE(overflow_just_above.has_value());
  EXPECT_EQ(childrenOf(*overflow_just_above), std::nullopt);

  // 3. Parent of extreme coordinates never overflows
  const auto parent_of_max = parentOf(*overflow_pos);
  ASSERT_TRUE(parent_of_max.has_value());
  EXPECT_EQ(parent_of_max->decode().x, std::numeric_limits<int32_t>::max() / 2);

  const auto parent_of_min = parentOf(*overflow_neg);
  ASSERT_TRUE(parent_of_min.has_value());
  EXPECT_EQ(parent_of_min->decode().x, std::numeric_limits<int32_t>::min() / 2);
}

TEST(HierarchyTest, ParentChildInvariants) {
  const CellCoord parents_to_test[] = {
      {.x = 0, .y = 0, .level = L1},
      {.x = 5, .y = -3, .level = L2},
      {.x = -1, .y = -1, .level = L3},
      {.x = 100, .y = -250, .level = L2},
  };

  for (const auto& p_coord : parents_to_test) {
    const auto parent_opt = CellKey::fromCellCoord(p_coord);
    ASSERT_TRUE(parent_opt.has_value());
    const CellKey parent = *parent_opt;

    const auto children_opt = childrenOf(parent);
    ASSERT_TRUE(children_opt.has_value());

    const auto& children = *children_opt;
    EXPECT_EQ(children.size(), 4U);

    // Invariant 1: parentOf(child) == parent for all 4 children
    for (size_t i = 0; i < 4; ++i) {
      const auto p_check = parentOf(children[i]);
      ASSERT_TRUE(p_check.has_value());
      EXPECT_EQ(*p_check, parent) << "Child index " << i << " did not map back to parent";
    }

    // Invariant 2: Children bounds exactly match parent bounds
    const Bounds2D pb = parent.bounds();
    const Bounds2D c0 = children[0].bounds();
    const Bounds2D c1 = children[1].bounds();
    const Bounds2D c2 = children[2].bounds();
    const Bounds2D c3 = children[3].bounds();

    // Check union min and max
    EXPECT_DOUBLE_EQ(c0.min_x, pb.min_x);
    EXPECT_DOUBLE_EQ(c0.min_y, pb.min_y);

    EXPECT_DOUBLE_EQ(c1.max_x, pb.max_x);
    EXPECT_DOUBLE_EQ(c1.min_y, pb.min_y);

    EXPECT_DOUBLE_EQ(c2.min_x, pb.min_x);
    EXPECT_DOUBLE_EQ(c2.max_y, pb.max_y);

    EXPECT_DOUBLE_EQ(c3.max_x, pb.max_x);
    EXPECT_DOUBLE_EQ(c3.max_y, pb.max_y);

    // Adjacency without gaps: c0 max_x == c1 min_x
    EXPECT_DOUBLE_EQ(c0.max_x, c1.min_x);
    EXPECT_DOUBLE_EQ(c2.max_x, c3.min_x);
    EXPECT_DOUBLE_EQ(c0.max_y, c2.min_y);
    EXPECT_DOUBLE_EQ(c1.max_y, c3.min_y);

    // Parent encloses all 4 children
    EXPECT_TRUE(pb.encloses(c0));
    EXPECT_TRUE(pb.encloses(c1));
    EXPECT_TRUE(pb.encloses(c2));
    EXPECT_TRUE(pb.encloses(c3));
  }
}

}  // namespace avr::spatial


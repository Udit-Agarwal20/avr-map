#include <gtest/gtest.h>

#include <optional>
#include <type_traits>
#include <unordered_set>

#include "avr_spatial/cell_key.hpp"

namespace avr::spatial {

TEST(CellKeyTest, BasicCreationAndDecode) {
  const CellCoord orig{.x = 12, .y = -34, .level = L1};
  const auto key_opt = CellKey::fromCellCoord(orig);
  ASSERT_TRUE(key_opt.has_value());
  const CellKey key = *key_opt;

  EXPECT_EQ(key.level, L1);

  const CellCoord dec = key.decode();
  EXPECT_EQ(dec.x, orig.x);
  EXPECT_EQ(dec.y, orig.y);
  EXPECT_EQ(dec.level, orig.level);
}

TEST(CellKeyTest, WorldPointCreation) {
  // At (0.12, 0.22) in meters at L0 (0.05m):
  // 0.12 * 20.0 = 2.4 -> cell x = 2
  // 0.22 * 20.0 = 4.4 -> cell y = 4
  const WorldPoint2D pt{.x = 0.12, .y = 0.22};
  const auto key_opt = CellKey::fromWorldPoint(pt, L0);
  ASSERT_TRUE(key_opt.has_value());
  const CellKey key = *key_opt;

  const CellCoord dec = key.decode();
  EXPECT_EQ(dec.x, 2);
  EXPECT_EQ(dec.y, 4);
  EXPECT_EQ(dec.level, L0);

  // Bounds check
  const Bounds2D b = key.bounds();
  EXPECT_TRUE(b.contains(pt));
}

TEST(CellKeyTest, InvalidLevelRejection) {
  EXPECT_EQ(CellKey::fromCellCoord({.x = 0, .y = 0, .level = 4}), std::nullopt);
  EXPECT_EQ(CellKey::fromCellCoord({.x = 0, .y = 0, .level = 5}), std::nullopt);
  EXPECT_EQ(CellKey::fromCellCoord({.x = 0, .y = 0, .level = 255}), std::nullopt);

  EXPECT_EQ(CellKey::fromGridCoord(GridCoord2D{.x = 0, .y = 0}, 4), std::nullopt);
  EXPECT_EQ(CellKey::fromWorldPoint(WorldPoint2D{.x = 0.0, .y = 0.0}, 4), std::nullopt);
}

TEST(CellKeyTest, EqualityAndOrdering) {
  const auto k1 = *CellKey::fromCellCoord({.x = 1, .y = 2, .level = L0});
  const auto k2 = *CellKey::fromCellCoord({.x = 1, .y = 2, .level = L0});
  const auto k3 = *CellKey::fromCellCoord({.x = 1, .y = 2, .level = L1});
  const auto k4 = *CellKey::fromCellCoord({.x = 2, .y = 2, .level = L0});

  EXPECT_EQ(k1, k2);
  EXPECT_NE(k1, k3);
  EXPECT_NE(k1, k4);

  // Deterministic ordering
  EXPECT_TRUE((k1 < k3) || (k3 < k1));
}

TEST(CellKeyTest, HashInUnorderedSet) {
  std::unordered_set<CellKey> set;

  const auto k1 = *CellKey::fromCellCoord({.x = 10, .y = -20, .level = L0});
  const auto k2 = *CellKey::fromCellCoord({.x = 10, .y = -20, .level = L1});
  const auto k3 = *CellKey::fromCellCoord({.x = 10, .y = -20, .level = L0});

  set.insert(k1);
  set.insert(k2);
  set.insert(k3);

  EXPECT_EQ(set.size(), 2U);
  EXPECT_TRUE(set.contains(k1));
  EXPECT_TRUE(set.contains(k2));
}

TEST(CellKeyTest, MemoryLayoutAndAlignment) {
  // Empirically verified under AppleClang 21.0.0 / Clang 18+ / GCC 13+ (64-bit systems)
  // uint64_t morton (8 bytes) + uint8_t level (1 byte) + 7 bytes natural tail padding = 16 bytes.
  EXPECT_EQ(sizeof(CellKey), 16U);
  EXPECT_EQ(alignof(CellKey), 8U);
  EXPECT_TRUE(std::is_trivially_copyable_v<CellKey>);
  EXPECT_TRUE(std::is_standard_layout_v<CellKey>);
}

}  // namespace avr::spatial


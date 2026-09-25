#include <gtest/gtest.h>

#include <limits>
#include <optional>
#include <type_traits>
#include <unordered_set>

#include "avr_spatial/cell_key.hpp"
#include "avr_spatial/tile_key.hpp"

namespace avr::spatial {

TEST(TileCoordinateTest, RequiredPromptExamples) {
  // cx = -1 -> tile_x = -1, local_x = 15
  const CellCoord c_neg1{.x = -1, .y = -1, .level = L0};
  const TileCoord t_neg1 = cellToTileCoord(c_neg1);
  EXPECT_EQ(t_neg1.x, -1);
  EXPECT_EQ(t_neg1.y, -1);
  const auto [lx_neg1, ly_neg1] = localCoordsFromIndex(localCellIndex(c_neg1.x, c_neg1.y));
  EXPECT_EQ(lx_neg1, 15);
  EXPECT_EQ(ly_neg1, 15);

  // cx = -16 -> tile_x = -1, local_x = 0
  const CellCoord c_neg16{.x = -16, .y = -16, .level = L0};
  const TileCoord t_neg16 = cellToTileCoord(c_neg16);
  EXPECT_EQ(t_neg16.x, -1);
  EXPECT_EQ(t_neg16.y, -1);
  const auto [lx_neg16, ly_neg16] = localCoordsFromIndex(localCellIndex(c_neg16.x, c_neg16.y));
  EXPECT_EQ(lx_neg16, 0);
  EXPECT_EQ(ly_neg16, 0);

  // cx = -17 -> tile_x = -2, local_x = 15
  const CellCoord c_neg17{.x = -17, .y = -17, .level = L0};
  const TileCoord t_neg17 = cellToTileCoord(c_neg17);
  EXPECT_EQ(t_neg17.x, -2);
  EXPECT_EQ(t_neg17.y, -2);
  const auto [lx_neg17, ly_neg17] = localCoordsFromIndex(localCellIndex(c_neg17.x, c_neg17.y));
  EXPECT_EQ(lx_neg17, 15);
  EXPECT_EQ(ly_neg17, 15);

  // Positive coordinates: cx = 0 -> tx = 0, lx = 0
  const CellCoord c_0{.x = 0, .y = 0, .level = L0};
  EXPECT_EQ(cellToTileCoord(c_0).x, 0);
  EXPECT_EQ(localCoordsFromIndex(localCellIndex(0, 0)).first, 0);

  // cx = 15 -> tx = 0, lx = 15
  EXPECT_EQ(cellToTileCoord(CellCoord{.x = 15, .y = 15, .level = L0}).x, 0);
  EXPECT_EQ(localCoordsFromIndex(localCellIndex(15, 15)).first, 15);

  // cx = 16 -> tx = 1, lx = 0
  EXPECT_EQ(cellToTileCoord(CellCoord{.x = 16, .y = 16, .level = L0}).x, 1);
  EXPECT_EQ(localCoordsFromIndex(localCellIndex(16, 16)).first, 0);
}

TEST(LocalIndexTest, ExhaustiveAll256Indices) {
  for (uint16_t idx = 0; idx < 256; ++idx) {
    const auto [lx, ly] = localCoordsFromIndex(idx);
    EXPECT_GE(lx, 0);
    EXPECT_LT(lx, 16);
    EXPECT_GE(ly, 0);
    EXPECT_LT(ly, 16);

    const uint16_t recomputed = localCellIndex(lx, ly);
    EXPECT_EQ(recomputed, idx);
  }
}

TEST(TileKeyTest, TileCellReconstructionRoundTrip) {
  const TileCoord tiles[] = {
      {.x = 0, .y = 0, .level = L0},
      {.x = -1, .y = -1, .level = L1},
      {.x = 12, .y = -34, .level = L2},
      {.x = -100, .y = 200, .level = L3},
  };

  for (const auto& tc : tiles) {
    const auto tk_opt = TileKey::fromTileCoord(tc);
    ASSERT_TRUE(tk_opt.has_value());
    const TileKey tk = *tk_opt;
    EXPECT_EQ(tk.decode(), tc);

    // Verify all 256 cells reconstruct into this tile
    for (uint16_t idx = 0; idx < 256; ++idx) {
      const auto cell_key_opt = cellKeyFromTileAndLocal(tk, idx);
      ASSERT_TRUE(cell_key_opt.has_value());
      const CellKey cell_key = *cell_key_opt;
      const CellCoord cell_coord = cell_key.decode();

      // Check cell maps back to the same tile key and same local index
      const auto derived_tile = TileKey::fromCellKey(cell_key);
      ASSERT_TRUE(derived_tile.has_value());
      EXPECT_EQ(*derived_tile, tk);

      const uint16_t derived_idx = localCellIndex(cell_coord.x, cell_coord.y);
      EXPECT_EQ(derived_idx, idx);
    }
  }
}

TEST(TileKeyTest, TileReconstructionOverflowHardening) {
  // Audit tile_x * 16 + local_x near coordinate extremes
  // Representable range of tile_x in int32_t: [-134217728, 134217727]

  // 1. Extreme representable tile coordinates
  const auto max_repr_tile = TileKey::fromTileCoord({.x = 134217727, .y = 134217727, .level = L0});
  ASSERT_TRUE(max_repr_tile.has_value());
  const auto max_cell = cellKeyFromTileAndLocal(*max_repr_tile, 255);  // lx=15, ly=15
  ASSERT_TRUE(max_cell.has_value());
  EXPECT_EQ(max_cell->decode().x, std::numeric_limits<int32_t>::max());

  const auto min_repr_tile = TileKey::fromTileCoord({.x = -134217728, .y = -134217728, .level = L0});
  ASSERT_TRUE(min_repr_tile.has_value());
  const auto min_cell = cellKeyFromTileAndLocal(*min_repr_tile, 0);  // lx=0, ly=0
  ASSERT_TRUE(min_cell.has_value());
  EXPECT_EQ(min_cell->decode().x, std::numeric_limits<int32_t>::min());

  // 2. Unrepresentable tile coordinates that would overflow int32_t
  // tx = INT32_MAX -> tx * 16 overflows
  const auto overflow_pos_tile = TileKey::fromTileCoord({.x = std::numeric_limits<int32_t>::max(), .y = 0, .level = L0});
  ASSERT_TRUE(overflow_pos_tile.has_value());
  EXPECT_EQ(cellKeyFromTileAndLocal(*overflow_pos_tile, 0), std::nullopt);

  // tx = INT32_MIN -> tx * 16 overflows
  const auto overflow_neg_tile = TileKey::fromTileCoord({.x = std::numeric_limits<int32_t>::min(), .y = 0, .level = L0});
  ASSERT_TRUE(overflow_neg_tile.has_value());
  EXPECT_EQ(cellKeyFromTileAndLocal(*overflow_neg_tile, 0), std::nullopt);

  // tx = 134217728 (just 1 above max representable tile) -> overflows int32_t
  const auto overflow_just_above = TileKey::fromTileCoord({.x = 134217728, .y = 0, .level = L0});
  ASSERT_TRUE(overflow_just_above.has_value());
  EXPECT_EQ(cellKeyFromTileAndLocal(*overflow_just_above, 0), std::nullopt);

  // Invalid local index >= 256
  EXPECT_EQ(cellKeyFromTileAndLocal(*max_repr_tile, 256), std::nullopt);
  EXPECT_EQ(cellKeyFromTileAndLocal(*max_repr_tile, 1000), std::nullopt);
}

TEST(TileKeyTest, InvalidLevelHardening) {
  EXPECT_EQ(TileKey::fromTileCoord({.x = 0, .y = 0, .level = 4}), std::nullopt);
  EXPECT_EQ(TileKey::fromTileCoord({.x = 0, .y = 0, .level = 255}), std::nullopt);
  EXPECT_EQ(cellToTileCoordSafe({.x = 0, .y = 0, .level = 4}), std::nullopt);
}

TEST(TileKeyTest, HashInUnorderedSet) {
  std::unordered_set<TileKey> set;
  const auto t1 = TileKey::fromTileCoord({.x = 1, .y = -1, .level = L0});
  const auto t2 = TileKey::fromTileCoord({.x = 1, .y = -1, .level = L1});
  const auto t3 = TileKey::fromTileCoord({.x = 1, .y = -1, .level = L0});

  ASSERT_TRUE(t1.has_value());
  ASSERT_TRUE(t2.has_value());
  ASSERT_TRUE(t3.has_value());

  set.insert(*t1);
  set.insert(*t2);
  set.insert(*t3);

  EXPECT_EQ(set.size(), 2U);
  EXPECT_TRUE(set.contains(*t1));
  EXPECT_TRUE(set.contains(*t2));
}

TEST(TileKeyTest, MemoryLayoutAndAlignment) {
  // Empirically verified under AppleClang 21.0.0 / Clang 18+ / GCC 13+ (64-bit systems)
  // uint64_t morton (8 bytes) + uint8_t level (1 byte) + 7 bytes natural tail padding = 16 bytes.
  EXPECT_EQ(sizeof(TileKey), 16U);
  EXPECT_EQ(alignof(TileKey), 8U);
  EXPECT_TRUE(std::is_trivially_copyable_v<TileKey>);
  EXPECT_TRUE(std::is_standard_layout_v<TileKey>);
}

}  // namespace avr::spatial


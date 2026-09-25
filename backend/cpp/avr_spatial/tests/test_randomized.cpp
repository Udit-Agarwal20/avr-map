#include <gtest/gtest.h>

#include <cstdint>
#include <random>

#include "avr_spatial/bounds.hpp"
#include "avr_spatial/cell_key.hpp"
#include "avr_spatial/morton.hpp"
#include "avr_spatial/tile_key.hpp"
#include "avr_spatial/zigzag.hpp"

namespace avr::spatial {

class RandomizedSpatialInvariantTest : public ::testing::Test {
 protected:
  static constexpr uint64_t FIXED_SEED = 26053ULL;
  static constexpr size_t ITERATION_COUNT = 100000;
};

TEST_F(RandomizedSpatialInvariantTest, ZigZagRoundTrip100k) {
  std::mt19937_64 rng(FIXED_SEED);
  std::uniform_int_distribution<int32_t> dist(INT32_MIN, INT32_MAX);

  for (size_t i = 0; i < ITERATION_COUNT; ++i) {
    const int32_t orig = dist(rng);
    const uint32_t enc = zigzagEncode(orig);
    const int32_t dec = zigzagDecode(enc);
    ASSERT_EQ(dec, orig) << "ZigZag failed on iteration " << i << " with value " << orig;
  }
}

TEST_F(RandomizedSpatialInvariantTest, MortonRoundTrip100k) {
  std::mt19937_64 rng(FIXED_SEED + 1);
  std::uniform_int_distribution<int32_t> dist(INT32_MIN, INT32_MAX);

  for (size_t i = 0; i < ITERATION_COUNT; ++i) {
    const int32_t x = dist(rng);
    const int32_t y = dist(rng);

    const uint64_t code = morton2DEncode(x, y);
    const auto [rx, ry] = morton2DDecodePair(code);

    ASSERT_EQ(rx, x) << "Morton X failed on iteration " << i;
    ASSERT_EQ(ry, y) << "Morton Y failed on iteration " << i;
  }
}

TEST_F(RandomizedSpatialInvariantTest, HierarchyParentChildInvariants100k) {
  std::mt19937_64 rng(FIXED_SEED + 2);
  // Keep coordinates within +/-1,000,000 cells (50 km radius) to avoid parent bounds overflowing double
  std::uniform_int_distribution<int32_t> coord_dist(-1000000, 1000000);
  std::uniform_int_distribution<uint32_t> level_dist(L1, L3);

  for (size_t i = 0; i < ITERATION_COUNT; ++i) {
    const int32_t px = coord_dist(rng);
    const int32_t py = coord_dist(rng);
    const uint8_t plevel = static_cast<uint8_t>(level_dist(rng));

    const auto parent_opt = CellKey::fromCellCoord({.x = px, .y = py, .level = plevel});
    ASSERT_TRUE(parent_opt.has_value());
    const CellKey parent = *parent_opt;
    const auto children_opt = childrenOf(parent);
    ASSERT_TRUE(children_opt.has_value());

    const auto& children = *children_opt;
    const Bounds2D pb = parent.bounds();

    for (const auto& child : children) {
      const auto back_parent = parentOf(child);
      ASSERT_TRUE(back_parent.has_value());
      ASSERT_EQ(*back_parent, parent) << "Child-to-parent mismatch on iteration " << i;

      // Half-open bound containment: child min must be >= parent min, child max <= parent max
      const Bounds2D cb = child.bounds();
      ASSERT_TRUE(pb.encloses(cb)) << "Parent bounds did not enclose child on iteration " << i;
    }
  }
}

TEST_F(RandomizedSpatialInvariantTest, TileDecompositionAndReconstruction100k) {
  std::mt19937_64 rng(FIXED_SEED + 3);
  // Scale down to prevent overflow when tile coordinate is multiplied by TILE_SIZE (16)
  std::uniform_int_distribution<int32_t> coord_dist(INT32_MIN / 32, INT32_MAX / 32);
  std::uniform_int_distribution<uint32_t> level_dist(L0, L3);

  for (size_t i = 0; i < ITERATION_COUNT; ++i) {
    const int32_t cx = coord_dist(rng);
    const int32_t cy = coord_dist(rng);
    const uint8_t level = static_cast<uint8_t>(level_dist(rng));

    const CellCoord cell_coord{.x = cx, .y = cy, .level = level};
    const TileCoord tile_coord = cellToTileCoord(cell_coord);
    const uint16_t local_idx = localCellIndex(cx, cy);

    ASSERT_LT(local_idx, 256U);

    const auto tk_opt = TileKey::fromTileCoord(tile_coord);
    ASSERT_TRUE(tk_opt.has_value());
    const TileKey tk = *tk_opt;

    const auto reconstructed_key_opt = cellKeyFromTileAndLocal(tk, local_idx);
    ASSERT_TRUE(reconstructed_key_opt.has_value());
    const CellKey reconstructed_key = *reconstructed_key_opt;
    const CellCoord rec_coord = reconstructed_key.decode();

    ASSERT_EQ(rec_coord.x, cx) << "Reconstructed X mismatch on iteration " << i;
    ASSERT_EQ(rec_coord.y, cy) << "Reconstructed Y mismatch on iteration " << i;
    ASSERT_EQ(rec_coord.level, level);
  }
}

}  // namespace avr::spatial

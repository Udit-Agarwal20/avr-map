#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "avr_mapping/uniform_map.hpp"
#include "avr_spatial/cell_key.hpp"
#include "avr_spatial/constants.hpp"
#include "avr_spatial/coordinate.hpp"
#include "avr_spatial/tile_key.hpp"

namespace {

using avr::mapping::PointXYZ;
using avr::mapping::UniformMap;
using avr::spatial::BASE_CELLS_PER_METER;
using avr::spatial::BASE_RESOLUTION_METERS;  // 0.05 m
using avr::spatial::CellCoord;
using avr::spatial::CellKey;
using avr::spatial::cellToTileCoord;
using avr::spatial::localCellIndex;
using avr::spatial::TileCoord;
using avr::spatial::TileKey;

// Helper to calculate cell center in meters from integer cell coordinate
double cellToCenter(int32_t cell) {
  return (static_cast<double>(cell) + 0.5) * BASE_RESOLUTION_METERS;
}

// Section 28: Tile boundary tests around -17, -16, -15, -1, 0, 15, 16, 17
TEST(UniformTileBoundaryTest, BoundaryCellsAcrossTileTransitions) {
  UniformMap map(200.0);  // large radius to comfortably accept all points

  const std::vector<int32_t> test_cx = {-17, -16, -15, -1, 0, 15, 16, 17};
  const int32_t fixed_cy = 2;

  std::vector<PointXYZ> points;
  points.reserve(test_cx.size());

  for (int32_t cx : test_cx) {
    points.push_back(PointXYZ{
        .x = cellToCenter(cx),
        .y = cellToCenter(fixed_cy),
        .z = 1.0,
    });
  }

  const auto stats = map.integrateFrame(points, 1);
  EXPECT_EQ(stats.accepted_points, test_cx.size());
  EXPECT_EQ(stats.cells_created, test_cx.size());

  for (int32_t cx : test_cx) {
    const CellCoord cell_coord{.x = cx, .y = fixed_cy, .level = 0};
    const auto expected_cell_key = CellKey::fromCellCoord(cell_coord);
    ASSERT_TRUE(expected_cell_key.has_value());

    const TileCoord expected_tile_coord = cellToTileCoord(cell_coord);
    const auto expected_tile_key = TileKey::fromTileCoord(expected_tile_coord);
    ASSERT_TRUE(expected_tile_key.has_value());

    const uint16_t expected_local_idx = localCellIndex(cx, fixed_cy);

    // Verify cell is found in map via CellKey
    const auto* cell_by_key = map.findCell(*expected_cell_key);
    ASSERT_NE(cell_by_key, nullptr);
    EXPECT_EQ(cell_by_key->observationCount(), 1u);

    // Verify cell is found in map via world coordinates
    const auto* cell_by_xy = map.findCell(cellToCenter(cx), cellToCenter(fixed_cy));
    ASSERT_NE(cell_by_xy, nullptr);
    EXPECT_EQ(cell_by_xy, cell_by_key);

    // Validate that expected_local_idx is within [0, 255]
    EXPECT_LT(expected_local_idx, 256u);
  }
}

// Section 29: Half-open world boundary tests with nextafter around 0, +/-0.05, +/-0.10, +/-1.00
TEST(UniformTileBoundaryTest, HalfOpenWorldBoundariesWithNextAfter) {
  UniformMap map(200.0);

  struct BoundaryTestCase {
    double input_x;
    int32_t expected_cell_x;
  };

  const std::vector<BoundaryTestCase> test_cases = {
      // 0.0 boundary: cell 0 is [0.0, 0.05), cell -1 is [-0.05, 0.0)
      {0.0, 0},
      {std::nextafter(0.0, -1.0), -1},
      {std::nextafter(0.0, 1.0), 0},

      // +0.05 boundary: cell 1 is [0.05, 0.10), cell 0 is [0.0, 0.05)
      {0.05, 1},
      {std::nextafter(0.05, 0.0), 0},
      {std::nextafter(0.05, 1.0), 1},

      // -0.05 boundary: cell -1 is [-0.05, 0.0), cell -2 is [-0.10, -0.05)
      {-0.05, -1},
      {std::nextafter(-0.05, -1.0), -2},
      {std::nextafter(-0.05, 0.0), -1},

      // +0.10 boundary: cell 2 is [0.10, 0.15), cell 1 is [0.05, 0.10)
      {0.10, 2},
      {std::nextafter(0.10, 0.0), 1},
      {std::nextafter(0.10, 1.0), 2},

      // -0.10 boundary: cell -2 is [-0.10, -0.05), cell -3 is [-0.15, -0.10)
      {-0.10, -2},
      {std::nextafter(-0.10, -1.0), -3},
      {std::nextafter(-0.10, 0.0), -2},

      // +1.00 boundary: 1.00 / 0.05 = 20. Cell 20 is [1.00, 1.05), cell 19 is [0.95, 1.00)
      {1.00, 20},
      {std::nextafter(1.00, 0.0), 19},
      {std::nextafter(1.00, 2.0), 20},

      // -1.00 boundary: -1.00 / 0.05 = -20. Cell -20 is [-1.00, -0.95), cell -21 is [-1.05, -1.00)
      {-1.00, -20},
      {std::nextafter(-1.00, -2.0), -21},
      {std::nextafter(-1.00, 0.0), -20},
  };

  for (size_t i = 0; i < test_cases.size(); ++i) {
    const auto& tc = test_cases[i];
    const PointXYZ pt{.x = tc.input_x, .y = 0.01, .z = static_cast<double>(i)};
    const auto stats = map.integrateFrame(std::span<const PointXYZ>(&pt, 1), i + 1);
    EXPECT_EQ(stats.accepted_points, 1u);

    // Verify cell lookup
    const CellCoord expected_coord{.x = tc.expected_cell_x, .y = 0, .level = 0};
    const auto key_opt = CellKey::fromCellCoord(expected_coord);
    ASSERT_TRUE(key_opt.has_value());

    const auto* cell = map.findCell(*key_opt);
    ASSERT_NE(cell, nullptr) << "Failed at test case " << i << " with input_x=" << tc.input_x
                             << ", expected_cell_x=" << tc.expected_cell_x;
    EXPECT_EQ(cell->lastSeenFrame(), i + 1);
  }
}

// Section 30: All four XY quadrants
TEST(UniformTileBoundaryTest, FourQuadrantsCoverageAndSnapshotOrdering) {
  UniformMap map(100.0);

  // Quadrants: Q1 (+,+), Q2 (-,+), Q3 (-,-), Q4 (+,-)
  const std::vector<PointXYZ> points = {
      PointXYZ{.x = 2.025, .y = 3.025, .z = 10.0},    // Q1: cx=40, cy=60
      PointXYZ{.x = -2.025, .y = 3.025, .z = 20.0},   // Q2: cx=-41, cy=60
      PointXYZ{.x = -2.025, .y = -3.025, .z = 30.0},  // Q3: cx=-41, cy=-61
      PointXYZ{.x = 2.025, .y = -3.025, .z = 40.0},   // Q4: cx=40, cy=-61
  };

  const auto stats = map.integrateFrame(points, 5);
  EXPECT_EQ(stats.accepted_points, 4u);
  EXPECT_EQ(stats.cells_created, 4u);
  EXPECT_EQ(map.activeCellCount(), 4u);

  // Check lookup in all quadrants
  for (const auto& pt : points) {
    const auto* cell = map.findCell(pt.x, pt.y);
    ASSERT_NE(cell, nullptr);
    EXPECT_EQ(cell->observationCount(), 1u);
  }

  // Check deterministic snapshot ordering
  const auto snap = map.snapshot();
  ASSERT_EQ(snap.size(), 4u);
  for (size_t i = 1; i < snap.size(); ++i) {
    EXPECT_LT(snap[i - 1].key, snap[i].key);
  }
}

// Section 34: Multiple cells in the same tile
TEST(UniformTileBoundaryTest, MultipleCellsSameTile) {
  UniformMap map(100.0);

  // Tile (0, 0) covers cells [0..15] x [0..15]
  std::vector<PointXYZ> points;
  // Insert 20 distinct cells within tile (0, 0)
  for (int32_t lx = 0; lx < 5; ++lx) {
    for (int32_t ly = 0; ly < 4; ++ly) {
      points.push_back(PointXYZ{
          .x = cellToCenter(lx),
          .y = cellToCenter(ly),
          .z = static_cast<double>(lx + ly),
      });
    }
  }

  const auto stats = map.integrateFrame(points, 1);
  EXPECT_EQ(stats.accepted_points, 20u);
  EXPECT_EQ(stats.cells_created, 20u);
  EXPECT_EQ(stats.tiles_created, 1u);

  EXPECT_EQ(map.tileCount(), 1u);
  EXPECT_EQ(map.activeCellCount(), 20u);

  const auto map_stats = map.mapStats();
  EXPECT_EQ(map_stats.tile_count, 1u);
  EXPECT_EQ(map_stats.active_cell_count, 20u);
  EXPECT_EQ(map_stats.allocated_cell_slots, 256u);
}

// Section 35: Multiple tiles crossing boundaries
TEST(UniformTileBoundaryTest, MultipleTilesCrossingBoundaries) {
  UniformMap map(100.0);

  // Point in tile (0, 0): cell (0, 0)
  // Point in tile (1, 0): cell (16, 0)
  // Point in tile (0, 1): cell (0, 16)
  // Point in tile (-1, -1): cell (-1, -1)
  const std::vector<PointXYZ> points = {
      PointXYZ{.x = cellToCenter(0), .y = cellToCenter(0), .z = 1.0},
      PointXYZ{.x = cellToCenter(16), .y = cellToCenter(0), .z = 2.0},
      PointXYZ{.x = cellToCenter(0), .y = cellToCenter(16), .z = 3.0},
      PointXYZ{.x = cellToCenter(-1), .y = cellToCenter(-1), .z = 4.0},
  };

  const auto stats = map.integrateFrame(points, 1);
  EXPECT_EQ(stats.accepted_points, 4u);
  EXPECT_EQ(stats.tiles_created, 4u);
  EXPECT_EQ(stats.cells_created, 4u);
  EXPECT_EQ(map.tileCount(), 4u);
  EXPECT_EQ(map.activeCellCount(), 4u);

  const auto map_stats = map.mapStats();
  EXPECT_EQ(map_stats.tile_count, 4u);
  EXPECT_EQ(map_stats.allocated_cell_slots, 4u * 256u);
}

// Section 27: Deterministic memory facts
TEST(UniformTileBoundaryTest, DeterministicMemoryFacts) {
  UniformMap map(100.0);

  std::vector<PointXYZ> points = {
      PointXYZ{.x = cellToCenter(0), .y = cellToCenter(0), .z = 1.0},
      PointXYZ{.x = cellToCenter(16), .y = cellToCenter(16), .z = 2.0},
      PointXYZ{.x = cellToCenter(-16), .y = cellToCenter(-16), .z = 3.0},
  };

  map.integrateFrame(points, 1);
  const auto s = map.mapStats();

  EXPECT_EQ(s.allocated_cell_slots, s.tile_count * 256u);
  EXPECT_EQ(s.tile_payload_bytes, s.tile_count * s.sizeof_uniform_tile);
  EXPECT_EQ(s.active_cell_payload_bytes, s.active_cell_count * s.sizeof_uniform_cell);
  EXPECT_EQ(s.sizeof_uniform_cell, sizeof(avr::mapping::UniformCell));
  EXPECT_EQ(s.sizeof_uniform_tile, sizeof(avr::mapping::UniformTile));
  EXPECT_EQ(sizeof(avr::mapping::UniformTile), 12368u);
  EXPECT_EQ(alignof(avr::mapping::UniformTile), 8u);
}

}  // namespace

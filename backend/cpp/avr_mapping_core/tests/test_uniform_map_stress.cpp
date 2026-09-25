#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <random>
#include <vector>

#include "avr_mapping/uniform_map.hpp"

namespace {

using avr::mapping::PointXYZ;
using avr::mapping::UniformMap;

// Generates a deterministic batch of at least 250,000 synthetic LiDAR points
std::vector<PointXYZ> generateDeterministicSyntheticFrame(size_t total_points = 250'000) {
  std::vector<PointXYZ> points;
  points.reserve(total_points);

  std::mt19937_64 rng(0xDEADBEEFCAFEULL);
  // Spread across [-40, +40] meters for dense multi-tile, multi-cell, positive & negative coverage
  std::uniform_real_distribution<double> coord_dist(-40.0, 40.0);
  std::uniform_real_distribution<double> z_dist(-5.0, 15.0);

  // Ingest the majority of valid points within 100 m extent
  const size_t valid_points = total_points - 600;
  for (size_t i = 0; i < valid_points; ++i) {
    points.push_back(PointXYZ{
        .x = coord_dist(rng),
        .y = coord_dist(rng),
        .z = z_dist(rng),
    });
  }

  // Include 100 non-finite points (NaN / Inf)
  const double nan_val = std::numeric_limits<double>::quiet_NaN();
  const double inf_val = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < 50; ++i) {
    points.push_back(PointXYZ{.x = nan_val, .y = coord_dist(rng), .z = 0.0});
    points.push_back(PointXYZ{.x = coord_dist(rng), .y = inf_val, .z = 0.0});
  }

  // Include 500 points intentionally outside 100m extent radius
  std::uniform_real_distribution<double> outside_dist(105.0, 150.0);
  for (size_t i = 0; i < 500; ++i) {
    points.push_back(PointXYZ{
        .x = outside_dist(rng),
        .y = outside_dist(rng),
        .z = z_dist(rng),
    });
  }

  return points;
}

// Section 42: Stress test with >= 250,000 points
TEST(UniformMapStressTest, Stress250kSyntheticPoints) {
  constexpr size_t kPointCount = 250'000;
  const auto points = generateDeterministicSyntheticFrame(kPointCount);
  ASSERT_EQ(points.size(), kPointCount);

  UniformMap map(100.0);

  const auto start = std::chrono::steady_clock::now();
  const auto stats = map.integrateFrame(points, 1);
  const auto end = std::chrono::steady_clock::now();

  const auto elapsed_ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
  std::cout << "[INFO] Integrated " << kPointCount << " points in " << elapsed_ms << " ms"
            << std::endl;

  // Internal consistency of FrameUpdateStats
  EXPECT_EQ(stats.input_points, kPointCount);
  EXPECT_EQ(stats.rejected_non_finite, 100u);
  EXPECT_EQ(stats.rejected_outside_extent, 500u);
  EXPECT_EQ(stats.rejected_out_of_range, 0u);
  EXPECT_EQ(stats.rejected_numeric, 0u);
  EXPECT_EQ(stats.accepted_points, kPointCount - 600u);

  EXPECT_EQ(stats.input_points, stats.accepted_points + stats.rejected_non_finite +
                                    stats.rejected_out_of_range + stats.rejected_outside_extent +
                                    stats.rejected_numeric);
  EXPECT_EQ(stats.accepted_points, stats.cells_created + stats.cells_updated);
  EXPECT_GT(stats.cells_created, 0u);
  EXPECT_GT(stats.cells_updated, 0u);
  EXPECT_GT(stats.tiles_created, 0u);
  EXPECT_GT(stats.unique_cells_touched, 0u);
  EXPECT_LE(stats.unique_cells_touched, stats.accepted_points);
  EXPECT_LE(stats.tiles_created, stats.cells_created);

  // Map state consistency
  EXPECT_EQ(map.activeCellCount(), stats.cells_created);
  EXPECT_EQ(map.tileCount(), stats.tiles_created);
  EXPECT_EQ(map.totalAcceptedObservations(), stats.accepted_points);

  // Structural statistics facts
  const auto s = map.mapStats();
  EXPECT_EQ(s.tile_count, map.tileCount());
  EXPECT_EQ(s.active_cell_count, map.activeCellCount());
  EXPECT_EQ(s.allocated_cell_slots, s.tile_count * 256u);
  EXPECT_EQ(s.tile_payload_bytes, s.tile_count * sizeof(avr::mapping::UniformTile));
  EXPECT_EQ(s.active_cell_payload_bytes, s.active_cell_count * sizeof(avr::mapping::UniformCell));

  // Snapshot verification
  const auto snap = map.snapshot();
  EXPECT_EQ(snap.size(), map.activeCellCount());
  for (size_t i = 1; i < snap.size(); ++i) {
    EXPECT_LT(snap[i - 1].key, snap[i].key);
  }
}

// Section 43: Repeatability across two fresh maps
TEST(UniformMapStressTest, RepeatabilityAcrossFreshMaps) {
  constexpr size_t kPointCount = 250'000;
  const auto points = generateDeterministicSyntheticFrame(kPointCount);

  UniformMap map1(100.0);
  UniformMap map2(100.0);

  const auto stats1 = map1.integrateFrame(points, 42);
  const auto stats2 = map2.integrateFrame(points, 42);

  // Compare FrameUpdateStats
  EXPECT_EQ(stats1.input_points, stats2.input_points);
  EXPECT_EQ(stats1.accepted_points, stats2.accepted_points);
  EXPECT_EQ(stats1.rejected_non_finite, stats2.rejected_non_finite);
  EXPECT_EQ(stats1.rejected_outside_extent, stats2.rejected_outside_extent);
  EXPECT_EQ(stats1.rejected_out_of_range, stats2.rejected_out_of_range);
  EXPECT_EQ(stats1.rejected_numeric, stats2.rejected_numeric);
  EXPECT_EQ(stats1.tiles_created, stats2.tiles_created);
  EXPECT_EQ(stats1.cells_created, stats2.cells_created);
  EXPECT_EQ(stats1.cells_updated, stats2.cells_updated);
  EXPECT_EQ(stats1.unique_cells_touched, stats2.unique_cells_touched);

  // Compare Map metrics
  EXPECT_EQ(map1.tileCount(), map2.tileCount());
  EXPECT_EQ(map1.activeCellCount(), map2.activeCellCount());
  EXPECT_EQ(map1.totalAcceptedObservations(), map2.totalAcceptedObservations());

  // Compare Snapshots
  const auto snap1 = map1.snapshot();
  const auto snap2 = map2.snapshot();

  ASSERT_EQ(snap1.size(), snap2.size());
  for (size_t i = 0; i < snap1.size(); ++i) {
    EXPECT_EQ(snap1[i].key, snap2[i].key);
    EXPECT_EQ(snap1[i].cell.observationCount(), snap2[i].cell.observationCount());
    EXPECT_DOUBLE_EQ(snap1[i].cell.meanZ(), snap2[i].cell.meanZ());
    EXPECT_DOUBLE_EQ(snap1[i].cell.m2Z(), snap2[i].cell.m2Z());
    EXPECT_DOUBLE_EQ(snap1[i].cell.minZ(), snap2[i].cell.minZ());
    EXPECT_DOUBLE_EQ(snap1[i].cell.maxZ(), snap2[i].cell.maxZ());
    EXPECT_EQ(snap1[i].cell.lastSeenFrame(), snap2[i].cell.lastSeenFrame());
  }
}

}  // namespace

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <vector>

#include "avr_mapping/evidence_signal_builder.hpp"
#include "avr_mapping/frame_evidence.hpp"
#include "avr_mapping/point_xyz.hpp"
#include "avr_spatial/cell_key.hpp"
#include "avr_spatial/constants.hpp"

using namespace avr::mapping;
using namespace avr::spatial;

namespace {

// Deterministic Linear Congruential Generator for reproducible stress data
struct DeterministicRng {
  uint64_t state{123456789ULL};

  uint32_t next() noexcept {
    state = state * 6364136223846793005ULL + 1442695040888963407ULL;
    return static_cast<uint32_t>(state >> 32);
  }

  double nextDouble(double min_val, double max_val) noexcept {
    const double norm = static_cast<double>(next()) / static_cast<double>(0xFFFFFFFFU);
    return min_val + norm * (max_val - min_val);
  }
};

std::vector<PointXYZ> generateDeterministicStressFrame(size_t total_points) {
  std::vector<PointXYZ> points;
  points.reserve(total_points);
  DeterministicRng rng;

  // Insert 500 deterministic invalid points
  // 1. Non-finite
  for (int i = 0; i < 200; ++i) {
    if (i % 2 == 0) {
      points.push_back(PointXYZ{std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0});
    } else {
      points.push_back(PointXYZ{0.0, std::numeric_limits<double>::infinity(), 0.0});
    }
  }
  // 2. Out-of-range XY
  for (int i = 0; i < 200; ++i) {
    points.push_back(PointXYZ{1e15, 1e15, 0.0});
  }

  // Generate valid points spanning multiple positive and negative tiles
  const size_t valid_points = total_points - points.size();
  for (size_t i = 0; i < valid_points; ++i) {
    // Coordinate range [-15.0, +15.0] meters spans multiple tiles in all 4 quadrants
    const double x = rng.nextDouble(-15.0, 15.0);
    const double y = rng.nextDouble(-15.0, 15.0);
    const double z = rng.nextDouble(-1.5, 3.5);
    points.push_back(PointXYZ{x, y, z});
  }

  return points;
}

}  // namespace

TEST(FrameEvidenceStressTest, Synthetic250kObservationsStressAndConsistency) {
  constexpr size_t POINT_COUNT = 250000;
  const auto points = generateDeterministicStressFrame(POINT_COUNT);
  ASSERT_EQ(points.size(), POINT_COUNT);

  FrameEvidence fe;
  const WorldPoint2D sensor_origin{0.0, 0.0};

  const auto start_build = std::chrono::steady_clock::now();
  const auto stats_opt = fe.buildFrame(points, 5001, sensor_origin);
  const auto end_build = std::chrono::steady_clock::now();
  const auto build_ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(end_build - start_build).count();

  ASSERT_TRUE(stats_opt.has_value());
  const auto& stats = *stats_opt;

  std::cout << "\n==================================================\n";
  std::cout << "M5 250,000 OBSERVATIONS STRESS REPORT:\n";
  std::cout << "Build time:            " << build_ms << " ms\n";
  std::cout << "Input points:          " << stats.input_points << "\n";
  std::cout << "Accepted points:       " << stats.accepted_points << "\n";
  std::cout << "Rejected non-finite:   " << stats.rejected_non_finite << "\n";
  std::cout << "Rejected out-of-range: " << stats.rejected_out_of_range << "\n";
  std::cout << "Rejected numeric:      " << stats.rejected_numeric << "\n";
  std::cout << "Active base cells:     " << stats.active_base_cells << "\n";
  std::cout << "Allocated tiles:       " << stats.tile_count << "\n";
  std::cout << "==================================================\n";

  // Section 56 consistency checks
  EXPECT_EQ(stats.input_points, POINT_COUNT);
  EXPECT_EQ(stats.rejected_non_finite, 200);
  EXPECT_EQ(stats.rejected_out_of_range, 200);
  EXPECT_EQ(stats.input_points,
            stats.accepted_points + stats.rejected_non_finite +
                stats.rejected_out_of_range + stats.rejected_numeric);
  EXPECT_TRUE(stats.isConsistent());
  EXPECT_GT(stats.active_base_cells, 1000);
  EXPECT_GT(stats.tile_count, 10);

  // Snapshot consistency
  const auto snapshot = fe.baseCellSnapshot();
  EXPECT_EQ(snapshot.size(), stats.active_base_cells);

  uint64_t total_snapshot_obs = 0;
  for (size_t i = 0; i < snapshot.size(); ++i) {
    if (i > 0) {
      EXPECT_LT(snapshot[i - 1].key, snapshot[i].key);  // strictly sorted
    }
    total_snapshot_obs += snapshot[i].cell.count();
  }
  EXPECT_EQ(total_snapshot_obs, stats.accepted_points);

  // Query 2,000 deterministic regions across L0-L3
  DeterministicRng query_rng;
  query_rng.state = 987654321ULL;

  for (int i = 0; i < 2000; ++i) {
    const double qx = query_rng.nextDouble(-12.0, 12.0);
    const double qy = query_rng.nextDouble(-12.0, 12.0);
    const uint8_t level = static_cast<uint8_t>(query_rng.next() % 4);

    const auto key = CellKey::fromWorldPoint(WorldPoint2D{qx, qy}, level);
    ASSERT_TRUE(key.has_value());

    const auto summary = fe.summarize(*key);
    EXPECT_GE(summary.density_score, 0.0);
    EXPECT_LE(summary.density_score, 1.0);

    if (summary.geometry_complexity.has_value()) {
      EXPECT_GE(*summary.geometry_complexity, 0.0);
      EXPECT_LE(*summary.geometry_complexity, 1.0);
      EXPECT_GE(summary.observation_count, 2);
      EXPECT_GE(summary.occupied_base_cells, 2);
    }

    const auto signals = makeResolutionSignals(fe, *key);
    ASSERT_TRUE(signals.has_value());
    EXPECT_TRUE(signals->isValid());
  }
}

TEST(FrameEvidenceStressTest, RepeatabilityAcrossFreshInstances) {
  constexpr size_t POINT_COUNT = 50000;
  const auto points = generateDeterministicStressFrame(POINT_COUNT);

  FrameEvidence fe1;
  FrameEvidence fe2;
  const WorldPoint2D origin{-2.5, 3.5};

  const auto stats1 = fe1.buildFrame(points, 42, origin);
  const auto stats2 = fe2.buildFrame(points, 42, origin);

  ASSERT_TRUE(stats1.has_value());
  ASSERT_TRUE(stats2.has_value());
  EXPECT_EQ(*stats1, *stats2);

  const auto snap1 = fe1.baseCellSnapshot();
  const auto snap2 = fe2.baseCellSnapshot();
  ASSERT_EQ(snap1.size(), snap2.size());
  for (size_t i = 0; i < snap1.size(); ++i) {
    EXPECT_EQ(snap1[i].key, snap2[i].key);
    EXPECT_EQ(snap1[i].cell, snap2[i].cell);
  }

  // Compare summaries across regions
  DeterministicRng rng;
  rng.state = 1122334455ULL;
  for (int i = 0; i < 200; ++i) {
    const double qx = rng.nextDouble(-10.0, 10.0);
    const double qy = rng.nextDouble(-10.0, 10.0);
    const uint8_t level = static_cast<uint8_t>(rng.next() % 4);
    const auto key = CellKey::fromWorldPoint(WorldPoint2D{qx, qy}, level);
    ASSERT_TRUE(key.has_value());

    const auto sum1 = fe1.summarize(*key);
    const auto sum2 = fe2.summarize(*key);
    EXPECT_EQ(sum1, sum2);

    const auto sig1 = makeResolutionSignals(fe1, *key);
    const auto sig2 = makeResolutionSignals(fe2, *key);
    EXPECT_EQ(sig1, sig2);
  }
}

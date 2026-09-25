#include <gtest/gtest.h>

#include <algorithm>
#include <limits>
#include <set>
#include <vector>

#include "avr_mapping/point_xyz.hpp"
#include "avr_mapping/recent_observation_cache.hpp"
#include "avr_spatial/cell_key.hpp"
#include "avr_spatial/constants.hpp"

namespace avr::mapping {
namespace {

TEST(RecentObservationSpatialIndexTest, OneIndexPerCandidateInvariant) {
  auto cache_opt = RecentObservationCache::create();
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;

  // Input contains valid points in different roots, some invalid points
  const std::vector<PointXYZ> pts = {
      {0.05, 0.05, 1.0},
      {0.15, 0.25, 2.0},
      {5.05, 5.05, 3.0},
      {-5.05, -5.05, 4.0},
      {std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0},
  };

  const auto stats_opt = cache.pushFrame(pts, 1, avr::spatial::WorldPoint2D{0.0, 0.0});
  ASSERT_TRUE(stats_opt.has_value());

  const auto* frame = cache.findFrame(1);
  ASSERT_NE(frame, nullptr);

  // Invariant 1: In one frame, sum of bucket index sizes == replay_candidates
  const auto bucket_keys = frame->bucketKeySnapshot();
  uint64_t total_bucket_indices = 0;
  for (const auto& k : bucket_keys) {
    total_bucket_indices += frame->findBucketIndices(k).size();
  }
  EXPECT_EQ(total_bucket_indices, stats_opt->replay_candidates);

  // Invariant 2: Across cache, stored_point_indices == stored_replay_candidates
  const auto cache_stats = cache.statistics();
  EXPECT_EQ(cache_stats.stored_point_indices, cache_stats.stored_replay_candidates);
  EXPECT_EQ(cache_stats.stored_point_indices, total_bucket_indices);
}

TEST(RecentObservationSpatialIndexTest, L0ToL3QueryAndParentChildPartition) {
  auto cache_opt = RecentObservationCache::create();
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;

  // L3 root covers 8x8 L0 cells (0.4m x 0.4m).
  // Populate points across all quadrants of L3 root at GridCoord (0, 0).
  // L0 cell size = 0.05m.
  // Quad SW: x in [0, 3], y in [0, 3] -> world [0.0, 0.2)
  // Quad SE: x in [4, 7], y in [0, 3] -> world [0.2, 0.4)
  // Quad NW: x in [0, 3], y in [4, 7]
  // Quad NE: x in [4, 7], y in [4, 7]
  std::vector<PointXYZ> pts;
  for (int gx = 0; gx < 8; ++gx) {
    for (int gy = 0; gy < 8; ++gy) {
      pts.push_back(PointXYZ{gx * 0.05 + 0.02, gy * 0.05 + 0.02, static_cast<double>(gx + gy)});
    }
  }
  ASSERT_EQ(pts.size(), 64ULL);

  ASSERT_TRUE(cache.pushFrame(pts, 10, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());

  const auto root_key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);

  // Query L3: must return all 64 points in that bucket
  std::vector<uint32_t> l3_indices;
  EXPECT_TRUE(cache.collectObservationIndices(10, root_key, l3_indices));
  EXPECT_EQ(l3_indices.size(), 64ULL);

  // Verify indices are strictly ascending
  for (size_t i = 1; i < l3_indices.size(); ++i) {
    EXPECT_GT(l3_indices[i], l3_indices[i - 1]);
  }

  // Children of L3 are 4 L2 cells
  const auto l2_children_opt = avr::spatial::childrenOf(root_key);
  ASSERT_TRUE(l2_children_opt.has_value());

  std::set<uint32_t> l2_union;
  size_t l2_total_count = 0;

  for (const auto& l2_child : *l2_children_opt) {
    std::vector<uint32_t> l2_indices;
    EXPECT_TRUE(cache.collectObservationIndices(10, l2_child, l2_indices));
    EXPECT_EQ(l2_indices.size(), 16ULL);  // 4x4 = 16 points per L2 cell

    for (const auto idx : l2_indices) {
      EXPECT_EQ(l2_union.count(idx), 0ULL);  // No duplicate across children
      l2_union.insert(idx);
    }
    l2_total_count += l2_indices.size();

    // Verify L2 -> 4 L1 children partition
    const auto l1_children_opt = avr::spatial::childrenOf(l2_child);
    ASSERT_TRUE(l1_children_opt.has_value());

    std::set<uint32_t> l1_union;
    for (const auto& l1_child : *l1_children_opt) {
      std::vector<uint32_t> l1_indices;
      EXPECT_TRUE(cache.collectObservationIndices(10, l1_child, l1_indices));
      EXPECT_EQ(l1_indices.size(), 4ULL);  // 2x2 = 4 points per L1 cell

      for (const auto idx : l1_indices) {
        EXPECT_EQ(l1_union.count(idx), 0ULL);
        l1_union.insert(idx);
      }

      // Verify L1 -> 4 L0 children partition
      const auto l0_children_opt = avr::spatial::childrenOf(l1_child);
      ASSERT_TRUE(l0_children_opt.has_value());

      std::set<uint32_t> l0_union;
      for (const auto& l0_child : *l0_children_opt) {
        std::vector<uint32_t> l0_indices;
        EXPECT_TRUE(cache.collectObservationIndices(10, l0_child, l0_indices));
        EXPECT_EQ(l0_indices.size(), 1ULL);  // 1x1 = 1 point per L0 cell
        l0_union.insert(l0_indices[0]);
      }
      const std::set<uint32_t> l1_child_set(l1_indices.begin(), l1_indices.end());
      EXPECT_EQ(l0_union, l1_child_set);
    }
    const std::set<uint32_t> l2_child_set(l2_indices.begin(), l2_indices.end());
    EXPECT_EQ(l1_union, l2_child_set);
    EXPECT_EQ(l1_union.size(), 16ULL);
  }

  // Parent/Child exact conservation: union of 4 L2 children equals L3 query
  EXPECT_EQ(l2_total_count, 64ULL);
  EXPECT_EQ(l2_union.size(), 64ULL);
  std::set<uint32_t> l3_set(l3_indices.begin(), l3_indices.end());
  EXPECT_EQ(l2_union, l3_set);
}

TEST(RecentObservationSpatialIndexTest, InterleavedArrivalParentChildPartitionDistinction) {
  auto cache_opt = RecentObservationCache::create();
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;

  const auto root_key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);
  const auto l2_children_opt = avr::spatial::childrenOf(root_key);
  ASSERT_TRUE(l2_children_opt.has_value());
  const auto& children = *l2_children_opt;
  ASSERT_EQ(children.size(), 4ULL);

  // Interleave points across child 0 and child 1 in temporal arrival order:
  // Child 0 (SW) center: (0.1, 0.1)
  // Child 1 (SE) center: (0.3, 0.1)
  // Input: [pt_in_child0, pt_in_child1, pt_in_child0, pt_in_child1]
  const std::vector<PointXYZ> interleaved_pts = {
      {0.1, 0.1, 1.0},  // index 0: child 0
      {0.3, 0.1, 2.0},  // index 1: child 1
      {0.1, 0.1, 3.0},  // index 2: child 0
      {0.3, 0.1, 4.0},  // index 3: child 1
  };

  ASSERT_TRUE(cache.pushFrame(interleaved_pts, 1, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());

  // Parent query returns all 4 indices in arrival order [0, 1, 2, 3]
  std::vector<uint32_t> parent_indices;
  EXPECT_TRUE(cache.collectObservationIndices(1, root_key, parent_indices));
  EXPECT_EQ(parent_indices, (std::vector<uint32_t>{0, 1, 2, 3}));

  // Child 0 query returns [0, 2] in arrival order
  std::vector<uint32_t> c0_indices;
  EXPECT_TRUE(cache.collectObservationIndices(1, children[0], c0_indices));
  EXPECT_EQ(c0_indices, (std::vector<uint32_t>{0, 2}));

  // Child 1 query returns [1, 3] in arrival order
  std::vector<uint32_t> c1_indices;
  EXPECT_TRUE(cache.collectObservationIndices(1, children[1], c1_indices));
  EXPECT_EQ(c1_indices, (std::vector<uint32_t>{1, 3}));

  // Canonical spatial concatenation [0, 2, 1, 3] != parent arrival order [0, 1, 2, 3]
  std::vector<uint32_t> concatenated;
  concatenated.insert(concatenated.end(), c0_indices.begin(), c0_indices.end());
  concatenated.insert(concatenated.end(), c1_indices.begin(), c1_indices.end());
  EXPECT_NE(concatenated, parent_indices);

  // But SET UNION / sorted concatenation EXACTLY equals parent query
  std::sort(concatenated.begin(), concatenated.end());
  EXPECT_EQ(concatenated, parent_indices);
}

TEST(RecentObservationSpatialIndexTest, NegativeCoordinateBoundaries) {
  auto cache_opt = RecentObservationCache::create();
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;

  // Coordinates around base grid boundaries: -17, -16, -15, -1, 0, 15, 16, 17
  const std::vector<int32_t> test_grids = {-17, -16, -15, -1, 0, 15, 16, 17};
  std::vector<PointXYZ> pts;

  for (const auto gx : test_grids) {
    for (const auto gy : test_grids) {
      // Cell center
      const double x = (gx + 0.5) * 0.05;
      const double y = (gy + 0.5) * 0.05;
      pts.push_back(PointXYZ{x, y, 0.0});
    }
  }

  ASSERT_TRUE(cache.pushFrame(pts, 1, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());

  // Query each grid point individually at L0
  for (size_t i = 0; i < pts.size(); ++i) {
    const auto& pt = pts[i];
    const auto gx = *avr::spatial::quantizeToBaseGridSafe(pt.x);
    const auto gy = *avr::spatial::quantizeToBaseGridSafe(pt.y);
    const auto l0_key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{gx, gy}, avr::spatial::L0);

    std::vector<uint32_t> indices;
    EXPECT_TRUE(cache.collectObservationIndices(1, l0_key, indices));
    ASSERT_EQ(indices.size(), 1ULL);
    EXPECT_EQ(indices[0], static_cast<uint32_t>(i));

    // Also verify indexing into raw points without copying
    const auto* frame = cache.findFrame(1);
    ASSERT_NE(frame, nullptr);
    EXPECT_DOUBLE_EQ(frame->rawPoints()[indices[0]].x, pt.x);
    EXPECT_DOUBLE_EQ(frame->rawPoints()[indices[0]].y, pt.y);
  }
}

TEST(RecentObservationSpatialIndexTest, MultiRootQueryIsolation) {
  auto cache_opt = RecentObservationCache::create();
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;

  // Points in 3 completely distinct roots:
  // Root 1: Grid (0, 0) -> world (0.05, 0.05)
  // Root 2: Grid (100, 100) -> world (5.05, 5.05)
  // Root 3: Grid (-100, -100) -> world (-5.05, -5.05)
  const std::vector<PointXYZ> pts = {
      {0.05, 0.05, 1.0},
      {5.05, 5.05, 2.0},
      {-5.05, -5.05, 3.0},
  };

  ASSERT_TRUE(cache.pushFrame(pts, 1, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());

  const auto root1 = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);
  const auto root2 = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{100, 100}, avr::spatial::L3);
  const auto root3 = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{-100, -100}, avr::spatial::L3);

  std::vector<uint32_t> idx1;
  std::vector<uint32_t> idx2;
  std::vector<uint32_t> idx3;

  EXPECT_TRUE(cache.collectObservationIndices(1, root1, idx1));
  EXPECT_TRUE(cache.collectObservationIndices(1, root2, idx2));
  EXPECT_TRUE(cache.collectObservationIndices(1, root3, idx3));

  // No cross-root leakage
  ASSERT_EQ(idx1.size(), 1ULL);
  EXPECT_EQ(idx1[0], 0ULL);

  ASSERT_EQ(idx2.size(), 1ULL);
  EXPECT_EQ(idx2[0], 1ULL);

  ASSERT_EQ(idx3.size(), 1ULL);
  EXPECT_EQ(idx3[0], 2ULL);

  // Unobserved root returns empty
  const auto unobserved_root = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{200, 200}, avr::spatial::L3);
  std::vector<uint32_t> unobserved_idx;
  EXPECT_TRUE(cache.collectObservationIndices(1, unobserved_root, unobserved_idx));
  EXPECT_TRUE(unobserved_idx.empty());
}

TEST(RecentObservationSpatialIndexTest, BucketKeySnapshotDeterminism) {
  auto cache_opt = RecentObservationCache::create();
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;

  // Insert points in non-sorted spatial order
  const std::vector<PointXYZ> pts = {
      {5.05, 5.05, 2.0},
      {-5.05, -5.05, 3.0},
      {0.05, 0.05, 1.0},
  };

  ASSERT_TRUE(cache.pushFrame(pts, 1, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());

  const auto keys = cache.bucketKeySnapshot(1);
  ASSERT_EQ(keys.size(), 3ULL);

  // Snapshot must be strictly sorted by canonical CellKey
  EXPECT_TRUE(std::is_sorted(keys.begin(), keys.end()));
}

}  // namespace
}  // namespace avr::mapping

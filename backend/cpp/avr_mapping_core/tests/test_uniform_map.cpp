#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "avr_mapping/uniform_map.hpp"
#include "avr_spatial/cell_key.hpp"
#include "avr_spatial/coordinate.hpp"

namespace {

using avr::mapping::PointXYZ;
using avr::mapping::SnapshotRecord;
using avr::mapping::UniformMap;
using avr::spatial::CellCoord;
using avr::spatial::CellKey;

// Section 33: Multiple points falling into the same cell
TEST(UniformMapTest, MultiplePointsSameCell) {
  UniformMap map(100.0);

  // All 5 points fall into cell (0, 0)
  const std::vector<PointXYZ> points = {
      PointXYZ{.x = 0.01, .y = 0.01, .z = 1.0}, PointXYZ{.x = 0.02, .y = 0.02, .z = 2.0},
      PointXYZ{.x = 0.03, .y = 0.03, .z = 3.0}, PointXYZ{.x = 0.04, .y = 0.04, .z = 4.0},
      PointXYZ{.x = 0.045, .y = 0.045, .z = 5.0},
  };

  const auto stats = map.integrateFrame(points, 10);
  EXPECT_EQ(stats.input_points, 5u);
  EXPECT_EQ(stats.accepted_points, 5u);
  EXPECT_EQ(stats.cells_created, 1u);
  EXPECT_EQ(stats.cells_updated, 4u);
  EXPECT_EQ(stats.unique_cells_touched, 1u);
  EXPECT_EQ(stats.tiles_created, 1u);

  EXPECT_EQ(map.tileCount(), 1u);
  EXPECT_EQ(map.activeCellCount(), 1u);
  EXPECT_EQ(map.totalAcceptedObservations(), 5u);

  const auto* cell = map.findCell(0.02, 0.02);
  ASSERT_NE(cell, nullptr);
  EXPECT_EQ(cell->observationCount(), 5u);
  EXPECT_DOUBLE_EQ(cell->meanZ(), 3.0);
  EXPECT_DOUBLE_EQ(cell->minZ(), 1.0);
  EXPECT_DOUBLE_EQ(cell->maxZ(), 5.0);
  EXPECT_EQ(cell->lastSeenFrame(), 10u);
}

// Section 36: Invalid points handling
TEST(UniformMapTest, InvalidPointsRejectionAndMapImmutability) {
  UniformMap map(100.0);

  const double nan_val = std::numeric_limits<double>::quiet_NaN();
  const double inf_val = std::numeric_limits<double>::infinity();

  const std::vector<PointXYZ> invalid_points = {
      PointXYZ{.x = nan_val, .y = 0.0, .z = 0.0},
      PointXYZ{.x = 0.0, .y = nan_val, .z = 0.0},
      PointXYZ{.x = 0.0, .y = 0.0, .z = nan_val},
      PointXYZ{.x = inf_val, .y = 0.0, .z = 0.0},
      PointXYZ{.x = -inf_val, .y = 0.0, .z = 0.0},
      PointXYZ{.x = 0.0, .y = inf_val, .z = 0.0},
      PointXYZ{.x = 0.0, .y = 0.0, .z = inf_val},
      PointXYZ{.x = 0.0, .y = 0.0, .z = -inf_val},
  };

  const auto stats = map.integrateFrame(invalid_points, 1);
  EXPECT_EQ(stats.input_points, invalid_points.size());
  EXPECT_EQ(stats.accepted_points, 0u);
  EXPECT_EQ(stats.rejected_non_finite, invalid_points.size());
  EXPECT_EQ(stats.tiles_created, 0u);
  EXPECT_EQ(stats.cells_created, 0u);

  EXPECT_EQ(map.tileCount(), 0u);
  EXPECT_EQ(map.activeCellCount(), 0u);
  EXPECT_EQ(map.totalAcceptedObservations(), 0u);
}

// Section 36 continued: Out of range coordinates
TEST(UniformMapTest, OutOfRangeCoordinateRejection) {
  // Set map radius to huge value to test coordinate range rejection past extent check
  UniformMap map(1e12);

  const std::vector<PointXYZ> out_of_range = {
      PointXYZ{.x = 1e11, .y = 0.0, .z = 1.0},
      PointXYZ{.x = -1e11, .y = 0.0, .z = 1.0},
  };

  const auto stats = map.integrateFrame(out_of_range, 1);
  EXPECT_EQ(stats.input_points, 2u);
  EXPECT_EQ(stats.accepted_points, 0u);
  EXPECT_EQ(stats.rejected_out_of_range, 2u);
  EXPECT_EQ(map.activeCellCount(), 0u);
}

// Section 37: Extent filter test with boundary and nextafter
TEST(UniformMapTest, ExtentFilterInsideBoundaryAndOutside) {
  UniformMap map(100.0);  // radius 100 m

  const double exactly_100 = 100.0;
  const double just_inside = std::nextafter(100.0, 0.0);
  const double just_outside = std::nextafter(100.0, 200.0);

  const std::vector<PointXYZ> points = {
      PointXYZ{.x = 50.0, .y = 0.0, .z = 1.0},          // Inside
      PointXYZ{.x = just_inside, .y = 0.0, .z = 2.0},   // Immediately inside
      PointXYZ{.x = exactly_100, .y = 0.0, .z = 3.0},   // Exactly on boundary (accepted)
      PointXYZ{.x = just_outside, .y = 0.0, .z = 4.0},  // Immediately outside (rejected)
      PointXYZ{.x = 101.0, .y = 0.0, .z = 5.0},         // Outside
  };

  const auto stats = map.integrateFrame(points, 1);
  EXPECT_EQ(stats.input_points, 5u);
  EXPECT_EQ(stats.accepted_points, 3u);
  EXPECT_EQ(stats.rejected_outside_extent, 2u);
  EXPECT_EQ(map.activeCellCount(), 3u);
}

// Milestone 2.1: Rejection classification precedence order
TEST(UniformMapTest, RejectionPrecedenceOrder) {
  UniformMap map(100.0);

  // 1. Non-finite should be rejected as rejected_non_finite even if out of range
  const PointXYZ pt_non_finite{.x = std::numeric_limits<double>::quiet_NaN(), .y = 1e300, .z = 0.0};
  const auto s1 = map.integrateFrame(std::span<const PointXYZ>(&pt_non_finite, 1), 1);
  EXPECT_EQ(s1.rejected_non_finite, 1u);
  EXPECT_EQ(s1.rejected_out_of_range, 0u);

  // 2. Finite but out-of-range should be rejected as rejected_out_of_range before extent check
  const PointXYZ pt_out_of_range{.x = 1e300, .y = 0.0, .z = 1.0};
  const auto s2 = map.integrateFrame(std::span<const PointXYZ>(&pt_out_of_range, 1), 2);
  EXPECT_EQ(s2.rejected_out_of_range, 1u);
  EXPECT_EQ(s2.rejected_outside_extent, 0u);

  // 3. Finite, representable, but outside extent radius
  const PointXYZ pt_outside_extent{.x = 105.0, .y = 0.0, .z = 1.0};
  const auto s3 = map.integrateFrame(std::span<const PointXYZ>(&pt_outside_extent, 1), 3);
  EXPECT_EQ(s3.rejected_outside_extent, 1u);
  EXPECT_EQ(s3.rejected_numeric, 0u);

  // 4. Valid location, but numeric arithmetic overflow on existing cell
  // First insert initial point
  const PointXYZ pt_init{.x = 0.025, .y = 0.025, .z = -1e308};
  const auto s_init = map.integrateFrame(std::span<const PointXYZ>(&pt_init, 1), 4);
  EXPECT_EQ(s_init.accepted_points, 1u);

  // Second point at same location with +1e308 -> delta = 2e308 (Inf) -> numeric failure
  const PointXYZ pt_overflow{.x = 0.025, .y = 0.025, .z = 1e308};
  const auto s4 = map.integrateFrame(std::span<const PointXYZ>(&pt_overflow, 1), 5);
  EXPECT_EQ(s4.rejected_numeric, 1u);
  EXPECT_EQ(s4.accepted_points, 0u);
}

// Milestone 2.1: Numeric rejection creates NO empty tile and does NOT mutate existing cell
TEST(UniformMapTest, NumericRejectionTransactionalIntegrity) {
  UniformMap map(100.0);

  // 1. Initial valid point
  const PointXYZ pt1{.x = 1.025, .y = 1.025, .z = -1e308};
  map.integrateFrame(std::span<const PointXYZ>(&pt1, 1), 10);
  EXPECT_EQ(map.tileCount(), 1u);
  EXPECT_EQ(map.activeCellCount(), 1u);
  EXPECT_EQ(map.totalAcceptedObservations(), 1u);

  const auto snap_before = map.snapshot();
  ASSERT_EQ(snap_before.size(), 1u);
  EXPECT_DOUBLE_EQ(snap_before[0].cell.meanZ(), -1e308);
  EXPECT_EQ(snap_before[0].cell.lastSeenFrame(), 10u);

  // 2. Point that fails numeric elevation update at the same cell
  const PointXYZ pt_overflow{.x = 1.025, .y = 1.025, .z = 1e308};
  const auto stats = map.integrateFrame(std::span<const PointXYZ>(&pt_overflow, 1), 20);
  EXPECT_EQ(stats.rejected_numeric, 1u);
  EXPECT_EQ(stats.accepted_points, 0u);

  // Map state must be completely unchanged
  EXPECT_EQ(map.tileCount(), 1u);
  EXPECT_EQ(map.activeCellCount(), 1u);
  EXPECT_EQ(map.totalAcceptedObservations(), 1u);

  const auto snap_after = map.snapshot();
  ASSERT_EQ(snap_after.size(), 1u);
  EXPECT_DOUBLE_EQ(snap_after[0].cell.meanZ(), -1e308);
  EXPECT_EQ(snap_after[0].cell.lastSeenFrame(), 10u);  // Not 20!
}

// Milestone 2.1: FrameUpdateStats counter partition invariants
TEST(UniformMapTest, FrameUpdateStatsPartitionInvariants) {
  UniformMap map(100.0);

  // First populate a cell with extreme negative value to trigger numeric rejection
  const PointXYZ init_pt{.x = 0.025, .y = 0.025, .z = -1e308};
  map.integrateFrame(std::span<const PointXYZ>(&init_pt, 1), 1);

  // Heterogeneous batch containing points of all categories
  const std::vector<PointXYZ> mixed_points = {
      PointXYZ{.x = std::numeric_limits<double>::quiet_NaN(), .y = 0.0, .z = 0.0},  // non-finite
      PointXYZ{.x = 1e300, .y = 0.0, .z = 0.0},                                      // out-of-range
      PointXYZ{.x = 150.0, .y = 0.0, .z = 0.0},                                      // outside extent
      PointXYZ{.x = 0.025, .y = 0.025, .z = 1e308},                                  // numeric rejection
      PointXYZ{.x = 0.025, .y = 0.025, .z = -1e308},                                 // accepted update
      PointXYZ{.x = 2.025, .y = 2.025, .z = 5.0},                                    // accepted new cell
  };

  const auto s = map.integrateFrame(mixed_points, 2);

  // Partition invariant
  EXPECT_EQ(s.input_points,
            s.accepted_points + s.rejected_non_finite + s.rejected_out_of_range +
                s.rejected_outside_extent + s.rejected_numeric);
  EXPECT_EQ(s.accepted_points, s.cells_created + s.cells_updated);
  EXPECT_LE(s.unique_cells_touched, s.accepted_points);
  EXPECT_LE(s.tiles_created, s.cells_created);
}

// Milestone 2.1: Map radius contract validation
TEST(UniformMapTest, MapRadiusContract) {
  UniformMap map(100.0);
  EXPECT_DOUBLE_EQ(map.radius(), 100.0);

  // Valid changes: radius >= 0.0 and finite
  EXPECT_TRUE(map.setRadius(50.0));
  EXPECT_DOUBLE_EQ(map.radius(), 50.0);

  EXPECT_TRUE(map.setRadius(0.0));
  EXPECT_DOUBLE_EQ(map.radius(), 0.0);

  // With radius 0.0, only point exactly at window center is accepted
  const PointXYZ pt_center{.x = 0.0, .y = 0.0, .z = 1.0};
  const PointXYZ pt_near{.x = 0.01, .y = 0.0, .z = 1.0};
  const auto s_zero_center = map.integrateFrame(std::span<const PointXYZ>(&pt_center, 1), 1);
  EXPECT_EQ(s_zero_center.accepted_points, 1u);
  const auto s_zero_near = map.integrateFrame(std::span<const PointXYZ>(&pt_near, 1), 2);
  EXPECT_EQ(s_zero_near.rejected_outside_extent, 1u);

  // Invalid radius changes: negative or non-finite
  EXPECT_FALSE(map.setRadius(-1.0));
  EXPECT_DOUBLE_EQ(map.radius(), 0.0);  // Preserved

  EXPECT_FALSE(map.setRadius(std::numeric_limits<double>::quiet_NaN()));
  EXPECT_DOUBLE_EQ(map.radius(), 0.0);

  EXPECT_FALSE(map.setRadius(std::numeric_limits<double>::infinity()));
  EXPECT_DOUBLE_EQ(map.radius(), 0.0);

  EXPECT_FALSE(map.setRadius(-std::numeric_limits<double>::infinity()));
  EXPECT_DOUBLE_EQ(map.radius(), 0.0);
}

// Milestone 2.1: Frame sequence contract (non-consecutive and repeated sequence identifiers)
TEST(UniformMapTest, FrameSequenceContractNonConsecutiveAndReused) {
  UniformMap map(100.0);

  const PointXYZ pt1{.x = 1.025, .y = 1.025, .z = 1.0};
  const PointXYZ pt2{.x = 2.025, .y = 2.025, .z = 2.0};

  // Frame 10 -> Frame 42 (non-consecutive)
  const auto s1 = map.integrateFrame(std::span<const PointXYZ>(&pt1, 1), 10);
  EXPECT_EQ(s1.unique_cells_touched, 1u);

  const auto s2 = map.integrateFrame(std::span<const PointXYZ>(&pt1, 1), 42);
  EXPECT_EQ(s2.unique_cells_touched, 1u);
  EXPECT_EQ(map.findCell(pt1.x, pt1.y)->lastSeenFrame(), 42u);

  // Frame 42 again (reusing exact frame_seq for another frame call):
  // Must NOT corrupt unique_cells_touched accounting!
  const auto s3 = map.integrateFrame(std::span<const PointXYZ>(&pt2, 1), 42);
  EXPECT_EQ(s3.unique_cells_touched, 1u);
  EXPECT_EQ(map.findCell(pt2.x, pt2.y)->lastSeenFrame(), 42u);
}

// Section 38 & 20: Window center invariance and API validation
TEST(UniformMapTest, WindowCenterInvariance) {
  UniformMap map(50.0);

  // Default center is (0, 0)
  const auto [cx0, cy0] = map.windowCenter();
  EXPECT_DOUBLE_EQ(cx0, 0.0);
  EXPECT_DOUBLE_EQ(cy0, 0.0);

  // Point at (10.025, 10.025)
  const PointXYZ pt{.x = 10.025, .y = 10.025, .z = 5.0};
  map.integrateFrame(std::span<const PointXYZ>(&pt, 1), 1);

  const auto* cell1 = map.findCell(pt.x, pt.y);
  ASSERT_NE(cell1, nullptr);

  // Obtain CellKey for pt
  const CellCoord expected_coord{.x = 200, .y = 200, .level = 0};
  const auto expected_key = CellKey::fromCellCoord(expected_coord);
  ASSERT_TRUE(expected_key.has_value());
  EXPECT_EQ(map.findCell(*expected_key), cell1);

  // Shift window center to (10.0, 10.0). Point pt is still within 50 m.
  EXPECT_TRUE(map.setWindowCenter(10.0, 10.0));
  const auto [cx1, cy1] = map.windowCenter();
  EXPECT_DOUBLE_EQ(cx1, 10.0);
  EXPECT_DOUBLE_EQ(cy1, 10.0);

  // Ingest pt again from frame 2
  map.integrateFrame(std::span<const PointXYZ>(&pt, 1), 2);

  // CellKey must remain identical! Window center MUST NOT change CellKey.
  const auto* cell2 = map.findCell(*expected_key);
  ASSERT_NE(cell2, nullptr);
  EXPECT_EQ(cell2->observationCount(), 2u);
  EXPECT_EQ(cell2->lastSeenFrame(), 2u);

  // Invalid window center update must fail and preserve existing center
  EXPECT_FALSE(map.setWindowCenter(std::numeric_limits<double>::quiet_NaN(), 0.0));
  EXPECT_FALSE(map.setWindowCenter(0.0, std::numeric_limits<double>::infinity()));
  const auto [cx_preserved, cy_preserved] = map.windowCenter();
  EXPECT_DOUBLE_EQ(cx_preserved, 10.0);
  EXPECT_DOUBLE_EQ(cy_preserved, 10.0);
}

// Section 8: Non-L0 key rejection in UniformMap API
TEST(UniformMapTest, NonL0KeyRejection) {
  UniformMap map(100.0);
  const PointXYZ pt{.x = 0.025, .y = 0.025, .z = 1.0};
  map.integrateFrame(std::span<const PointXYZ>(&pt, 1), 1);

  // Valid L0 key lookup succeeds
  const auto l0_key = CellKey::fromCellCoord(CellCoord{.x = 0, .y = 0, .level = 0});
  ASSERT_TRUE(l0_key.has_value());
  EXPECT_NE(map.findCell(*l0_key), nullptr);

  // L1..L3 keys must be rejected
  for (uint8_t lvl = 1; lvl <= 3; ++lvl) {
    const auto l_key = CellKey::fromCellCoord(CellCoord{.x = 0, .y = 0, .level = lvl});
    ASSERT_TRUE(l_key.has_value());
    EXPECT_EQ(map.findCell(*l_key), nullptr);
  }
}

// Section 40: Snapshot determinism with different input orderings
TEST(UniformMapTest, SnapshotDeterminismDifferentInputOrder) {
  UniformMap map1(100.0);
  UniformMap map2(100.0);

  const std::vector<PointXYZ> points_order_a = {
      PointXYZ{.x = 1.025, .y = 2.025, .z = 1.0},
      PointXYZ{.x = -3.025, .y = 4.025, .z = 2.0},
      PointXYZ{.x = 5.025, .y = -6.025, .z = 3.0},
      PointXYZ{.x = -7.025, .y = -8.025, .z = 4.0},
      PointXYZ{.x = 0.025, .y = 0.025, .z = 5.0},
  };

  // Reverse ordering
  std::vector<PointXYZ> points_order_b = points_order_a;
  std::reverse(points_order_b.begin(), points_order_b.end());

  map1.integrateFrame(points_order_a, 1);
  map2.integrateFrame(points_order_b, 1);

  const auto snap1 = map1.snapshot();
  const auto snap2 = map2.snapshot();

  ASSERT_EQ(snap1.size(), snap2.size());
  ASSERT_EQ(snap1.size(), 5u);

  for (size_t i = 0; i < snap1.size(); ++i) {
    EXPECT_EQ(snap1[i].key, snap2[i].key);
    EXPECT_EQ(snap1[i].cell.observationCount(), snap2[i].cell.observationCount());
    EXPECT_DOUBLE_EQ(snap1[i].cell.meanZ(), snap2[i].cell.meanZ());
  }

  // Verify strict ordering
  for (size_t i = 1; i < snap1.size(); ++i) {
    EXPECT_LT(snap1[i - 1].key, snap1[i].key);
  }
}

// Section 41 & Milestone 2.1: Reset test with Policy A capacity semantics
TEST(UniformMapTest, ResetRestoresCleanStateAndPermitsReuse) {
  UniformMap map(100.0);

  const std::vector<PointXYZ> points = {
      PointXYZ{.x = 0.025, .y = 0.025, .z = 1.0},
      PointXYZ{.x = 1.025, .y = 1.025, .z = 2.0},
      PointXYZ{.x = -2.025, .y = -2.025, .z = 3.0},
  };

  map.integrateFrame(points, 1);
  EXPECT_EQ(map.tileCount(), 3u);
  EXPECT_EQ(map.activeCellCount(), 3u);
  EXPECT_EQ(map.totalAcceptedObservations(), 3u);

  // Call reset
  map.reset();

  EXPECT_EQ(map.tileCount(), 0u);
  EXPECT_EQ(map.activeCellCount(), 0u);
  EXPECT_EQ(map.totalAcceptedObservations(), 0u);
  EXPECT_TRUE(map.snapshot().empty());
  EXPECT_EQ(map.findCell(0.025, 0.025), nullptr);

  // Re-insert observations after reset
  const PointXYZ pt{.x = 5.025, .y = 5.025, .z = 10.0};
  const auto stats = map.integrateFrame(std::span<const PointXYZ>(&pt, 1), 2);
  EXPECT_EQ(stats.accepted_points, 1u);
  EXPECT_EQ(stats.cells_created, 1u);
  EXPECT_EQ(map.tileCount(), 1u);
  EXPECT_EQ(map.activeCellCount(), 1u);

  const auto* cell = map.findCell(pt.x, pt.y);
  ASSERT_NE(cell, nullptr);
  EXPECT_EQ(cell->observationCount(), 1u);
  EXPECT_DOUBLE_EQ(cell->meanZ(), 10.0);
}

}  // namespace

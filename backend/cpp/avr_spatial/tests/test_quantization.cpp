#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <random>

#include "avr_spatial/bounds.hpp"
#include "avr_spatial/cell_key.hpp"
#include "avr_spatial/coordinate.hpp"

namespace avr::spatial {

TEST(QuantizationTest, HalfOpenBoundariesExactAndNear) {
  // x = 0.000000 -> cell 0
  EXPECT_EQ(quantizeToBaseGrid(0.0), 0);

  // x = 0.04999999999999999 (immediately below 0.05) -> cell 0
  const double below_0_05 = std::nextafter(0.05, 0.0);
  EXPECT_EQ(quantizeToBaseGrid(below_0_05), 0);

  // x = 0.050000 exactly -> cell 1
  EXPECT_EQ(quantizeToBaseGrid(0.05), 1);

  // x = 0.05000000000000001 (immediately above 0.05) -> cell 1
  const double above_0_05 = std::nextafter(0.05, 1.0);
  EXPECT_EQ(quantizeToBaseGrid(above_0_05), 1);

  // x = -0.000001 (immediately below 0.0) -> cell -1
  const double below_zero = std::nextafter(0.0, -1.0);
  EXPECT_EQ(quantizeToBaseGrid(below_zero), -1);
  EXPECT_EQ(quantizeToBaseGrid(-0.000001), -1);

  // x = -0.050000 exactly -> cell -1 (since interval is [-0.05, 0.0))
  EXPECT_EQ(quantizeToBaseGrid(-0.05), -1);

  // x immediately below -0.05 -> cell -2
  const double below_neg_0_05 = std::nextafter(-0.05, -1.0);
  EXPECT_EQ(quantizeToBaseGrid(below_neg_0_05), -2);
}

TEST(QuantizationTest, ExplicitBoundariesAndNextafter) {
  // Required explicit cases: 0.00, 0.05, 0.10, 0.15, 0.20, 0.25, 0.30, 0.35, 0.50, 1.00
  // and their negative equivalents.
  // Particular focus on 0.15, -0.15, 0.35, -0.35 which break under naive division by 0.05.
  const double explicit_boundaries[] = {
      0.00, 0.05, 0.10, 0.15, 0.20, 0.25, 0.30, 0.35, 0.50, 1.00,
      -0.05, -0.10, -0.15, -0.20, -0.25, -0.30, -0.35, -0.50, -1.00,
  };

  for (double boundary : explicit_boundaries) {
    const int32_t expected_cell = static_cast<int32_t>(std::round(boundary * BASE_CELLS_PER_METER));

    // Exact boundary must belong to expected_cell
    const auto q_exact = quantizeToBaseGrid(boundary);
    ASSERT_TRUE(q_exact.has_value());
    EXPECT_EQ(*q_exact, expected_cell) << "Exact boundary failure at " << boundary;

    // Immediately below boundary must belong to expected_cell - 1
    const double below = std::nextafter(boundary, -std::numeric_limits<double>::infinity());
    const auto q_below = quantizeToBaseGrid(below);
    ASSERT_TRUE(q_below.has_value());
    EXPECT_EQ(*q_below, expected_cell - 1) << "Below boundary failure at " << boundary;

    // Immediately above boundary must belong to expected_cell
    const double above = std::nextafter(boundary, std::numeric_limits<double>::infinity());
    const auto q_above = quantizeToBaseGrid(above);
    ASSERT_TRUE(q_above.has_value());
    EXPECT_EQ(*q_above, expected_cell) << "Above boundary failure at " << boundary;
  }
}

TEST(QuantizationTest, SequentialLargeBoundaryRoundTrip200k) {
  // Wide deterministic sequential test: 200,001 indices from -100,000 to +100,000
  // cell i -> calculate physical min boundary -> quantize -> returns exactly i
  constexpr int32_t kMinIndex = -100000;
  constexpr int32_t kMaxIndex = 100000;

  int mismatches = 0;
  for (int32_t i = kMinIndex; i <= kMaxIndex; ++i) {
    const double min_x = static_cast<double>(i) / BASE_CELLS_PER_METER;
    const auto q = quantizeToBaseGrid(min_x);
    if (!q.has_value() || *q != i) {
      ++mismatches;
    }
  }

  EXPECT_EQ(mismatches, 0) << "200,001 sequential round-trip had mismatches!";
}

TEST(QuantizationTest, BoundsQuantizationConsistencyAllLevels) {
  // Test bounds / quantization consistency for all 4 hierarchy levels
  std::mt19937_64 rng(0xCAFEBABEDEADULL);
  std::uniform_int_distribution<int32_t> dist(-50000, 50000);

  constexpr size_t kChecksPerLevel = 25000;

  for (uint8_t lvl = 0; lvl < LEVEL_COUNT; ++lvl) {
    for (size_t iter = 0; iter < kChecksPerLevel; ++iter) {
      const int32_t cx = dist(rng);
      const int32_t cy = dist(rng);

      const auto key_opt = CellKey::fromCellCoord(CellCoord{.x = cx, .y = cy, .level = lvl});
      ASSERT_TRUE(key_opt.has_value());
      const CellKey key = *key_opt;

      const Bounds2D b = key.bounds();

      // 1. Min corner must quantize back to original cell
      const auto min_cell = worldToCellCoord(WorldPoint2D{.x = b.min_x, .y = b.min_y}, lvl);
      ASSERT_TRUE(min_cell.has_value());
      EXPECT_EQ(min_cell->x, cx);
      EXPECT_EQ(min_cell->y, cy);
      EXPECT_EQ(min_cell->level, lvl);

      // 2. nextafter(max_x, min_x) belongs to current cell where representable in floating point
      const double cpm = cellsPerMeter(lvl);
      const double inside_x = std::nextafter(b.max_x, b.min_x);
      const double inside_y = std::nextafter(b.max_y, b.min_y);

      if (inside_x * cpm < static_cast<double>(static_cast<int64_t>(cx) + 1)) {
        const auto inside_cell_x = worldToCellCoord(WorldPoint2D{.x = inside_x, .y = b.min_y}, lvl);
        ASSERT_TRUE(inside_cell_x.has_value());
        EXPECT_EQ(inside_cell_x->x, cx);
      }

      if (inside_y * cpm < static_cast<double>(static_cast<int64_t>(cy) + 1)) {
        const auto inside_cell_y = worldToCellCoord(WorldPoint2D{.x = b.min_x, .y = inside_y}, lvl);
        ASSERT_TRUE(inside_cell_y.has_value());
        EXPECT_EQ(inside_cell_y->y, cy);
      }

      // 3. max_x must belong to adjacent cell
      const auto adjacent_x_cell = worldToCellCoord(WorldPoint2D{.x = b.max_x, .y = b.min_y}, lvl);
      ASSERT_TRUE(adjacent_x_cell.has_value());
      EXPECT_EQ(adjacent_x_cell->x, cx + 1);
      EXPECT_EQ(adjacent_x_cell->y, cy);
    }
  }
}

TEST(QuantizationTest, NonFiniteWorldInput) {
  const double nan_val = std::numeric_limits<double>::quiet_NaN();
  const double pos_inf = std::numeric_limits<double>::infinity();
  const double neg_inf = -std::numeric_limits<double>::infinity();

  // 1D quantization
  EXPECT_EQ(quantizeToBaseGrid(nan_val), std::nullopt);
  EXPECT_EQ(quantizeToBaseGrid(pos_inf), std::nullopt);
  EXPECT_EQ(quantizeToBaseGrid(neg_inf), std::nullopt);

  // 2D world points
  EXPECT_EQ(worldToGridCoord(WorldPoint2D{.x = nan_val, .y = 0.0}), std::nullopt);
  EXPECT_EQ(worldToGridCoord(WorldPoint2D{.x = 0.0, .y = nan_val}), std::nullopt);
  EXPECT_EQ(worldToGridCoord(WorldPoint2D{.x = pos_inf, .y = 0.0}), std::nullopt);
  EXPECT_EQ(worldToGridCoord(WorldPoint2D{.x = 0.0, .y = neg_inf}), std::nullopt);

  // World to CellCoord
  EXPECT_EQ(worldToCellCoord(WorldPoint2D{.x = nan_val, .y = 0.0}, L0), std::nullopt);
  EXPECT_EQ(worldToCellCoord(WorldPoint2D{.x = pos_inf, .y = 0.0}, L1), std::nullopt);
  EXPECT_EQ(worldToCellCoord(WorldPoint2D{.x = neg_inf, .y = 0.0}, L2), std::nullopt);

  // CellKey from world point
  EXPECT_EQ(CellKey::fromWorldPoint(WorldPoint2D{.x = nan_val, .y = 0.0}, L0), std::nullopt);
  EXPECT_EQ(CellKey::fromWorldPoint(WorldPoint2D{.x = pos_inf, .y = 0.0}, L0), std::nullopt);
  EXPECT_EQ(CellKey::fromWorldPoint(WorldPoint2D{.x = neg_inf, .y = 0.0}, L0), std::nullopt);
}

TEST(QuantizationTest, OutOfRangeWorldInput) {
  // Extreme metric coordinates that would overflow int32_t
  const double extreme_pos = 1.0e12;   // 1 trillion meters -> 20 trillion cells > INT32_MAX
  const double extreme_neg = -1.0e12;  // -1 trillion meters < INT32_MIN

  EXPECT_EQ(quantizeToBaseGrid(extreme_pos), std::nullopt);
  EXPECT_EQ(quantizeToBaseGrid(extreme_neg), std::nullopt);

  EXPECT_EQ(worldToGridCoord(WorldPoint2D{.x = extreme_pos, .y = 0.0}), std::nullopt);
  EXPECT_EQ(worldToGridCoord(WorldPoint2D{.x = 0.0, .y = extreme_neg}), std::nullopt);

  EXPECT_EQ(worldToCellCoord(WorldPoint2D{.x = extreme_pos, .y = 0.0}, L0), std::nullopt);
  EXPECT_EQ(CellKey::fromWorldPoint(WorldPoint2D{.x = extreme_pos, .y = 0.0}, L0), std::nullopt);

  // Even larger floats near double max
  EXPECT_EQ(quantizeToBaseGrid(1.0e300), std::nullopt);
  EXPECT_EQ(quantizeToBaseGrid(-1.0e300), std::nullopt);
}

TEST(QuantizationTest, MultipleDistanceIntervals) {
  // 1.00m boundaries (1.00 * 20.0 = 20 base cells)
  EXPECT_EQ(quantizeToBaseGrid(1.00), 20);
  EXPECT_EQ(quantizeToBaseGrid(std::nextafter(1.00, 0.0)), 19);
  EXPECT_EQ(quantizeToBaseGrid(std::nextafter(1.00, 2.0)), 20);

  // -1.00m boundaries (-1.00 * 20.0 = -20 base cells)
  // Interval [-1.00, -0.95) belongs to cell -20
  EXPECT_EQ(quantizeToBaseGrid(-1.00), -20);
  EXPECT_EQ(quantizeToBaseGrid(std::nextafter(-1.00, -2.0)), -21);
  EXPECT_EQ(quantizeToBaseGrid(std::nextafter(-1.00, 0.0)), -20);
}

TEST(QuantizationTest, LevelMapping) {
  // bx = 7 -> L0: 7, L1: 3, L2: 1, L3: 0
  EXPECT_EQ(baseToLevelCoord(7, L0), 7);
  EXPECT_EQ(baseToLevelCoord(7, L1), 3);
  EXPECT_EQ(baseToLevelCoord(7, L2), 1);
  EXPECT_EQ(baseToLevelCoord(7, L3), 0);

  // bx = -1 -> L0: -1, L1: -1, L2: -1, L3: -1
  EXPECT_EQ(baseToLevelCoord(-1, L0), -1);
  EXPECT_EQ(baseToLevelCoord(-1, L1), -1);
  EXPECT_EQ(baseToLevelCoord(-1, L2), -1);
  EXPECT_EQ(baseToLevelCoord(-1, L3), -1);

  // bx = -9 -> L0: -9, L1: -5, L2: -3, L3: -2
  EXPECT_EQ(baseToLevelCoord(-9, L0), -9);
  EXPECT_EQ(baseToLevelCoord(-9, L1), -5);
  EXPECT_EQ(baseToLevelCoord(-9, L2), -3);
  EXPECT_EQ(baseToLevelCoord(-9, L3), -2);
}

TEST(QuantizationTest, PhysicalRangeSanity) {
  // 100m rolling map radius requires at least [-100m, +100m]
  // In base cells at 0.05m: +/-100m = +/-2,000 cells.
  // int32_t capacity: [-2,147,483,648, +2,147,483,647]
  // Span: > 107,374 km, vastly exceeding 100m rolling map.
  const WorldPoint2D far_pos{.x = 100.0, .y = 100.0};
  const WorldPoint2D far_neg{.x = -100.0, .y = -100.0};

  const auto g_pos = worldToGridCoord(far_pos);
  const auto g_neg = worldToGridCoord(far_neg);

  ASSERT_TRUE(g_pos.has_value());
  ASSERT_TRUE(g_neg.has_value());

  EXPECT_EQ(g_pos->x, 2000);
  EXPECT_EQ(g_pos->y, 2000);
  EXPECT_EQ(g_neg->x, -2000);
  EXPECT_EQ(g_neg->y, -2000);
}

TEST(QuantizationTest, BoundsHalfOpenIntervals) {
  const CellCoord cell{.x = 0, .y = 0, .level = L0};
  const Bounds2D b = cellBounds(cell);

  EXPECT_DOUBLE_EQ(b.min_x, 0.0);
  EXPECT_DOUBLE_EQ(b.min_y, 0.0);
  EXPECT_DOUBLE_EQ(b.max_x, 0.05);
  EXPECT_DOUBLE_EQ(b.max_y, 0.05);

  EXPECT_TRUE(b.contains(0.0, 0.0));
  EXPECT_TRUE(b.contains(0.049999, 0.049999));
  EXPECT_FALSE(b.contains(0.05, 0.0));
  EXPECT_FALSE(b.contains(0.0, 0.05));
  EXPECT_FALSE(b.contains(-0.0001, 0.0));
}

TEST(QuantizationTest, NonZeroOriginBoundaries) {
  const double origins[] = {1.0, -1.0, 123.45, -987.65};
  const int32_t indices[] = {-100, -17, -1, 0, 1, 3, 7, 16, 100};

  for (double origin : origins) {
    const double o_cell_target = std::round(origin * BASE_CELLS_PER_METER);
    for (int32_t idx : indices) {
      const double world_min = origin + static_cast<double>(idx) / BASE_CELLS_PER_METER;

      // Exact generated boundary -> intended cell
      const auto q_exact = quantizeToBaseGrid(world_min, origin);
      ASSERT_TRUE(q_exact.has_value());
      EXPECT_EQ(*q_exact, idx) << "Exact boundary failure: origin=" << origin << ", idx=" << idx;

      // Immediately above boundary -> intended cell
      const double above = std::nextafter(world_min, std::numeric_limits<double>::infinity());
      const auto q_above = quantizeToBaseGrid(above, origin);
      ASSERT_TRUE(q_above.has_value());
      EXPECT_EQ(*q_above, idx) << "Above boundary failure: origin=" << origin << ", idx=" << idx;

      // Immediately below boundary -> strictly previous cell
      // where the representable values genuinely satisfy those spatial relations.
      const double below = std::nextafter(world_min, -std::numeric_limits<double>::infinity());
      const auto q_below = quantizeToBaseGrid(below, origin);
      ASSERT_TRUE(q_below.has_value());
      if (below * BASE_CELLS_PER_METER < (o_cell_target + idx)) {
        EXPECT_EQ(*q_below, idx - 1)
            << "Below boundary failure: origin=" << origin << ", idx=" << idx;
      } else {
        EXPECT_EQ(*q_below, idx)
            << "Float rounding edge case at boundary: origin=" << origin << ", idx=" << idx;
      }
    }
  }
}

TEST(QuantizationTest, RandomizedNonZeroOrigin25k) {
  // Deterministic randomized test with >= 25,000 cases.
  // Origin aligned to integer base grid in [-10000 m, +10000 m] (ADR-007).
  // Cell indices in safe range [-100000, +100000].
  // Verifies that canonical world_min accurately reconstructs the intended integer index.
  std::mt19937_64 rng(26053ULL);
  std::uniform_int_distribution<int32_t> origin_cell_dist(-200000, 200000);
  std::uniform_int_distribution<int32_t> index_dist(-100000, 100000);

  constexpr size_t kCases = 25000;
  size_t mismatches = 0;

  for (size_t i = 0; i < kCases; ++i) {
    const int32_t origin_cell = origin_cell_dist(rng);
    const double origin = static_cast<double>(origin_cell) / BASE_CELLS_PER_METER;
    const int32_t expected_idx = index_dist(rng);
    const double world_min = static_cast<double>(origin_cell + expected_idx) / BASE_CELLS_PER_METER;

    const auto q = quantizeToBaseGrid(world_min, origin);
    if (!q.has_value() || *q != expected_idx) {
      ++mismatches;
    }
  }

  EXPECT_EQ(mismatches, 0U) << "Randomized non-zero origin quantization had mismatches!";
}

TEST(QuantizationTest, CanonicalOriginNextafterLargeDeterministicSweep) {
  // Verifies canonical origin = 0.0 with nextafter tests across 20,001 boundaries.
  // The actual AVR map grid is anchored at canonical origin (0.0, 0.0).
  // Unambiguous half-open ownership [min, max) must be preserved for all boundaries.
  constexpr int32_t kMinIdx = -10000;
  constexpr int32_t kMaxIdx = 10000;

  for (int32_t idx = kMinIdx; idx <= kMaxIdx; ++idx) {
    const double boundary = static_cast<double>(idx) / BASE_CELLS_PER_METER;

    // Exact boundary must belong to intended cell
    const auto q_exact = quantizeToBaseGrid(boundary, 0.0);
    ASSERT_TRUE(q_exact.has_value());
    EXPECT_EQ(*q_exact, idx) << "Canonical exact boundary failure at idx=" << idx;

    // Immediately above boundary must belong to intended cell
    const double above = std::nextafter(boundary, std::numeric_limits<double>::infinity());
    const auto q_above = quantizeToBaseGrid(above, 0.0);
    ASSERT_TRUE(q_above.has_value());
    EXPECT_EQ(*q_above, idx) << "Canonical above boundary failure at idx=" << idx;

    // Immediately below boundary must belong to previous cell where representable
    // values genuinely satisfy below * BASE_CELLS_PER_METER < idx in IEEE 754 arithmetic.
    const double below = std::nextafter(boundary, -std::numeric_limits<double>::infinity());
    const auto q_below = quantizeToBaseGrid(below, 0.0);
    ASSERT_TRUE(q_below.has_value());
    if (below * BASE_CELLS_PER_METER < idx) {
      EXPECT_EQ(*q_below, idx - 1) << "Canonical below boundary failure at idx=" << idx;
    } else {
      EXPECT_EQ(*q_below, idx) << "Canonical float limit edge case at idx=" << idx;
    }
  }
}

TEST(QuantizationTest, OriginValidation) {
  const double nan_val = std::numeric_limits<double>::quiet_NaN();
  const double pos_inf = std::numeric_limits<double>::infinity();
  const double neg_inf = -std::numeric_limits<double>::infinity();

  // 1D quantization with invalid origins must return std::nullopt
  EXPECT_EQ(quantizeToBaseGrid(1.0, nan_val), std::nullopt);
  EXPECT_EQ(quantizeToBaseGrid(1.0, pos_inf), std::nullopt);
  EXPECT_EQ(quantizeToBaseGrid(1.0, neg_inf), std::nullopt);

  // 2D worldToGridCoord with non-finite origins
  EXPECT_EQ(worldToGridCoord(WorldPoint2D{.x = 1.0, .y = 1.0}, WorldPoint2D{.x = nan_val, .y = 0.0}),
            std::nullopt);
  EXPECT_EQ(worldToGridCoord(WorldPoint2D{.x = 1.0, .y = 1.0}, WorldPoint2D{.x = 0.0, .y = nan_val}),
            std::nullopt);
  EXPECT_EQ(worldToGridCoord(WorldPoint2D{.x = 1.0, .y = 1.0}, WorldPoint2D{.x = pos_inf, .y = 0.0}),
            std::nullopt);
  EXPECT_EQ(worldToGridCoord(WorldPoint2D{.x = 1.0, .y = 1.0}, WorldPoint2D{.x = 0.0, .y = neg_inf}),
            std::nullopt);

  // 2D worldToCellCoord with non-finite origins
  EXPECT_EQ(worldToCellCoord(WorldPoint2D{.x = 1.0, .y = 1.0}, L0, WorldPoint2D{.x = nan_val, .y = 0.0}),
            std::nullopt);
  EXPECT_EQ(worldToCellCoord(WorldPoint2D{.x = 1.0, .y = 1.0}, L1, WorldPoint2D{.x = pos_inf, .y = 0.0}),
            std::nullopt);
  EXPECT_EQ(worldToCellCoord(WorldPoint2D{.x = 1.0, .y = 1.0}, L2, WorldPoint2D{.x = 0.0, .y = neg_inf}),
            std::nullopt);

  // CellKey from world point with non-finite origins
  EXPECT_EQ(
      CellKey::fromWorldPoint(WorldPoint2D{.x = 1.0, .y = 1.0}, L0, WorldPoint2D{.x = nan_val, .y = 0.0}),
      std::nullopt);
  EXPECT_EQ(
      CellKey::fromWorldPoint(WorldPoint2D{.x = 1.0, .y = 1.0}, L0, WorldPoint2D{.x = pos_inf, .y = 0.0}),
      std::nullopt);
  EXPECT_EQ(
      CellKey::fromWorldPoint(WorldPoint2D{.x = 1.0, .y = 1.0}, L0, WorldPoint2D{.x = 0.0, .y = neg_inf}),
      std::nullopt);
}

TEST(QuantizationTest, MetricOriginConversionValidAligned) {
  // Required aligned metric origins: 0.0, 0.05, -0.05, 0.10, -0.10, 1.0, -1.0, 123.45, -987.65
  const struct TestCase {
    double metric;
    int32_t expected_cell;
  } cases[] = {
      {0.0, 0},
      {0.05, 1},
      {-0.05, -1},
      {0.10, 2},
      {-0.10, -2},
      {1.0, 20},
      {-1.0, -20},
      {123.45, 2469},
      {-987.65, -19753},
  };

  for (const auto& tc : cases) {
    const auto cell_opt = metricOriginToGridCell(tc.metric);
    ASSERT_TRUE(cell_opt.has_value()) << "Failed for aligned origin: " << tc.metric;
    EXPECT_EQ(*cell_opt, tc.expected_cell);

    const auto origin_opt = metricOriginToGridOrigin(WorldPoint2D{.x = tc.metric, .y = tc.metric});
    ASSERT_TRUE(origin_opt.has_value());
    EXPECT_EQ(origin_opt->x, tc.expected_cell);
    EXPECT_EQ(origin_opt->y, tc.expected_cell);
  }
}

TEST(QuantizationTest, MetricOriginConversionInvalidNonAligned) {
  // Required non-aligned origins: 0.01, 0.02, 0.033, 0.049, 0.051, 1.001, -0.014, -0.049, -0.051
  const double non_aligned[] = {
      0.01, 0.02, 0.033, 0.049, 0.051, 1.001, -0.014, -0.049, -0.051,
  };

  for (double origin : non_aligned) {
    EXPECT_EQ(metricOriginToGridCell(origin), std::nullopt)
        << "Non-aligned origin did not fail: " << origin;

    EXPECT_EQ(metricOriginToGridOrigin(WorldPoint2D{.x = origin, .y = 0.0}), std::nullopt);
    EXPECT_EQ(metricOriginToGridOrigin(WorldPoint2D{.x = 0.0, .y = origin}), std::nullopt);
    EXPECT_EQ(metricOriginToGridOrigin(WorldPoint2D{.x = origin, .y = origin}), std::nullopt);

    // Public quantize/worldToGrid overloads with metric origin must reject non-aligned origin
    EXPECT_EQ(quantizeToBaseGrid(1.0, origin), std::nullopt);
    EXPECT_EQ(quantizeToBaseGridSafe(1.0, origin), std::nullopt);
    EXPECT_EQ(worldToGridCoord(WorldPoint2D{.x = 1.0, .y = 1.0}, WorldPoint2D{.x = origin, .y = 0.0}),
              std::nullopt);
    EXPECT_EQ(worldToCellCoord(WorldPoint2D{.x = 1.0, .y = 1.0}, L0, WorldPoint2D{.x = 0.0, .y = origin}),
              std::nullopt);
    EXPECT_EQ(
        CellKey::fromWorldPoint(WorldPoint2D{.x = 1.0, .y = 1.0}, L0, WorldPoint2D{.x = origin, .y = origin}),
        std::nullopt);
  }
}

TEST(QuantizationTest, IntegerOriginCheckedArithmeticOffsets) {
  // Offsets: 0, 1, -1, 17, -17, 100000, -100000
  const int32_t offsets[] = {0, 1, -1, 17, -17, 100000, -100000};
  const double test_worlds[] = {-15.2, -1.0, -0.05, 0.0, 0.05, 2.5, 50.0};

  for (int32_t offset : offsets) {
    const GridOrigin2D origin{.x = offset, .y = offset};
    for (double w : test_worlds) {
      const auto global_base = quantizeToBaseGridSafe(w);
      ASSERT_TRUE(global_base.has_value());

      // Checked offset
      const auto local_base = checkedCellOffset(*global_base, offset);
      ASSERT_TRUE(local_base.has_value());
      EXPECT_EQ(*local_base, *global_base - offset);

      // Quantize with integer origin
      const auto q_int = quantizeToBaseGridSafe(w, offset);
      ASSERT_TRUE(q_int.has_value());
      EXPECT_EQ(*q_int, *local_base);

      // 2D worldToGridCoord with GridOrigin2D
      const auto g_2d = worldToGridCoord(WorldPoint2D{.x = w, .y = w}, origin);
      ASSERT_TRUE(g_2d.has_value());
      EXPECT_EQ(g_2d->x, *local_base);
      EXPECT_EQ(g_2d->y, *local_base);
    }
  }
}

TEST(QuantizationTest, ExtremeOriginArithmetic) {
  // global_cell - origin_cell near INT32_MIN and INT32_MAX
  // Underflow: INT32_MIN - 1
  EXPECT_EQ(checkedCellOffset(std::numeric_limits<int32_t>::min(), 1), std::nullopt);

  // Overflow: INT32_MAX - (-1)
  EXPECT_EQ(checkedCellOffset(std::numeric_limits<int32_t>::max(), -1), std::nullopt);

  // Extreme underflow: INT32_MIN - INT32_MAX
  EXPECT_EQ(checkedCellOffset(std::numeric_limits<int32_t>::min(), std::numeric_limits<int32_t>::max()),
            std::nullopt);

  // Extreme overflow: INT32_MAX - INT32_MIN
  EXPECT_EQ(checkedCellOffset(std::numeric_limits<int32_t>::max(), std::numeric_limits<int32_t>::min()),
            std::nullopt);

  // Representable boundaries
  EXPECT_EQ(checkedCellOffset(std::numeric_limits<int32_t>::max(), 0),
            std::numeric_limits<int32_t>::max());
  EXPECT_EQ(checkedCellOffset(std::numeric_limits<int32_t>::min(), 0),
            std::numeric_limits<int32_t>::min());
  EXPECT_EQ(checkedCellOffset(0, std::numeric_limits<int32_t>::max()),
            -std::numeric_limits<int32_t>::max());
  EXPECT_EQ(checkedCellOffset(std::numeric_limits<int32_t>::max(), std::numeric_limits<int32_t>::max()), 0);
  EXPECT_EQ(checkedCellOffset(std::numeric_limits<int32_t>::min(), std::numeric_limits<int32_t>::min()), 0);
  EXPECT_EQ(checkedCellOffset(std::numeric_limits<int32_t>::min() + 1, 1),
            std::numeric_limits<int32_t>::min());
  EXPECT_EQ(checkedCellOffset(std::numeric_limits<int32_t>::max() - 1, -1),
            std::numeric_limits<int32_t>::max());

  // Failures through public quantization API when origin offset causes overflow
  EXPECT_EQ(quantizeToBaseGridSafe(0.0, std::numeric_limits<int32_t>::min()), std::nullopt);
}

TEST(QuantizationTest, CanonicalMapRequirementAround100m) {
  // AVR canonical map origin = GridOrigin2D{0, 0}
  const GridOrigin2D canonical_origin = GridOrigin2D::canonical();
  EXPECT_EQ(canonical_origin.x, 0);
  EXPECT_EQ(canonical_origin.y, 0);

  // World coordinates around -100 m, 0 m, +100 m
  // -100.0 m -> expected base cell -2000
  //    0.0 m -> expected base cell 0
  // +100.0 m -> expected base cell +2000
  const struct TestCase {
    double coord;
    int32_t expected_cell;
  } cases[] = {
      {-100.0, -2000},
      {0.0, 0},
      {100.0, 2000},
  };

  for (const auto& tc : cases) {
    // Exact coordinate
    const auto q_exact = worldToGridCoord(WorldPoint2D{.x = tc.coord, .y = tc.coord}, canonical_origin);
    ASSERT_TRUE(q_exact.has_value());
    EXPECT_EQ(q_exact->x, tc.expected_cell);
    EXPECT_EQ(q_exact->y, tc.expected_cell);

    // Immediately above boundary -> expected_cell
    const double above = std::nextafter(tc.coord, std::numeric_limits<double>::infinity());
    const auto q_above = worldToGridCoord(WorldPoint2D{.x = above, .y = above}, canonical_origin);
    ASSERT_TRUE(q_above.has_value());
    EXPECT_EQ(q_above->x, tc.expected_cell);
    EXPECT_EQ(q_above->y, tc.expected_cell);

    // Immediately below boundary -> expected_cell - 1
    const double below = std::nextafter(tc.coord, -std::numeric_limits<double>::infinity());
    const auto q_below = worldToGridCoord(WorldPoint2D{.x = below, .y = below}, canonical_origin);
    ASSERT_TRUE(q_below.has_value());
    EXPECT_EQ(q_below->x, tc.expected_cell - 1);
    EXPECT_EQ(q_below->y, tc.expected_cell - 1);
  }
}

}  // namespace avr::spatial


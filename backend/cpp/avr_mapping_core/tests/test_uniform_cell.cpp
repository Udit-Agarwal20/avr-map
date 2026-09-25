#include <gtest/gtest.h>

#include <limits>
#include <type_traits>

#include "avr_mapping/uniform_cell.hpp"

namespace {

using avr::mapping::UniformCell;

TEST(UniformCellTest, InitialState) {
  UniformCell cell;
  EXPECT_EQ(cell.observationCount(), 0u);
  EXPECT_DOUBLE_EQ(cell.meanZ(), 0.0);
  EXPECT_DOUBLE_EQ(cell.m2Z(), 0.0);
  EXPECT_DOUBLE_EQ(cell.minZ(), 0.0);
  EXPECT_DOUBLE_EQ(cell.maxZ(), 0.0);
  EXPECT_DOUBLE_EQ(cell.populationVariance(), 0.0);
  EXPECT_DOUBLE_EQ(cell.sampleVariance(), 0.0);
  EXPECT_EQ(cell.lastSeenFrame(), 0u);
}

// Section 13: First observation handling
TEST(UniformCellTest, FirstObservationContract) {
  UniformCell cell;
  constexpr double z = 3.14159;
  constexpr uint64_t frame = 100;

  EXPECT_TRUE(cell.tryUpdate(z, frame));

  EXPECT_EQ(cell.observationCount(), 1u);
  EXPECT_DOUBLE_EQ(cell.meanZ(), z);
  EXPECT_DOUBLE_EQ(cell.m2Z(), 0.0);
  EXPECT_DOUBLE_EQ(cell.minZ(), z);
  EXPECT_DOUBLE_EQ(cell.maxZ(), z);
  EXPECT_DOUBLE_EQ(cell.populationVariance(), 0.0);
  EXPECT_DOUBLE_EQ(cell.sampleVariance(), 0.0);
  EXPECT_EQ(cell.lastSeenFrame(), frame);
}

// Section 14 & 39: Subsequent observations and frame sequence
TEST(UniformCellTest, SubsequentObservationsAndFrameSequence) {
  UniformCell cell;
  EXPECT_TRUE(cell.tryUpdate(2.0, 10));
  EXPECT_EQ(cell.lastSeenFrame(), 10u);

  // Non-consecutive frame update
  EXPECT_TRUE(cell.tryUpdate(4.0, 42));
  EXPECT_EQ(cell.observationCount(), 2u);
  EXPECT_DOUBLE_EQ(cell.meanZ(), 3.0);
  EXPECT_DOUBLE_EQ(cell.minZ(), 2.0);
  EXPECT_DOUBLE_EQ(cell.maxZ(), 4.0);
  EXPECT_EQ(cell.lastSeenFrame(), 42u);

  EXPECT_TRUE(cell.tryUpdate(6.0, 42));
  EXPECT_EQ(cell.observationCount(), 3u);
  EXPECT_DOUBLE_EQ(cell.meanZ(), 4.0);
  EXPECT_DOUBLE_EQ(cell.minZ(), 2.0);
  EXPECT_DOUBLE_EQ(cell.maxZ(), 6.0);
  EXPECT_EQ(cell.lastSeenFrame(), 42u);
}

// Milestone 2.1: Numeric rejection preserves last_seen_frame and prior state
TEST(UniformCellTest, NumericRejectionPreservesPriorStateAndLastSeenFrame) {
  UniformCell cell;
  EXPECT_TRUE(cell.tryUpdate(-1e308, 15));
  EXPECT_EQ(cell.observationCount(), 1u);
  EXPECT_EQ(cell.lastSeenFrame(), 15u);
  EXPECT_DOUBLE_EQ(cell.meanZ(), -1e308);

  // Ingest extreme value that causes delta overflow
  EXPECT_FALSE(cell.tryUpdate(1e308, 99));

  // Cell state and frame sequence MUST be completely preserved
  EXPECT_EQ(cell.observationCount(), 1u);
  EXPECT_EQ(cell.lastSeenFrame(), 15u);  // Not 99!
  EXPECT_DOUBLE_EQ(cell.meanZ(), -1e308);

  // Subsequent valid update still works
  EXPECT_TRUE(cell.tryUpdate(-1e308, 105));
  EXPECT_EQ(cell.observationCount(), 2u);
  EXPECT_EQ(cell.lastSeenFrame(), 105u);
}

TEST(UniformCellTest, MemoryLayoutAndTrivialCopy) {
  static_assert(std::is_trivially_copyable_v<UniformCell>);
  // Size remains strictly 48 bytes with uint64 observation count
  EXPECT_EQ(sizeof(UniformCell), 48u);
  EXPECT_EQ(alignof(UniformCell), 8u);
}

TEST(UniformCellTest, Reset) {
  UniformCell cell;
  cell.update(10.0, 1);
  cell.update(20.0, 2);
  EXPECT_EQ(cell.observationCount(), 2u);

  cell.reset();
  EXPECT_EQ(cell.observationCount(), 0u);
  EXPECT_DOUBLE_EQ(cell.meanZ(), 0.0);
  EXPECT_EQ(cell.lastSeenFrame(), 0u);
}

}  // namespace

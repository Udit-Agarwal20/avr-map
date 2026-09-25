#include <gtest/gtest.h>

#include <optional>

#include "avr_spatial/constants.hpp"

namespace avr::spatial {

TEST(ResolutionTest, HierarchyConstants) {
  EXPECT_DOUBLE_EQ(BASE_RESOLUTION_METERS, 0.05);
  EXPECT_DOUBLE_EQ(BASE_CELLS_PER_METER, 20.0);
  EXPECT_EQ(LEVEL_COUNT, 4);
  EXPECT_EQ(L0, 0);
  EXPECT_EQ(L1, 1);
  EXPECT_EQ(L2, 2);
  EXPECT_EQ(L3, 3);
}

TEST(ResolutionTest, ExactDyadicValues) {
  EXPECT_DOUBLE_EQ(resolutionMeters(L0), 0.05);
  EXPECT_DOUBLE_EQ(resolutionMeters(L1), 0.10);
  EXPECT_DOUBLE_EQ(resolutionMeters(L2), 0.20);
  EXPECT_DOUBLE_EQ(resolutionMeters(L3), 0.40);

  // Safe query
  EXPECT_EQ(resolutionMetersSafe(L0), 0.05);
  EXPECT_EQ(resolutionMetersSafe(L1), 0.10);
  EXPECT_EQ(resolutionMetersSafe(L2), 0.20);
  EXPECT_EQ(resolutionMetersSafe(L3), 0.40);

  // Dyadic doubling property
  EXPECT_DOUBLE_EQ(resolutionMeters(L1), resolutionMeters(L0) * 2.0);
  EXPECT_DOUBLE_EQ(resolutionMeters(L2), resolutionMeters(L1) * 2.0);
  EXPECT_DOUBLE_EQ(resolutionMeters(L3), resolutionMeters(L2) * 2.0);
}

TEST(ResolutionTest, CellsPerMeter) {
  EXPECT_DOUBLE_EQ(cellsPerMeter(L0), 20.0);
  EXPECT_DOUBLE_EQ(cellsPerMeter(L1), 10.0);
  EXPECT_DOUBLE_EQ(cellsPerMeter(L2), 5.0);
  EXPECT_DOUBLE_EQ(cellsPerMeter(L3), 2.5);

  EXPECT_EQ(cellsPerMeterSafe(L0), 20.0);
  EXPECT_EQ(cellsPerMeterSafe(L1), 10.0);
  EXPECT_EQ(cellsPerMeterSafe(L2), 5.0);
  EXPECT_EQ(cellsPerMeterSafe(L3), 2.5);
}

TEST(ResolutionTest, LevelScale) {
  EXPECT_EQ(levelScale(L0), 1U);
  EXPECT_EQ(levelScale(L1), 2U);
  EXPECT_EQ(levelScale(L2), 4U);
  EXPECT_EQ(levelScale(L3), 8U);

  EXPECT_EQ(levelScaleSafe(L0), 1U);
  EXPECT_EQ(levelScaleSafe(L1), 2U);
  EXPECT_EQ(levelScaleSafe(L2), 4U);
  EXPECT_EQ(levelScaleSafe(L3), 8U);
}

TEST(ResolutionTest, LevelValidation) {
  EXPECT_TRUE(isValidLevel(0));
  EXPECT_TRUE(isValidLevel(1));
  EXPECT_TRUE(isValidLevel(2));
  EXPECT_TRUE(isValidLevel(3));

  EXPECT_FALSE(isValidLevel(4));
  EXPECT_FALSE(isValidLevel(5));
  EXPECT_FALSE(isValidLevel(255));
}

TEST(ResolutionTest, InvalidLevelHardening) {
  // Invalid levels must never cause shift UB, overflow, or fake resolutions
  const uint8_t invalid_levels[] = {4, 5, 31, 32, 63, 64, 128, 255};

  for (uint8_t lvl : invalid_levels) {
    EXPECT_FALSE(isValidLevel(lvl));
    EXPECT_DOUBLE_EQ(resolutionMeters(lvl), 0.0);
    EXPECT_EQ(resolutionMetersSafe(lvl), std::nullopt);

    EXPECT_DOUBLE_EQ(cellsPerMeter(lvl), 0.0);
    EXPECT_EQ(cellsPerMeterSafe(lvl), std::nullopt);

    EXPECT_EQ(levelScale(lvl), 0U);
    EXPECT_EQ(levelScaleSafe(lvl), std::nullopt);
  }
}

}  // namespace avr::spatial


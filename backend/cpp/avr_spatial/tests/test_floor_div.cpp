#include <gtest/gtest.h>

#include <cstdint>
#include <limits>
#include <optional>

#include "avr_spatial/floor_div.hpp"

namespace avr::spatial {

TEST(FloorDivTest, RequiredPromptExamples) {
  // Positive numerator, positive divisor
  EXPECT_EQ(floorDiv(5, 2), 2);
  EXPECT_EQ(floorDiv(4, 2), 2);
  EXPECT_EQ(detail::floorDivPositive(5, 2), 2);
  EXPECT_EQ(detail::floorDivPositive(4, 2), 2);

  // Negative numerator, positive divisor
  EXPECT_EQ(floorDiv(-1, 2), -1);
  EXPECT_EQ(floorDiv(-2, 2), -1);
  EXPECT_EQ(floorDiv(-3, 2), -2);
  EXPECT_EQ(detail::floorDivPositive(-1, 2), -1);
  EXPECT_EQ(detail::floorDivPositive(-2, 2), -1);
  EXPECT_EQ(detail::floorDivPositive(-3, 2), -2);

  // Positive numerator, negative divisor
  EXPECT_EQ(floorDiv(1, -2), -1);

  // Negative numerator, negative divisor
  EXPECT_EQ(floorDiv(-1, -2), 0);
  EXPECT_EQ(floorDiv(-3, -2), 1);
}

TEST(FloorDivTest, PowersOfTwoDivisors) {
  // Level 1: divisor 2
  EXPECT_EQ(floorDiv(7, 2), 3);
  EXPECT_EQ(floorDiv(-1, 2), -1);
  EXPECT_EQ(floorDiv(-9, 2), -5);
  EXPECT_EQ(detail::floorDivPositive(7, 2), 3);
  EXPECT_EQ(detail::floorDivPositive(-1, 2), -1);
  EXPECT_EQ(detail::floorDivPositive(-9, 2), -5);

  // Level 2: divisor 4
  EXPECT_EQ(floorDiv(7, 4), 1);
  EXPECT_EQ(floorDiv(-1, 4), -1);
  EXPECT_EQ(floorDiv(-9, 4), -3);
  EXPECT_EQ(detail::floorDivPositive(7, 4), 1);
  EXPECT_EQ(detail::floorDivPositive(-1, 4), -1);
  EXPECT_EQ(detail::floorDivPositive(-9, 4), -3);

  // Level 3: divisor 8
  EXPECT_EQ(floorDiv(7, 8), 0);
  EXPECT_EQ(floorDiv(-1, 8), -1);
  EXPECT_EQ(floorDiv(-9, 8), -2);
  EXPECT_EQ(detail::floorDivPositive(7, 8), 0);
  EXPECT_EQ(detail::floorDivPositive(-1, 8), -1);
  EXPECT_EQ(detail::floorDivPositive(-9, 8), -2);

  // Tile dimension: divisor 16
  EXPECT_EQ(floorDiv(15, 16), 0);
  EXPECT_EQ(floorDiv(16, 16), 1);
  EXPECT_EQ(floorDiv(-1, 16), -1);
  EXPECT_EQ(floorDiv(-16, 16), -1);
  EXPECT_EQ(floorDiv(-17, 16), -2);
  EXPECT_EQ(detail::floorDivPositive(15, 16), 0);
  EXPECT_EQ(detail::floorDivPositive(16, 16), 1);
  EXPECT_EQ(detail::floorDivPositive(-1, 16), -1);
  EXPECT_EQ(detail::floorDivPositive(-16, 16), -1);
  EXPECT_EQ(detail::floorDivPositive(-17, 16), -2);
}

TEST(FloorDivTest, ZeroDivisionReturnsNullopt) {
  EXPECT_EQ(floorDiv(10, 0), std::nullopt);
  EXPECT_EQ(floorDiv(-10, 0), std::nullopt);
  EXPECT_EQ(floorDiv(0, 0), std::nullopt);

  EXPECT_EQ(floorDivSafe(10, 0), std::nullopt);
  EXPECT_EQ(floorDivSafe(-10, 0), std::nullopt);
  EXPECT_EQ(floorDivSafe(0, 0), std::nullopt);
}

TEST(FloorDivTest, MinSignedDivisionOverflowReturnsNullopt) {
  // INT32_MIN / -1 cannot be represented in int32_t (would equal +2,147,483,648)
  EXPECT_EQ(floorDiv(INT32_MIN, -1), std::nullopt);
  EXPECT_EQ(floorDivSafe(INT32_MIN, -1), std::nullopt);

  // INT64_MIN / -1
  EXPECT_EQ(floorDiv<int64_t>(std::numeric_limits<int64_t>::min(), -1LL), std::nullopt);
  EXPECT_EQ(floorDivSafe<int64_t>(std::numeric_limits<int64_t>::min(), -1LL), std::nullopt);

  // INT16_MIN / -1
  EXPECT_EQ(floorDiv<int16_t>(std::numeric_limits<int16_t>::min(), -1), std::nullopt);
  EXPECT_EQ(floorDiv<int8_t>(std::numeric_limits<int8_t>::min(), -1), std::nullopt);
}

TEST(FloorDivTest, ExtremeValues) {
  EXPECT_EQ(floorDiv(INT32_MAX, 1), INT32_MAX);
  EXPECT_EQ(floorDiv(INT32_MIN, 1), INT32_MIN);
  EXPECT_EQ(detail::floorDivPositive(INT32_MAX, 1), INT32_MAX);
  EXPECT_EQ(detail::floorDivPositive(INT32_MIN, 1), INT32_MIN);

  EXPECT_EQ(floorDiv(INT32_MAX, 2), INT32_MAX / 2);
  EXPECT_EQ(floorDiv(INT32_MIN, 2), INT32_MIN / 2);
  EXPECT_EQ(detail::floorDivPositive(INT32_MAX, 2), INT32_MAX / 2);
  EXPECT_EQ(detail::floorDivPositive(INT32_MIN, 2), INT32_MIN / 2);

  // INT32_MAX / -1 is representable: -INT32_MAX
  EXPECT_EQ(floorDiv(INT32_MAX, -1), -INT32_MAX);
  EXPECT_EQ(floorDivSafe(INT32_MAX, -1), -INT32_MAX);

  // 64-bit types
  EXPECT_EQ(floorDiv<int64_t>(-1LL, 2LL), -1LL);
  EXPECT_EQ(floorDiv<int64_t>(-100LL, 16LL), -7LL);
  EXPECT_EQ(detail::floorDivPositive<int64_t>(-1LL, 2LL), -1LL);
  EXPECT_EQ(detail::floorDivPositive<int64_t>(-100LL, 16LL), -7LL);
}

}  // namespace avr::spatial


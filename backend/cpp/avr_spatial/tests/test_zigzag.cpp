#include <gtest/gtest.h>

#include <cstdint>

#include "avr_spatial/zigzag.hpp"

namespace avr::spatial {

TEST(ZigZagTest, CanonicalMappings) {
  EXPECT_EQ(zigzagEncode(0), 0U);
  EXPECT_EQ(zigzagEncode(-1), 1U);
  EXPECT_EQ(zigzagEncode(1), 2U);
  EXPECT_EQ(zigzagEncode(-2), 3U);
  EXPECT_EQ(zigzagEncode(2), 4U);
  EXPECT_EQ(zigzagEncode(-3), 5U);
  EXPECT_EQ(zigzagEncode(3), 6U);
}

TEST(ZigZagTest, DecodeMappings) {
  EXPECT_EQ(zigzagDecode(0U), 0);
  EXPECT_EQ(zigzagDecode(1U), -1);
  EXPECT_EQ(zigzagDecode(2U), 1);
  EXPECT_EQ(zigzagDecode(3U), -2);
  EXPECT_EQ(zigzagDecode(4U), 2);
  EXPECT_EQ(zigzagDecode(5U), -3);
  EXPECT_EQ(zigzagDecode(6U), 3);
}

TEST(ZigZagTest, ExtremeValues) {
  // INT32_MAX -> UINT32_MAX - 1
  EXPECT_EQ(zigzagEncode(INT32_MAX), UINT32_MAX - 1U);
  EXPECT_EQ(zigzagDecode(UINT32_MAX - 1U), INT32_MAX);

  // INT32_MIN -> UINT32_MAX
  EXPECT_EQ(zigzagEncode(INT32_MIN), UINT32_MAX);
  EXPECT_EQ(zigzagDecode(UINT32_MAX), INT32_MIN);

  // Tile boundaries
  for (int32_t val : {15, 16, 17, -15, -16, -17}) {
    EXPECT_EQ(zigzagDecode(zigzagEncode(val)), val);
  }
}

TEST(ZigZagTest, RoundTripExhaustiveSmallRange) {
  for (int32_t i = -10000; i <= 10000; ++i) {
    const uint32_t enc = zigzagEncode(i);
    const int32_t dec = zigzagDecode(enc);
    EXPECT_EQ(dec, i);
  }
}

}  // namespace avr::spatial

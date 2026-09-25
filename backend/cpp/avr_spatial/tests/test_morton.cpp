#include <gtest/gtest.h>

#include <cstdint>

#include "avr_spatial/morton.hpp"

namespace avr::spatial {

TEST(MortonTest, PRDSection22Example) {
  // PRD Section 22:
  // x bits = 1 0 1 (5)
  // y bits = 0 1 1 (3)
  // interleave (x2 y2 x1 y1 x0 y0) -> 1 0 0 1 1 1 (39)
  const uint32_t ux = 5U;
  const uint32_t uy = 3U;

  const uint64_t code = morton2DEncodeUnsigned(ux, uy);
  EXPECT_EQ(code, 39ULL);

  uint32_t dec_x = 0;
  uint32_t dec_y = 0;
  morton2DDecodeUnsigned(code, dec_x, dec_y);
  EXPECT_EQ(dec_x, ux);
  EXPECT_EQ(dec_y, uy);
}

TEST(MortonTest, SignedZeroOrigin) {
  const uint64_t code = morton2DEncode(0, 0);
  EXPECT_EQ(code, 0ULL);

  int32_t x = -1;
  int32_t y = -1;
  morton2DDecode(code, x, y);
  EXPECT_EQ(x, 0);
  EXPECT_EQ(y, 0);
}

TEST(MortonTest, SignedQuadrantsRoundTrip) {
  const std::pair<int32_t, int32_t> test_cases[] = {
      {1, 1},
      {-1, 1},
      {-1, -1},
      {1, -1},
      {2000, 2000},
      {-2000, 2000},
      {-2000, -2000},
      {2000, -2000},
      {15, 15},
      {-16, -16},
  };

  for (const auto& [tx, ty] : test_cases) {
    const uint64_t code = morton2DEncode(tx, ty);
    const auto [rx, ry] = morton2DDecodePair(code);
    EXPECT_EQ(rx, tx) << "Failed for x=" << tx << ", y=" << ty;
    EXPECT_EQ(ry, ty) << "Failed for x=" << tx << ", y=" << ty;
  }
}

TEST(MortonTest, ExtremeCoordinates) {
  const int32_t extremes[] = {INT32_MIN, INT32_MAX, 0, -1, 1};

  for (int32_t x : extremes) {
    for (int32_t y : extremes) {
      const uint64_t code = morton2DEncode(x, y);
      const auto [rx, ry] = morton2DDecodePair(code);
      EXPECT_EQ(rx, x) << "Extreme failed for x=" << x << ", y=" << y;
      EXPECT_EQ(ry, y) << "Extreme failed for x=" << x << ", y=" << y;
    }
  }
}

}  // namespace avr::spatial

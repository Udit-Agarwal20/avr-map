#pragma once

#include <cstdint>
#include <utility>

#include "avr_spatial/zigzag.hpp"

namespace avr::spatial {

static_assert(sizeof(uint64_t) == 8, "uint64_t must be exactly 64 bits");
static_assert(sizeof(uint32_t) == 4, "uint32_t must be exactly 32 bits");

/**
 * @brief Dilates 32 bits of an unsigned integer into even bit positions of a 64-bit integer.
 *
 * Example: 0b1011 -> 0b01000101 (inserts zeros in odd bit positions)
 */
[[nodiscard]] constexpr uint64_t dilate32(uint32_t val) noexcept {
  uint64_t x = val;
  x = (x | (x << 16)) & 0x0000FFFF0000FFFFULL;
  x = (x | (x << 8)) & 0x00FF00FF00FF00FFULL;
  x = (x | (x << 4)) & 0x0F0F0F0F0F0F0F0FULL;
  x = (x | (x << 2)) & 0x3333333333333333ULL;
  x = (x | (x << 1)) & 0x5555555555555555ULL;
  return x;
}

/**
 * @brief Compacts the even bit positions of a 64-bit integer into a 32-bit unsigned integer.
 *
 * Inverse of dilate32().
 */
[[nodiscard]] constexpr uint32_t compact64(uint64_t val) noexcept {
  uint64_t x = val & 0x5555555555555555ULL;
  x = (x ^ (x >> 1)) & 0x3333333333333333ULL;
  x = (x ^ (x >> 2)) & 0x0F0F0F0F0F0F0F0FULL;
  x = (x ^ (x >> 4)) & 0x00FF00FF00FF00FFULL;
  x = (x ^ (x >> 8)) & 0x0000FFFF0000FFFFULL;
  x = (x ^ (x >> 16)) & 0x00000000FFFFFFFFULL;
  return static_cast<uint32_t>(x);
}

/**
 * @brief Interleaves unsigned 32-bit (ux, uy) coordinates into a 64-bit Morton key.
 *
 * Implements the exact canonical bit pattern from PRD Section 22 & TRD Section 53:
 *   (x2 y2 x1 y1 x0 y0 ...)
 * where x bits occupy odd bit positions (1, 3, 5, ...) and y bits occupy even bit positions (0, 2, 4, ...).
 *
 * Example:
 *   ux = 5 (1 0 1)
 *   uy = 3 (0 1 1)
 *   interleave -> 1 0 0 1 1 1 (39)
 */
[[nodiscard]] constexpr uint64_t morton2DEncodeUnsigned(uint32_t ux, uint32_t uy) noexcept {
  return dilate32(uy) | (dilate32(ux) << 1);
}

/**
 * @brief Deinterleaves a 64-bit Morton key into unsigned 32-bit (ux, uy) coordinates.
 */
constexpr void morton2DDecodeUnsigned(uint64_t code, uint32_t& ux, uint32_t& uy) noexcept {
  uy = compact64(code);
  ux = compact64(code >> 1);
}

/**
 * @brief Encodes signed 32-bit (x, y) coordinates into a 64-bit Morton key via ZigZag mapping.
 */
[[nodiscard]] constexpr uint64_t morton2DEncode(int32_t x, int32_t y) noexcept {
  return morton2DEncodeUnsigned(zigzagEncode(x), zigzagEncode(y));
}

/**
 * @brief Decodes a 64-bit Morton key back into signed 32-bit (x, y) coordinates.
 */
constexpr void morton2DDecode(uint64_t code, int32_t& x, int32_t& y) noexcept {
  uint32_t ux = 0;
  uint32_t uy = 0;
  morton2DDecodeUnsigned(code, ux, uy);
  x = zigzagDecode(ux);
  y = zigzagDecode(uy);
}

/**
 * @brief Decodes a 64-bit Morton key returning an std::pair of signed (x, y) coordinates.
 */
[[nodiscard]] constexpr std::pair<int32_t, int32_t> morton2DDecodePair(uint64_t code) noexcept {
  int32_t x = 0;
  int32_t y = 0;
  morton2DDecode(code, x, y);
  return {x, y};
}

}  // namespace avr::spatial

namespace AVR {
namespace spatial = ::avr::spatial;
}

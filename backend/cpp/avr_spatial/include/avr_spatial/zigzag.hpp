#pragma once

#include <cstdint>

namespace avr::spatial {

static_assert(sizeof(int32_t) == 4, "int32_t must be exactly 32 bits");
static_assert(sizeof(uint32_t) == 4, "uint32_t must be exactly 32 bits");

/**
 * @brief Encodes a signed 32-bit integer to an unsigned 32-bit integer using ZigZag encoding.
 *
 * Maps integers symmetrically around zero so small magnitudes yield small unsigned values:
 *    0 -> 0
 *   -1 -> 1
 *    1 -> 2
 *   -2 -> 3
 *    2 -> 4
 *   -3 -> 5
 *    3 -> 6
 *   INT32_MAX -> UINT32_MAX - 1
 *   INT32_MIN -> UINT32_MAX
 *
 * Uses purely unsigned bitwise operations to eliminate signed-overflow undefined behavior.
 */
[[nodiscard]] constexpr uint32_t zigzagEncode(int32_t n) noexcept {
  return (static_cast<uint32_t>(n) << 1) ^ static_cast<uint32_t>(n >> 31);
}

/**
 * @brief Decodes an unsigned 32-bit integer back to a signed 32-bit integer using ZigZag decoding.
 */
[[nodiscard]] constexpr int32_t zigzagDecode(uint32_t u) noexcept {
  return static_cast<int32_t>((u >> 1) ^ static_cast<uint32_t>(-static_cast<int32_t>(u & 1U)));
}

}  // namespace avr::spatial

namespace AVR {
namespace spatial = ::avr::spatial;
}

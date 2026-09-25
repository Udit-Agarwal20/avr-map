#pragma once

#include <cassert>
#include <concepts>
#include <limits>
#include <optional>
#include <type_traits>

namespace avr::spatial {

/**
 * @brief Computes mathematically correct integer floor division with explicit validity reporting:
 *        floor(a / b).
 *
 * Contract:
 * - Returns std::nullopt if b == 0 (division by zero is undefined).
 * - Returns std::nullopt if a == numeric_limits<T>::min() and b == -1 for signed types
 *   (signed integer overflow is unrepresentable in two's complement).
 * - Otherwise returns the exact floor quotient:
 *     floorDivSafe( 5,  2) =  2
 *     floorDivSafe( 4,  2) =  2
 *     floorDivSafe(-1,  2) = -1
 *     floorDivSafe(-2,  2) = -1
 *     floorDivSafe(-3,  2) = -2
 *     floorDivSafe( 1, -2) = -1
 *     floorDivSafe(-1, -2) =  0
 *     floorDivSafe(-3, -2) =  1
 *
 * Never invokes undefined behavior. Never throws exceptions.
 */
template <std::integral T>
[[nodiscard]] constexpr std::optional<T> floorDivSafe(T a, T b) noexcept {
  if (b == 0) [[unlikely]] {
    return std::nullopt;
  }

  if constexpr (std::is_signed_v<T>) {
    if (a == std::numeric_limits<T>::min() && b == -1) [[unlikely]] {
      return std::nullopt;  // Guard against signed overflow UB (INT_MIN / -1)
    }
  }

  T q = a / b;
  T r = a % b;

  if (r != 0 && ((a < 0) ^ (b < 0))) {
    --q;
  }

  return q;
}

/**
 * @brief General floor division API returning std::optional<T>.
 *
 * Alias for floorDivSafe. Returns std::nullopt for invalid divisor (0) or signed overflow (min / -1).
 */
template <std::integral T>
[[nodiscard]] constexpr std::optional<T> floorDiv(T a, T b) noexcept {
  return floorDivSafe(a, b);
}

namespace detail {

/**
 * @brief High-performance hierarchy-oriented floor division for strictly positive divisors (b > 0).
 *
 * Contract:
 * - Precondition: b > 0.
 * - Used exclusively in internal spatial math where divisors are verified compile-time constants
 *   or validated dyadic scales (1, 2, 4, 8, 16).
 * - In debug mode: asserts b > 0.
 * - In release mode: defensive fallback avoids division-by-zero UB without returning a deceptive
 *   valid coordinate.
 * - NOT part of the public spatial API. Public callers must use floorDiv / floorDivSafe.
 */
template <std::integral T>
[[nodiscard]] constexpr T floorDivPositive(T a, T b) noexcept {
  assert(b > 0 && "floorDivPositive requires strictly positive divisor (b > 0)");
  if (b <= 0) [[unlikely]] {
    return std::numeric_limits<T>::min();
  }

  T q = a / b;
  T r = a % b;

  if (r != 0 && a < 0) {
    --q;
  }

  return q;
}

}  // namespace detail

}  // namespace avr::spatial

namespace AVR {
namespace spatial = ::avr::spatial;
}


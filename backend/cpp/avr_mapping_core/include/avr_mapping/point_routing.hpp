#pragma once

#include <cstdint>

namespace avr::mapping {

/**
 * @brief Historical static-fusion authorization provenance for an observation.
 *
 * Distinguishes observations authorized to contribute to persistent static map geometry
 * from those excluded due to confirmed dynamic motion or perception filtering.
 *
 * Invariant:
 * Geometry validity (finite coordinates, within spatial grid bounds) is evaluated
 * independently. PointRouting answers only whether a geometrically valid observation
 * is authorized to enter the persistent static map.
 */
enum class PointRouting : uint8_t {
  STATIC_FUSABLE = 0,
  DYNAMIC_EXCLUDED = 1,
};

[[nodiscard]] constexpr bool isValidPointRouting(PointRouting r) noexcept {
  return r == PointRouting::STATIC_FUSABLE || r == PointRouting::DYNAMIC_EXCLUDED;
}

}  // namespace avr::mapping

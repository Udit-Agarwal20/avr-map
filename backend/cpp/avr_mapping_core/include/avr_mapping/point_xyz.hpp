#pragma once

#include <cmath>

namespace avr::mapping {

/**
 * @brief Lightweight 3D point in continuous map coordinate frame (odom).
 *
 * Represents an input LiDAR return in meters.
 * Valid only if all coordinates x, y, z are finite.
 */
struct PointXYZ {
  double x{0.0};
  double y{0.0};
  double z{0.0};

  constexpr bool operator==(const PointXYZ&) const noexcept = default;

  [[nodiscard]] constexpr bool isFinite() const noexcept {
    return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
  }
};

}  // namespace avr::mapping

namespace AVR {
namespace mapping = ::avr::mapping;
}

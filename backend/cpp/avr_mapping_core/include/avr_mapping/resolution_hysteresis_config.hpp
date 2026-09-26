#pragma once

#include <cstdint>

namespace avr::mapping {

/**
 * @brief Configuration parameters for temporal resolution hysteresis.
 */
struct ResolutionHysteresisConfig {
  /// Default prototype threshold: 10 stable evidence-backed region observations.
  /// Note: This is an uncalibrated prototype default; not claimed optimal.
  uint32_t coarsen_stable_observations{10};

  [[nodiscard]] constexpr bool isValid() const noexcept {
    return coarsen_stable_observations >= 1;
  }

  constexpr bool operator==(const ResolutionHysteresisConfig&) const noexcept = default;
};

}  // namespace avr::mapping

namespace AVR {
namespace mapping = ::avr::mapping;
}

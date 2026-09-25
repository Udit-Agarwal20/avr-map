#pragma once

#include <cstdint>

#include "avr_mapping/elevation_stats.hpp"

namespace avr::mapping {

/**
 * @brief Fixed-resolution 5 cm uniform 2.5D cell representation.
 *
 * Contains strictly the baseline state required for M2:
 * - observation count (uint64_t)
 * - running mean z
 * - sum of squared differences M2 z
 * - z_min
 * - z_max
 * - last_seen_frame (uint64_t)
 *
 * Guarantees:
 * - O(1) memory per cell.
 * - Memory footprint exactly sizeof(UniformCell) == 48 bytes.
 * - Transactional safety: failed numeric updates never partially update cell state or last_seen_frame.
 */
class UniformCell {
 public:
  constexpr UniformCell() noexcept = default;

  /**
   * @brief Attempts to update cell with an accepted elevation observation.
   *        Returns false if numeric arithmetic fails, leaving all cell state and
   *        last_seen_frame untouched.
   */
  bool tryUpdate(double z, uint64_t frame_seq) noexcept {
    if (!elevation_.tryUpdate(z)) {
      return false;
    }
    last_seen_frame_ = frame_seq;
    return true;
  }

  void update(double z, uint64_t frame_seq) noexcept {
    tryUpdate(z, frame_seq);
  }

  [[nodiscard]] constexpr uint64_t observationCount() const noexcept {
    return elevation_.count();
  }
  [[nodiscard]] constexpr double meanZ() const noexcept {
    return elevation_.mean();
  }
  [[nodiscard]] constexpr double m2Z() const noexcept {
    return elevation_.m2();
  }
  [[nodiscard]] constexpr double minZ() const noexcept {
    return elevation_.min();
  }
  [[nodiscard]] constexpr double maxZ() const noexcept {
    return elevation_.max();
  }
  [[nodiscard]] double populationVariance() const noexcept {
    return elevation_.populationVariance();
  }
  [[nodiscard]] double sampleVariance() const noexcept {
    return elevation_.sampleVariance();
  }
  [[nodiscard]] constexpr uint64_t lastSeenFrame() const noexcept {
    return last_seen_frame_;
  }
  [[nodiscard]] const ElevationStats& elevation() const noexcept {
    return elevation_;
  }

  void reset() noexcept {
    elevation_.reset();
    last_seen_frame_ = 0;
  }

  constexpr bool operator==(const UniformCell&) const noexcept = default;

 private:
  ElevationStats elevation_{};
  uint64_t last_seen_frame_{0};
};

}  // namespace avr::mapping

namespace AVR {
namespace mapping = ::avr::mapping;
}

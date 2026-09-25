#pragma once

#include <cstdint>

#include "avr_mapping/elevation_stats.hpp"

namespace avr::mapping {

/**
 * @brief Frame-local evidence cell storing aggregated elevation statistics.
 *
 * Contains only online Welford statistics for observations within this canonical L0 (5 cm) cell.
 * Does NOT store individual points or point history (strictly O(1) memory per cell).
 *
 * NOTE: EvidenceCell is temporary measurement support for a single logical sensor frame.
 * It is NOT persistent map state.
 */
class EvidenceCell {
 public:
  constexpr EvidenceCell() noexcept = default;

  [[nodiscard]] const ElevationStats& elevation() const noexcept {
    return elevation_;
  }

  [[nodiscard]] ElevationStats& elevation() noexcept {
    return elevation_;
  }

  [[nodiscard]] uint64_t count() const noexcept {
    return elevation_.count();
  }

  /**
   * @brief Transactionally attempts to ingest an elevation observation z.
   *
   * @param z Elevation in meters.
   * @return true if accepted and updated, false if rejected due to non-finite or extreme values.
   */
  bool update(double z) noexcept {
    return elevation_.tryUpdate(z);
  }

  /**
   * @brief Transactionally merges another EvidenceCell into this cell using parallel Welford.
   *
   * @param other Evidence cell to merge.
   * @return true if merged successfully, false if rejected due to arithmetic overflow.
   */
  bool merge(const EvidenceCell& other) noexcept {
    return elevation_.tryMerge(other.elevation_);
  }

  void reset() noexcept {
    elevation_.reset();
  }

  constexpr bool operator==(const EvidenceCell& other) const noexcept = default;

 private:
  ElevationStats elevation_{};
};

}  // namespace avr::mapping

namespace AVR {
namespace mapping = ::avr::mapping;
namespace mapping_core = ::avr::mapping;
}

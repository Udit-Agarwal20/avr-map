#pragma once

#include <array>
#include <bitset>
#include <cassert>
#include <cstdint>

#include "avr_mapping/evidence_cell.hpp"
#include "avr_spatial/constants.hpp"

namespace avr::mapping {

/**
 * @brief Fixed 16x16 cell tile containing exactly 256 Level 0 (5 cm) EvidenceCell slots.
 *
 * Manages a dense contiguous array of EvidenceCell plus bitset activity tracking.
 * Strictly zero heap allocation per cell update.
 *
 * NOTE: EvidenceTile is temporary measurement support for a single logical sensor frame.
 * It is NOT persistent map state, NOT UniformTile, and NOT AdaptiveTile.
 */
class EvidenceTile {
 public:
  static constexpr size_t CELL_COUNT = avr::spatial::TILE_CELL_COUNT;  // 256

  EvidenceTile() = default;

  [[nodiscard]] const EvidenceCell& cell(uint16_t local_idx) const noexcept {
    assert(local_idx < CELL_COUNT && "local_idx must be in [0, 255]");
    return cells_[local_idx];
  }

  [[nodiscard]] EvidenceCell& cell(uint16_t local_idx) noexcept {
    assert(local_idx < CELL_COUNT && "local_idx must be in [0, 255]");
    return cells_[local_idx];
  }

  [[nodiscard]] bool isCellActive(uint16_t local_idx) const noexcept {
    assert(local_idx < CELL_COUNT && "local_idx must be in [0, 255]");
    return active_mask_.test(local_idx);
  }

  [[nodiscard]] uint16_t activeCellCount() const noexcept {
    return active_count_;
  }

  [[nodiscard]] const std::bitset<CELL_COUNT>& activeMask() const noexcept {
    return active_mask_;
  }

  void activateCell(uint16_t local_idx) noexcept {
    assert(local_idx < CELL_COUNT && "local_idx must be in [0, 255]");
    if (!active_mask_.test(local_idx)) {
      active_mask_.set(local_idx);
      ++active_count_;
    }
  }

  /**
   * @brief Result of updating an evidence cell slot.
   */
  struct UpdateResult {
    bool success{false};
    bool became_active{false};
  };

  /**
   * @brief Ingests elevation z into the cell at local_idx.
   *
   * @param local_idx Local cell slot index [0, 255].
   * @param z Elevation in meters.
   * @return UpdateResult with success status and whether the cell transitioned from inactive to active.
   */
  UpdateResult updateCell(uint16_t local_idx, double z) noexcept {
    assert(local_idx < CELL_COUNT && "local_idx must be in [0, 255]");
    const bool was_active = active_mask_.test(local_idx);
    if (cells_[local_idx].update(z)) {
      if (!was_active) {
        active_mask_.set(local_idx);
        ++active_count_;
        return UpdateResult{.success = true, .became_active = true};
      }
      return UpdateResult{.success = true, .became_active = false};
    }
    return UpdateResult{.success = false, .became_active = false};
  }

  /**
   * @brief Resets all active cells and clears the activity mask.
   */
  void reset() noexcept {
    for (size_t i = 0; i < CELL_COUNT; ++i) {
      if (active_mask_.test(i)) {
        cells_[i].reset();
      }
    }
    active_mask_.reset();
    active_count_ = 0;
  }

  bool operator==(const EvidenceTile& other) const noexcept = default;

 private:
  std::array<EvidenceCell, CELL_COUNT> cells_{};
  std::bitset<CELL_COUNT> active_mask_{};
  uint16_t active_count_{0};
};

}  // namespace avr::mapping

namespace AVR {
namespace mapping = ::avr::mapping;
namespace mapping_core = ::avr::mapping;
}

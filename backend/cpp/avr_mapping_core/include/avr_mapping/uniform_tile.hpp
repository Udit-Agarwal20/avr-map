#pragma once

#include <array>
#include <bitset>
#include <cassert>
#include <cstdint>

#include "avr_mapping/uniform_cell.hpp"
#include "avr_spatial/constants.hpp"

namespace avr::mapping {

/**
 * @brief Fixed 16x16 cell tile containing exactly 256 L0 (5 cm) uniform cell slots.
 *
 * Manages dense contiguous array of UniformCell plus bitset activity tracking.
 * Does not allocate individual cells on heap.
 * Includes generation-tagged frame touch tracking for unique_cells_touched metrics without heap allocation.
 */
class UniformTile {
 public:
  static constexpr size_t CELL_COUNT = avr::spatial::TILE_CELL_COUNT;  // 256

  UniformTile() = default;

  [[nodiscard]] const UniformCell& cell(uint16_t local_idx) const noexcept {
    assert(local_idx < CELL_COUNT && "local_idx must be in [0, 255]");
    return cells_[local_idx];
  }

  [[nodiscard]] UniformCell& cell(uint16_t local_idx) noexcept {
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

  void setCell(uint16_t local_idx, const UniformCell& cell) noexcept {
    assert(local_idx < CELL_COUNT && "local_idx must be in [0, 255]");
    cells_[local_idx] = cell;
  }

  void activateCell(uint16_t local_idx) noexcept {
    assert(local_idx < CELL_COUNT && "local_idx must be in [0, 255]");
    if (!active_mask_.test(local_idx)) {
      active_mask_.set(local_idx);
      ++active_count_;
    }
  }

  /**
   * @brief Attempts numeric update of an active cell. Returns true if arithmetic succeeded.
   */
  bool tryUpdateCell(uint16_t local_idx, double z, uint64_t frame_seq) noexcept {
    assert(local_idx < CELL_COUNT && "local_idx must be in [0, 255]");
    return cells_[local_idx].tryUpdate(z, frame_seq);
  }

  /**
   * @brief Updates cell at local_idx with observation z and frame_seq.
   * @return true if the cell transitioned from inactive to active, false if already active or failed.
   */
  bool updateCell(uint16_t local_idx, double z, uint64_t frame_seq) noexcept {
    assert(local_idx < CELL_COUNT && "local_idx must be in [0, 255]");
    const bool is_new = !active_mask_.test(local_idx);
    if (cells_[local_idx].tryUpdate(z, frame_seq)) {
      if (is_new) {
        active_mask_.set(local_idx);
        ++active_count_;
      }
      return is_new;
    }
    return false;
  }

  /**
   * @brief Marks a cell slot as touched during the specified integration generation.
   * @return true if this was the first touch of this slot in this generation, false if already touched.
   */
  bool markCellTouchedInFrame(uint16_t local_idx, uint64_t generation) noexcept {
    assert(local_idx < CELL_COUNT && "local_idx must be in [0, 255]");
    if (last_touched_generation_ != generation) {
      last_touched_generation_ = generation;
      frame_touched_mask_.reset();
    }
    if (!frame_touched_mask_.test(local_idx)) {
      frame_touched_mask_.set(local_idx);
      return true;
    }
    return false;
  }

  void reset() noexcept {
    cells_.fill(UniformCell{});
    active_mask_.reset();
    active_count_ = 0;
    frame_touched_mask_.reset();
    last_touched_generation_ = 0;
  }

 private:
  std::array<UniformCell, CELL_COUNT> cells_{};
  std::bitset<CELL_COUNT> active_mask_{};
  std::bitset<CELL_COUNT> frame_touched_mask_{};
  uint64_t last_touched_generation_{0};
  uint16_t active_count_{0};
};

}  // namespace avr::mapping

namespace AVR {
namespace mapping = ::avr::mapping;
}

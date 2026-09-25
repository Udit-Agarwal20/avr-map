#pragma once

#include <array>
#include <bitset>
#include <cassert>
#include <cstdint>

#include "avr_mapping/adaptive_cell_payload.hpp"
#include "avr_spatial/constants.hpp"

namespace avr::mapping {

/**
 * @brief Fixed 16x16 cell tile holding persistent payloads and structural status masks.
 *
 * Each tile manages 256 hierarchy-node cell slots matching level-specific TileKey conventions.
 * Maintains compact bitsets to track cell lifecycle states:
 * - ACTIVE_PAYLOAD: Active measurement payload owned by an active hierarchy LEAF.
 * - RETIRED_PARENT_CACHE: Immutable historical summary owned by an INTERNAL node after split.
 * - REPLAY_REQUIRED: Active LEAF whose historical measurements could not be reconstructed
 *   from aggregate parent statistics alone and requires raw observation replay.
 * - RETIRED_CACHE_INCOMPLETE: Indicates a retired parent cache was itself historically incomplete
 *   (i.e. parent was already replay-required before split).
 *
 * Structural Invariants:
 * - (active_payload_mask & retired_cache_mask) == 0 (a cell is active LEAF OR retired parent cache, never both).
 * - retired_cache_incomplete_mask is a strict subset of retired_cache_mask.
 * - Zero heap allocations per cell.
 */
class AdaptivePayloadTile {
 public:
  static constexpr size_t CELL_COUNT = avr::spatial::TILE_CELL_COUNT;  // 256

  AdaptivePayloadTile() = default;

  [[nodiscard]] bool isActive(uint16_t local_idx) const noexcept {
    if (local_idx >= CELL_COUNT) [[unlikely]] {
      return false;
    }
    return active_payload_mask_.test(local_idx);
  }

  [[nodiscard]] bool isRetiredCache(uint16_t local_idx) const noexcept {
    if (local_idx >= CELL_COUNT) [[unlikely]] {
      return false;
    }
    return retired_cache_mask_.test(local_idx);
  }

  [[nodiscard]] bool isReplayRequired(uint16_t local_idx) const noexcept {
    if (local_idx >= CELL_COUNT) [[unlikely]] {
      return false;
    }
    return replay_required_mask_.test(local_idx);
  }

  [[nodiscard]] bool isRetiredCacheIncomplete(uint16_t local_idx) const noexcept {
    if (local_idx >= CELL_COUNT) [[unlikely]] {
      return false;
    }
    return retired_cache_incomplete_mask_.test(local_idx);
  }

  [[nodiscard]] const AdaptiveCellPayload& cell(uint16_t local_idx) const noexcept {
    assert(local_idx < CELL_COUNT && "AdaptivePayloadTile::cell index out of range");
    return cells_[local_idx];
  }

  [[nodiscard]] AdaptiveCellPayload& cell(uint16_t local_idx) noexcept {
    assert(local_idx < CELL_COUNT && "AdaptivePayloadTile::cell index out of range");
    return cells_[local_idx];
  }

  void setActivePayload(uint16_t local_idx, const AdaptiveCellPayload& payload) noexcept {
    if (local_idx >= CELL_COUNT) [[unlikely]] {
      return;
    }
    cells_[local_idx] = payload;
    active_payload_mask_.set(local_idx);
    retired_cache_mask_.reset(local_idx);
    retired_cache_incomplete_mask_.reset(local_idx);
  }

  void setRetiredCache(uint16_t local_idx, const AdaptiveCellPayload& payload, bool incomplete = false) noexcept {
    if (local_idx >= CELL_COUNT) [[unlikely]] {
      return;
    }
    cells_[local_idx] = payload;
    retired_cache_mask_.set(local_idx);
    active_payload_mask_.reset(local_idx);
    if (incomplete) {
      retired_cache_incomplete_mask_.set(local_idx);
    } else {
      retired_cache_incomplete_mask_.reset(local_idx);
    }
  }

  void setReplayRequired(uint16_t local_idx, bool required = true) noexcept {
    if (local_idx >= CELL_COUNT) [[unlikely]] {
      return;
    }
    if (required) {
      replay_required_mask_.set(local_idx);
    } else {
      replay_required_mask_.reset(local_idx);
    }
  }

  void clearCell(uint16_t local_idx) noexcept {
    if (local_idx >= CELL_COUNT) [[unlikely]] {
      return;
    }
    active_payload_mask_.reset(local_idx);
    retired_cache_mask_.reset(local_idx);
    replay_required_mask_.reset(local_idx);
    retired_cache_incomplete_mask_.reset(local_idx);
    cells_[local_idx] = AdaptiveCellPayload{};
  }

  [[nodiscard]] size_t activeCount() const noexcept {
    return active_payload_mask_.count();
  }

  [[nodiscard]] size_t retiredCacheCount() const noexcept {
    return retired_cache_mask_.count();
  }

  [[nodiscard]] size_t replayRequiredCount() const noexcept {
    return replay_required_mask_.count();
  }

  [[nodiscard]] size_t retiredCacheIncompleteCount() const noexcept {
    return retired_cache_incomplete_mask_.count();
  }

  [[nodiscard]] bool isEmpty() const noexcept {
    return active_payload_mask_.none() &&
           retired_cache_mask_.none() &&
           replay_required_mask_.none() &&
           retired_cache_incomplete_mask_.none();
  }

  [[nodiscard]] const std::bitset<CELL_COUNT>& activeMask() const noexcept {
    return active_payload_mask_;
  }

  [[nodiscard]] const std::bitset<CELL_COUNT>& retiredCacheMask() const noexcept {
    return retired_cache_mask_;
  }

  [[nodiscard]] const std::bitset<CELL_COUNT>& replayRequiredMask() const noexcept {
    return replay_required_mask_;
  }

  [[nodiscard]] const std::bitset<CELL_COUNT>& retiredCacheIncompleteMask() const noexcept {
    return retired_cache_incomplete_mask_;
  }

 private:
  std::array<AdaptiveCellPayload, CELL_COUNT> cells_{};
  std::bitset<CELL_COUNT> active_payload_mask_{};
  std::bitset<CELL_COUNT> retired_cache_mask_{};
  std::bitset<CELL_COUNT> replay_required_mask_{};
  std::bitset<CELL_COUNT> retired_cache_incomplete_mask_{};
};

}  // namespace avr::mapping

namespace AVR {
namespace mapping = ::avr::mapping;
}

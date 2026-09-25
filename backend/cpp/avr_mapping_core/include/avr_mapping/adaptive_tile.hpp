#pragma once

#include <bitset>
#include <cassert>
#include <cstdint>

#include "avr_spatial/constants.hpp"

namespace avr::mapping {

/**
 * @brief Structural state of an allocated hierarchy cell node.
 *
 * ABSENT: No node is allocated at this CellKey.
 * LEAF: Active map owner at this resolution.
 * INTERNAL: Not an active map owner; four direct children exist at the next finer level.
 */
enum class NodeState : uint8_t {
  ABSENT = 0,
  LEAF = 1,
  INTERNAL = 2,
};

/**
 * @brief Fixed 16x16 cell tile managing topological presence and leaf states.
 *
 * Each tile holds 256 cell slots representing a single level-specific TileKey.
 * Uses compact 256-bit bitsets for presence and leaf status:
 *   present = 0          -> ABSENT
 *   present = 1, leaf = 1 -> LEAF
 *   present = 1, leaf = 0 -> INTERNAL
 *
 * Invariant: leaf_mask & ~present_mask is always zero (leaf_mask is a subset of present_mask).
 * Does not allocate heap memory per cell node.
 * Local indices >= 256 are safely guarded against out-of-bounds access.
 */
class AdaptiveTile {
 public:
  static constexpr size_t CELL_COUNT = avr::spatial::TILE_CELL_COUNT;  // 256

  AdaptiveTile() = default;

  [[nodiscard]] NodeState nodeState(uint16_t local_idx) const noexcept {
    if (local_idx >= CELL_COUNT) [[unlikely]] {
      return NodeState::ABSENT;
    }
    if (!present_mask_.test(local_idx)) {
      return NodeState::ABSENT;
    }
    return leaf_mask_.test(local_idx) ? NodeState::LEAF : NodeState::INTERNAL;
  }

  [[nodiscard]] bool isPresent(uint16_t local_idx) const noexcept {
    if (local_idx >= CELL_COUNT) [[unlikely]] {
      return false;
    }
    return present_mask_.test(local_idx);
  }

  [[nodiscard]] bool isLeaf(uint16_t local_idx) const noexcept {
    if (local_idx >= CELL_COUNT) [[unlikely]] {
      return false;
    }
    return present_mask_.test(local_idx) && leaf_mask_.test(local_idx);
  }

  [[nodiscard]] bool isInternal(uint16_t local_idx) const noexcept {
    if (local_idx >= CELL_COUNT) [[unlikely]] {
      return false;
    }
    return present_mask_.test(local_idx) && !leaf_mask_.test(local_idx);
  }

  void setLeaf(uint16_t local_idx) noexcept {
    if (local_idx >= CELL_COUNT) [[unlikely]] {
      return;
    }
    if (!present_mask_.test(local_idx)) {
      present_mask_.set(local_idx);
      ++present_count_;
    }
    if (!leaf_mask_.test(local_idx)) {
      leaf_mask_.set(local_idx);
      ++leaf_count_;
    }
  }

  void setInternal(uint16_t local_idx) noexcept {
    if (local_idx >= CELL_COUNT) [[unlikely]] {
      return;
    }
    if (!present_mask_.test(local_idx)) {
      present_mask_.set(local_idx);
      ++present_count_;
    }
    if (leaf_mask_.test(local_idx)) {
      leaf_mask_.reset(local_idx);
      --leaf_count_;
    }
  }

  void eraseNode(uint16_t local_idx) noexcept {
    if (local_idx >= CELL_COUNT) [[unlikely]] {
      return;
    }
    if (present_mask_.test(local_idx)) {
      present_mask_.reset(local_idx);
      --present_count_;
      if (leaf_mask_.test(local_idx)) {
        leaf_mask_.reset(local_idx);
        --leaf_count_;
      }
    }
  }

  [[nodiscard]] bool empty() const noexcept {
    return present_count_ == 0;
  }

  void reset() noexcept {
    present_mask_.reset();
    leaf_mask_.reset();
    present_count_ = 0;
    leaf_count_ = 0;
  }

  [[nodiscard]] uint16_t presentCount() const noexcept {
    return present_count_;
  }

  [[nodiscard]] uint16_t leafCount() const noexcept {
    return leaf_count_;
  }

  [[nodiscard]] uint16_t internalCount() const noexcept {
    return static_cast<uint16_t>(present_count_ - leaf_count_);
  }

  [[nodiscard]] const std::bitset<CELL_COUNT>& presentMask() const noexcept {
    return present_mask_;
  }

  [[nodiscard]] const std::bitset<CELL_COUNT>& leafMask() const noexcept {
    return leaf_mask_;
  }

  [[nodiscard]] bool isAcquired() const noexcept {
    return is_acquired_;
  }

  void setAcquired(bool acquired) noexcept {
    is_acquired_ = acquired;
  }

  [[nodiscard]] bool validateInvariants() const noexcept {
    if ((leaf_mask_ & ~present_mask_).any()) {
      return false;
    }
    if (present_mask_.count() != present_count_) {
      return false;
    }
    if (leaf_mask_.count() != leaf_count_) {
      return false;
    }
    if (leaf_count_ > present_count_) {
      return false;
    }
    return true;
  }

 private:
  std::bitset<CELL_COUNT> present_mask_{};
  std::bitset<CELL_COUNT> leaf_mask_{};
  uint16_t present_count_{0};
  uint16_t leaf_count_{0};
  bool is_acquired_{false};
};

}  // namespace avr::mapping

namespace AVR {
namespace mapping = ::avr::mapping;
}

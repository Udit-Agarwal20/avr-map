#pragma once

#include <cassert>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

#include "avr_mapping/adaptive_tile.hpp"

namespace avr::mapping {

/**
 * @brief Block-allocated pool for AdaptiveTile memory reuse.
 *
 * Avoids per-mutation OS heap allocations by allocating tiles in contiguous blocks
 * and managing a reusable free list. Guarantees that AdaptiveTile pointer addresses
 * remain stable throughout their acquired lifetime.
 *
 * Memory & Ownership Contract:
 * - TilePool is explicitly non-copyable and non-movable.
 * - Double release of an acquired tile or releasing an unowned pointer is strictly rejected.
 * - Zero block capacity input is normalized to DEFAULT_BLOCK_CAPACITY.
 */
class TilePool {
 public:
  static constexpr size_t DEFAULT_BLOCK_CAPACITY = 64;

  explicit TilePool(size_t block_capacity = DEFAULT_BLOCK_CAPACITY)
      : block_capacity_(block_capacity > 0 ? block_capacity : DEFAULT_BLOCK_CAPACITY) {}

  ~TilePool() = default;

  TilePool(const TilePool&) = delete;
  TilePool& operator=(const TilePool&) = delete;
  TilePool(TilePool&&) = delete;
  TilePool& operator=(TilePool&&) = delete;

  /**
   * @brief Acquires a clean, reset AdaptiveTile from the pool.
   *
   * Contract:
   * - Returns a valid acquired tile pointer.
   * - If the free list is empty, a new block of block_capacity_ tiles is allocated.
   * - Acquired tiles are always reset and marked acquired before returning.
   */
  AdaptiveTile* acquireTile() {
    if (free_list_.empty()) {
      allocateBlock();
    }
    AdaptiveTile* tile = free_list_.back();
    free_list_.pop_back();
    tile->reset();
    tile->setAcquired(true);
    ++in_use_;
    return tile;
  }

  /**
   * @brief Releases an active AdaptiveTile back to the pool for reuse.
   *
   * Contract:
   * - Releasing ptr invalidates the caller's ownership of ptr.
   * - Callers must not access, read, or write to ptr after a successful release.
   * - A released tile may later be reused and returned by a subsequent acquireTile() call.
   * - Rejects nullptr, foreign pointers, and double-release of already-released tiles.
   * - Resets the tile state immediately upon release.
   * @return true if successfully released, false if rejected (unowned, nullptr, or double release).
   */
  bool releaseTile(AdaptiveTile* tile) noexcept {
    if (tile == nullptr) {
      return false;
    }
    if (!owns(tile)) {
      return false;
    }
    if (!tile->isAcquired()) {
      // Prevent double release and free list corruption
      return false;
    }
    tile->setAcquired(false);
    tile->reset();
    if (in_use_ > 0) {
      --in_use_;
    }
    free_list_.push_back(tile);
    return true;
  }

  /**
   * @brief Reclaims all acquired tiles back into the free list.
   *
   * Retains allocated block capacity for real-time reuse without releasing heap blocks.
   */
  void reset() noexcept {
    free_list_.clear();
    free_list_.reserve(capacity());
    for (auto& block : blocks_) {
      for (size_t i = 0; i < block_capacity_; ++i) {
        block[i].reset();
        block[i].setAcquired(false);
        free_list_.push_back(&block[i]);
      }
    }
    in_use_ = 0;
  }

  [[nodiscard]] size_t capacity() const noexcept {
    if (blocks_.empty() || block_capacity_ == 0) {
      return 0;
    }
    if (blocks_.size() > std::numeric_limits<size_t>::max() / block_capacity_) {
      return std::numeric_limits<size_t>::max();
    }
    return blocks_.size() * block_capacity_;
  }

  [[nodiscard]] size_t inUse() const noexcept {
    return in_use_;
  }

  [[nodiscard]] size_t freeCount() const noexcept {
    return free_list_.size();
  }

  [[nodiscard]] size_t blockCount() const noexcept {
    return blocks_.size();
  }

  [[nodiscard]] size_t blockSize() const noexcept {
    return block_capacity_;
  }

  /**
   * @brief Validates if a tile pointer belongs to one of the allocated blocks.
   *
   * Uses standards-defined pointer equality between object pointers to ensure 100% portable
   * ownership testing without relying on relational comparisons across unrelated heap allocations.
   */
  [[nodiscard]] bool owns(const AdaptiveTile* ptr) const noexcept {
    if (ptr == nullptr) {
      return false;
    }
    for (const auto& block : blocks_) {
      for (size_t i = 0; i < block_capacity_; ++i) {
        if (&block[i] == ptr) {
          return true;
        }
      }
    }
    return false;
  }

  /**
   * @brief Validates if a tile pointer belongs to this pool and is currently acquired.
   */
  [[nodiscard]] bool isAcquired(const AdaptiveTile* ptr) const noexcept {
    return owns(ptr) && ptr != nullptr && ptr->isAcquired();
  }

  [[nodiscard]] bool containsPointer(const AdaptiveTile* ptr) const noexcept {
    return owns(ptr);
  }

 private:
  void allocateBlock() {
    auto block = std::make_unique<AdaptiveTile[]>(block_capacity_);
    free_list_.reserve(free_list_.size() + block_capacity_);
    for (size_t i = 0; i < block_capacity_; ++i) {
      block[i].setAcquired(false);
      free_list_.push_back(&block[i]);
    }
    blocks_.push_back(std::move(block));
  }

  size_t block_capacity_{DEFAULT_BLOCK_CAPACITY};
  size_t in_use_{0};
  std::vector<std::unique_ptr<AdaptiveTile[]>> blocks_{};
  std::vector<AdaptiveTile*> free_list_{};
};

}  // namespace avr::mapping

namespace AVR {
namespace mapping = ::avr::mapping;
}

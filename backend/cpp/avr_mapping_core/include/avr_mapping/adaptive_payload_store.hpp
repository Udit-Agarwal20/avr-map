#pragma once

#include <optional>
#include <unordered_map>
#include <utility>
#include <vector>

#include "avr_mapping/adaptive_cell_payload.hpp"
#include "avr_mapping/adaptive_payload_tile.hpp"
#include "avr_mapping/persistent_map_stats.hpp"
#include "avr_spatial/cell_key.hpp"
#include "avr_spatial/tile_key.hpp"

namespace avr::mapping {

/**
 * @brief Sparse tile-based storage for persistent adaptive cell payloads.
 *
 * Implements M7 tiled persistent storage:
 * - Maps TileKey (16x16 nodes at level L0-L3) to AdaptivePayloadTile.
 * - Confines allocations to sparse tile insertions and rehashes (zero per-cell allocations).
 * - Distinguishes ACTIVE_PAYLOAD (owned by active leaves), RETIRED_PARENT_CACHE (owned by
 *   internal nodes after split), and REPLAY_REQUIRED state.
 * - Provides deterministic sorted snapshots ordered by CellKey.
 * - Cleans up logically empty tiles via pruneEmptyTiles().
 */
class AdaptivePayloadStore {
 public:
  AdaptivePayloadStore() = default;
  ~AdaptivePayloadStore() = default;

  AdaptivePayloadStore(const AdaptivePayloadStore&) = default;
  AdaptivePayloadStore& operator=(const AdaptivePayloadStore&) = default;
  AdaptivePayloadStore(AdaptivePayloadStore&&) noexcept = default;
  AdaptivePayloadStore& operator=(AdaptivePayloadStore&&) noexcept = default;

  /**
   * @brief Queries active measurement payload for an active hierarchy LEAF.
   *        Returns nullptr if the node is absent or not active.
   */
  [[nodiscard]] const AdaptiveCellPayload* findActivePayload(avr::spatial::CellKey key) const noexcept;

  /**
   * @brief Queries immutable historical cache for an INTERNAL hierarchy node.
   *        Returns nullptr if the node has no retired cache.
   */
  [[nodiscard]] const AdaptiveCellPayload* findRetiredCache(avr::spatial::CellKey key) const noexcept;

  /**
   * @brief Returns true if this active leaf is marked as requiring raw observation replay.
   */
  [[nodiscard]] bool requiresReplay(avr::spatial::CellKey key) const noexcept;

  /**
   * @brief Returns true if this retired parent cache was historically incomplete upon split.
   */
  [[nodiscard]] bool isRetiredCacheIncomplete(avr::spatial::CellKey key) const noexcept;

  /**
   * @brief Sets or updates active payload for a leaf cell.
   */
  void setActivePayload(avr::spatial::CellKey key, const AdaptiveCellPayload& payload);

  /**
   * @brief Transitions active payload to immutable retired cache upon parent split.
   */
  void setRetiredCache(avr::spatial::CellKey key, const AdaptiveCellPayload& payload, bool incomplete = false);

  /**
   * @brief Sets or clears the replay-required flag for a cell.
   */
  void setReplayRequired(avr::spatial::CellKey key, bool required = true);

  /**
   * @brief Clears all payload, cache, and replay metadata for a cell.
   */
  void clearCell(avr::spatial::CellKey key);

  /**
   * @brief Removes logically empty tiles from the sparse container.
   */
  void pruneEmptyTiles() noexcept;

  /**
   * @brief Logically clears all tiles. Container bucket capacity may remain allocated.
   */
  void reset() noexcept;

  [[nodiscard]] size_t tileCount() const noexcept {
    return tiles_.size();
  }

  [[nodiscard]] bool isEmpty() const noexcept {
    return tiles_.empty();
  }

  [[nodiscard]] PayloadStoreStats statistics() const noexcept;

  /**
   * @brief Returns deterministically sorted snapshot of all active payloads (by CellKey).
   */
  [[nodiscard]] std::vector<std::pair<avr::spatial::CellKey, AdaptiveCellPayload>> activePayloadSnapshot() const;

  /**
   * @brief Returns deterministically sorted snapshot of all retired parent caches (by CellKey).
   */
  [[nodiscard]] std::vector<std::pair<avr::spatial::CellKey, AdaptiveCellPayload>> retiredCacheSnapshot() const;

  /**
   * @brief Returns deterministically sorted snapshot of all leaves marked REPLAY_REQUIRED.
   */
  [[nodiscard]] std::vector<avr::spatial::CellKey> replayRequiredSnapshot() const;

  /**
   * @brief Returns deterministically sorted snapshot of all retired caches marked incomplete.
   */
  [[nodiscard]] std::vector<avr::spatial::CellKey> retiredCacheIncompleteSnapshot() const;

  [[nodiscard]] const std::unordered_map<avr::spatial::TileKey, AdaptivePayloadTile>& tiles() const noexcept {
    return tiles_;
  }

 private:
  std::unordered_map<avr::spatial::TileKey, AdaptivePayloadTile> tiles_{};
};

}  // namespace avr::mapping

namespace AVR {
namespace mapping = ::avr::mapping;
}

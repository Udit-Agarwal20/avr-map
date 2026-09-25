#pragma once

#include <array>
#include <optional>
#include <unordered_map>
#include <vector>

#include "avr_mapping/adaptive_tile.hpp"
#include "avr_mapping/hierarchy_stats.hpp"
#include "avr_mapping/tile_pool.hpp"
#include "avr_spatial/cell_key.hpp"
#include "avr_spatial/coordinate.hpp"
#include "avr_spatial/tile_key.hpp"

namespace avr::mapping {

/**
 * @brief Sparse Dyadic Adaptive Hierarchy (L0=5cm, L1=10cm, L2=20cm, L3=40cm).
 *
 * Implements mixed-resolution active leaf spatial partitioning.
 *
 * Key Invariants:
 * - Exactly one active leaf owns each location within allocated root coverage.
 * - Sparse allocation roots exist exclusively at L3 (40 cm).
 * - A parent node is active LEAF OR its four direct children are active.
 * - Non-recursive merge: direct children must all be LEAF to merge.
 * - No heuristic/decision policy (distance, semantics, dynamic objects) belongs here;
 *   this class manages structural topology only.
 *
 * Structural Allocation Contract:
 * - Leaf lookup (findLeafContainingBase): zero heap allocation.
 * - Node mutation inside already allocated hierarchy tiles: zero per-node heap allocation.
 * - Structural allocations are confined to:
 *     1. TilePool block growth when all existing free tiles are exhausted.
 *     2. TileStore unordered_map bucket node creation and table rehashing.
 *     3. Internal vector growth during snapshot retrieval.
 *
 * Threading & Lifecycle Contract:
 * - AdaptiveHierarchy is NOT internally multi-writer thread-safe.
 * - Architecture specifies one dedicated map worker thread as the single topology writer.
 * - Explicitly non-copyable and non-movable to preserve absolute reference safety.
 */
class AdaptiveHierarchy {
 public:
  explicit AdaptiveHierarchy(size_t pool_block_capacity = TilePool::DEFAULT_BLOCK_CAPACITY);
  ~AdaptiveHierarchy();

  AdaptiveHierarchy(const AdaptiveHierarchy&) = delete;
  AdaptiveHierarchy& operator=(const AdaptiveHierarchy&) = delete;
  AdaptiveHierarchy(AdaptiveHierarchy&&) = delete;
  AdaptiveHierarchy& operator=(AdaptiveHierarchy&&) = delete;

  /**
   * @brief Ensures an L3 root exists covering the given base-grid coordinate.
   *
   * If an L3 root already exists (as LEAF or INTERNAL), returns its CellKey.
   * Otherwise allocates an L3 LEAF root and returns its CellKey.
   */
  std::optional<avr::spatial::CellKey> ensureRootForBase(const avr::spatial::GridCoord2D& base);

  /**
   * @brief Returns true if an L3 root has been allocated covering the given base coordinate.
   */
  [[nodiscard]] bool hasRootForBase(const avr::spatial::GridCoord2D& base) const noexcept;

  /**
   * @brief Returns the L3 CellKey covering the given base coordinate regardless of allocation.
   */
  [[nodiscard]] static std::optional<avr::spatial::CellKey> rootKeyForBase(
      const avr::spatial::GridCoord2D& base) noexcept;

  /**
   * @brief Finds the active LEAF cell key owning the given base-grid coordinate.
   *
   * Returns std::nullopt if the coordinate lies outside all allocated L3 root footprints.
   * Zero heap allocations required for lookup.
   */
  [[nodiscard]] std::optional<avr::spatial::CellKey> findLeafContainingBase(
      const avr::spatial::GridCoord2D& base) const noexcept;

  /**
   * @brief Queries the structural state (ABSENT, LEAF, INTERNAL) of a CellKey.
   */
  [[nodiscard]] NodeState nodeState(const avr::spatial::CellKey& key) const noexcept;

  [[nodiscard]] bool isLeaf(const avr::spatial::CellKey& key) const noexcept {
    return nodeState(key) == NodeState::LEAF;
  }

  [[nodiscard]] bool isInternal(const avr::spatial::CellKey& key) const noexcept {
    return nodeState(key) == NodeState::INTERNAL;
  }

  [[nodiscard]] bool isAllocated(const avr::spatial::CellKey& key) const noexcept {
    return nodeState(key) != NodeState::ABSENT;
  }

  /**
   * @brief Splits an active LEAF node into four direct children at level - 1.
   *
   * Requirements:
   * - parent must exist, be LEAF, and have level > L0.
   * - children must not already exist in the hierarchy.
   *
   * On success: parent becomes INTERNAL; four children are created as LEAF.
   * On failure: returns false with zero hierarchy mutation.
   */
  bool splitLeaf(const avr::spatial::CellKey& parent_key);

  /**
   * @brief Merges four direct leaf children into their parent node.
   *
   * Requirements:
   * - parent must exist, be INTERNAL, and have level > L0.
   * - all four direct children must exist and currently be LEAF.
   * - Non-recursive: if any direct child is INTERNAL, merge is rejected.
   *
   * On success: four children are erased; parent becomes LEAF.
   * If a child tile becomes empty, it is released back to TilePool.
   * On failure: returns false with zero hierarchy mutation.
   */
  bool mergeChildren(const avr::spatial::CellKey& parent_key);

  /**
   * @brief Refines the hierarchy at the given base coordinate down to requested_level.
   *
   * Ensures root exists, locates the owning leaf, and repeatedly splits only the owning
   * ancestor path until requested_level is reached.
   * If the current owning leaf is already at or finer than requested_level, does not coarsen.
   *
   * Transaction and Failure Contract:
   * - If requested_level is invalid (>= 4), rejected immediately with zero hierarchy mutation.
   * - If base grid coordinates are unrepresentable, rejected immediately with zero hierarchy mutation.
   * - Under a valid topology, once root and owning leaf are resolved, every subsequent splitLeaf
   *   along the path to requested_level is guaranteed to succeed barring system std::bad_alloc.
   * - Therefore, refineToLevel either reaches requested_level or does not mutate topology.
   */
  std::optional<avr::spatial::CellKey> refineToLevel(
      const avr::spatial::GridCoord2D& base,
      uint8_t requested_level);

  /**
   * @brief Validates all structural topology and tiling invariants.
   */
  [[nodiscard]] TopologyValidationResult validateInvariants() const;

  /**
   * @brief Returns exact structural statistics and memory payload facts.
   */
  [[nodiscard]] HierarchyStats statistics() const noexcept;

  [[nodiscard]] const TilePool& tilePool() const noexcept {
    return tile_pool_;
  }

  [[nodiscard]] size_t tileCount() const noexcept {
    return tiles_.size();
  }

  [[nodiscard]] size_t nodeCount() const noexcept {
    return node_count_;
  }

  [[nodiscard]] size_t leafCount() const noexcept {
    return leaf_count_;
  }

  [[nodiscard]] size_t internalCount() const noexcept {
    return internal_count_;
  }

  /**
   * @brief Returns sorted vector of CellKeys for all active LEAF nodes.
   */
  [[nodiscard]] std::vector<avr::spatial::CellKey> leafSnapshot() const;

  /**
   * @brief Returns sorted vector of all allocated hierarchy nodes with their states.
   */
  [[nodiscard]] std::vector<TopologyRecord> topologySnapshot() const;

  /**
   * @brief Clears all logical hierarchy nodes and tiles.
   *
   * Reclaims all tiles to TilePool while retaining allocated pool block capacity for reuse.
   */
  void reset() noexcept;

 private:
  [[nodiscard]] const AdaptiveTile* findTile(const avr::spatial::TileKey& tile_key) const noexcept;
  [[nodiscard]] AdaptiveTile* findTile(const avr::spatial::TileKey& tile_key) noexcept;

  TilePool tile_pool_;
  std::unordered_map<avr::spatial::TileKey, AdaptiveTile*> tiles_{};

  size_t node_count_{0};
  size_t leaf_count_{0};
  size_t internal_count_{0};

  std::array<size_t, 4> leaf_count_by_level_{0, 0, 0, 0};
  std::array<size_t, 4> internal_count_by_level_{0, 0, 0, 0};
};

}  // namespace avr::mapping

namespace AVR {
namespace mapping = ::avr::mapping;
}

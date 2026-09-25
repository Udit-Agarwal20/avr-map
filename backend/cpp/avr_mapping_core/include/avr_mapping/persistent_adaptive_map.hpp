#pragma once

#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

#include "avr_mapping/adaptive_cell_payload.hpp"
#include "avr_mapping/adaptive_hierarchy.hpp"
#include "avr_mapping/adaptive_payload_store.hpp"
#include "avr_mapping/frame_evidence.hpp"
#include "avr_mapping/persistent_map_stats.hpp"
#include "avr_mapping/resolution_decision.hpp"
#include "avr_spatial/cell_key.hpp"
#include "avr_spatial/coordinate.hpp"

namespace avr::mapping {

/**
 * @brief Persistent multi-frame adaptive map layer coordinating hierarchy and payload state.
 *
 * Implements M7 persistent adaptive mapping foundation:
 * - Owns structural topology (AdaptiveHierarchy) and measurement state (AdaptivePayloadStore).
 * - Guarantees strict single-writer thread confinement: non-copyable and non-movable.
 * - Coordinated refinement (refineToLevel / splitLeaf): structural quadtree split transitions
 *   active parent payload to immutable RETIRED_PARENT_CACHE, leaves children with NO fabricated
 *   aggregate statistics, and marks children as REPLAY_REQUIRED.
 * - Multi-frame elevation fusion: merges FrameEvidence into active leaf payloads using
 *   transactional two-phase PREPARE/COMMIT. Rejects duplicate or out-of-order frame sequences.
 * - Enforces active-leaf-only fusion: INTERNAL nodes never receive observation updates.
 * - Verifies observation conservation: sum of leaf observations equals FrameEvidence.accepted_points.
 * - Cross-store invariant validation via validateInvariants().
 */
class PersistentAdaptiveMap {
 public:
  explicit PersistentAdaptiveMap(size_t pool_block_capacity = TilePool::DEFAULT_BLOCK_CAPACITY);
  ~PersistentAdaptiveMap() = default;

  PersistentAdaptiveMap(const PersistentAdaptiveMap&) = delete;
  PersistentAdaptiveMap& operator=(const PersistentAdaptiveMap&) = delete;
  PersistentAdaptiveMap(PersistentAdaptiveMap&&) = delete;
  PersistentAdaptiveMap& operator=(PersistentAdaptiveMap&&) = delete;

  /**
   * @brief Ensures a coarse L3 root exists covering base grid coordinates.
   */
  std::optional<avr::spatial::CellKey> ensureRootForBase(const avr::spatial::GridCoord2D& base);

  /**
   * @brief Coordinates structural split of an active leaf and transitions its payload state.
   *
   * Transition Rules:
   * - If parent had ACTIVE_PAYLOAD:
   *     1. parent becomes INTERNAL.
   *     2. parent active payload moves to RETIRED_PARENT_CACHE (immutable).
   *     3. if parent was REPLAY_REQUIRED, cache is marked incomplete.
   *     4. all four children are marked REPLAY_REQUIRED with no fabricated payload.
   * - If parent had NO active payload:
   *     - if parent was REPLAY_REQUIRED, all four children become REPLAY_REQUIRED.
   *     - if parent was NOT replay-required, children are NOT replay-required.
   *
   * @param parent_key CellKey of the active LEAF to split (must have level > L0).
   * @param reasons Decision reasons associated with the split.
   * @return true if split succeeded, false if node is invalid, not a leaf, or out of tiles.
   */
  bool splitLeaf(
      const avr::spatial::CellKey& parent_key,
      ResolutionReasonBits reasons = ResolutionReasonBits{});

  /**
   * @brief Coordinates multi-level refinement along the owning ancestor path to target_level [0, 3].
   */
  bool refineToLevel(
      const avr::spatial::GridCoord2D& base,
      uint8_t target_level,
      ResolutionReasonBits reasons = ResolutionReasonBits{});

  /**
   * @brief Transactionally fuses current FrameEvidence into active persistent leaves.
   *
   * Uses two-phase PREPARE / COMMIT:
   * - Validates frame sequencing (frame_seq > last_fused_frame_seq).
   * - Maps active base cells to current owning leaves (or would-be L3 roots).
   * - Validates candidate fusion for each unique leaf.
   * - Verifies observation conservation against evidence.accepted_points.
   * - Creates missing L3 roots and commits payload updates atomically.
   *
   * @param evidence Valid FrameEvidence from sensor frame.
   * @param reasons Decision context for the current frame update.
   * @return Populated PersistentFusionStats on success, std::nullopt on any failure.
   */
  std::optional<PersistentFusionStats> fuseFrameEvidence(
      const FrameEvidence& evidence,
      ResolutionReasonBits reasons = ResolutionReasonBits{});

  /**
   * @brief Validates all cross-subsystem structural and payload invariants.
   */
  [[nodiscard]] PersistentMapValidationResult validateInvariants() const;

  /**
   * @brief Returns unified statistics for hierarchy, payload store, and frame sequencing.
   */
  [[nodiscard]] PersistentMapStats statistics() const noexcept;

  // Read-only inspection
  [[nodiscard]] const AdaptiveHierarchy& hierarchy() const noexcept {
    return hierarchy_;
  }

  [[nodiscard]] const AdaptivePayloadStore& payloadStore() const noexcept {
    return payload_store_;
  }

  [[nodiscard]] uint64_t lastFusedFrameSeq() const noexcept {
    return last_fused_frame_seq_;
  }

  [[nodiscard]] bool hasFusedFrames() const noexcept {
    return has_fused_frames_;
  }

  [[nodiscard]] const AdaptiveCellPayload* findActivePayload(avr::spatial::CellKey key) const noexcept {
    return payload_store_.findActivePayload(key);
  }

  [[nodiscard]] const AdaptiveCellPayload* findRetiredCache(avr::spatial::CellKey key) const noexcept {
    return payload_store_.findRetiredCache(key);
  }

  [[nodiscard]] bool requiresReplay(avr::spatial::CellKey key) const noexcept {
    return payload_store_.requiresReplay(key);
  }

  [[nodiscard]] bool isRetiredCacheIncomplete(avr::spatial::CellKey key) const noexcept {
    return payload_store_.isRetiredCacheIncomplete(key);
  }

  [[nodiscard]] std::vector<std::pair<avr::spatial::CellKey, AdaptiveCellPayload>> activePayloadSnapshot() const {
    return payload_store_.activePayloadSnapshot();
  }

  [[nodiscard]] std::vector<std::pair<avr::spatial::CellKey, AdaptiveCellPayload>> retiredCacheSnapshot() const {
    return payload_store_.retiredCacheSnapshot();
  }

  [[nodiscard]] std::vector<avr::spatial::CellKey> replayRequiredSnapshot() const {
    return payload_store_.replayRequiredSnapshot();
  }

  [[nodiscard]] std::vector<avr::spatial::CellKey> retiredCacheIncompleteSnapshot() const {
    return payload_store_.retiredCacheIncompleteSnapshot();
  }

  /**
   * @brief Clears all hierarchy topology, payload tiles, and sequencing state.
   *        Allocated container and pool block capacities remain reserved for reuse.
   */
  void reset() noexcept;

 private:
  friend struct PersistentAdaptiveMapTestAccess;

  AdaptiveHierarchy hierarchy_;
  AdaptivePayloadStore payload_store_;
  uint64_t last_fused_frame_seq_{0};
  bool has_fused_frames_{false};
  uint64_t total_frames_fused_{0};
  uint64_t total_observations_fused_{0};
};

}  // namespace avr::mapping

namespace AVR {
namespace mapping = ::avr::mapping;
}

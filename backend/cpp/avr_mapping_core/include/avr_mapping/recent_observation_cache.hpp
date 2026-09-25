#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>
#include <span>
#include <vector>

#include "avr_mapping/cached_observation_frame.hpp"
#include "avr_mapping/replay_cache_stats.hpp"

namespace avr::mapping {

/**
 * @brief Bounded FIFO cache of recent raw observation frames for future split replay.
 *
 * Characteristics:
 * - Retains up to max_frames recent sensor frames (canonical prototype default: 3).
 * - Enforces strictly increasing frame_seq (rejects duplicate and out-of-order frames).
 * - Two-phase push: prepares new frame in temporary state; commits and evicts oldest only on success.
 * - Coarse L3 spatial bucket indexing by point indices (zero duplicate raw point copies).
 * - Explicit non-copyable, movable ownership.
 * - Pure replay data source: does NOT mutate PersistentAdaptiveMap or AdaptiveHierarchy.
 */
class RecentObservationCache {
 public:
  using PointIndex = CachedObservationFrame::PointIndex;

  /**
   * @brief Factory constructor that validates configuration before instantiation.
   *
   * @param config Configuration parameters (requires max_frames >= 1).
   * @return Populated RecentObservationCache, or std::nullopt if config is invalid.
   */
  [[nodiscard]] static std::optional<RecentObservationCache> create(
      ReplayCacheConfig config = ReplayCacheConfig{}) noexcept;

  ~RecentObservationCache() = default;

  RecentObservationCache(const RecentObservationCache&) = delete;
  RecentObservationCache& operator=(const RecentObservationCache&) = delete;

  RecentObservationCache(RecentObservationCache&&) noexcept = default;
  RecentObservationCache& operator=(RecentObservationCache&&) noexcept = default;

  /**
   * @brief Transactionally pushes a new raw observation frame into the cache.
   *
   * @param raw_points Raw sensor input points.
   * @param frame_seq Global sequence number (must be > newestFrameSeq).
   * @param sensor_origin Sensor position in world coordinates (must be finite).
   * @return ReplayCacheFrameStats on successful commit, std::nullopt on any validation failure.
   */
  std::optional<ReplayCacheFrameStats> pushFrame(
      std::span<const PointXYZ> raw_points,
      uint64_t frame_seq,
      avr::spatial::WorldPoint2D sensor_origin);

  /**
   * @brief Finds and returns a non-owning pointer to the cached observation frame.
   *
   * LIFETIME & OWNERSHIP CONTRACT:
   * The returned pointer is borrowed read-only access. Caller must not retain or use it
   * across any mutating operation on this cache that could evict or invalidate frames,
   * including:
   * - A subsequent pushFrame() call that triggers FIFO eviction;
   * - reset();
   * - Destruction of RecentObservationCache.
   * No long-term pointer stability is guaranteed.
   *
   * @param frame_seq Global frame sequence number to find.
   * @return Const pointer to CachedObservationFrame, or nullptr if absent.
   */
  [[nodiscard]] const CachedObservationFrame* findFrame(uint64_t frame_seq) const noexcept;

  [[nodiscard]] bool containsFrame(uint64_t frame_seq) const noexcept;

  [[nodiscard]] size_t frameCount() const noexcept {
    return frames_.size();
  }

  [[nodiscard]] std::optional<uint64_t> oldestFrameSeq() const noexcept;

  [[nodiscard]] std::optional<uint64_t> newestFrameSeq() const noexcept;

  [[nodiscard]] std::vector<uint64_t> frameSequenceSnapshot() const;

  [[nodiscard]] std::vector<avr::spatial::CellKey> bucketKeySnapshot(uint64_t frame_seq) const;

  /**
   * @brief Queries matching point indices in a given cached frame for an L0-L3 CellKey.
   *
   * QUERY CONTRACT:
   * - output is unconditionally cleared before any operation.
   * - If frame_seq is not retained in the cache: returns false, output is empty.
   * - If region level is invalid (> L3) or ancestor navigation fails: returns false, output is empty.
   * - If region is valid but no observations fall within its containing L3 bucket:
   *   returns true, output is empty (valid query with zero matching observations).
   * - Matching point indices are strictly ascending (output[i] < output[i+1]).
   *
   * @param frame_seq Target cached frame sequence number.
   * @param region Valid L0-L3 CellKey region.
   * @param output Vector to receive matching indices in original observation arrival order.
   * @return true if query is valid, false on query failure.
   */
  bool collectObservationIndices(
      uint64_t frame_seq,
      const avr::spatial::CellKey& region,
      std::vector<PointIndex>& output) const;

  [[nodiscard]] ReplayCacheStats statistics() const noexcept;

  [[nodiscard]] const ReplayCacheConfig& config() const noexcept {
    return config_;
  }

  void reset() noexcept;

 private:
  explicit RecentObservationCache(ReplayCacheConfig config) noexcept;

  ReplayCacheConfig config_{};
  std::deque<CachedObservationFrame> frames_{};
};

}  // namespace avr::mapping

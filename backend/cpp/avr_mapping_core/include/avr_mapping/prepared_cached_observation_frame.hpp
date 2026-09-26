#pragma once

#include <cstdint>
#include <optional>
#include <type_traits>
#include <utility>

#include "avr_mapping/cached_observation_frame.hpp"
#include "avr_mapping/replay_cache_stats.hpp"
#include "avr_spatial/coordinate.hpp"

namespace avr::mapping {

/**
 * @brief Move-only prepared raw sensor observation frame for staged cache commit.
 *
 * Characteristics:
 * - Owns all raw observations, coarse L3 spatial index, sensor origin, and frame metrics.
 * - Retains zero pointers or references into caller input memory or RecentObservationCache.
 * - Move-only semantics (non-copyable) to prevent accidental raw point vector duplicates.
 * - Independent lifecycle: caller may prepare frame, execute arbitrary read-only queries or
 *   historical replay against RecentObservationCache, and subsequently commit or discard.
 * - Discarding an uncommitted prepared frame is completely side-effect-free on RecentObservationCache.
 */
class PreparedCachedObservationFrame {
 public:
  PreparedCachedObservationFrame() = default;
  ~PreparedCachedObservationFrame() = default;

  // Non-copyable, move-only
  PreparedCachedObservationFrame(const PreparedCachedObservationFrame&) = delete;
  PreparedCachedObservationFrame& operator=(const PreparedCachedObservationFrame&) = delete;

  PreparedCachedObservationFrame(PreparedCachedObservationFrame&&) noexcept = default;
  PreparedCachedObservationFrame& operator=(PreparedCachedObservationFrame&&) noexcept = default;

  explicit PreparedCachedObservationFrame(CachedObservationFrame frame) noexcept
      : frame_(std::move(frame)) {}

  /**
   * @brief Returns true if this object holds a valid, unextracted prepared frame.
   */
  [[nodiscard]] bool isValid() const noexcept {
    return frame_.has_value();
  }

  /**
   * @brief Global frame sequence number of the prepared frame (0 if invalid/moved-from).
   */
  [[nodiscard]] uint64_t frameSeq() const noexcept {
    return frame_.has_value() ? frame_->frameSeq() : 0;
  }

  /**
   * @brief Sensor origin in continuous world coordinates.
   */
  [[nodiscard]] avr::spatial::WorldPoint2D sensorOrigin() const noexcept {
    return frame_.has_value() ? frame_->sensorOrigin() : avr::spatial::WorldPoint2D{0.0, 0.0};
  }

  /**
   * @brief Pre-calculated frame-local caching metrics.
   */
  [[nodiscard]] const ReplayCacheFrameStats& statistics() const noexcept {
    static const ReplayCacheFrameStats kEmptyStats{};
    return frame_.has_value() ? frame_->statistics() : kEmptyStats;
  }

  /**
   * @brief Read-only view of the underlying CachedObservationFrame.
   */
  [[nodiscard]] const CachedObservationFrame& frame() const noexcept {
    return *frame_;
  }

  /**
   * @brief Extracts the underlying CachedObservationFrame, invalidating this prepared wrapper.
   */
  [[nodiscard]] CachedObservationFrame extractFrame() && noexcept {
    auto frame = std::move(*frame_);
    frame_.reset();
    return frame;
  }

 private:
  std::optional<CachedObservationFrame> frame_{std::nullopt};
};

static_assert(!std::is_copy_constructible_v<PreparedCachedObservationFrame>);
static_assert(!std::is_copy_assignable_v<PreparedCachedObservationFrame>);
static_assert(std::is_move_constructible_v<PreparedCachedObservationFrame>);
static_assert(std::is_move_assignable_v<PreparedCachedObservationFrame>);

}  // namespace avr::mapping

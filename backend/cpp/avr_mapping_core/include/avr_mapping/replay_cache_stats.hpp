#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>

namespace avr::mapping {

/**
 * @brief Overflow-safe arithmetic helper for saturating multiplication on size_t.
 */
inline constexpr size_t saturatingMultiply(size_t a, size_t b) noexcept {
  if (a == 0 || b == 0) {
    return 0;
  }
  if (a > std::numeric_limits<size_t>::max() / b) {
    return std::numeric_limits<size_t>::max();
  }
  return a * b;
}

/**
 * @brief Overflow-safe arithmetic helper for saturating addition on size_t.
 */
inline constexpr size_t saturatingAdd(size_t a, size_t b) noexcept {
  if (std::numeric_limits<size_t>::max() - a < b) {
    return std::numeric_limits<size_t>::max();
  }
  return a + b;
}

/**
 * @brief Overflow-safe arithmetic helper for saturating addition on uint64_t.
 */
inline constexpr uint64_t saturatingAddU64(uint64_t a, uint64_t b) noexcept {
  if (std::numeric_limits<uint64_t>::max() - a < b) {
    return std::numeric_limits<uint64_t>::max();
  }
  return a + b;
}

/**
 * @brief Configuration settings for RecentObservationCache.
 */
struct ReplayCacheConfig {
  size_t max_frames{3};  ///< Canonical prototype default: 3 frames (configurable, not claimed optimal).

  [[nodiscard]] constexpr bool isValid() const noexcept {
    return max_frames >= 1;
  }

  constexpr bool operator==(const ReplayCacheConfig&) const noexcept = default;
  constexpr bool operator!=(const ReplayCacheConfig&) const noexcept = default;
};

/**
 * @brief Frame-local caching and indexing metrics for a single raw sensor frame.
 */
struct ReplayCacheFrameStats {
  uint64_t frame_seq{0};
  uint64_t input_points{0};
  uint64_t replay_candidates{0};
  uint64_t non_finite_unindexed{0};
  uint64_t out_of_range_unindexed{0};
  size_t bucket_count{0};
  uint64_t point_indices_count{0};
  size_t raw_point_bytes{0};
  size_t index_bytes{0};

  constexpr bool operator==(const ReplayCacheFrameStats&) const noexcept = default;
  constexpr bool operator!=(const ReplayCacheFrameStats&) const noexcept = default;
};

/**
 * @brief Structural statistics and payload accounting across RecentObservationCache.
 *
 * All byte payload calculations saturate at std::numeric_limits<size_t>::max()
 * to prevent unchecked integer overflow in public telemetry.
 */
struct ReplayCacheStats {
  size_t max_frames{0};
  size_t stored_frames{0};
  uint64_t stored_raw_points{0};
  uint64_t stored_replay_candidates{0};
  size_t stored_bucket_count{0};
  uint64_t stored_point_indices{0};
  std::optional<uint64_t> oldest_frame_seq{std::nullopt};
  std::optional<uint64_t> newest_frame_seq{std::nullopt};
  size_t raw_point_payload_bytes{0};
  size_t index_payload_bytes{0};

  constexpr bool operator==(const ReplayCacheStats&) const noexcept = default;
  constexpr bool operator!=(const ReplayCacheStats&) const noexcept = default;
};

}  // namespace avr::mapping


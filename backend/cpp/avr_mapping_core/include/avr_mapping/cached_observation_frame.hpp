#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <span>
#include <unordered_map>
#include <vector>

#include "avr_mapping/point_xyz.hpp"
#include "avr_mapping/replay_cache_stats.hpp"
#include "avr_spatial/bounds.hpp"
#include "avr_spatial/cell_key.hpp"
#include "avr_spatial/constants.hpp"
#include "avr_spatial/coordinate.hpp"

namespace avr::mapping {

/**
 * @brief Immutable cached record of a single raw sensor observation frame.
 *
 * Retains:
 * - Exact original raw PointXYZ sequence in original order (including invalid points).
 * - Coarse L3 (40 cm) spatial bucket index storing integer point indices into raw_points.
 * - Sensor origin in world coordinates.
 * - Frame-local caching metrics.
 */
class CachedObservationFrame {
 public:
  using PointIndex = uint32_t;

  CachedObservationFrame(const CachedObservationFrame&) = delete;
  CachedObservationFrame& operator=(const CachedObservationFrame&) = delete;

  CachedObservationFrame(CachedObservationFrame&&) noexcept = default;
  CachedObservationFrame& operator=(CachedObservationFrame&&) noexcept = default;

  ~CachedObservationFrame() = default;

  /**
   * @brief Factory constructor that prepares and validates a frame record.
   *
   * @param raw_points Span of input points in original sensor acquisition order.
   * @param frame_seq Global monotonic frame sequence number.
   * @param sensor_origin Sensor position in world coordinates (must be finite).
   * @return Populated CachedObservationFrame, or std::nullopt on non-finite origin
   *         or if raw_points.size() exceeds UINT32_MAX.
   */
  [[nodiscard]] static std::optional<CachedObservationFrame> create(
      std::span<const PointXYZ> raw_points,
      uint64_t frame_seq,
      avr::spatial::WorldPoint2D sensor_origin) {
    if (!std::isfinite(sensor_origin.x) || !std::isfinite(sensor_origin.y)) {
      return std::nullopt;
    }

    if (raw_points.size() > std::numeric_limits<PointIndex>::max()) {
      return std::nullopt;
    }

    CachedObservationFrame frame{};
    frame.frame_seq_ = frame_seq;
    frame.sensor_origin_ = sensor_origin;
    frame.raw_points_.assign(raw_points.begin(), raw_points.end());

    frame.stats_.frame_seq = frame_seq;
    frame.stats_.input_points = raw_points.size();

    for (size_t i = 0; i < raw_points.size(); ++i) {
      const auto& pt = raw_points[i];
      if (!pt.isFinite()) {
        ++frame.stats_.non_finite_unindexed;
        continue;
      }

      const auto gx = avr::spatial::quantizeToBaseGridSafe(pt.x);
      const auto gy = avr::spatial::quantizeToBaseGridSafe(pt.y);
      if (!gx.has_value() || !gy.has_value()) {
        ++frame.stats_.out_of_range_unindexed;
        continue;
      }

      const avr::spatial::GridCoord2D grid_coord{.x = *gx, .y = *gy};
      const auto l3_key_opt = avr::spatial::CellKey::fromGridCoord(grid_coord, avr::spatial::L3);
      if (!l3_key_opt.has_value()) {
        ++frame.stats_.out_of_range_unindexed;
        continue;
      }

      const auto l3_key = *l3_key_opt;
      frame.buckets_[l3_key].push_back(static_cast<PointIndex>(i));
      ++frame.stats_.replay_candidates;
    }

    frame.stats_.bucket_count = frame.buckets_.size();
    frame.stats_.point_indices_count = frame.stats_.replay_candidates;
    frame.stats_.raw_point_bytes = saturatingMultiply(frame.raw_points_.size(), sizeof(PointXYZ));
    frame.stats_.index_bytes = saturatingMultiply(
        static_cast<size_t>(frame.stats_.point_indices_count), sizeof(PointIndex));

    return frame;
  }

  [[nodiscard]] uint64_t frameSeq() const noexcept {
    return frame_seq_;
  }

  [[nodiscard]] const avr::spatial::WorldPoint2D& sensorOrigin() const noexcept {
    return sensor_origin_;
  }

  [[nodiscard]] std::span<const PointXYZ> rawPoints() const noexcept {
    return raw_points_;
  }

  [[nodiscard]] const std::vector<PointXYZ>& rawPointsVector() const noexcept {
    return raw_points_;
  }

  [[nodiscard]] size_t pointCount() const noexcept {
    return raw_points_.size();
  }

  [[nodiscard]] size_t bucketCount() const noexcept {
    return buckets_.size();
  }

  [[nodiscard]] const ReplayCacheFrameStats& statistics() const noexcept {
    return stats_;
  }

  /**
   * @brief Returns a deterministic list of populated L3 bucket keys sorted in canonical CellKey order.
   */
  [[nodiscard]] std::vector<avr::spatial::CellKey> bucketKeySnapshot() const {
    std::vector<avr::spatial::CellKey> keys;
    keys.reserve(buckets_.size());
    for (const auto& [key, _] : buckets_) {
      keys.push_back(key);
    }
    std::sort(keys.begin(), keys.end());
    return keys;
  }

  /**
   * @brief Returns read-only slice of point indices in the requested L3 bucket.
   *        Returns empty span if bucket is not present.
   */
  [[nodiscard]] std::span<const PointIndex> findBucketIndices(
      const avr::spatial::CellKey& l3_key) const noexcept {
    const auto it = buckets_.find(l3_key);
    if (it == buckets_.end()) {
      return {};
    }
    return it->second;
  }

  /**
   * @brief Collects raw point indices for any valid L0-L3 CellKey region.
   *
   * Inspects exclusively the single containing L3 bucket and filters candidate
   * points using exact canonical integer grid membership.
   *
   * @param region Requested L0, L1, L2, or L3 CellKey.
   * @param output Output vector, cleared first, then populated with matching indices
   *               in original ascending order.
   * @return true if region is valid level, false otherwise.
   */
  bool collectObservationIndices(
      const avr::spatial::CellKey& region,
      std::vector<PointIndex>& output) const {
    output.clear();

    if (!avr::spatial::isValidLevel(region.level)) {
      return false;
    }

    // Derive containing L3 bucket key via exact ancestor navigation
    avr::spatial::CellKey l3_key = region;
    while (l3_key.level < avr::spatial::L3) {
      const auto parent_opt = avr::spatial::parentOf(l3_key);
      if (!parent_opt.has_value()) {
        return false;
      }
      l3_key = *parent_opt;
    }

    const auto it = buckets_.find(l3_key);
    if (it == buckets_.end()) {
      return true;
    }

    if (region.level == avr::spatial::L3) {
      output.assign(it->second.begin(), it->second.end());
      return true;
    }

    output.reserve(it->second.size());
    for (const auto idx : it->second) {
      const auto& pt = raw_points_[idx];
      const auto gx = avr::spatial::quantizeToBaseGridSafe(pt.x);
      const auto gy = avr::spatial::quantizeToBaseGridSafe(pt.y);
      if (!gx.has_value() || !gy.has_value()) {
        continue;
      }

      const avr::spatial::GridCoord2D grid_coord{.x = *gx, .y = *gy};
      const auto pt_key_opt = avr::spatial::CellKey::fromGridCoord(grid_coord, region.level);
      if (pt_key_opt.has_value() && *pt_key_opt == region) {
        output.push_back(idx);
      }
    }

    return true;
  }

 private:
  CachedObservationFrame() = default;

  uint64_t frame_seq_{0};
  avr::spatial::WorldPoint2D sensor_origin_{0.0, 0.0};
  std::vector<PointXYZ> raw_points_{};
  std::unordered_map<avr::spatial::CellKey, std::vector<PointIndex>> buckets_{};
  ReplayCacheFrameStats stats_{};
};

}  // namespace avr::mapping

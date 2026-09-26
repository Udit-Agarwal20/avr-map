#pragma once

#include <cstdint>
#include <optional>
#include <span>

#include "avr_mapping/point_xyz.hpp"
#include "avr_mapping/semantic_types.hpp"
#include "avr_spatial/coordinate.hpp"

namespace avr::mapping {

/**
 * @brief Abstract perception provider interface for per-frame point classification.
 *
 * Implementations process raw LiDAR frames to produce SemanticObservation predictions.
 * Returning std::nullopt indicates temporary provider unavailability (triggers degraded fallback).
 */
class SemanticProvider {
 public:
  virtual ~SemanticProvider() = default;

  /**
   * @brief Infers semantic classes and motion states for the provided point cloud.
   *
   * @param raw_points Full unfiltered incoming LiDAR point returns in odom coordinates.
   * @param frame_seq Logical sensor frame sequence number.
   * @param sensor_origin Sensor position in odom coordinates.
   * @return Populated SemanticFrameResult, or std::nullopt if provider is temporarily unavailable.
   */
  [[nodiscard]] virtual std::optional<SemanticFrameResult> infer(
      std::span<const PointXYZ> raw_points,
      uint64_t frame_seq,
      avr::spatial::WorldPoint2D sensor_origin) = 0;
};

}  // namespace avr::mapping

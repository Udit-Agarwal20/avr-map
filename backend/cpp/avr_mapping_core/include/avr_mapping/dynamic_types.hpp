#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "avr_mapping/point_xyz.hpp"
#include "avr_mapping/semantic_types.hpp"

namespace avr::mapping {

/**
 * @brief Owned representation of a confirmed-dynamic LiDAR point return.
 *
 * Guaranteed to originate from an observation classified as MotionClassification::CONFIRMED_DYNAMIC
 * with motion_confidence >= min_motion_confidence and valid, finite, quantizable geometry.
 *
 * Stored in pipeline input coordinate frame.
 */
struct DynamicPoint {
  uint32_t source_point_index{0};
  PointXYZ position{};
  SemanticClass semantic_class{SemanticClass::UNKNOWN};
  float semantic_confidence{0.0f};
  float motion_confidence{0.0f};

  constexpr bool operator==(const DynamicPoint&) const noexcept = default;
};

/**
 * @brief Owned set of confirmed-dynamic points extracted from a single frame.
 *
 * Points are strictly preserved in ascending source_point_index / arrival order.
 * Current-frame only; never persisted across frames.
 */
struct DynamicPointSet {
  uint64_t frame_seq{0};
  std::vector<DynamicPoint> points{};

  constexpr bool operator==(const DynamicPointSet&) const noexcept = default;
};

/**
 * @brief Configuration parameters for deterministic 2.5D spatial clustering.
 *
 * Two points p and q are direct neighbors iff:
 *   (px - qx)^2 + (py - qy)^2 <= cluster_radius_xy_m^2
 *   AND
 *   abs(pz - qz) <= max_vertical_gap_m.
 *
 * Default values are PROVISIONAL ENGINEERING DEFAULTS (not calibrated physical truth).
 */
struct DynamicClusterConfig {
  double cluster_radius_xy_m{0.50};
  double max_vertical_gap_m{0.80};
  uint32_t min_cluster_points{5};
  float min_semantic_confidence{0.50f};

  [[nodiscard]] bool isValid() const noexcept {
    return std::isfinite(cluster_radius_xy_m) && cluster_radius_xy_m > 0.0 &&
           std::isfinite(max_vertical_gap_m) && max_vertical_gap_m >= 0.0 &&
           min_cluster_points >= 1 &&
           std::isfinite(min_semantic_confidence) &&
           min_semantic_confidence >= 0.0f && min_semantic_confidence <= 1.0f;
  }

  constexpr bool operator==(const DynamicClusterConfig&) const noexcept = default;
};

/**
 * @brief Outcome status of the dynamic spatial clustering routine.
 */
enum class DynamicClusteringStatus : uint8_t {
  SUCCESS = 0,
  INVALID_CONFIG = 1,
  RESOURCE_ERROR = 2,
  NUMERIC_ERROR = 3,
  INVALID_INPUT = 4,
};

[[nodiscard]] constexpr const char* dynamicClusteringStatusToString(
    DynamicClusteringStatus status) noexcept {
  switch (status) {
    case DynamicClusteringStatus::SUCCESS:
      return "SUCCESS";
    case DynamicClusteringStatus::INVALID_CONFIG:
      return "INVALID_CONFIG";
    case DynamicClusteringStatus::RESOURCE_ERROR:
      return "RESOURCE_ERROR";
    case DynamicClusteringStatus::NUMERIC_ERROR:
      return "NUMERIC_ERROR";
    case DynamicClusteringStatus::INVALID_INPUT:
      return "INVALID_INPUT";
  }
  return "UNKNOWN";
}

/**
 * @brief Conservation accounting diagnostics for dynamic spatial clustering.
 */
struct DynamicClusteringDiagnostics {
  uint32_t input_points{0};
  uint32_t clustered_points{0};
  uint32_t noise_points{0};

  constexpr bool operator==(const DynamicClusteringDiagnostics&) const noexcept = default;
};

/**
 * @brief Connected component representing an identified dynamic object in a single frame.
 *
 * Current-frame cluster IDs have NO temporal identity.
 */
struct DynamicObjectCluster {
  uint32_t cluster_local_id{0};
  uint32_t canonical_source_point_index{0};

  std::vector<uint32_t> member_source_indices{};

  PointXYZ centroid{};
  PointXYZ aabb_min{};
  PointXYZ aabb_max{};

  SemanticClass dominant_class{SemanticClass::UNKNOWN};
  float mean_semantic_confidence{0.0f};
  float mean_motion_confidence{0.0f};

  constexpr bool operator==(const DynamicObjectCluster&) const noexcept = default;
};

/**
 * @brief Complete output of single-frame dynamic spatial clustering.
 */
struct DynamicClusteringResult {
  DynamicClusteringStatus status{DynamicClusteringStatus::SUCCESS};
  std::vector<DynamicObjectCluster> clusters{};
  std::vector<uint32_t> noise_source_indices{};
  DynamicClusteringDiagnostics diagnostics{};

  [[nodiscard]] constexpr bool isSuccess() const noexcept {
    return status == DynamicClusteringStatus::SUCCESS;
  }

  constexpr bool operator==(const DynamicClusteringResult&) const noexcept = default;
};

}  // namespace avr::mapping

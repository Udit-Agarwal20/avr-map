#pragma once

#include "avr_mapping/dynamic_types.hpp"

namespace avr::mapping {

/**
 * @brief Standalone deterministic 2.5D spatial clusterer for dynamic point returns.
 *
 * Implements single-frame connected-component discovery over an anisotropic neighborhood:
 *   (px - qx)^2 + (py - qy)^2 <= cluster_radius_xy_m^2
 *   AND
 *   abs(pz - qz) <= max_vertical_gap_m.
 *
 * Has zero dependencies on mapping, cache, or hysteresis data structures.
 */
class DynamicClusterer {
 public:
  /**
   * @brief Performs deterministic 2.5D spatial clustering on a set of dynamic points.
   *
   * @param point_set Set of confirmed-dynamic points to cluster (must have finite positions).
   * @param config Clustering spatial thresholds and minimum point count.
   * @return Clustering result containing clusters, noise indices, diagnostics, and status.
   */
  [[nodiscard]] static DynamicClusteringResult cluster(
      const DynamicPointSet& point_set,
      const DynamicClusterConfig& config);

  /**
   * @brief Instance helper delegating to static cluster routine.
   */
  [[nodiscard]] DynamicClusteringResult operator()(
      const DynamicPointSet& point_set,
      const DynamicClusterConfig& config) const {
    return cluster(point_set, config);
  }
};

}  // namespace avr::mapping

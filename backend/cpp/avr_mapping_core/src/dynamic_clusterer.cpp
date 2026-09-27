#include "avr_mapping/dynamic_clusterer.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <new>
#include <numeric>
#include <optional>
#include <unordered_map>
#include <vector>

namespace avr::mapping {

namespace {

struct VoxelCoord3D {
  int64_t vx{0};
  int64_t vy{0};
  int64_t vz{0};

  constexpr bool operator==(const VoxelCoord3D&) const noexcept = default;
};

struct VoxelCoord3DHash {
  size_t operator()(const VoxelCoord3D& c) const noexcept {
    // Explicitly unsigned 64-bit hashing and mixing to prevent any signed integer overflow
    size_t h1 = std::hash<int64_t>{}(c.vx);
    size_t h2 = std::hash<int64_t>{}(c.vy);
    size_t h3 = std::hash<int64_t>{}(c.vz);
    size_t seed = h1;
    seed ^= h2 + 0x9e3779b97f4a7c15ULL + (seed << 6) + (seed >> 2);
    seed ^= h3 + 0x9e3779b97f4a7c15ULL + (seed << 6) + (seed >> 2);
    return seed;
  }
};

/**
 * @brief Safely quantizes a finite coordinate into an integer voxel coordinate.
 *
 * Guarantees that coordinate / voxel_size is finite, does not overflow int64_t,
 * and leaves room for +/- 1 neighbor traversal offsets without signed integer overflow.
 */
[[nodiscard]] std::optional<int64_t> safeVoxelCoord(
    double coordinate,
    double voxel_size) noexcept {
  if (!std::isfinite(coordinate) || !std::isfinite(voxel_size) || voxel_size <= 0.0) {
    return std::nullopt;
  }
  const double div = coordinate / voxel_size;
  if (!std::isfinite(div)) {
    return std::nullopt;
  }
  const double scaled = std::floor(div);
  if (!std::isfinite(scaled)) {
    return std::nullopt;
  }
  // Safe bounds: int64_t range is ~ [-9.22e18, 9.22e18].
  // Restricting to [-9.0e18, 9.0e18] guarantees safe conversion and safe +/- 1 neighbor offsets.
  constexpr double kMinSafeVoxelCoord = -9.0e18;
  constexpr double kMaxSafeVoxelCoord = 9.0e18;
  if (scaled < kMinSafeVoxelCoord || scaled > kMaxSafeVoxelCoord) {
    return std::nullopt;
  }
  return static_cast<int64_t>(scaled);
}

}  // namespace

DynamicClusteringResult DynamicClusterer::cluster(
    const DynamicPointSet& point_set,
    const DynamicClusterConfig& config) {
  // 1. Config validation
  if (!config.isValid()) {
    DynamicClusteringResult res{};
    res.status = DynamicClusteringStatus::INVALID_CONFIG;
    return res;
  }

  const double r_xy_sq = config.cluster_radius_xy_m * config.cluster_radius_xy_m;
  if (!std::isfinite(r_xy_sq)) {
    DynamicClusteringResult res{};
    res.status = DynamicClusteringStatus::NUMERIC_ERROR;
    return res;
  }

  // 2. Empty point set early-exit
  if (point_set.points.empty()) {
    DynamicClusteringResult res{};
    res.status = DynamicClusteringStatus::SUCCESS;
    res.diagnostics.input_points = 0;
    res.diagnostics.clustered_points = 0;
    res.diagnostics.noise_points = 0;
    return res;
  }

  try {
    const size_t num_points = point_set.points.size();
    const double v_xy = config.cluster_radius_xy_m;
    const double v_z = (config.max_vertical_gap_m > 0.0) ? config.max_vertical_gap_m : v_xy;
    if (!std::isfinite(v_z) || v_z <= 0.0) {
      DynamicClusteringResult res{};
      res.status = DynamicClusteringStatus::NUMERIC_ERROR;
      return res;
    }

    // 3. Public DynamicPoint contract validation & checked voxel quantization
    std::vector<VoxelCoord3D> point_voxels;
    point_voxels.reserve(num_points);
    for (const auto& pt : point_set.points) {
      if (!isValidSemanticClass(pt.semantic_class)) {
        DynamicClusteringResult res{};
        res.status = DynamicClusteringStatus::INVALID_INPUT;
        return res;
      }
      if (!std::isfinite(pt.semantic_confidence) ||
          pt.semantic_confidence < 0.0f || pt.semantic_confidence > 1.0f) {
        DynamicClusteringResult res{};
        res.status = DynamicClusteringStatus::INVALID_INPUT;
        return res;
      }
      if (!std::isfinite(pt.motion_confidence) ||
          pt.motion_confidence < 0.0f || pt.motion_confidence > 1.0f) {
        DynamicClusteringResult res{};
        res.status = DynamicClusteringStatus::INVALID_INPUT;
        return res;
      }
      if (!pt.position.isFinite()) {
        DynamicClusteringResult res{};
        res.status = DynamicClusteringStatus::NUMERIC_ERROR;
        return res;
      }
      auto vx = safeVoxelCoord(pt.position.x, v_xy);
      auto vy = safeVoxelCoord(pt.position.y, v_xy);
      auto vz = safeVoxelCoord(pt.position.z, v_z);
      if (!vx.has_value() || !vy.has_value() || !vz.has_value()) {
        DynamicClusteringResult res{};
        res.status = DynamicClusteringStatus::NUMERIC_ERROR;
        return res;
      }
      point_voxels.push_back(VoxelCoord3D{*vx, *vy, *vz});
    }

    // 4. Duplicate source index validation and canonical ordering
    std::vector<uint32_t> canonical_order(num_points);
    std::iota(canonical_order.begin(), canonical_order.end(), 0u);
    std::sort(canonical_order.begin(), canonical_order.end(),
        [&point_set](uint32_t a, uint32_t b) {
          return point_set.points[a].source_point_index <
                 point_set.points[b].source_point_index;
        });

    for (size_t k = 1; k < num_points; ++k) {
      if (point_set.points[canonical_order[k - 1]].source_point_index ==
          point_set.points[canonical_order[k]].source_point_index) {
        DynamicClusteringResult res{};
        res.status = DynamicClusteringStatus::INVALID_INPUT;
        return res;
      }
    }

    // 5. Populate spatial voxel hash using canonical order
    std::unordered_map<VoxelCoord3D, std::vector<uint32_t>, VoxelCoord3DHash> grid;
    grid.reserve(num_points);

    for (uint32_t orig_idx : canonical_order) {
      grid[point_voxels[orig_idx]].push_back(orig_idx);
    }

    // 6. Deterministic connected-component traversal via iterative BFS
    std::vector<bool> visited(num_points, false);
    std::vector<DynamicObjectCluster> completed_clusters;
    std::vector<uint32_t> noise_source_indices;

    const double max_vz = config.max_vertical_gap_m;

    for (uint32_t orig_idx : canonical_order) {
      if (visited[orig_idx]) {
        continue;
      }

      std::vector<uint32_t> component_local_indices;
      std::vector<uint32_t> queue;
      queue.push_back(orig_idx);
      visited[orig_idx] = true;
      size_t queue_head = 0;

      while (queue_head < queue.size()) {
        uint32_t curr = queue[queue_head++];
        component_local_indices.push_back(curr);

        const auto& p_curr = point_set.points[curr].position;
        const auto& cvoxel = point_voxels[curr];

        std::vector<uint32_t> candidate_neighbors;
        for (int dx = -1; dx <= 1; ++dx) {
          for (int dy = -1; dy <= 1; ++dy) {
            for (int dz = -1; dz <= 1; ++dz) {
              VoxelCoord3D ncoord{cvoxel.vx + dx, cvoxel.vy + dy, cvoxel.vz + dz};
              auto it = grid.find(ncoord);
              if (it != grid.end()) {
                for (uint32_t cand_idx : it->second) {
                  if (!visited[cand_idx]) {
                    const auto& p_cand = point_set.points[cand_idx].position;
                    double diff_x = p_curr.x - p_cand.x;
                    double diff_y = p_curr.y - p_cand.y;
                    double diff_z = std::abs(p_curr.z - p_cand.z);
                    if ((diff_x * diff_x + diff_y * diff_y <= r_xy_sq) &&
                        (diff_z <= max_vz)) {
                      candidate_neighbors.push_back(cand_idx);
                      visited[cand_idx] = true;
                    }
                  }
                }
              }
            }
          }
        }

        // Sort candidate neighbors ascending by source_point_index for canonical traversal order
        std::sort(candidate_neighbors.begin(), candidate_neighbors.end(),
            [&point_set](uint32_t a, uint32_t b) {
              return point_set.points[a].source_point_index <
                     point_set.points[b].source_point_index;
            });
        for (uint32_t nb : candidate_neighbors) {
          queue.push_back(nb);
        }
      }

      std::sort(component_local_indices.begin(), component_local_indices.end(),
          [&point_set](uint32_t a, uint32_t b) {
            return point_set.points[a].source_point_index <
                   point_set.points[b].source_point_index;
          });

      if (component_local_indices.size() < config.min_cluster_points) {
        for (uint32_t loc_idx : component_local_indices) {
          noise_source_indices.push_back(point_set.points[loc_idx].source_point_index);
        }
      } else {
        DynamicObjectCluster cluster{};
        cluster.member_source_indices.reserve(component_local_indices.size());
        for (uint32_t loc_idx : component_local_indices) {
          cluster.member_source_indices.push_back(point_set.points[loc_idx].source_point_index);
        }
        cluster.canonical_source_point_index = cluster.member_source_indices.front();

        // Compute Centroid & AABB using double precision accumulators
        double sum_x = 0.0;
        double sum_y = 0.0;
        double sum_z = 0.0;
        double min_x = std::numeric_limits<double>::infinity();
        double min_y = std::numeric_limits<double>::infinity();
        double min_z = std::numeric_limits<double>::infinity();
        double max_x = -std::numeric_limits<double>::infinity();
        double max_y = -std::numeric_limits<double>::infinity();
        double max_z = -std::numeric_limits<double>::infinity();

        double total_semantic_conf = 0.0;
        double total_motion_conf = 0.0;

        // Semantic voting: classes 1..7 (non-UNKNOWN)
        double class_weights[8] = {0.0};

        for (uint32_t loc_idx : component_local_indices) {
          const auto& pt = point_set.points[loc_idx];
          sum_x += pt.position.x;
          sum_y += pt.position.y;
          sum_z += pt.position.z;

          min_x = std::min(min_x, pt.position.x);
          min_y = std::min(min_y, pt.position.y);
          min_z = std::min(min_z, pt.position.z);
          max_x = std::max(max_x, pt.position.x);
          max_y = std::max(max_y, pt.position.y);
          max_z = std::max(max_z, pt.position.z);

          total_semantic_conf += pt.semantic_confidence;
          total_motion_conf += pt.motion_confidence;

          if (pt.semantic_class != SemanticClass::UNKNOWN &&
              pt.semantic_confidence >= config.min_semantic_confidence) {
            uint8_t sc_val = static_cast<uint8_t>(pt.semantic_class);
            if (sc_val < 8) {
              class_weights[sc_val] += pt.semantic_confidence;
            }
          }
        }

        const double member_count_d = static_cast<double>(component_local_indices.size());
        cluster.centroid = PointXYZ{
            .x = sum_x / member_count_d,
            .y = sum_y / member_count_d,
            .z = sum_z / member_count_d,
        };
        cluster.aabb_min = PointXYZ{.x = min_x, .y = min_y, .z = min_z};
        cluster.aabb_max = PointXYZ{.x = max_x, .y = max_y, .z = max_z};

        cluster.mean_semantic_confidence = static_cast<float>(total_semantic_conf / member_count_d);
        cluster.mean_motion_confidence = static_cast<float>(total_motion_conf / member_count_d);

        // Dominant semantic class: highest accumulated weight among qualified non-UNKNOWN classes.
        // Tie-breaker: lowest SemanticClass numeric value (preserved by strictly greater comparison).
        SemanticClass dominant = SemanticClass::UNKNOWN;
        double max_weight = 0.0;
        for (uint8_t c = 1; c < 8; ++c) {
          if (class_weights[c] > max_weight) {
            max_weight = class_weights[c];
            dominant = static_cast<SemanticClass>(c);
          }
        }
        cluster.dominant_class = dominant;

        completed_clusters.push_back(std::move(cluster));
      }
    }

    // 7. Sort completed clusters by canonical key ascending and assign local IDs 0..K-1
    std::sort(completed_clusters.begin(), completed_clusters.end(),
        [](const auto& a, const auto& b) {
          return a.canonical_source_point_index < b.canonical_source_point_index;
        });

    for (uint32_t k = 0; k < static_cast<uint32_t>(completed_clusters.size()); ++k) {
      completed_clusters[k].cluster_local_id = k;
    }

    // 8. Sort noise source indices ascending
    std::sort(noise_source_indices.begin(), noise_source_indices.end());

    // 9. Assemble result and diagnostics
    DynamicClusteringResult result{};
    result.status = DynamicClusteringStatus::SUCCESS;
    result.diagnostics.input_points = static_cast<uint32_t>(num_points);
    uint32_t clustered_total = 0;
    for (const auto& cl : completed_clusters) {
      clustered_total += static_cast<uint32_t>(cl.member_source_indices.size());
    }
    result.diagnostics.clustered_points = clustered_total;
    result.diagnostics.noise_points = static_cast<uint32_t>(noise_source_indices.size());
    result.clusters = std::move(completed_clusters);
    result.noise_source_indices = std::move(noise_source_indices);

    return result;
  } catch (const std::bad_alloc&) {
    DynamicClusteringResult res{};
    res.status = DynamicClusteringStatus::RESOURCE_ERROR;
    res.diagnostics.input_points = static_cast<uint32_t>(point_set.points.size());
    return res;
  }
}

}  // namespace avr::mapping

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <vector>

#include "avr_mapping/dynamic_clusterer.hpp"
#include "avr_mapping/dynamic_types.hpp"

namespace avr::mapping {
namespace {

struct XorShift64 {
  uint64_t state{0x123456789ABCDEF0ULL};

  uint64_t next() noexcept {
    uint64_t x = state;
    x ^= x << 13;
    x ^= x >> 7;
    x ^= x << 17;
    state = x;
    return x;
  }

  double uniformDouble(double min_val, double max_val) noexcept {
    double u = static_cast<double>(next() >> 11) / static_cast<double>(1ULL << 53);
    return min_val + u * (max_val - min_val);
  }

  uint32_t uniformInt(uint32_t min_val, uint32_t max_val) noexcept {
    if (min_val >= max_val) return min_val;
    return min_val + static_cast<uint32_t>(next() % (max_val - min_val + 1));
  }
};

}  // namespace

TEST(DynamicClusteringStressTest, TenThousandDynamicPointSetsStress) {
  // Requirement 72:
  // Deterministic clustering stress covering at least 10,000 DynamicPointSets.
  XorShift64 rng{};
  constexpr uint32_t kTotalIterations = 10000;

  DynamicClusterConfig default_config{
      .cluster_radius_xy_m = 0.50,
      .max_vertical_gap_m = 0.80,
      .min_cluster_points = 5,
      .min_semantic_confidence = 0.50f,
  };

  for (uint32_t iter = 0; iter < kTotalIterations; ++iter) {
    DynamicPointSet pts{};
    pts.frame_seq = iter + 1;

    uint32_t pattern = iter % 11;
    switch (pattern) {
      case 0: {
        // 0: empty
        break;
      }
      case 1: {
        // 1: single point (must become noise)
        pts.points.push_back(DynamicPoint{
            .source_point_index = 0,
            .position = PointXYZ{
                .x = rng.uniformDouble(-10.0, 10.0),
                .y = rng.uniformDouble(-10.0, 10.0),
                .z = rng.uniformDouble(-2.0, 2.0),
            },
            .semantic_class = SemanticClass::VEHICLE,
            .semantic_confidence = static_cast<float>(rng.uniformDouble(0.5, 1.0)),
            .motion_confidence = static_cast<float>(rng.uniformDouble(0.7, 1.0)),
        });
        break;
      }
      case 2: {
        // 2: all noise (several isolated points separated by > 5m)
        uint32_t count = rng.uniformInt(2, 6);
        for (uint32_t p = 0; p < count; ++p) {
          pts.points.push_back(DynamicPoint{
              .source_point_index = p,
              .position = PointXYZ{
                  .x = p * 10.0,
                  .y = p * 10.0,
                  .z = 0.0,
              },
              .semantic_class = SemanticClass::PEDESTRIAN,
              .semantic_confidence = 0.8f,
              .motion_confidence = 0.9f,
          });
        }
        break;
      }
      case 3: {
        // 3: one tight cluster (5 to 20 points within 0.3m)
        uint32_t count = rng.uniformInt(5, 20);
        double center_x = rng.uniformDouble(-20.0, 20.0);
        double center_y = rng.uniformDouble(-20.0, 20.0);
        double center_z = rng.uniformDouble(-1.0, 1.0);
        for (uint32_t p = 0; p < count; ++p) {
          pts.points.push_back(DynamicPoint{
              .source_point_index = p,
              .position = PointXYZ{
                  .x = center_x + rng.uniformDouble(-0.15, 0.15),
                  .y = center_y + rng.uniformDouble(-0.15, 0.15),
                  .z = center_z + rng.uniformDouble(-0.10, 0.10),
              },
              .semantic_class = SemanticClass::VEHICLE,
              .semantic_confidence = 0.9f,
              .motion_confidence = 0.95f,
          });
        }
        break;
      }
      case 4: {
        // 4: multiple clusters (2 to 4 clusters spaced at intervals of 5.0m)
        uint32_t num_clusters = rng.uniformInt(2, 4);
        uint32_t src_idx = 0;
        for (uint32_t c = 0; c < num_clusters; ++c) {
          uint32_t c_pts = rng.uniformInt(5, 10);
          double base_x = c * 8.0;
          double base_y = c * 8.0;
          for (uint32_t p = 0; p < c_pts; ++p) {
            pts.points.push_back(DynamicPoint{
                .source_point_index = src_idx++,
                .position = PointXYZ{
                    .x = base_x + rng.uniformDouble(-0.1, 0.1),
                    .y = base_y + rng.uniformDouble(-0.1, 0.1),
                    .z = rng.uniformDouble(-0.1, 0.1),
                },
                .semantic_class = SemanticClass::VEHICLE,
                .semantic_confidence = 0.85f,
                .motion_confidence = 0.9f,
            });
          }
        }
        break;
      }
      case 5: {
        // 5: negative coordinates
        uint32_t count = rng.uniformInt(5, 15);
        double base_x = -150.0;
        double base_y = -150.0;
        for (uint32_t p = 0; p < count; ++p) {
          pts.points.push_back(DynamicPoint{
              .source_point_index = p,
              .position = PointXYZ{
                  .x = base_x + rng.uniformDouble(-0.15, 0.15),
                  .y = base_y + rng.uniformDouble(-0.15, 0.15),
                  .z = -5.0 + rng.uniformDouble(-0.1, 0.1),
              },
              .semantic_class = SemanticClass::VEHICLE,
              .semantic_confidence = 0.9f,
              .motion_confidence = 0.9f,
          });
        }
        break;
      }
      case 6: {
        // 6: dense pathological neighborhood (30 to 60 points in a 0.1m box)
        uint32_t count = rng.uniformInt(30, 60);
        for (uint32_t p = 0; p < count; ++p) {
          pts.points.push_back(DynamicPoint{
              .source_point_index = p,
              .position = PointXYZ{
                  .x = rng.uniformDouble(0.0, 0.1),
                  .y = rng.uniformDouble(0.0, 0.1),
                  .z = rng.uniformDouble(0.0, 0.1),
              },
              .semantic_class = SemanticClass::PEDESTRIAN,
              .semantic_confidence = 0.9f,
              .motion_confidence = 0.9f,
          });
        }
        break;
      }
      case 7: {
        // 7: vertical separation (points at same XY, but separated by 1.5m in Z)
        for (uint32_t p = 0; p < 5; ++p) {
          pts.points.push_back(DynamicPoint{
              .source_point_index = p,
              .position = PointXYZ{.x = 1.0, .y = 1.0, .z = 0.0 + 0.02 * p},
              .semantic_class = SemanticClass::VEHICLE,
              .semantic_confidence = 0.9f,
              .motion_confidence = 0.9f,
          });
        }
        for (uint32_t p = 5; p < 10; ++p) {
          pts.points.push_back(DynamicPoint{
              .source_point_index = p,
              .position = PointXYZ{.x = 1.0, .y = 1.0, .z = 1.5 + 0.02 * (p - 5)},
              .semantic_class = SemanticClass::VEHICLE,
              .semantic_confidence = 0.9f,
              .motion_confidence = 0.9f,
          });
        }
        break;
      }
      case 8: {
        // 8: mixed semantics (VEHICLE, PEDESTRIAN, UNKNOWN, various confidences)
        for (uint32_t p = 0; p < 8; ++p) {
          SemanticClass sc = (p % 3 == 0) ? SemanticClass::VEHICLE :
                             (p % 3 == 1) ? SemanticClass::PEDESTRIAN : SemanticClass::UNKNOWN;
          pts.points.push_back(DynamicPoint{
              .source_point_index = p,
              .position = PointXYZ{.x = 0.05 * p, .y = 0.0, .z = 0.0},
              .semantic_class = sc,
              .semantic_confidence = static_cast<float>(rng.uniformDouble(0.2, 0.95)),
              .motion_confidence = static_cast<float>(rng.uniformDouble(0.7, 0.99)),
          });
        }
        break;
      }
      case 9: {
        // 9: large valid coordinates (e.g. 50,000m)
        uint32_t count = rng.uniformInt(5, 12);
        double base_x = 50000.0;
        double base_y = 50000.0;
        for (uint32_t p = 0; p < count; ++p) {
          pts.points.push_back(DynamicPoint{
              .source_point_index = p,
              .position = PointXYZ{
                  .x = base_x + 0.05 * p,
                  .y = base_y + 0.05 * p,
                  .z = 100.0,
              },
              .semantic_class = SemanticClass::VEHICLE,
              .semantic_confidence = 0.9f,
              .motion_confidence = 0.95f,
          });
        }
        break;
      }
      case 10: {
        // 10: negative coordinates with tight spacing
        uint32_t count = rng.uniformInt(5, 15);
        double base_x = -5000.0;
        double base_y = -5000.0;
        for (uint32_t p = 0; p < count; ++p) {
          pts.points.push_back(DynamicPoint{
              .source_point_index = p,
              .position = PointXYZ{
                  .x = base_x + rng.uniformDouble(-0.02, 0.02),
                  .y = base_y + rng.uniformDouble(-0.02, 0.02),
                  .z = -10.0 + rng.uniformDouble(-0.01, 0.01),
              },
              .semantic_class = SemanticClass::VEHICLE,
              .semantic_confidence = 0.9f,
              .motion_confidence = 0.9f,
          });
        }
        break;
      }
    }

    // Periodically assign non-contiguous source IDs (Requirement 33)
    if (iter % 3 == 0) {
      for (size_t p = 0; p < pts.points.size(); ++p) {
        pts.points[p].source_point_index = static_cast<uint32_t>(p * 19 + 7);
      }
    }

    // Periodically shuffle input vector ordering (Requirement 33)
    if (iter % 2 == 1 && pts.points.size() > 1) {
      for (size_t p = pts.points.size() - 1; p > 0; --p) {
        size_t j = static_cast<size_t>(rng.uniformInt(0, static_cast<uint32_t>(p)));
        std::swap(pts.points[p], pts.points[j]);
      }
    }

    // Periodically use small safe radius or zero vertical gap (Requirement 33)
    DynamicClusterConfig run_config = default_config;
    if (iter % 50 == 0) {
      run_config.cluster_radius_xy_m = 0.05;
      run_config.max_vertical_gap_m = 0.10;
    } else if (iter % 75 == 0) {
      run_config.max_vertical_gap_m = 0.0;
    }

    auto res = DynamicClusterer::cluster(pts, run_config);
    ASSERT_EQ(res.status, DynamicClusteringStatus::SUCCESS);
    ASSERT_EQ(res.diagnostics.input_points, pts.points.size());
    ASSERT_EQ(res.diagnostics.clustered_points + res.diagnostics.noise_points,
              res.diagnostics.input_points);

    // Validate cluster properties
    uint32_t prev_canonical = 0;
    std::vector<uint32_t> accounted_source_indices;
    accounted_source_indices.reserve(pts.points.size());

    for (uint32_t c = 0; c < res.clusters.size(); ++c) {
      const auto& cl = res.clusters[c];
      EXPECT_EQ(cl.cluster_local_id, c);
      EXPECT_GE(cl.member_source_indices.size(), run_config.min_cluster_points);
      EXPECT_EQ(cl.canonical_source_point_index, cl.member_source_indices.front());

      if (c > 0) {
        EXPECT_GT(cl.canonical_source_point_index, prev_canonical);
      }
      prev_canonical = cl.canonical_source_point_index;

      // Check member indices strictly sorted ascending
      for (size_t m = 1; m < cl.member_source_indices.size(); ++m) {
        EXPECT_GT(cl.member_source_indices[m], cl.member_source_indices[m - 1]);
      }
      for (uint32_t idx : cl.member_source_indices) {
        accounted_source_indices.push_back(idx);
      }

      // Check centroid within AABB
      EXPECT_GE(cl.centroid.x, cl.aabb_min.x - 1e-6);
      EXPECT_LE(cl.centroid.x, cl.aabb_max.x + 1e-6);
      EXPECT_GE(cl.centroid.y, cl.aabb_min.y - 1e-6);
      EXPECT_LE(cl.centroid.y, cl.aabb_max.y + 1e-6);
      EXPECT_GE(cl.centroid.z, cl.aabb_min.z - 1e-6);
      EXPECT_LE(cl.centroid.z, cl.aabb_max.z + 1e-6);

      // Check mean confidences in [0, 1]
      EXPECT_GE(cl.mean_semantic_confidence, 0.0f);
      EXPECT_LE(cl.mean_semantic_confidence, 1.0f);
      EXPECT_GE(cl.mean_motion_confidence, 0.0f);
      EXPECT_LE(cl.mean_motion_confidence, 1.0f);
    }

    // Check noise indices strictly sorted ascending
    for (size_t n = 1; n < res.noise_source_indices.size(); ++n) {
      EXPECT_GT(res.noise_source_indices[n], res.noise_source_indices[n - 1]);
    }
    for (uint32_t idx : res.noise_source_indices) {
      accounted_source_indices.push_back(idx);
    }

    // Every unique source index appears exactly once!
    std::sort(accounted_source_indices.begin(), accounted_source_indices.end());
    std::vector<uint32_t> expected_indices;
    expected_indices.reserve(pts.points.size());
    for (const auto& p : pts.points) {
      expected_indices.push_back(p.source_point_index);
    }
    std::sort(expected_indices.begin(), expected_indices.end());
    EXPECT_EQ(accounted_source_indices, expected_indices);
  }
}

}  // namespace avr::mapping

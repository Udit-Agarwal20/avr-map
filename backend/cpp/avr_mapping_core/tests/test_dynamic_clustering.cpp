#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <iostream>
#include <vector>

#include "avr_mapping/dynamic_clusterer.hpp"
#include "avr_mapping/dynamic_types.hpp"

namespace avr::mapping {

TEST(DynamicClusteringTest, ABIAndLayoutReporting) {
  std::cout << "[ABI] sizeof(PointXYZ) = " << sizeof(PointXYZ)
            << ", alignof(PointXYZ) = " << alignof(PointXYZ) << std::endl;
  std::cout << "[ABI] sizeof(DynamicPoint) = " << sizeof(DynamicPoint)
            << ", alignof(DynamicPoint) = " << alignof(DynamicPoint) << std::endl;
  std::cout << "[ABI] offsetof(DynamicPoint, source_point_index) = "
            << offsetof(DynamicPoint, source_point_index) << std::endl;
  std::cout << "[ABI] offsetof(DynamicPoint, position) = "
            << offsetof(DynamicPoint, position) << std::endl;
  std::cout << "[ABI] offsetof(DynamicPoint, semantic_class) = "
            << offsetof(DynamicPoint, semantic_class) << std::endl;
  std::cout << "[ABI] offsetof(DynamicPoint, semantic_confidence) = "
            << offsetof(DynamicPoint, semantic_confidence) << std::endl;
  std::cout << "[ABI] offsetof(DynamicPoint, motion_confidence) = "
            << offsetof(DynamicPoint, motion_confidence) << std::endl;

  std::cout << "[ABI] sizeof(DynamicPointSet) = " << sizeof(DynamicPointSet)
            << ", alignof(DynamicPointSet) = " << alignof(DynamicPointSet) << std::endl;
  std::cout << "[ABI] sizeof(DynamicObjectCluster) = " << sizeof(DynamicObjectCluster)
            << ", alignof(DynamicObjectCluster) = " << alignof(DynamicObjectCluster) << std::endl;
  std::cout << "[ABI] sizeof(DynamicClusteringDiagnostics) = " << sizeof(DynamicClusteringDiagnostics)
            << ", alignof(DynamicClusteringDiagnostics) = " << alignof(DynamicClusteringDiagnostics) << std::endl;
  std::cout << "[ABI] sizeof(DynamicClusteringResult) = " << sizeof(DynamicClusteringResult)
            << ", alignof(DynamicClusteringResult) = " << alignof(DynamicClusteringResult) << std::endl;

  EXPECT_EQ(sizeof(PointXYZ), 24u);
  EXPECT_EQ(alignof(PointXYZ), 8u);
  EXPECT_EQ(sizeof(DynamicPoint), 48u);
  EXPECT_EQ(alignof(DynamicPoint), 8u);
  EXPECT_EQ(offsetof(DynamicPoint, source_point_index), 0u);
  EXPECT_EQ(offsetof(DynamicPoint, position), 8u);
  EXPECT_EQ(offsetof(DynamicPoint, semantic_class), 32u);
  EXPECT_EQ(offsetof(DynamicPoint, semantic_confidence), 36u);
  EXPECT_EQ(offsetof(DynamicPoint, motion_confidence), 40u);

  static_assert(static_cast<uint8_t>(DynamicClusteringStatus::SUCCESS) == 0);
  static_assert(static_cast<uint8_t>(DynamicClusteringStatus::INVALID_CONFIG) == 1);
  static_assert(static_cast<uint8_t>(DynamicClusteringStatus::RESOURCE_ERROR) == 2);
  static_assert(static_cast<uint8_t>(DynamicClusteringStatus::NUMERIC_ERROR) == 3);
  static_assert(static_cast<uint8_t>(DynamicClusteringStatus::INVALID_INPUT) == 4);
}

TEST(DynamicClusteringTest, ConfigValidation) {
  DynamicClusterConfig valid_config{};
  EXPECT_TRUE(valid_config.isValid());

  DynamicClusterConfig invalid_radius = valid_config;
  invalid_radius.cluster_radius_xy_m = 0.0;
  EXPECT_FALSE(invalid_radius.isValid());
  invalid_radius.cluster_radius_xy_m = -0.5;
  EXPECT_FALSE(invalid_radius.isValid());
  invalid_radius.cluster_radius_xy_m = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(invalid_radius.isValid());

  DynamicClusterConfig invalid_gap = valid_config;
  invalid_gap.max_vertical_gap_m = -0.01;
  EXPECT_FALSE(invalid_gap.isValid());
  invalid_gap.max_vertical_gap_m = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(invalid_gap.isValid());

  DynamicClusterConfig invalid_min_pts = valid_config;
  invalid_min_pts.min_cluster_points = 0;
  EXPECT_FALSE(invalid_min_pts.isValid());

  DynamicClusterConfig invalid_conf = valid_config;
  invalid_conf.min_semantic_confidence = -0.1f;
  EXPECT_FALSE(invalid_conf.isValid());
  invalid_conf.min_semantic_confidence = 1.1f;
  EXPECT_FALSE(invalid_conf.isValid());

  DynamicPointSet pts{};
  pts.points.push_back(DynamicPoint{
      .source_point_index = 0,
      .position = PointXYZ{.x = 1.0, .y = 1.0, .z = 0.0},
      .semantic_class = SemanticClass::VEHICLE,
      .semantic_confidence = 0.9f,
      .motion_confidence = 0.9f,
  });

  auto res = DynamicClusterer::cluster(pts, invalid_radius);
  EXPECT_EQ(res.status, DynamicClusteringStatus::INVALID_CONFIG);
  EXPECT_FALSE(res.isSuccess());
  EXPECT_TRUE(res.clusters.empty());
}

TEST(DynamicClusteringTest, EmptyInputHandling) {
  DynamicPointSet empty_set{};
  DynamicClusterConfig config{};
  auto res = DynamicClusterer::cluster(empty_set, config);
  EXPECT_EQ(res.status, DynamicClusteringStatus::SUCCESS);
  EXPECT_TRUE(res.isSuccess());
  EXPECT_TRUE(res.clusters.empty());
  EXPECT_TRUE(res.noise_source_indices.empty());
  EXPECT_EQ(res.diagnostics.input_points, 0u);
  EXPECT_EQ(res.diagnostics.clustered_points, 0u);
  EXPECT_EQ(res.diagnostics.noise_points, 0u);
}

TEST(DynamicClusteringTest, NumericInputValidation) {
  DynamicClusterConfig config{};
  DynamicPointSet invalid_set{};
  invalid_set.points.push_back(DynamicPoint{
      .source_point_index = 0,
      .position = PointXYZ{.x = std::numeric_limits<double>::quiet_NaN(), .y = 1.0, .z = 0.0},
      .semantic_class = SemanticClass::VEHICLE,
      .semantic_confidence = 0.9f,
      .motion_confidence = 0.9f,
  });

  auto res = DynamicClusterer::cluster(invalid_set, config);
  EXPECT_EQ(res.status, DynamicClusteringStatus::NUMERIC_ERROR);
  EXPECT_FALSE(res.isSuccess());

  DynamicPointSet inf_set{};
  inf_set.points.push_back(DynamicPoint{
      .source_point_index = 0,
      .position = PointXYZ{.x = 1.0, .y = std::numeric_limits<double>::infinity(), .z = 0.0},
      .semantic_class = SemanticClass::VEHICLE,
      .semantic_confidence = 0.9f,
      .motion_confidence = 0.9f,
  });

  res = DynamicClusterer::cluster(inf_set, config);
  EXPECT_EQ(res.status, DynamicClusteringStatus::NUMERIC_ERROR);
}

TEST(DynamicClusteringTest, ChainConnectivityGolden) {
  // Requirement 48:
  // Points A, B, C where A-B neighbors, B-C neighbors, but A-C not direct neighbors.
  // Proves connected-component transitivity.
  DynamicClusterConfig config{
      .cluster_radius_xy_m = 0.50,
      .max_vertical_gap_m = 0.80,
      .min_cluster_points = 3,
  };

  DynamicPointSet set{};
  set.points.push_back(DynamicPoint{
      .source_point_index = 10,
      .position = PointXYZ{.x = 0.0, .y = 0.0, .z = 0.0},
      .semantic_class = SemanticClass::VEHICLE,
      .semantic_confidence = 0.9f,
      .motion_confidence = 0.95f,
  });
  set.points.push_back(DynamicPoint{
      .source_point_index = 20,
      .position = PointXYZ{.x = 0.40, .y = 0.0, .z = 0.0},  // dist to A = 0.40 <= 0.50
      .semantic_class = SemanticClass::VEHICLE,
      .semantic_confidence = 0.9f,
      .motion_confidence = 0.95f,
  });
  set.points.push_back(DynamicPoint{
      .source_point_index = 30,
      .position = PointXYZ{.x = 0.80, .y = 0.0, .z = 0.0},  // dist to B = 0.40 <= 0.50, dist to A = 0.80 > 0.50
      .semantic_class = SemanticClass::VEHICLE,
      .semantic_confidence = 0.9f,
      .motion_confidence = 0.95f,
  });

  auto res = DynamicClusterer::cluster(set, config);
  ASSERT_EQ(res.status, DynamicClusteringStatus::SUCCESS);
  ASSERT_EQ(res.clusters.size(), 1u);
  EXPECT_TRUE(res.noise_source_indices.empty());

  const auto& cl = res.clusters[0];
  EXPECT_EQ(cl.cluster_local_id, 0u);
  EXPECT_EQ(cl.canonical_source_point_index, 10u);
  ASSERT_EQ(cl.member_source_indices.size(), 3u);
  EXPECT_EQ(cl.member_source_indices[0], 10u);
  EXPECT_EQ(cl.member_source_indices[1], 20u);
  EXPECT_EQ(cl.member_source_indices[2], 30u);

  EXPECT_NEAR(cl.centroid.x, 0.40, 1e-6);
  EXPECT_NEAR(cl.centroid.y, 0.0, 1e-6);
  EXPECT_NEAR(cl.centroid.z, 0.0, 1e-6);

  EXPECT_NEAR(cl.aabb_min.x, 0.0, 1e-6);
  EXPECT_NEAR(cl.aabb_max.x, 0.80, 1e-6);
}

TEST(DynamicClusteringTest, VerticalSeparationGolden) {
  // Requirement 49:
  // Same XY positions, separated in Z by more than max_vertical_gap_m.
  // Expected: separate components.
  DynamicClusterConfig config{
      .cluster_radius_xy_m = 0.50,
      .max_vertical_gap_m = 0.80,
      .min_cluster_points = 2,
  };

  DynamicPointSet set{};
  // Group 1 at Z = 0.0
  set.points.push_back(DynamicPoint{
      .source_point_index = 1,
      .position = PointXYZ{.x = 1.0, .y = 1.0, .z = 0.0},
      .semantic_class = SemanticClass::VEHICLE,
      .semantic_confidence = 0.8f,
      .motion_confidence = 0.9f,
  });
  set.points.push_back(DynamicPoint{
      .source_point_index = 2,
      .position = PointXYZ{.x = 1.1, .y = 1.0, .z = 0.1},
      .semantic_class = SemanticClass::VEHICLE,
      .semantic_confidence = 0.8f,
      .motion_confidence = 0.9f,
  });

  // Group 2 at Z = 1.5 (gap = 1.4m > 0.80m)
  set.points.push_back(DynamicPoint{
      .source_point_index = 3,
      .position = PointXYZ{.x = 1.0, .y = 1.0, .z = 1.5},
      .semantic_class = SemanticClass::VEHICLE,
      .semantic_confidence = 0.8f,
      .motion_confidence = 0.9f,
  });
  set.points.push_back(DynamicPoint{
      .source_point_index = 4,
      .position = PointXYZ{.x = 1.1, .y = 1.0, .z = 1.6},
      .semantic_class = SemanticClass::VEHICLE,
      .semantic_confidence = 0.8f,
      .motion_confidence = 0.9f,
  });

  auto res = DynamicClusterer::cluster(set, config);
  ASSERT_EQ(res.status, DynamicClusteringStatus::SUCCESS);
  ASSERT_EQ(res.clusters.size(), 2u);
  EXPECT_TRUE(res.noise_source_indices.empty());

  EXPECT_EQ(res.clusters[0].cluster_local_id, 0u);
  EXPECT_EQ(res.clusters[0].canonical_source_point_index, 1u);
  EXPECT_EQ(res.clusters[0].member_source_indices.size(), 2u);

  EXPECT_EQ(res.clusters[1].cluster_local_id, 1u);
  EXPECT_EQ(res.clusters[1].canonical_source_point_index, 3u);
  EXPECT_EQ(res.clusters[1].member_source_indices.size(), 2u);
}

TEST(DynamicClusteringTest, HorizontalSeparationGolden) {
  // Requirement 50:
  // Z similar, XY distance greater than radius.
  // Expected: separate components.
  DynamicClusterConfig config{
      .cluster_radius_xy_m = 0.50,
      .max_vertical_gap_m = 0.80,
      .min_cluster_points = 2,
  };

  DynamicPointSet set{};
  // Group 1 at (0, 0, 0)
  set.points.push_back(DynamicPoint{
      .source_point_index = 10,
      .position = PointXYZ{.x = 0.0, .y = 0.0, .z = 0.0},
      .semantic_class = SemanticClass::PEDESTRIAN,
      .semantic_confidence = 0.85f,
      .motion_confidence = 0.9f,
  });
  set.points.push_back(DynamicPoint{
      .source_point_index = 11,
      .position = PointXYZ{.x = 0.1, .y = 0.0, .z = 0.0},
      .semantic_class = SemanticClass::PEDESTRIAN,
      .semantic_confidence = 0.85f,
      .motion_confidence = 0.9f,
  });

  // Group 2 at (10, 10, 0)
  set.points.push_back(DynamicPoint{
      .source_point_index = 20,
      .position = PointXYZ{.x = 10.0, .y = 10.0, .z = 0.0},
      .semantic_class = SemanticClass::PEDESTRIAN,
      .semantic_confidence = 0.85f,
      .motion_confidence = 0.9f,
  });
  set.points.push_back(DynamicPoint{
      .source_point_index = 21,
      .position = PointXYZ{.x = 10.1, .y = 10.0, .z = 0.0},
      .semantic_class = SemanticClass::PEDESTRIAN,
      .semantic_confidence = 0.85f,
      .motion_confidence = 0.9f,
  });

  auto res = DynamicClusterer::cluster(set, config);
  ASSERT_EQ(res.status, DynamicClusteringStatus::SUCCESS);
  ASSERT_EQ(res.clusters.size(), 2u);
  EXPECT_TRUE(res.noise_source_indices.empty());
  EXPECT_EQ(res.clusters[0].canonical_source_point_index, 10u);
  EXPECT_EQ(res.clusters[1].canonical_source_point_index, 20u);
}

TEST(DynamicClusteringTest, BoundaryInclusionGolden) {
  // Requirement 51:
  // Exactly dx^2 + dy^2 == radius^2 and abs(dz) == max_vertical_gap_m must count as neighbors.
  DynamicClusterConfig config{
      .cluster_radius_xy_m = 0.50,
      .max_vertical_gap_m = 0.80,
      .min_cluster_points = 2,
  };

  // 0.3^2 + 0.4^2 = 0.09 + 0.16 = 0.25 = 0.50^2
  // dz = 0.80 == max_vertical_gap_m
  DynamicPointSet set{};
  set.points.push_back(DynamicPoint{
      .source_point_index = 100,
      .position = PointXYZ{.x = 0.0, .y = 0.0, .z = 0.0},
      .semantic_class = SemanticClass::VEHICLE,
      .semantic_confidence = 0.9f,
      .motion_confidence = 0.9f,
  });
  set.points.push_back(DynamicPoint{
      .source_point_index = 101,
      .position = PointXYZ{.x = 0.30, .y = 0.40, .z = 0.80},
      .semantic_class = SemanticClass::VEHICLE,
      .semantic_confidence = 0.9f,
      .motion_confidence = 0.9f,
  });

  auto res = DynamicClusterer::cluster(set, config);
  ASSERT_EQ(res.status, DynamicClusteringStatus::SUCCESS);
  ASSERT_EQ(res.clusters.size(), 1u);
  EXPECT_EQ(res.clusters[0].member_source_indices.size(), 2u);
  EXPECT_TRUE(res.noise_source_indices.empty());
}

TEST(DynamicClusteringTest, NegativeCoordinateGolden) {
  // Requirement 52:
  // Spatial hash must handle negative coordinates using mathematical floor semantics.
  DynamicClusterConfig config{
      .cluster_radius_xy_m = 0.50,
      .max_vertical_gap_m = 0.80,
      .min_cluster_points = 2,
  };

  DynamicPointSet set{};
  // Points straddling negative voxel boundaries (e.g. around -0.50, -1.0)
  set.points.push_back(DynamicPoint{
      .source_point_index = 5,
      .position = PointXYZ{.x = -0.49, .y = -0.49, .z = -0.2},
      .semantic_class = SemanticClass::VEHICLE,
      .semantic_confidence = 0.8f,
      .motion_confidence = 0.8f,
  });
  set.points.push_back(DynamicPoint{
      .source_point_index = 6,
      .position = PointXYZ{.x = -0.51, .y = -0.49, .z = -0.2},
      .semantic_class = SemanticClass::VEHICLE,
      .semantic_confidence = 0.8f,
      .motion_confidence = 0.8f,
  });

  auto res = DynamicClusterer::cluster(set, config);
  ASSERT_EQ(res.status, DynamicClusteringStatus::SUCCESS);
  ASSERT_EQ(res.clusters.size(), 1u);
  EXPECT_EQ(res.clusters[0].member_source_indices.size(), 2u);
  EXPECT_TRUE(res.noise_source_indices.empty());
}

TEST(DynamicClusteringTest, LargeCoordinateGolden) {
  // Requirement 53:
  // Large coordinates within bounds must not overflow signed integers in hash computation.
  DynamicClusterConfig config{
      .cluster_radius_xy_m = 0.50,
      .max_vertical_gap_m = 0.80,
      .min_cluster_points = 2,
  };

  DynamicPointSet set{};
  set.points.push_back(DynamicPoint{
      .source_point_index = 1,
      .position = PointXYZ{.x = 50000.1, .y = 50000.1, .z = 100.0},
      .semantic_class = SemanticClass::VEHICLE,
      .semantic_confidence = 0.8f,
      .motion_confidence = 0.8f,
  });
  set.points.push_back(DynamicPoint{
      .source_point_index = 2,
      .position = PointXYZ{.x = 50000.3, .y = 50000.2, .z = 100.1},
      .semantic_class = SemanticClass::VEHICLE,
      .semantic_confidence = 0.8f,
      .motion_confidence = 0.8f,
  });

  auto res = DynamicClusterer::cluster(set, config);
  ASSERT_EQ(res.status, DynamicClusteringStatus::SUCCESS);
  ASSERT_EQ(res.clusters.size(), 1u);
  EXPECT_EQ(res.clusters[0].member_source_indices.size(), 2u);
}

TEST(DynamicClusteringTest, MixedClassesGolden) {
  // Requirement 54:
  // 3 high-confidence VEHICLE points + 2 high-confidence PEDESTRIAN points.
  // Weighted semantic vote determines deterministic dominant class.
  DynamicClusterConfig config{
      .cluster_radius_xy_m = 0.50,
      .max_vertical_gap_m = 0.80,
      .min_cluster_points = 5,
      .min_semantic_confidence = 0.50f,
  };

  DynamicPointSet set{};
  // 3 vehicle points, confidence 0.80 -> total weight = 2.40
  for (uint32_t i = 0; i < 3; ++i) {
    set.points.push_back(DynamicPoint{
        .source_point_index = i,
        .position = PointXYZ{.x = 0.1 * i, .y = 0.0, .z = 0.0},
        .semantic_class = SemanticClass::VEHICLE,
        .semantic_confidence = 0.80f,
        .motion_confidence = 0.90f,
    });
  }
  // 2 pedestrian points, confidence 0.90 -> total weight = 1.80
  for (uint32_t i = 3; i < 5; ++i) {
    set.points.push_back(DynamicPoint{
        .source_point_index = i,
        .position = PointXYZ{.x = 0.1 * i, .y = 0.0, .z = 0.0},
        .semantic_class = SemanticClass::PEDESTRIAN,
        .semantic_confidence = 0.90f,
        .motion_confidence = 0.90f,
    });
  }

  auto res = DynamicClusterer::cluster(set, config);
  ASSERT_EQ(res.status, DynamicClusteringStatus::SUCCESS);
  ASSERT_EQ(res.clusters.size(), 1u);
  EXPECT_EQ(res.clusters[0].dominant_class, SemanticClass::VEHICLE);
  EXPECT_NEAR(res.clusters[0].mean_semantic_confidence, (3 * 0.80f + 2 * 0.90f) / 5.0f, 1e-5);
  EXPECT_NEAR(res.clusters[0].mean_motion_confidence, 0.90f, 1e-5);
}

TEST(DynamicClusteringTest, SemanticTieGolden) {
  // Requirement 55:
  // Equal weighted class vote. Tie-break: lowest canonical SemanticClass value.
  // GROUND = 1, VEHICLE = 5
  DynamicClusterConfig config{
      .cluster_radius_xy_m = 0.50,
      .max_vertical_gap_m = 0.80,
      .min_cluster_points = 2,
      .min_semantic_confidence = 0.50f,
  };

  DynamicPointSet set{};
  set.points.push_back(DynamicPoint{
      .source_point_index = 1,
      .position = PointXYZ{.x = 0.0, .y = 0.0, .z = 0.0},
      .semantic_class = SemanticClass::VEHICLE,
      .semantic_confidence = 0.80f,
      .motion_confidence = 0.90f,
  });
  set.points.push_back(DynamicPoint{
      .source_point_index = 2,
      .position = PointXYZ{.x = 0.1, .y = 0.0, .z = 0.0},
      .semantic_class = SemanticClass::GROUND,
      .semantic_confidence = 0.80f,
      .motion_confidence = 0.90f,
  });

  auto res = DynamicClusterer::cluster(set, config);
  ASSERT_EQ(res.status, DynamicClusteringStatus::SUCCESS);
  ASSERT_EQ(res.clusters.size(), 1u);
  // GROUND (1) < VEHICLE (5), so GROUND must win the tie-break!
  EXPECT_EQ(res.clusters[0].dominant_class, SemanticClass::GROUND);
}

TEST(DynamicClusteringTest, AllUnknownOrLowConfidenceGolden) {
  // Requirement 56:
  // Cluster with no confidence-qualified meaningful semantic class -> dominant_class = UNKNOWN.
  DynamicClusterConfig config{
      .cluster_radius_xy_m = 0.50,
      .max_vertical_gap_m = 0.80,
      .min_cluster_points = 2,
      .min_semantic_confidence = 0.70f,
  };

  DynamicPointSet set{};
  // Point 1: UNKNOWN class
  set.points.push_back(DynamicPoint{
      .source_point_index = 1,
      .position = PointXYZ{.x = 0.0, .y = 0.0, .z = 0.0},
      .semantic_class = SemanticClass::UNKNOWN,
      .semantic_confidence = 0.99f,
      .motion_confidence = 0.90f,
  });
  // Point 2: VEHICLE but low confidence (0.40 < min 0.70)
  set.points.push_back(DynamicPoint{
      .source_point_index = 2,
      .position = PointXYZ{.x = 0.1, .y = 0.0, .z = 0.0},
      .semantic_class = SemanticClass::VEHICLE,
      .semantic_confidence = 0.40f,
      .motion_confidence = 0.90f,
  });

  auto res = DynamicClusterer::cluster(set, config);
  ASSERT_EQ(res.status, DynamicClusteringStatus::SUCCESS);
  ASSERT_EQ(res.clusters.size(), 1u);
  EXPECT_EQ(res.clusters[0].dominant_class, SemanticClass::UNKNOWN);
}

TEST(DynamicClusteringTest, NoiseContractAndConservation) {
  // Requirement 32, 45, 46:
  // Component size < min_cluster_points -> noise points.
  // Cluster members + noise == total points. Original source indices preserved.
  DynamicClusterConfig config{
      .cluster_radius_xy_m = 0.50,
      .max_vertical_gap_m = 0.80,
      .min_cluster_points = 4,
  };

  DynamicPointSet set{};
  // Valid cluster of 4 points at (0, 0, 0)
  for (uint32_t i = 0; i < 4; ++i) {
    set.points.push_back(DynamicPoint{
        .source_point_index = 100 + i,
        .position = PointXYZ{.x = 0.05 * i, .y = 0.0, .z = 0.0},
        .semantic_class = SemanticClass::VEHICLE,
        .semantic_confidence = 0.85f,
        .motion_confidence = 0.90f,
    });
  }

  // Noise component of 2 points at (5, 5, 0)
  for (uint32_t i = 0; i < 2; ++i) {
    set.points.push_back(DynamicPoint{
        .source_point_index = 200 + i,
        .position = PointXYZ{.x = 5.0 + 0.05 * i, .y = 5.0, .z = 0.0},
        .semantic_class = SemanticClass::VEHICLE,
        .semantic_confidence = 0.85f,
        .motion_confidence = 0.90f,
    });
  }

  // Single isolated noise point at (10, 10, 0)
  set.points.push_back(DynamicPoint{
      .source_point_index = 300,
      .position = PointXYZ{.x = 10.0, .y = 10.0, .z = 0.0},
      .semantic_class = SemanticClass::PEDESTRIAN,
      .semantic_confidence = 0.85f,
      .motion_confidence = 0.90f,
  });

  auto res = DynamicClusterer::cluster(set, config);
  ASSERT_EQ(res.status, DynamicClusteringStatus::SUCCESS);
  ASSERT_EQ(res.clusters.size(), 1u);
  EXPECT_EQ(res.clusters[0].member_source_indices.size(), 4u);
  EXPECT_EQ(res.clusters[0].cluster_local_id, 0u);
  EXPECT_EQ(res.clusters[0].canonical_source_point_index, 100u);

  EXPECT_EQ(res.noise_source_indices.size(), 3u);
  EXPECT_EQ(res.noise_source_indices[0], 200u);
  EXPECT_EQ(res.noise_source_indices[1], 201u);
  EXPECT_EQ(res.noise_source_indices[2], 300u);

  EXPECT_EQ(res.diagnostics.input_points, 7u);
  EXPECT_EQ(res.diagnostics.clustered_points, 4u);
  EXPECT_EQ(res.diagnostics.noise_points, 3u);
  EXPECT_EQ(res.diagnostics.clustered_points + res.diagnostics.noise_points,
            res.diagnostics.input_points);
}

TEST(DynamicClusteringTest, DensePathologicalNeighborhood) {
  // Requirement 30:
  // Dense input test with many points in one spatial neighborhood.
  // Verifies correctness, determinism, and no recursion / stack overflow.
  DynamicClusterConfig config{
      .cluster_radius_xy_m = 0.50,
      .max_vertical_gap_m = 0.80,
      .min_cluster_points = 5,
  };

  DynamicPointSet set{};
  constexpr uint32_t kNumPoints = 500;
  for (uint32_t i = 0; i < kNumPoints; ++i) {
    double offset = static_cast<double>(i) * 0.0005;
    set.points.push_back(DynamicPoint{
        .source_point_index = i,
        .position = PointXYZ{.x = offset, .y = offset, .z = offset},
        .semantic_class = SemanticClass::VEHICLE,
        .semantic_confidence = 0.9f,
        .motion_confidence = 0.95f,
    });
  }

  auto res = DynamicClusterer::cluster(set, config);
  ASSERT_EQ(res.status, DynamicClusteringStatus::SUCCESS);
  ASSERT_EQ(res.clusters.size(), 1u);
  EXPECT_EQ(res.clusters[0].member_source_indices.size(), kNumPoints);
  EXPECT_EQ(res.diagnostics.input_points, kNumPoints);
  EXPECT_EQ(res.diagnostics.clustered_points, kNumPoints);
  EXPECT_EQ(res.diagnostics.noise_points, 0u);
}

TEST(DynamicClusteringTest, RepeatabilityTest) {
  // Requirement 73:
  // Two independent runs on identical input produce bit-identical results.
  DynamicClusterConfig config{
      .cluster_radius_xy_m = 0.50,
      .max_vertical_gap_m = 0.80,
      .min_cluster_points = 3,
  };

  DynamicPointSet set{};
  for (uint32_t i = 0; i < 20; ++i) {
    set.points.push_back(DynamicPoint{
        .source_point_index = i * 2,
        .position = PointXYZ{
            .x = (i % 4) * 0.1 + (i / 4) * 2.0,
            .y = (i % 4) * 0.1,
            .z = (i % 4) * 0.05,
        },
        .semantic_class = (i % 2 == 0) ? SemanticClass::VEHICLE : SemanticClass::PEDESTRIAN,
        .semantic_confidence = 0.80f + 0.01f * (i % 10),
        .motion_confidence = 0.90f,
    });
  }

  auto res1 = DynamicClusterer::cluster(set, config);
  auto res2 = DynamicClusterer::cluster(set, config);

  EXPECT_EQ(res1.status, res2.status);
  EXPECT_EQ(res1.diagnostics, res2.diagnostics);
  EXPECT_EQ(res1.noise_source_indices, res2.noise_source_indices);
  ASSERT_EQ(res1.clusters.size(), res2.clusters.size());
  for (size_t k = 0; k < res1.clusters.size(); ++k) {
    EXPECT_EQ(res1.clusters[k], res2.clusters[k]);
  }
}

TEST(DynamicClusteringTest, StandaloneDependencyAudit) {
  // Requirement 64:
  // DynamicClusterer can be called without constructing map or cache objects.
  DynamicClusterer clusterer{};
  DynamicPointSet set{};
  DynamicClusterConfig config{};
  auto res = clusterer(set, config);
  EXPECT_EQ(res.status, DynamicClusteringStatus::SUCCESS);
}

TEST(DynamicClusteringTest, SafeVoxelQuantizationTinyRadius) {
  // Requirement 5:
  // cluster_radius_xy_m = std::numeric_limits<double>::min() with ordinary coordinate x = 1000.0.
  // Must return clean NUMERIC_ERROR, no UB, no sanitizer issue.
  DynamicClusterConfig config{
      .cluster_radius_xy_m = std::numeric_limits<double>::min(),
      .max_vertical_gap_m = 0.80,
      .min_cluster_points = 1,
  };
  DynamicPointSet set{};
  set.points.push_back(DynamicPoint{
      .source_point_index = 0,
      .position = PointXYZ{.x = 1000.0, .y = 0.0, .z = 0.0},
      .semantic_class = SemanticClass::VEHICLE,
      .semantic_confidence = 0.9f,
      .motion_confidence = 0.9f,
  });

  auto res = DynamicClusterer::cluster(set, config);
  EXPECT_EQ(res.status, DynamicClusteringStatus::NUMERIC_ERROR);
  EXPECT_FALSE(res.isSuccess());
  EXPECT_TRUE(res.clusters.empty());
  EXPECT_TRUE(res.noise_source_indices.empty());
}

TEST(DynamicClusteringTest, SafeVoxelQuantizationVerticalSafety) {
  // Requirement 6:
  // Extremely small positive vertical threshold with ordinary coordinate z = 1000.0.
  DynamicClusterConfig config{
      .cluster_radius_xy_m = 0.50,
      .max_vertical_gap_m = std::numeric_limits<double>::min(),
      .min_cluster_points = 1,
  };
  DynamicPointSet set{};
  set.points.push_back(DynamicPoint{
      .source_point_index = 0,
      .position = PointXYZ{.x = 0.0, .y = 0.0, .z = 1000.0},
      .semantic_class = SemanticClass::VEHICLE,
      .semantic_confidence = 0.9f,
      .motion_confidence = 0.9f,
  });

  auto res = DynamicClusterer::cluster(set, config);
  EXPECT_EQ(res.status, DynamicClusteringStatus::NUMERIC_ERROR);
  EXPECT_FALSE(res.isSuccess());
  EXPECT_TRUE(res.clusters.empty());
  EXPECT_TRUE(res.noise_source_indices.empty());
}

TEST(DynamicClusteringTest, LargeSafeRatioGolden) {
  // Requirement 9:
  // Largest comfortably representable positive and negative voxel coordinates near checked boundary.
  // Tested ratio: +/- 8.99e18 with voxel_size = 1.0.
  DynamicClusterConfig config{
      .cluster_radius_xy_m = 1.0,
      .max_vertical_gap_m = 1.0,
      .min_cluster_points = 2,
  };

  DynamicPointSet set_pos{};
  set_pos.points.push_back(DynamicPoint{
      .source_point_index = 1,
      .position = PointXYZ{.x = 8.99e18, .y = 0.0, .z = 0.0},
      .semantic_class = SemanticClass::VEHICLE,
      .semantic_confidence = 0.9f,
      .motion_confidence = 0.9f,
  });
  set_pos.points.push_back(DynamicPoint{
      .source_point_index = 2,
      .position = PointXYZ{.x = 8.99e18 + 0.5, .y = 0.0, .z = 0.0},
      .semantic_class = SemanticClass::VEHICLE,
      .semantic_confidence = 0.9f,
      .motion_confidence = 0.9f,
  });

  auto res_pos = DynamicClusterer::cluster(set_pos, config);
  EXPECT_EQ(res_pos.status, DynamicClusteringStatus::SUCCESS);
  EXPECT_TRUE(res_pos.isSuccess());
  EXPECT_EQ(res_pos.clusters.size(), 1u);

  DynamicPointSet set_neg{};
  set_neg.points.push_back(DynamicPoint{
      .source_point_index = 10,
      .position = PointXYZ{.x = -8.99e18, .y = 0.0, .z = 0.0},
      .semantic_class = SemanticClass::VEHICLE,
      .semantic_confidence = 0.9f,
      .motion_confidence = 0.9f,
  });
  set_neg.points.push_back(DynamicPoint{
      .source_point_index = 11,
      .position = PointXYZ{.x = -8.99e18 + 0.5, .y = 0.0, .z = 0.0},
      .semantic_class = SemanticClass::VEHICLE,
      .semantic_confidence = 0.9f,
      .motion_confidence = 0.9f,
  });

  auto res_neg = DynamicClusterer::cluster(set_neg, config);
  EXPECT_EQ(res_neg.status, DynamicClusteringStatus::SUCCESS);
  EXPECT_TRUE(res_neg.isSuccess());
  EXPECT_EQ(res_neg.clusters.size(), 1u);
}

TEST(DynamicClusteringTest, OutOfRangeRatioGolden) {
  // Requirement 10:
  // Finite coordinate + finite positive voxel size whose ratio exceeds integer range.
  DynamicClusterConfig config{
      .cluster_radius_xy_m = 1e-19,
      .max_vertical_gap_m = 0.80,
      .min_cluster_points = 1,
  };
  DynamicPointSet set{};
  set.points.push_back(DynamicPoint{
      .source_point_index = 1,
      .position = PointXYZ{.x = 1000.0, .y = 0.0, .z = 0.0},
      .semantic_class = SemanticClass::VEHICLE,
      .semantic_confidence = 0.9f,
      .motion_confidence = 0.9f,
  });

  auto res = DynamicClusterer::cluster(set, config);
  EXPECT_EQ(res.status, DynamicClusteringStatus::NUMERIC_ERROR);
  EXPECT_FALSE(res.isSuccess());
  EXPECT_TRUE(res.clusters.empty());
  EXPECT_TRUE(res.noise_source_indices.empty());
}

TEST(DynamicClusteringTest, NegativeOutOfRangeRatio) {
  // Requirement 11:
  // Negative coordinate whose ratio exceeds integer range.
  DynamicClusterConfig config{
      .cluster_radius_xy_m = 1e-19,
      .max_vertical_gap_m = 0.80,
      .min_cluster_points = 1,
  };
  DynamicPointSet set{};
  set.points.push_back(DynamicPoint{
      .source_point_index = 1,
      .position = PointXYZ{.x = -1000.0, .y = 0.0, .z = 0.0},
      .semantic_class = SemanticClass::VEHICLE,
      .semantic_confidence = 0.9f,
      .motion_confidence = 0.9f,
  });

  auto res = DynamicClusterer::cluster(set, config);
  EXPECT_EQ(res.status, DynamicClusteringStatus::NUMERIC_ERROR);
  EXPECT_FALSE(res.isSuccess());
  EXPECT_TRUE(res.clusters.empty());
  EXPECT_TRUE(res.noise_source_indices.empty());
}

TEST(DynamicClusteringTest, DuplicateSourceIndexRejected) {
  // Requirement 13 & 24:
  // Duplicate source_point_index makes cluster membership ambiguous -> reject deterministically with INVALID_INPUT.
  // Clusters empty, noise empty.
  DynamicClusterConfig config{
      .cluster_radius_xy_m = 0.50,
      .max_vertical_gap_m = 0.80,
      .min_cluster_points = 2,
  };
  DynamicPointSet set{};
  set.points.push_back(DynamicPoint{
      .source_point_index = 42,
      .position = PointXYZ{.x = 1.0, .y = 1.0, .z = 0.0},
      .semantic_class = SemanticClass::VEHICLE,
      .semantic_confidence = 0.9f,
      .motion_confidence = 0.9f,
  });
  set.points.push_back(DynamicPoint{
      .source_point_index = 42,  // Duplicate!
      .position = PointXYZ{.x = 1.1, .y = 1.0, .z = 0.0},
      .semantic_class = SemanticClass::VEHICLE,
      .semantic_confidence = 0.9f,
      .motion_confidence = 0.9f,
  });

  auto res = DynamicClusterer::cluster(set, config);
  EXPECT_EQ(res.status, DynamicClusteringStatus::INVALID_INPUT);
  EXPECT_FALSE(res.isSuccess());
  EXPECT_TRUE(res.clusters.empty());
  EXPECT_TRUE(res.noise_source_indices.empty());
}

TEST(DynamicClusteringTest, InputOrderIndependenceAndNoMutation) {
  // Requirements 14, 16, 17:
  // Same logical DynamicPointSet in different vector orders produce 100% bit-identical results.
  // Caller input is not mutated.
  DynamicClusterConfig config{
      .cluster_radius_xy_m = 0.50,
      .max_vertical_gap_m = 0.80,
      .min_cluster_points = 3,
  };

  DynamicPoint p1{
      .source_point_index = 10,
      .position = PointXYZ{.x = 0.0, .y = 0.0, .z = 0.0},
      .semantic_class = SemanticClass::VEHICLE,
      .semantic_confidence = 0.85f,
      .motion_confidence = 0.90f,
  };
  DynamicPoint p2{
      .source_point_index = 25,
      .position = PointXYZ{.x = 0.2, .y = 0.0, .z = 0.0},
      .semantic_class = SemanticClass::VEHICLE,
      .semantic_confidence = 0.80f,
      .motion_confidence = 0.95f,
  };
  DynamicPoint p3{
      .source_point_index = 100,
      .position = PointXYZ{.x = 0.4, .y = 0.0, .z = 0.0},
      .semantic_class = SemanticClass::VEHICLE,
      .semantic_confidence = 0.90f,
      .motion_confidence = 0.85f,
  };
  DynamicPoint p_noise{
      .source_point_index = 500,
      .position = PointXYZ{.x = 5.0, .y = 5.0, .z = 0.0},
      .semantic_class = SemanticClass::PEDESTRIAN,
      .semantic_confidence = 0.70f,
      .motion_confidence = 0.80f,
  };

  DynamicPointSet set_asc{};
  set_asc.points = {p1, p2, p3, p_noise};
  const DynamicPointSet original_copy = set_asc;

  DynamicPointSet set_shuffled{};
  set_shuffled.points = {p3, p_noise, p1, p2};

  auto res1 = DynamicClusterer::cluster(set_asc, config);
  auto res2 = DynamicClusterer::cluster(set_shuffled, config);

  // Input must not be mutated
  EXPECT_EQ(set_asc, original_copy);

  // Exact identity across all fields
  EXPECT_EQ(res1.status, res2.status);
  EXPECT_EQ(res1.diagnostics, res2.diagnostics);
  EXPECT_EQ(res1.noise_source_indices, res2.noise_source_indices);
  ASSERT_EQ(res1.clusters.size(), res2.clusters.size());
  for (size_t k = 0; k < res1.clusters.size(); ++k) {
    EXPECT_EQ(res1.clusters[k].cluster_local_id, res2.clusters[k].cluster_local_id);
    EXPECT_EQ(res1.clusters[k].canonical_source_point_index, res2.clusters[k].canonical_source_point_index);
    EXPECT_EQ(res1.clusters[k].member_source_indices, res2.clusters[k].member_source_indices);
    EXPECT_EQ(res1.clusters[k].centroid, res2.clusters[k].centroid);
    EXPECT_EQ(res1.clusters[k].aabb_min, res2.clusters[k].aabb_min);
    EXPECT_EQ(res1.clusters[k].aabb_max, res2.clusters[k].aabb_max);
    EXPECT_EQ(res1.clusters[k].dominant_class, res2.clusters[k].dominant_class);
    EXPECT_EQ(res1.clusters[k].mean_semantic_confidence, res2.clusters[k].mean_semantic_confidence);
    EXPECT_EQ(res1.clusters[k].mean_motion_confidence, res2.clusters[k].mean_motion_confidence);
  }

  // Conservation check: sum cluster member counts + noise count == input point count
  EXPECT_EQ(res1.diagnostics.clustered_points + res1.diagnostics.noise_points, 4u);
}

TEST(DynamicClusteringTest, NonContiguousSourceIndices) {
  // Requirement 15:
  // Source indices: 2, 10, 100, 5000.
  // Valid standalone set, output preserves original source indices.
  DynamicClusterConfig config{
      .cluster_radius_xy_m = 0.50,
      .max_vertical_gap_m = 0.80,
      .min_cluster_points = 2,
  };
  DynamicPointSet set{};
  set.points.push_back(DynamicPoint{
      .source_point_index = 5000,
      .position = PointXYZ{.x = 0.0, .y = 0.0, .z = 0.0},
      .semantic_class = SemanticClass::VEHICLE,
      .semantic_confidence = 0.9f,
      .motion_confidence = 0.9f,
  });
  set.points.push_back(DynamicPoint{
      .source_point_index = 2,
      .position = PointXYZ{.x = 0.1, .y = 0.0, .z = 0.0},
      .semantic_class = SemanticClass::VEHICLE,
      .semantic_confidence = 0.9f,
      .motion_confidence = 0.9f,
  });
  set.points.push_back(DynamicPoint{
      .source_point_index = 100,
      .position = PointXYZ{.x = 5.0, .y = 5.0, .z = 0.0},
      .semantic_class = SemanticClass::PEDESTRIAN,
      .semantic_confidence = 0.9f,
      .motion_confidence = 0.9f,
  });
  set.points.push_back(DynamicPoint{
      .source_point_index = 10,
      .position = PointXYZ{.x = 5.1, .y = 5.0, .z = 0.0},
      .semantic_class = SemanticClass::PEDESTRIAN,
      .semantic_confidence = 0.9f,
      .motion_confidence = 0.9f,
  });

  auto res = DynamicClusterer::cluster(set, config);
  ASSERT_EQ(res.status, DynamicClusteringStatus::SUCCESS);
  ASSERT_EQ(res.clusters.size(), 2u);

  // Cluster 0: points 2, 5000 (canonical = 2)
  EXPECT_EQ(res.clusters[0].canonical_source_point_index, 2u);
  ASSERT_EQ(res.clusters[0].member_source_indices.size(), 2u);
  EXPECT_EQ(res.clusters[0].member_source_indices[0], 2u);
  EXPECT_EQ(res.clusters[0].member_source_indices[1], 5000u);

  // Cluster 1: points 10, 100 (canonical = 10)
  EXPECT_EQ(res.clusters[1].canonical_source_point_index, 10u);
  ASSERT_EQ(res.clusters[1].member_source_indices.size(), 2u);
  EXPECT_EQ(res.clusters[1].member_source_indices[0], 10u);
  EXPECT_EQ(res.clusters[1].member_source_indices[1], 100u);
}

TEST(DynamicClusteringTest, ZeroVerticalGapDirect) {
  // Requirement 26:
  // max_vertical_gap_m = 0.
  // Points with identical z may cluster if XY condition passes.
  // Points with any non-zero dz do not.
  DynamicClusterConfig config{
      .cluster_radius_xy_m = 0.50,
      .max_vertical_gap_m = 0.0,
      .min_cluster_points = 2,
  };
  EXPECT_TRUE(config.isValid());

  DynamicPointSet set{};
  // Points 1 and 2 have dz == 0 -> should cluster
  set.points.push_back(DynamicPoint{
      .source_point_index = 1,
      .position = PointXYZ{.x = 1.0, .y = 1.0, .z = 0.50},
      .semantic_class = SemanticClass::VEHICLE,
      .semantic_confidence = 0.9f,
      .motion_confidence = 0.9f,
  });
  set.points.push_back(DynamicPoint{
      .source_point_index = 2,
      .position = PointXYZ{.x = 1.1, .y = 1.0, .z = 0.50},
      .semantic_class = SemanticClass::VEHICLE,
      .semantic_confidence = 0.9f,
      .motion_confidence = 0.9f,
  });

  // Point 3 has dz = 0.001 != 0 -> must NOT cluster with 1 and 2
  set.points.push_back(DynamicPoint{
      .source_point_index = 3,
      .position = PointXYZ{.x = 1.05, .y = 1.0, .z = 0.501},
      .semantic_class = SemanticClass::VEHICLE,
      .semantic_confidence = 0.9f,
      .motion_confidence = 0.9f,
  });

  auto res = DynamicClusterer::cluster(set, config);
  ASSERT_EQ(res.status, DynamicClusteringStatus::SUCCESS);
  ASSERT_EQ(res.clusters.size(), 1u);
  EXPECT_EQ(res.clusters[0].member_source_indices.size(), 2u);
  EXPECT_EQ(res.clusters[0].member_source_indices[0], 1u);
  EXPECT_EQ(res.clusters[0].member_source_indices[1], 2u);

  ASSERT_EQ(res.noise_source_indices.size(), 1u);
  EXPECT_EQ(res.noise_source_indices[0], 3u);
}

TEST(DynamicClusteringTest, VerySmallButSafeVoxelSize) {
  // Requirement 27:
  // Small radius that still yields representable voxel coords (e.g. 0.005m).
  // Separates "small" from "unsafe".
  DynamicClusterConfig config{
      .cluster_radius_xy_m = 0.005,
      .max_vertical_gap_m = 0.010,
      .min_cluster_points = 2,
  };
  EXPECT_TRUE(config.isValid());

  DynamicPointSet set{};
  set.points.push_back(DynamicPoint{
      .source_point_index = 10,
      .position = PointXYZ{.x = 10.000, .y = 10.000, .z = 1.000},
      .semantic_class = SemanticClass::VEHICLE,
      .semantic_confidence = 0.9f,
      .motion_confidence = 0.9f,
  });
  set.points.push_back(DynamicPoint{
      .source_point_index = 20,
      .position = PointXYZ{.x = 10.003, .y = 10.000, .z = 1.000},
      .semantic_class = SemanticClass::VEHICLE,
      .semantic_confidence = 0.9f,
      .motion_confidence = 0.9f,
  });

  auto res = DynamicClusterer::cluster(set, config);
  EXPECT_EQ(res.status, DynamicClusteringStatus::SUCCESS);
  ASSERT_EQ(res.clusters.size(), 1u);
  EXPECT_EQ(res.clusters[0].member_source_indices.size(), 2u);
}

TEST(DynamicClusteringTest, ConfigEdgeCasesExtended) {
  // Requirement 25:
  // Explicitly test:
  // radius NaN, Inf, 0, negative
  // vertical NaN, Inf, negative
  // min_cluster_points == 0
  // semantic confidence threshold NaN, Inf, negative, >1
  // All deterministic INVALID_CONFIG.
  DynamicPointSet pts{};
  pts.points.push_back(DynamicPoint{
      .source_point_index = 1,
      .position = PointXYZ{.x = 0.0, .y = 0.0, .z = 0.0},
      .semantic_class = SemanticClass::VEHICLE,
      .semantic_confidence = 0.9f,
      .motion_confidence = 0.9f,
  });

  auto check_invalid = [&](DynamicClusterConfig cfg) {
    EXPECT_FALSE(cfg.isValid());
    auto r = DynamicClusterer::cluster(pts, cfg);
    EXPECT_EQ(r.status, DynamicClusteringStatus::INVALID_CONFIG);
    EXPECT_TRUE(r.clusters.empty());
    EXPECT_TRUE(r.noise_source_indices.empty());
  };

  DynamicClusterConfig base{};

  // Radius variations
  {
    auto c = base; c.cluster_radius_xy_m = std::numeric_limits<double>::quiet_NaN(); check_invalid(c);
    c = base; c.cluster_radius_xy_m = std::numeric_limits<double>::infinity(); check_invalid(c);
    c = base; c.cluster_radius_xy_m = 0.0; check_invalid(c);
    c = base; c.cluster_radius_xy_m = -1.0; check_invalid(c);
  }

  // Vertical gap variations
  {
    auto c = base; c.max_vertical_gap_m = std::numeric_limits<double>::quiet_NaN(); check_invalid(c);
    c = base; c.max_vertical_gap_m = std::numeric_limits<double>::infinity(); check_invalid(c);
    c = base; c.max_vertical_gap_m = -0.001; check_invalid(c);
  }

  // Min cluster points
  {
    auto c = base; c.min_cluster_points = 0; check_invalid(c);
  }

  // Semantic confidence threshold variations
  {
    auto c = base; c.min_semantic_confidence = std::numeric_limits<float>::quiet_NaN(); check_invalid(c);
    c = base; c.min_semantic_confidence = std::numeric_limits<float>::infinity(); check_invalid(c);
    c = base; c.min_semantic_confidence = -0.01f; check_invalid(c);
    c = base; c.min_semantic_confidence = 1.01f; check_invalid(c);
  }
}

TEST(DynamicClusteringTest, InvalidSemanticEnumGolden) {
  // Requirement 8:
  // Construct valid finite DynamicPoint with semantic_class = static_cast<SemanticClass>(255).
  // Expected: INVALID_INPUT, 0 clusters, 0 noise, ASan/UBSan clean.
  DynamicClusterConfig config{};
  DynamicPointSet set{};
  set.points.push_back(DynamicPoint{
      .source_point_index = 0,
      .position = PointXYZ{.x = 1.0, .y = 1.0, .z = 0.0},
      .semantic_class = static_cast<SemanticClass>(255),
      .semantic_confidence = 0.9f,
      .motion_confidence = 0.9f,
  });

  auto res = DynamicClusterer::cluster(set, config);
  EXPECT_EQ(res.status, DynamicClusteringStatus::INVALID_INPUT);
  EXPECT_FALSE(res.isSuccess());
  EXPECT_TRUE(res.clusters.empty());
  EXPECT_TRUE(res.noise_source_indices.empty());
}

TEST(DynamicClusteringTest, InvalidSemanticConfidenceTests) {
  // Requirement 9:
  // Test independently: NaN, +Inf, -Inf, -0.01, 1.01 -> INVALID_INPUT.
  DynamicClusterConfig config{};
  std::vector<float> bad_confs = {
      std::numeric_limits<float>::quiet_NaN(),
      std::numeric_limits<float>::infinity(),
      -std::numeric_limits<float>::infinity(),
      -0.01f,
      1.01f,
  };

  for (float bad_val : bad_confs) {
    DynamicPointSet set{};
    set.points.push_back(DynamicPoint{
        .source_point_index = 0,
        .position = PointXYZ{.x = 1.0, .y = 1.0, .z = 0.0},
        .semantic_class = SemanticClass::VEHICLE,
        .semantic_confidence = bad_val,
        .motion_confidence = 0.9f,
    });

    auto res = DynamicClusterer::cluster(set, config);
    EXPECT_EQ(res.status, DynamicClusteringStatus::INVALID_INPUT) << "Failed for val: " << bad_val;
    EXPECT_FALSE(res.isSuccess());
    EXPECT_TRUE(res.clusters.empty());
    EXPECT_TRUE(res.noise_source_indices.empty());
  }
}

TEST(DynamicClusteringTest, InvalidMotionConfidenceTests) {
  // Requirement 10:
  // Test independently: NaN, +Inf, -Inf, -0.01, 1.01 -> INVALID_INPUT.
  DynamicClusterConfig config{};
  std::vector<float> bad_confs = {
      std::numeric_limits<float>::quiet_NaN(),
      std::numeric_limits<float>::infinity(),
      -std::numeric_limits<float>::infinity(),
      -0.01f,
      1.01f,
  };

  for (float bad_val : bad_confs) {
    DynamicPointSet set{};
    set.points.push_back(DynamicPoint{
        .source_point_index = 0,
        .position = PointXYZ{.x = 1.0, .y = 1.0, .z = 0.0},
        .semantic_class = SemanticClass::VEHICLE,
        .semantic_confidence = 0.9f,
        .motion_confidence = bad_val,
    });

    auto res = DynamicClusterer::cluster(set, config);
    EXPECT_EQ(res.status, DynamicClusteringStatus::INVALID_INPUT) << "Failed for val: " << bad_val;
    EXPECT_FALSE(res.isSuccess());
    EXPECT_TRUE(res.clusters.empty());
    EXPECT_TRUE(res.noise_source_indices.empty());
  }
}

TEST(DynamicClusteringTest, ValidConfidenceBoundaries) {
  // Requirement 11:
  // Explicitly test semantic_confidence = 0.0, 1.0 and motion_confidence = 0.0, 1.0.
  // All are structurally valid; clustering proceeds normally.
  DynamicClusterConfig config{
      .cluster_radius_xy_m = 0.50,
      .max_vertical_gap_m = 0.80,
      .min_cluster_points = 2,
      .min_semantic_confidence = 0.50f,
  };

  DynamicPointSet set{};
  set.points.push_back(DynamicPoint{
      .source_point_index = 1,
      .position = PointXYZ{.x = 0.0, .y = 0.0, .z = 0.0},
      .semantic_class = SemanticClass::VEHICLE,
      .semantic_confidence = 0.0f,
      .motion_confidence = 0.0f,
  });
  set.points.push_back(DynamicPoint{
      .source_point_index = 2,
      .position = PointXYZ{.x = 0.1, .y = 0.0, .z = 0.0},
      .semantic_class = SemanticClass::VEHICLE,
      .semantic_confidence = 1.0f,
      .motion_confidence = 1.0f,
  });

  auto res = DynamicClusterer::cluster(set, config);
  EXPECT_EQ(res.status, DynamicClusteringStatus::SUCCESS);
  EXPECT_TRUE(res.isSuccess());
  ASSERT_EQ(res.clusters.size(), 1u);
  EXPECT_EQ(res.clusters[0].member_source_indices.size(), 2u);
  // Point 2 (conf 1.0 >= 0.5) voted VEHICLE; Point 1 (conf 0.0 < 0.5) did not
  EXPECT_EQ(res.clusters[0].dominant_class, SemanticClass::VEHICLE);
  EXPECT_NEAR(res.clusters[0].mean_semantic_confidence, 0.50f, 1e-6);
  EXPECT_NEAR(res.clusters[0].mean_motion_confidence, 0.50f, 1e-6);
}

TEST(DynamicClusteringTest, ValidLowConfidenceBelowThreshold) {
  // Requirement 7:
  // Values inside [0, 1] but below config.min_semantic_confidence are VALID input.
  // They simply do not cast a semantic class vote.
  DynamicClusterConfig config{
      .cluster_radius_xy_m = 0.50,
      .max_vertical_gap_m = 0.80,
      .min_cluster_points = 2,
      .min_semantic_confidence = 0.70f,
  };

  DynamicPointSet set{};
  set.points.push_back(DynamicPoint{
      .source_point_index = 1,
      .position = PointXYZ{.x = 0.0, .y = 0.0, .z = 0.0},
      .semantic_class = SemanticClass::PEDESTRIAN,
      .semantic_confidence = 0.30f,  // < 0.70 threshold
      .motion_confidence = 0.80f,
  });
  set.points.push_back(DynamicPoint{
      .source_point_index = 2,
      .position = PointXYZ{.x = 0.1, .y = 0.0, .z = 0.0},
      .semantic_class = SemanticClass::PEDESTRIAN,
      .semantic_confidence = 0.40f,  // < 0.70 threshold
      .motion_confidence = 0.80f,
  });

  auto res = DynamicClusterer::cluster(set, config);
  EXPECT_EQ(res.status, DynamicClusteringStatus::SUCCESS);
  ASSERT_EQ(res.clusters.size(), 1u);
  // Neither point met 0.70 threshold -> dominant class is UNKNOWN
  EXPECT_EQ(res.clusters[0].dominant_class, SemanticClass::UNKNOWN);
}

TEST(DynamicClusteringTest, ExtremeHashGolden) {
  // Requirements 14, 15, 16:
  // Use successfully quantized voxel coordinates near the permitted +/- 9e18 boundary.
  // Insert/query them through actual spatial hash.
  // Expected: SUCCESS, correct bucket retrieval, ASan clean, UBSan clean.
  DynamicClusterConfig config{
      .cluster_radius_xy_m = 1.0,
      .max_vertical_gap_m = 1.0,
      .min_cluster_points = 2,
  };

  // Group at +8.99e18
  DynamicPointSet set{};
  set.points.push_back(DynamicPoint{
      .source_point_index = 1,
      .position = PointXYZ{.x = 8.99e18, .y = 8.99e18, .z = 8.99e18},
      .semantic_class = SemanticClass::VEHICLE,
      .semantic_confidence = 0.9f,
      .motion_confidence = 0.9f,
  });
  set.points.push_back(DynamicPoint{
      .source_point_index = 2,
      .position = PointXYZ{.x = 8.99e18 + 0.2, .y = 8.99e18 + 0.2, .z = 8.99e18 + 0.2},
      .semantic_class = SemanticClass::VEHICLE,
      .semantic_confidence = 0.9f,
      .motion_confidence = 0.9f,
  });

  // Group at -8.99e18
  set.points.push_back(DynamicPoint{
      .source_point_index = 10,
      .position = PointXYZ{.x = -8.99e18, .y = -8.99e18, .z = -8.99e18},
      .semantic_class = SemanticClass::PEDESTRIAN,
      .semantic_confidence = 0.9f,
      .motion_confidence = 0.9f,
  });
  set.points.push_back(DynamicPoint{
      .source_point_index = 11,
      .position = PointXYZ{.x = -8.99e18 + 0.2, .y = -8.99e18 + 0.2, .z = -8.99e18 + 0.2},
      .semantic_class = SemanticClass::PEDESTRIAN,
      .semantic_confidence = 0.9f,
      .motion_confidence = 0.9f,
  });

  auto res = DynamicClusterer::cluster(set, config);
  EXPECT_EQ(res.status, DynamicClusteringStatus::SUCCESS);
  ASSERT_EQ(res.clusters.size(), 2u);
  EXPECT_EQ(res.clusters[0].canonical_source_point_index, 1u);
  EXPECT_EQ(res.clusters[0].member_source_indices.size(), 2u);
  EXPECT_EQ(res.clusters[1].canonical_source_point_index, 10u);
  EXPECT_EQ(res.clusters[1].member_source_indices.size(), 2u);
  EXPECT_TRUE(res.noise_source_indices.empty());
}

}  // namespace avr::mapping



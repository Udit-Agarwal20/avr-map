#include <gtest/gtest.h>

#include <random>
#include <vector>

#include "avr_mapping/adaptive_hierarchy.hpp"
#include "avr_spatial/constants.hpp"

namespace avr::mapping {

TEST(HierarchyRandomizedTest, Stress50kOperationsAnd100kLookups) {
  AdaptiveHierarchy hierarchy;
  std::mt19937 rng(26053);

  std::uniform_int_distribution<int32_t> root_dist(-15, 15);  // Grid of 31x31 L3 root zones
  std::uniform_int_distribution<int32_t> local_offset_dist(0, 7);
  std::uniform_int_distribution<int> op_dist(0, 4);
  std::uniform_int_distribution<int> level_dist(0, 3);

  std::vector<avr::spatial::GridCoord2D> root_origins;
  root_origins.reserve(100);

  // Seed with initial roots
  for (int i = 0; i < 8; ++i) {
    const int32_t rx = root_dist(rng) * 8;
    const int32_t ry = root_dist(rng) * 8;
    hierarchy.ensureRootForBase(avr::spatial::GridCoord2D{.x = rx, .y = ry});
    root_origins.push_back(avr::spatial::GridCoord2D{.x = rx, .y = ry});
  }

  // 1. Execute at least 50,000 structural topology operations
  constexpr size_t NUM_OPS = 50000;
  for (size_t op = 0; op < NUM_OPS; ++op) {
    const int action = op_dist(rng);

    if (action == 0 && root_origins.size() < 64) {
      // Allocate new root
      const int32_t rx = root_dist(rng) * 8;
      const int32_t ry = root_dist(rng) * 8;
      const avr::spatial::GridCoord2D base{.x = rx, .y = ry};
      hierarchy.ensureRootForBase(base);
      root_origins.push_back(base);
    } else if (action == 1) {
      // Refine a random location inside an allocated root
      std::uniform_int_distribution<size_t> root_idx_dist(0, root_origins.size() - 1);
      const auto& r = root_origins[root_idx_dist(rng)];
      const avr::spatial::GridCoord2D pt{
          .x = r.x + local_offset_dist(rng),
          .y = r.y + local_offset_dist(rng),
      };
      const uint8_t req_level = static_cast<uint8_t>(level_dist(rng));
      hierarchy.refineToLevel(pt, req_level);
    } else if (action == 2) {
      // Split a leaf
      std::uniform_int_distribution<size_t> root_idx_dist(0, root_origins.size() - 1);
      const auto& r = root_origins[root_idx_dist(rng)];
      const avr::spatial::GridCoord2D pt{
          .x = r.x + local_offset_dist(rng),
          .y = r.y + local_offset_dist(rng),
      };
      const auto leaf = hierarchy.findLeafContainingBase(pt);
      if (leaf.has_value() && leaf->level > avr::spatial::L0) {
        hierarchy.splitLeaf(*leaf);
      }
    } else if (action == 3) {
      // Attempt merge on an internal parent
      std::uniform_int_distribution<size_t> root_idx_dist(0, root_origins.size() - 1);
      const auto& r = root_origins[root_idx_dist(rng)];
      const avr::spatial::GridCoord2D pt{
          .x = r.x + local_offset_dist(rng),
          .y = r.y + local_offset_dist(rng),
      };
      const auto leaf = hierarchy.findLeafContainingBase(pt);
      if (leaf.has_value() && leaf->level < avr::spatial::L3) {
        const auto parent = avr::spatial::parentOf(*leaf);
        if (parent.has_value() && hierarchy.isInternal(*parent)) {
          hierarchy.mergeChildren(*parent);
        }
      }
    } else {
      // Ownership query
      std::uniform_int_distribution<size_t> root_idx_dist(0, root_origins.size() - 1);
      const auto& r = root_origins[root_idx_dist(rng)];
      const avr::spatial::GridCoord2D pt{
          .x = r.x + local_offset_dist(rng),
          .y = r.y + local_offset_dist(rng),
      };
      const auto leaf = hierarchy.findLeafContainingBase(pt);
      ASSERT_TRUE(leaf.has_value());
    }

    if ((op + 1) % 100 == 0) {
      const auto val = hierarchy.validateInvariants();
      ASSERT_TRUE(val.valid) << "Invariant failed at op " << op << ": " << val.error_message;
    }
  }

  // Final invariant check after 50,000 operations
  const auto final_val = hierarchy.validateInvariants();
  ASSERT_TRUE(final_val.valid) << "Final invariant failed: " << final_val.error_message;

  // 2. Execute at least 100,000 random ownership lookups inside allocated root footprints
  constexpr size_t NUM_LOOKUPS = 100000;
  std::uniform_int_distribution<size_t> root_idx_dist(0, root_origins.size() - 1);
  for (size_t lk = 0; lk < NUM_LOOKUPS; ++lk) {
    const auto& r = root_origins[root_idx_dist(rng)];
    const avr::spatial::GridCoord2D pt{
        .x = r.x + local_offset_dist(rng),
        .y = r.y + local_offset_dist(rng),
    };
    const auto leaf = hierarchy.findLeafContainingBase(pt);
    ASSERT_TRUE(leaf.has_value());
    EXPECT_TRUE(hierarchy.isLeaf(*leaf));
    EXPECT_FALSE(hierarchy.isInternal(*leaf));
  }
}

TEST(HierarchyRandomizedTest, RepeatabilityAcrossFreshInstances) {
  auto run_sequence = [](AdaptiveHierarchy& h, uint32_t seed) {
    std::mt19937 rng(seed);
    std::uniform_int_distribution<int32_t> root_dist(-10, 10);
    std::uniform_int_distribution<int32_t> offset_dist(0, 7);
    std::uniform_int_distribution<int> op_dist(0, 3);
    std::uniform_int_distribution<int> level_dist(0, 3);

    std::vector<avr::spatial::GridCoord2D> roots;
    for (int i = 0; i < 5; ++i) {
      const auto base = avr::spatial::GridCoord2D{.x = root_dist(rng) * 8, .y = root_dist(rng) * 8};
      h.ensureRootForBase(base);
      roots.push_back(base);
    }

    for (size_t op = 0; op < 5000; ++op) {
      const int act = op_dist(rng);
      std::uniform_int_distribution<size_t> idx_dist(0, roots.size() - 1);
      const auto& r = roots[idx_dist(rng)];
      const auto pt = avr::spatial::GridCoord2D{.x = r.x + offset_dist(rng), .y = r.y + offset_dist(rng)};

      if (act == 0 && roots.size() < 20) {
        const auto new_root = avr::spatial::GridCoord2D{.x = root_dist(rng) * 8, .y = root_dist(rng) * 8};
        h.ensureRootForBase(new_root);
        roots.push_back(new_root);
      } else if (act == 1) {
        h.refineToLevel(pt, static_cast<uint8_t>(level_dist(rng)));
      } else if (act == 2) {
        const auto leaf = h.findLeafContainingBase(pt);
        if (leaf.has_value() && leaf->level > avr::spatial::L0) {
          h.splitLeaf(*leaf);
        }
      } else {
        const auto leaf = h.findLeafContainingBase(pt);
        if (leaf.has_value() && leaf->level < avr::spatial::L3) {
          const auto parent = avr::spatial::parentOf(*leaf);
          if (parent.has_value() && h.isInternal(*parent)) {
            h.mergeChildren(*parent);
          }
        }
      }
    }
  };

  AdaptiveHierarchy h1;
  AdaptiveHierarchy h2;

  run_sequence(h1, 26053);
  run_sequence(h2, 26053);

  EXPECT_TRUE(h1.validateInvariants());
  EXPECT_TRUE(h2.validateInvariants());

  const auto s1 = h1.statistics();
  const auto s2 = h2.statistics();

  EXPECT_EQ(s1.tile_count, s2.tile_count);
  EXPECT_EQ(s1.node_count, s2.node_count);
  EXPECT_EQ(s1.leaf_count, s2.leaf_count);
  EXPECT_EQ(s1.internal_count, s2.internal_count);
  EXPECT_EQ(s1.leaf_count_by_level, s2.leaf_count_by_level);
  EXPECT_EQ(s1.internal_count_by_level, s2.internal_count_by_level);

  EXPECT_EQ(s1.pool_capacity, s2.pool_capacity);
  EXPECT_EQ(s1.pool_in_use, s2.pool_in_use);
  EXPECT_EQ(s1.pool_free, s2.pool_free);
  EXPECT_EQ(s1.pool_block_count, s2.pool_block_count);

  const auto snap1 = h1.leafSnapshot();
  const auto snap2 = h2.leafSnapshot();
  EXPECT_EQ(snap1, snap2);

  const auto top1 = h1.topologySnapshot();
  const auto top2 = h2.topologySnapshot();
  EXPECT_EQ(top1, top2);
}

}  // namespace avr::mapping

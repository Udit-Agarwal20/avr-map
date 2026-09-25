#include <gtest/gtest.h>

#include <vector>

#include "avr_mapping/adaptive_payload_store.hpp"
#include "avr_mapping/adaptive_payload_tile.hpp"
#include "avr_spatial/cell_key.hpp"
#include "avr_spatial/constants.hpp"

namespace avr::mapping {
namespace {

TEST(AdaptivePayloadStoreTest, InitialStateIsEmpty) {
  AdaptivePayloadStore store{};
  EXPECT_TRUE(store.isEmpty());
  EXPECT_EQ(store.tileCount(), 0ULL);

  const auto coord = avr::spatial::CellCoord{.x = 0, .y = 0, .level = avr::spatial::L3};
  const auto key = *avr::spatial::CellKey::fromCellCoord(coord);

  EXPECT_EQ(store.findActivePayload(key), nullptr);
  EXPECT_EQ(store.findRetiredCache(key), nullptr);
  EXPECT_FALSE(store.requiresReplay(key));
  EXPECT_FALSE(store.isRetiredCacheIncomplete(key));
}

TEST(AdaptivePayloadStoreTest, SetActivePayloadAndRetrieve) {
  AdaptivePayloadStore store{};

  const auto coord = avr::spatial::CellCoord{.x = 5, .y = 10, .level = avr::spatial::L2};
  const auto key = *avr::spatial::CellKey::fromCellCoord(coord);

  ElevationStats elev{};
  elev.update(3.0);
  const AdaptiveCellPayload payload(elev, 1, 1, 1, ResolutionReasonBits{});

  store.setActivePayload(key, payload);

  EXPECT_EQ(store.tileCount(), 1ULL);
  EXPECT_FALSE(store.isEmpty());

  const auto* found = store.findActivePayload(key);
  ASSERT_NE(found, nullptr);
  EXPECT_EQ(*found, payload);
  EXPECT_EQ(store.findRetiredCache(key), nullptr);
}

TEST(AdaptivePayloadStoreTest, SetRetiredCacheAndRetrieve) {
  AdaptivePayloadStore store{};

  const auto coord = avr::spatial::CellCoord{.x = 2, .y = 3, .level = avr::spatial::L3};
  const auto key = *avr::spatial::CellKey::fromCellCoord(coord);

  ElevationStats elev{};
  elev.update(10.0);
  const AdaptiveCellPayload payload(elev, 5, 8, 2, ResolutionReasonBits{});

  store.setRetiredCache(key, payload, /*incomplete=*/true);

  EXPECT_EQ(store.findActivePayload(key), nullptr);
  const auto* found_cache = store.findRetiredCache(key);
  ASSERT_NE(found_cache, nullptr);
  EXPECT_EQ(*found_cache, payload);
  EXPECT_TRUE(store.isRetiredCacheIncomplete(key));
}

TEST(AdaptivePayloadStoreTest, SetReplayRequiredAndQuery) {
  AdaptivePayloadStore store{};

  const auto coord = avr::spatial::CellCoord{.x = 1, .y = 1, .level = avr::spatial::L1};
  const auto key = *avr::spatial::CellKey::fromCellCoord(coord);

  EXPECT_FALSE(store.requiresReplay(key));
  store.setReplayRequired(key, true);
  EXPECT_TRUE(store.requiresReplay(key));

  store.setReplayRequired(key, false);
  EXPECT_FALSE(store.requiresReplay(key));
  // Tile became empty, so prune removes it
  store.pruneEmptyTiles();
  EXPECT_TRUE(store.isEmpty());
}

TEST(AdaptivePayloadStoreTest, ClearCellRemovesState) {
  AdaptivePayloadStore store{};

  const auto coord = avr::spatial::CellCoord{.x = 0, .y = 0, .level = avr::spatial::L0};
  const auto key = *avr::spatial::CellKey::fromCellCoord(coord);

  ElevationStats elev{};
  elev.update(2.0);
  store.setActivePayload(key, AdaptiveCellPayload(elev, 1, 1, 1, ResolutionReasonBits{}));
  store.setReplayRequired(key, true);

  EXPECT_NE(store.findActivePayload(key), nullptr);
  EXPECT_TRUE(store.requiresReplay(key));

  store.clearCell(key);

  EXPECT_EQ(store.findActivePayload(key), nullptr);
  EXPECT_FALSE(store.requiresReplay(key));
  EXPECT_TRUE(store.isEmpty());
}

TEST(AdaptivePayloadStoreTest, StatisticsCalculation) {
  AdaptivePayloadStore store{};

  const auto k1 = *avr::spatial::CellKey::fromCellCoord(
      avr::spatial::CellCoord{.x = 0, .y = 0, .level = avr::spatial::L3});
  const auto k2 = *avr::spatial::CellKey::fromCellCoord(
      avr::spatial::CellCoord{.x = 1, .y = 0, .level = avr::spatial::L3});

  ElevationStats e1{};
  e1.update(1.0);
  e1.update(2.0);
  store.setActivePayload(k1, AdaptiveCellPayload(e1, 1, 1, 1, ResolutionReasonBits{}));

  ElevationStats e2{};
  e2.update(5.0);
  e2.update(6.0);
  e2.update(7.0);
  store.setRetiredCache(k2, AdaptiveCellPayload(e2, 1, 2, 2, ResolutionReasonBits{}), true);

  const auto stats = store.statistics();
  EXPECT_EQ(stats.payload_tile_count, 1ULL);
  EXPECT_EQ(stats.active_payload_count, 1ULL);
  EXPECT_EQ(stats.retired_cache_count, 1ULL);
  EXPECT_EQ(stats.retired_cache_incomplete_count, 1ULL);
  EXPECT_EQ(stats.active_observation_count, 2ULL);
  EXPECT_EQ(stats.retired_cache_observation_count, 3ULL);
  EXPECT_EQ(stats.sizeof_payload, sizeof(AdaptiveCellPayload));
  EXPECT_EQ(stats.sizeof_payload_tile, sizeof(AdaptivePayloadTile));
  EXPECT_EQ(stats.payload_tile_bytes, stats.payload_tile_count * sizeof(AdaptivePayloadTile));
}

TEST(AdaptivePayloadStoreTest, SnapshotsAreSortedByCellKey) {
  AdaptivePayloadStore store{};

  // Insert in scrambled order across multiple tiles
  const std::vector<avr::spatial::CellCoord> coords = {
      {.x = 20, .y = 25, .level = avr::spatial::L2},
      {.x = -5, .y = -8, .level = avr::spatial::L2},
      {.x = 0, .y = 0, .level = avr::spatial::L2},
      {.x = 15, .y = 15, .level = avr::spatial::L2},
      {.x = -17, .y = 3, .level = avr::spatial::L2},
  };

  for (const auto& c : coords) {
    const auto key = *avr::spatial::CellKey::fromCellCoord(c);
    ElevationStats elev{};
    elev.update(1.0);
    store.setActivePayload(key, AdaptiveCellPayload(elev, 1, 1, 1, ResolutionReasonBits{}));
  }

  const auto snapshot = store.activePayloadSnapshot();
  ASSERT_EQ(snapshot.size(), coords.size());

  for (size_t i = 0; i + 1 < snapshot.size(); ++i) {
    EXPECT_LT(snapshot[i].first, snapshot[i + 1].first)
        << "Snapshot must be strictly sorted by CellKey";
  }
}

TEST(AdaptivePayloadStoreTest, NegativeCoordinatesAndTileBoundaries) {
  AdaptivePayloadStore store{};

  const std::vector<int32_t> test_coords = {-17, -16, -15, -1, 0, 15, 16, 17};

  for (const int32_t x : test_coords) {
    for (const int32_t y : test_coords) {
      const auto coord = avr::spatial::CellCoord{.x = x, .y = y, .level = avr::spatial::L1};
      const auto key = *avr::spatial::CellKey::fromCellCoord(coord);

      ElevationStats elev{};
      elev.update(static_cast<double>(x + y));
      store.setActivePayload(key, AdaptiveCellPayload(elev, 1, 1, 1, ResolutionReasonBits{}));

      const auto* found = store.findActivePayload(key);
      ASSERT_NE(found, nullptr);
      EXPECT_DOUBLE_EQ(found->elevation().mean(), static_cast<double>(x + y));
    }
  }

  EXPECT_GT(store.tileCount(), 1ULL);
  store.reset();
  EXPECT_TRUE(store.isEmpty());
}

TEST(AdaptivePayloadStoreTest, AdaptivePayloadTileMaskInvariants) {
  AdaptivePayloadTile tile{};
  EXPECT_TRUE(tile.isEmpty());
  EXPECT_EQ(tile.activeCount(), 0ULL);
  EXPECT_EQ(tile.retiredCacheCount(), 0ULL);
  EXPECT_EQ(tile.replayRequiredCount(), 0ULL);
  EXPECT_EQ(tile.retiredCacheIncompleteCount(), 0ULL);

  ElevationStats elev{};
  elev.update(1.0);
  const AdaptiveCellPayload payload(elev, 1, 1, 1, ResolutionReasonBits{});

  // Set active on slot 5
  tile.setActivePayload(5, payload);
  EXPECT_TRUE(tile.isActive(5));
  EXPECT_FALSE(tile.isRetiredCache(5));
  EXPECT_EQ((tile.activeMask() & tile.retiredCacheMask()).count(), 0ULL);

  // Set retired cache with incomplete=true on slot 10
  tile.setRetiredCache(10, payload, /*incomplete=*/true);
  EXPECT_FALSE(tile.isActive(10));
  EXPECT_TRUE(tile.isRetiredCache(10));
  EXPECT_TRUE(tile.isRetiredCacheIncomplete(10));

  // Invariant: incomplete mask must be a subset of retired cache mask
  EXPECT_EQ((tile.retiredCacheIncompleteMask() & ~tile.retiredCacheMask()).count(), 0ULL);
  EXPECT_EQ((tile.activeMask() & tile.retiredCacheMask()).count(), 0ULL);

  // Transition slot 10 back to active: must clear retired cache and incomplete flags
  tile.setActivePayload(10, payload);
  EXPECT_TRUE(tile.isActive(10));
  EXPECT_FALSE(tile.isRetiredCache(10));
  EXPECT_FALSE(tile.isRetiredCacheIncomplete(10));
  EXPECT_EQ((tile.retiredCacheIncompleteMask() & ~tile.retiredCacheMask()).count(), 0ULL);

  // Clear slot 5
  tile.clearCell(5);
  EXPECT_FALSE(tile.isActive(5));
  EXPECT_FALSE(tile.isRetiredCache(5));
}

TEST(AdaptivePayloadStoreTest, ReplayRequiredAndRetiredIncompleteSnapshots) {
  AdaptivePayloadStore store{};

  const std::vector<avr::spatial::CellCoord> replay_coords = {
      {.x = 10, .y = 10, .level = avr::spatial::L2},
      {.x = -5, .y = 2, .level = avr::spatial::L2},
      {.x = 0, .y = 0, .level = avr::spatial::L2},
  };
  for (const auto& c : replay_coords) {
    const auto key = *avr::spatial::CellKey::fromCellCoord(c);
    store.setReplayRequired(key, true);
  }

  const auto replay_snap = store.replayRequiredSnapshot();
  ASSERT_EQ(replay_snap.size(), replay_coords.size());
  for (size_t i = 0; i + 1 < replay_snap.size(); ++i) {
    EXPECT_LT(replay_snap[i], replay_snap[i + 1]);
  }

  ElevationStats elev{};
  elev.update(2.0);
  const AdaptiveCellPayload payload(elev, 1, 1, 1, ResolutionReasonBits{});
  store.setRetiredCache(replay_snap[0], payload, /*incomplete=*/true);
  store.setRetiredCache(replay_snap[1], payload, /*incomplete=*/false);

  const auto incomplete_snap = store.retiredCacheIncompleteSnapshot();
  ASSERT_EQ(incomplete_snap.size(), 1ULL);
  EXPECT_EQ(incomplete_snap[0], replay_snap[0]);
}

TEST(AdaptivePayloadStoreTest, TileLayoutContract) {
  EXPECT_EQ(sizeof(AdaptivePayloadTile), 18560);
  EXPECT_EQ(alignof(AdaptivePayloadTile), 8);
}

}  // namespace
}  // namespace avr::mapping

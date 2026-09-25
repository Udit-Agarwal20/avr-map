#include "avr_mapping/adaptive_payload_store.hpp"

#include <algorithm>

namespace avr::mapping {

const AdaptiveCellPayload* AdaptivePayloadStore::findActivePayload(avr::spatial::CellKey key) const noexcept {
  const auto tile_key_opt = avr::spatial::TileKey::fromCellKey(key);
  if (!tile_key_opt.has_value()) {
    return nullptr;
  }
  const auto it = tiles_.find(*tile_key_opt);
  if (it == tiles_.end()) {
    return nullptr;
  }
  const auto coord = key.decode();
  const uint16_t local_idx = avr::spatial::localCellIndex(coord.x, coord.y);
  if (!it->second.isActive(local_idx)) {
    return nullptr;
  }
  return &it->second.cell(local_idx);
}

const AdaptiveCellPayload* AdaptivePayloadStore::findRetiredCache(avr::spatial::CellKey key) const noexcept {
  const auto tile_key_opt = avr::spatial::TileKey::fromCellKey(key);
  if (!tile_key_opt.has_value()) {
    return nullptr;
  }
  const auto it = tiles_.find(*tile_key_opt);
  if (it == tiles_.end()) {
    return nullptr;
  }
  const auto coord = key.decode();
  const uint16_t local_idx = avr::spatial::localCellIndex(coord.x, coord.y);
  if (!it->second.isRetiredCache(local_idx)) {
    return nullptr;
  }
  return &it->second.cell(local_idx);
}

bool AdaptivePayloadStore::requiresReplay(avr::spatial::CellKey key) const noexcept {
  const auto tile_key_opt = avr::spatial::TileKey::fromCellKey(key);
  if (!tile_key_opt.has_value()) {
    return false;
  }
  const auto it = tiles_.find(*tile_key_opt);
  if (it == tiles_.end()) {
    return false;
  }
  const auto coord = key.decode();
  const uint16_t local_idx = avr::spatial::localCellIndex(coord.x, coord.y);
  return it->second.isReplayRequired(local_idx);
}

bool AdaptivePayloadStore::isRetiredCacheIncomplete(avr::spatial::CellKey key) const noexcept {
  const auto tile_key_opt = avr::spatial::TileKey::fromCellKey(key);
  if (!tile_key_opt.has_value()) {
    return false;
  }
  const auto it = tiles_.find(*tile_key_opt);
  if (it == tiles_.end()) {
    return false;
  }
  const auto coord = key.decode();
  const uint16_t local_idx = avr::spatial::localCellIndex(coord.x, coord.y);
  return it->second.isRetiredCacheIncomplete(local_idx);
}

void AdaptivePayloadStore::setActivePayload(avr::spatial::CellKey key, const AdaptiveCellPayload& payload) {
  const auto tile_key_opt = avr::spatial::TileKey::fromCellKey(key);
  if (!tile_key_opt.has_value()) {
    return;
  }
  const auto coord = key.decode();
  const uint16_t local_idx = avr::spatial::localCellIndex(coord.x, coord.y);
  tiles_[*tile_key_opt].setActivePayload(local_idx, payload);
}

void AdaptivePayloadStore::setRetiredCache(
    avr::spatial::CellKey key,
    const AdaptiveCellPayload& payload,
    bool incomplete) {
  const auto tile_key_opt = avr::spatial::TileKey::fromCellKey(key);
  if (!tile_key_opt.has_value()) {
    return;
  }
  const auto coord = key.decode();
  const uint16_t local_idx = avr::spatial::localCellIndex(coord.x, coord.y);
  tiles_[*tile_key_opt].setRetiredCache(local_idx, payload, incomplete);
}

void AdaptivePayloadStore::setReplayRequired(avr::spatial::CellKey key, bool required) {
  const auto tile_key_opt = avr::spatial::TileKey::fromCellKey(key);
  if (!tile_key_opt.has_value()) {
    return;
  }
  const auto coord = key.decode();
  const uint16_t local_idx = avr::spatial::localCellIndex(coord.x, coord.y);
  if (required) {
    tiles_[*tile_key_opt].setReplayRequired(local_idx, true);
  } else {
    const auto it = tiles_.find(*tile_key_opt);
    if (it != tiles_.end()) {
      it->second.setReplayRequired(local_idx, false);
      if (it->second.isEmpty()) {
        tiles_.erase(it);
      }
    }
  }
}

void AdaptivePayloadStore::clearCell(avr::spatial::CellKey key) {
  const auto tile_key_opt = avr::spatial::TileKey::fromCellKey(key);
  if (!tile_key_opt.has_value()) {
    return;
  }
  const auto it = tiles_.find(*tile_key_opt);
  if (it == tiles_.end()) {
    return;
  }
  const auto coord = key.decode();
  const uint16_t local_idx = avr::spatial::localCellIndex(coord.x, coord.y);
  it->second.clearCell(local_idx);
  if (it->second.isEmpty()) {
    tiles_.erase(it);
  }
}

void AdaptivePayloadStore::pruneEmptyTiles() noexcept {
  for (auto it = tiles_.begin(); it != tiles_.end();) {
    if (it->second.isEmpty()) {
      it = tiles_.erase(it);
    } else {
      ++it;
    }
  }
}

void AdaptivePayloadStore::reset() noexcept {
  tiles_.clear();
}

PayloadStoreStats AdaptivePayloadStore::statistics() const noexcept {
  PayloadStoreStats stats{};
  stats.payload_tile_count = tiles_.size();
  stats.sizeof_payload = sizeof(AdaptiveCellPayload);
  stats.sizeof_payload_tile = sizeof(AdaptivePayloadTile);
  stats.payload_tile_bytes = stats.payload_tile_count * stats.sizeof_payload_tile;

  for (const auto& [tile_key, tile] : tiles_) {
    stats.active_payload_count += tile.activeCount();
    stats.retired_cache_count += tile.retiredCacheCount();
    stats.replay_required_count += tile.replayRequiredCount();
    stats.retired_cache_incomplete_count += tile.retiredCacheIncompleteCount();

    const auto& active_mask = tile.activeMask();
    const auto& retired_mask = tile.retiredCacheMask();

    for (size_t i = 0; i < AdaptivePayloadTile::CELL_COUNT; ++i) {
      const uint16_t local_idx = static_cast<uint16_t>(i);
      if (active_mask.test(i)) {
        stats.active_observation_count += tile.cell(local_idx).observationCount();
      }
      if (retired_mask.test(i)) {
        stats.retired_cache_observation_count += tile.cell(local_idx).observationCount();
      }
    }
  }

  return stats;
}

std::vector<std::pair<avr::spatial::CellKey, AdaptiveCellPayload>>
AdaptivePayloadStore::activePayloadSnapshot() const {
  std::vector<std::pair<avr::spatial::CellKey, AdaptiveCellPayload>> snapshot;

  for (const auto& [tile_key, tile] : tiles_) {
    const auto& active_mask = tile.activeMask();
    for (size_t i = 0; i < AdaptivePayloadTile::CELL_COUNT; ++i) {
      if (active_mask.test(i)) {
        const auto cell_key_opt = avr::spatial::cellKeyFromTileAndLocal(
            tile_key, static_cast<uint16_t>(i));
        if (cell_key_opt.has_value()) {
          snapshot.emplace_back(*cell_key_opt, tile.cell(static_cast<uint16_t>(i)));
        }
      }
    }
  }

  std::sort(snapshot.begin(), snapshot.end(), [](const auto& a, const auto& b) {
    return a.first < b.first;
  });

  return snapshot;
}

std::vector<std::pair<avr::spatial::CellKey, AdaptiveCellPayload>>
AdaptivePayloadStore::retiredCacheSnapshot() const {
  std::vector<std::pair<avr::spatial::CellKey, AdaptiveCellPayload>> snapshot;

  for (const auto& [tile_key, tile] : tiles_) {
    const auto& retired_mask = tile.retiredCacheMask();
    for (size_t i = 0; i < AdaptivePayloadTile::CELL_COUNT; ++i) {
      if (retired_mask.test(i)) {
        const auto cell_key_opt = avr::spatial::cellKeyFromTileAndLocal(
            tile_key, static_cast<uint16_t>(i));
        if (cell_key_opt.has_value()) {
          snapshot.emplace_back(*cell_key_opt, tile.cell(static_cast<uint16_t>(i)));
        }
      }
    }
  }

  std::sort(snapshot.begin(), snapshot.end(), [](const auto& a, const auto& b) {
    return a.first < b.first;
  });

  return snapshot;
}

std::vector<avr::spatial::CellKey>
AdaptivePayloadStore::replayRequiredSnapshot() const {
  std::vector<avr::spatial::CellKey> snapshot;

  for (const auto& [tile_key, tile] : tiles_) {
    const auto& mask = tile.replayRequiredMask();
    for (size_t i = 0; i < AdaptivePayloadTile::CELL_COUNT; ++i) {
      if (mask.test(i)) {
        const auto cell_key_opt = avr::spatial::cellKeyFromTileAndLocal(
            tile_key, static_cast<uint16_t>(i));
        if (cell_key_opt.has_value()) {
          snapshot.push_back(*cell_key_opt);
        }
      }
    }
  }

  std::sort(snapshot.begin(), snapshot.end());
  return snapshot;
}

std::vector<avr::spatial::CellKey>
AdaptivePayloadStore::retiredCacheIncompleteSnapshot() const {
  std::vector<avr::spatial::CellKey> snapshot;

  for (const auto& [tile_key, tile] : tiles_) {
    const auto& mask = tile.retiredCacheIncompleteMask();
    for (size_t i = 0; i < AdaptivePayloadTile::CELL_COUNT; ++i) {
      if (mask.test(i)) {
        const auto cell_key_opt = avr::spatial::cellKeyFromTileAndLocal(
            tile_key, static_cast<uint16_t>(i));
        if (cell_key_opt.has_value()) {
          snapshot.push_back(*cell_key_opt);
        }
      }
    }
  }

  std::sort(snapshot.begin(), snapshot.end());
  return snapshot;
}

}  // namespace avr::mapping

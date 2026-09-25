#include "avr_mapping/uniform_map.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace avr::mapping {

namespace {

constexpr size_t safeMul(size_t a, size_t b) noexcept {
  if (a == 0 || b == 0) {
    return 0;
  }
  if (a > std::numeric_limits<size_t>::max() / b) {
    return std::numeric_limits<size_t>::max();
  }
  return a * b;
}

}  // namespace

UniformMap::UniformMap(double radius_m) {
  if (std::isfinite(radius_m) && radius_m >= 0.0) {
    radius_ = radius_m;
    radius_sq_ = radius_m * radius_m;
  }
}

FrameUpdateStats UniformMap::integrateFrame(
    std::span<const PointXYZ> points,
    uint64_t frame_seq) {
  FrameUpdateStats stats;
  stats.frame_seq = frame_seq;
  stats.input_points = points.size();

  // Advance internal integration generation for unique_cells_touched tracking without temporary heap allocation
  ++integration_generation_;

  for (const auto& pt : points) {
    // 1. Non-finite coordinate rejection
    if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
      ++stats.rejected_non_finite;
      continue;
    }

    // 2. Canonical base-grid representability / out-of-range rejection
    const auto gx = avr::spatial::quantizeToBaseGridSafe(pt.x);
    const auto gy = avr::spatial::quantizeToBaseGridSafe(pt.y);
    if (!gx.has_value() || !gy.has_value()) {
      ++stats.rejected_out_of_range;
      continue;
    }

    // 3. Local-map extent filtering
    const double dx = pt.x - window_center_x_;
    const double dy = pt.y - window_center_y_;
    const double dist_sq = dx * dx + dy * dy;

    if (!std::isfinite(dist_sq) || dist_sq > radius_sq_) {
      ++stats.rejected_outside_extent;
      continue;
    }

    const avr::spatial::CellCoord cell_coord{.x = *gx, .y = *gy, .level = 0};
    const auto cell_key_opt = avr::spatial::CellKey::fromCellCoord(cell_coord);
    if (!cell_key_opt.has_value()) [[unlikely]] {
      ++stats.rejected_out_of_range;
      continue;
    }

    const avr::spatial::TileCoord tile_coord = avr::spatial::cellToTileCoord(cell_coord);
    const auto tile_key_opt = avr::spatial::TileKey::fromTileCoord(tile_coord);
    if (!tile_key_opt.has_value()) [[unlikely]] {
      ++stats.rejected_out_of_range;
      continue;
    }
    const avr::spatial::TileKey tile_key = *tile_key_opt;
    const uint16_t local_idx = avr::spatial::localCellIndex(*gx, *gy);

    // 4. Cell / elevation numerical update validity & transactional mutation
    auto it = tiles_.find(tile_key);
    const bool tile_existed = (it != tiles_.end());

    if (!tile_existed) {
      // Validate candidate update before creating a new tile
      UniformCell trial_cell;
      if (!trial_cell.tryUpdate(pt.z, frame_seq)) {
        ++stats.rejected_numeric;
        continue;
      }

      // Candidate is valid: create tile and commit cell
      it = tiles_.try_emplace(tile_key).first;
      ++stats.tiles_created;

      UniformTile& tile = it->second;
      tile.setCell(local_idx, trial_cell);
      tile.activateCell(local_idx);

      ++stats.cells_created;
      ++active_cell_count_;
      ++stats.accepted_points;
      ++total_observations_;

      if (tile.markCellTouchedInFrame(local_idx, integration_generation_)) {
        ++stats.unique_cells_touched;
      }
    } else {
      UniformTile& tile = it->second;
      if (!tile.isCellActive(local_idx)) {
        UniformCell trial_cell;
        if (!trial_cell.tryUpdate(pt.z, frame_seq)) {
          ++stats.rejected_numeric;
          continue;
        }

        tile.setCell(local_idx, trial_cell);
        tile.activateCell(local_idx);

        ++stats.cells_created;
        ++active_cell_count_;
        ++stats.accepted_points;
        ++total_observations_;

        if (tile.markCellTouchedInFrame(local_idx, integration_generation_)) {
          ++stats.unique_cells_touched;
        }
      } else {
        // Cell already active: attempt in-place numeric update
        if (!tile.tryUpdateCell(local_idx, pt.z, frame_seq)) {
          ++stats.rejected_numeric;
          continue;
        }

        ++stats.cells_updated;
        ++stats.accepted_points;
        ++total_observations_;

        if (tile.markCellTouchedInFrame(local_idx, integration_generation_)) {
          ++stats.unique_cells_touched;
        }
      }
    }
  }

  return stats;
}

bool UniformMap::setWindowCenter(double x, double y) noexcept {
  if (!std::isfinite(x) || !std::isfinite(y)) {
    return false;
  }
  window_center_x_ = x;
  window_center_y_ = y;
  return true;
}

std::pair<double, double> UniformMap::windowCenter() const noexcept {
  return {window_center_x_, window_center_y_};
}

bool UniformMap::setRadius(double radius_m) noexcept {
  if (!std::isfinite(radius_m) || radius_m < 0.0) {
    return false;
  }
  radius_ = radius_m;
  radius_sq_ = radius_m * radius_m;
  return true;
}

double UniformMap::radius() const noexcept {
  return radius_;
}

void UniformMap::reserveTiles(size_t tile_count) {
  tiles_.reserve(tile_count);
}

const UniformCell* UniformMap::findCell(const avr::spatial::CellKey& key) const noexcept {
  if (key.level != 0) {
    return nullptr;
  }
  const auto cell_coord = key.decode();
  const auto tile_coord = avr::spatial::cellToTileCoord(cell_coord);
  const auto tile_key_opt = avr::spatial::TileKey::fromTileCoord(tile_coord);
  if (!tile_key_opt.has_value()) {
    return nullptr;
  }
  auto it = tiles_.find(*tile_key_opt);
  if (it == tiles_.end()) {
    return nullptr;
  }
  const uint16_t local_idx = avr::spatial::localCellIndex(cell_coord.x, cell_coord.y);
  if (!it->second.isCellActive(local_idx)) {
    return nullptr;
  }
  return &it->second.cell(local_idx);
}

const UniformCell* UniformMap::findCell(double x, double y) const noexcept {
  if (!std::isfinite(x) || !std::isfinite(y)) {
    return nullptr;
  }
  const auto gx = avr::spatial::quantizeToBaseGridSafe(x);
  const auto gy = avr::spatial::quantizeToBaseGridSafe(y);
  if (!gx.has_value() || !gy.has_value()) {
    return nullptr;
  }
  const avr::spatial::CellCoord cell_coord{.x = *gx, .y = *gy, .level = 0};
  const auto cell_key_opt = avr::spatial::CellKey::fromCellCoord(cell_coord);
  if (!cell_key_opt.has_value()) {
    return nullptr;
  }
  return findCell(*cell_key_opt);
}

const UniformCell* UniformMap::findCell(const avr::spatial::WorldPoint2D& pt) const noexcept {
  return findCell(pt.x, pt.y);
}

std::vector<SnapshotRecord> UniformMap::snapshot() const {
  std::vector<SnapshotRecord> records;
  records.reserve(active_cell_count_);
  for (const auto& [tile_key, tile] : tiles_) {
    const auto& mask = tile.activeMask();
    for (size_t i = 0; i < UniformTile::CELL_COUNT; ++i) {
      if (mask.test(i)) {
        const auto key_opt =
            avr::spatial::cellKeyFromTileAndLocal(tile_key, static_cast<uint16_t>(i));
        if (key_opt.has_value()) {
          records.push_back(
              SnapshotRecord{.key = *key_opt, .cell = tile.cell(static_cast<uint16_t>(i))});
        }
      }
    }
  }
  std::sort(
      records.begin(), records.end(),
      [](const SnapshotRecord& a, const SnapshotRecord& b) { return a.key < b.key; });
  return records;
}

void UniformMap::reset() noexcept {
  tiles_.clear();
  active_cell_count_ = 0;
  total_observations_ = 0;
}

MapStats UniformMap::statistics() const noexcept {
  MapStats s;
  s.tile_count = tiles_.size();
  s.active_cell_count = active_cell_count_;
  s.allocated_cell_slots = safeMul(s.tile_count, UniformTile::CELL_COUNT);
  s.total_accepted_observations = total_observations_;
  s.bucket_count = tiles_.bucket_count();
  s.load_factor = tiles_.load_factor();
  s.sizeof_uniform_cell = sizeof(UniformCell);
  s.sizeof_uniform_tile = sizeof(UniformTile);
  s.tile_payload_bytes = safeMul(s.tile_count, sizeof(UniformTile));
  s.active_cell_payload_bytes = safeMul(s.active_cell_count, sizeof(UniformCell));
  return s;
}

}  // namespace avr::mapping

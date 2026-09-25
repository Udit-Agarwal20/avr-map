#include "avr_mapping/frame_evidence.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <limits>

#include "avr_mapping/evidence_signal_builder.hpp"
#include "avr_spatial/bounds.hpp"
#include "avr_spatial/constants.hpp"

namespace avr::mapping {

std::optional<FrameEvidenceStats> FrameEvidence::buildFrame(
    std::span<const PointXYZ> points,
    uint64_t frame_seq,
    avr::spatial::WorldPoint2D sensor_origin) {
  // Frame-global validation: sensor_origin must be strictly finite in both axes
  if (!std::isfinite(sensor_origin.x) || !std::isfinite(sensor_origin.y)) {
    // Preserve previous valid frame evidence unmodified
    return std::nullopt;
  }

  // Clear previous logical frame evidence; hash container capacity is preserved
  reset();

  stats_.frame_seq = frame_seq;
  stats_.sensor_origin = sensor_origin;

  for (const auto& pt : points) {
    ++stats_.input_points;

    // 1. Validate finite coordinates
    if (!pt.isFinite()) {
      ++stats_.rejected_non_finite;
      continue;
    }

    // 2. Canonical base-grid quantization (L0 / 5 cm)
    const auto gx = avr::spatial::quantizeToBaseGridSafe(pt.x);
    const auto gy = avr::spatial::quantizeToBaseGridSafe(pt.y);
    if (!gx.has_value() || !gy.has_value()) {
      ++stats_.rejected_out_of_range;
      continue;
    }

    // 3. Grid coordinates and L0 CellCoord
    const avr::spatial::GridCoord2D grid_coord{.x = *gx, .y = *gy};
    const avr::spatial::CellCoord cell_coord =
        avr::spatial::gridToCellCoord(grid_coord, avr::spatial::L0);

    // 4. Derive L0 TileCoord and local cell index
    const avr::spatial::TileCoord tile_coord = avr::spatial::cellToTileCoord(cell_coord);
    const auto tile_key_opt = avr::spatial::TileKey::fromTileCoord(tile_coord);
    if (!tile_key_opt.has_value()) {
      ++stats_.rejected_out_of_range;
      continue;
    }
    const auto tile_key = *tile_key_opt;
    const uint16_t local_idx = avr::spatial::localCellIndex(cell_coord.x, cell_coord.y);

    // 5. Lookup or insert sparse L0 tile
    auto [it, inserted] = tiles_.try_emplace(tile_key);
    auto& tile = it->second;

    // 6. Update cell elevation
    const auto update_res = tile.updateCell(local_idx, pt.z);
    if (update_res.success) {
      ++stats_.accepted_points;
      if (update_res.became_active) {
        ++stats_.active_base_cells;
      }
    } else {
      // If tile was newly inserted but cell update failed, do not leave empty tile allocated
      if (inserted && tile.activeCellCount() == 0) {
        tiles_.erase(it);
      }
      ++stats_.rejected_numeric;
    }
  }

  stats_.tile_count = static_cast<uint32_t>(tiles_.size());
  has_frame_ = true;
  return stats_;
}

void FrameEvidence::reset() noexcept {
  tiles_.clear();
  stats_ = FrameEvidenceStats{};
  has_frame_ = false;
}

EvidenceSummary FrameEvidence::summarize(avr::spatial::CellKey region) const noexcept {
  if (!avr::spatial::isValidLevel(region.level)) {
    return EvidenceSummary{.region = region};
  }

  const uint32_t capacity = 1U << (2U * region.level);
  const auto coord = region.decode();
  const int32_t scale = static_cast<int32_t>(avr::spatial::levelScale(region.level));

  const int64_t base_x_min_64 = static_cast<int64_t>(coord.x) * scale;
  const int64_t base_y_min_64 = static_cast<int64_t>(coord.y) * scale;
  const int64_t base_x_max_64 = base_x_min_64 + scale - 1;
  const int64_t base_y_max_64 = base_y_min_64 + scale - 1;

  if (base_x_min_64 < std::numeric_limits<int32_t>::min() ||
      base_x_max_64 > std::numeric_limits<int32_t>::max() ||
      base_y_min_64 < std::numeric_limits<int32_t>::min() ||
      base_y_max_64 > std::numeric_limits<int32_t>::max()) {
    return EvidenceSummary{.region = region, .base_cell_capacity = capacity};
  }

  uint64_t observation_count = 0;
  uint32_t occupied_base_cells = 0;
  ElevationStats aggregate_elevation{};

  for (int32_t dy = 0; dy < scale; ++dy) {
    for (int32_t dx = 0; dx < scale; ++dx) {
      const int32_t gx = static_cast<int32_t>(base_x_min_64 + dx);
      const int32_t gy = static_cast<int32_t>(base_y_min_64 + dy);

      const avr::spatial::CellCoord base_coord{.x = gx, .y = gy, .level = avr::spatial::L0};
      const avr::spatial::TileCoord tile_coord = avr::spatial::cellToTileCoord(base_coord);
      const auto tile_key = avr::spatial::TileKey::fromTileCoord(tile_coord);
      if (!tile_key.has_value()) {
        continue;
      }

      const auto it = tiles_.find(*tile_key);
      if (it == tiles_.end()) {
        continue;
      }

      const uint16_t local_idx = avr::spatial::localCellIndex(gx, gy);
      if (!it->second.isCellActive(local_idx)) {
        continue;
      }

      const auto& cell = it->second.cell(local_idx);
      ++occupied_base_cells;
      observation_count += cell.count();
      aggregate_elevation.merge(cell.elevation());
    }
  }

  double density_score = 0.0;
  if (capacity > 0) {
    density_score = static_cast<double>(occupied_base_cells) / static_cast<double>(capacity);
    density_score = std::clamp(density_score, 0.0, 1.0);
  }

  EvidenceSummary summary{
      .region = region,
      .observation_count = observation_count,
      .occupied_base_cells = occupied_base_cells,
      .base_cell_capacity = capacity,
      .density_score = density_score,
  };

  if (occupied_base_cells > 0 && observation_count > 0) {
    summary.elevation_mean = aggregate_elevation.mean();
    summary.elevation_variance = aggregate_elevation.populationVariance();
    summary.z_min = aggregate_elevation.min();
    summary.z_max = aggregate_elevation.max();

    // Vertical-relief geometry complexity requires >= 2 observations AND >= 2 distinct base cells
    if (observation_count >= 2 && occupied_base_cells >= 2) {
      const double span = *summary.z_max - *summary.z_min;
      const double width = avr::spatial::resolutionMeters(region.level);
      if (std::isfinite(span) && span >= 0.0 && std::isfinite(width) && width > 0.0) {
        const double ratio = span / width;
        if (std::isfinite(ratio)) {
          summary.geometry_complexity = std::clamp(ratio, 0.0, 1.0);
        }
      }
    }
  }

  return summary;
}

std::optional<ResolutionSignals> FrameEvidence::makeResolutionSignals(
    avr::spatial::CellKey region,
    SemanticRelevance semantic,
    DynamicRelevance dynamic,
    std::optional<double> uncertainty) const noexcept {
  return avr::mapping::makeResolutionSignals(*this, region, semantic, dynamic, uncertainty);
}

std::vector<BaseCellSnapshotEntry> FrameEvidence::baseCellSnapshot() const {
  std::vector<BaseCellSnapshotEntry> snapshot;
  snapshot.reserve(stats_.active_base_cells);

  for (const auto& [tile_key, tile] : tiles_) {
    const auto active_mask = tile.activeMask();
    for (size_t local_idx = 0; local_idx < EvidenceTile::CELL_COUNT; ++local_idx) {
      if (active_mask.test(local_idx)) {
        const auto cell_key_opt = avr::spatial::cellKeyFromTileAndLocal(
            tile_key, static_cast<uint16_t>(local_idx));
        if (cell_key_opt.has_value()) {
          snapshot.push_back(BaseCellSnapshotEntry{
              .key = *cell_key_opt,
              .cell = tile.cell(static_cast<uint16_t>(local_idx)),
          });
        }
      }
    }
  }

  std::sort(snapshot.begin(), snapshot.end());
  return snapshot;
}

}  // namespace avr::mapping

#pragma once

#include <span>
#include <unordered_map>
#include <utility>
#include <vector>

#include "avr_mapping/map_stats.hpp"
#include "avr_mapping/point_xyz.hpp"
#include "avr_mapping/uniform_cell.hpp"
#include "avr_mapping/uniform_tile.hpp"
#include "avr_spatial/cell_key.hpp"
#include "avr_spatial/coordinate.hpp"
#include "avr_spatial/tile_key.hpp"

namespace avr::mapping {

/**
 * @brief Deterministic cell record for map snapshots.
 */
struct SnapshotRecord {
  avr::spatial::CellKey key;
  UniformCell cell;

  constexpr bool operator==(const SnapshotRecord&) const noexcept = default;
};

/**
 * @brief Uniform 5 cm (Level 0) 2.5D Baseline Map Engine.
 *
 * Implements a fixed-resolution sparse tiled 2.5D elevation grid.
 * Serves as the ground-truth empirical baseline for the future adaptive AVR map.
 *
 * Key invariants:
 * - Spatial ownership is governed exclusively by canonical GridOrigin2D::canonical().
 * - Resolution is locked at Level 0 (5 cm); no L1..L3 cells are supported.
 * - Window center is an extent acceptance filter; it never shifts the underlying integer grid.
 * - Cell elevation is aggregated via numerically stable Welford online variance in O(1) memory.
 * - Snapshots are strictly sorted by CellKey for 100% determinism.
 * - Rejection precedence order: non-finite -> out_of_range -> outside_extent -> numeric -> accepted.
 * - Allocation contract:
 *   - No heap allocation is required when updating an already allocated tile/cell.
 *   - Structural allocation: new tile node creation and hash-table rehash when capacity grows.
 *   - Temporary per-frame allocation: none.
 *   - Per-observation auxiliary allocation: none for existing tile/cell updates.
 * - Frame sequence contract: frame_seq identifies the logical sensor frame; unique cells touched
 *   are tracked via an internal monotonic generation counter (one internal generation per
 *   integrateFrame call) with total immunity to frame_seq ordering or reuse. Resetting the map
 *   clears all tiles so stale touched masks cannot be exposed. Unsigned 64-bit generation arithmetic
 *   has no UB; uint64 generation exhaustion is outside practical runtime scope.
 * - Reset capacity semantics (Policy A): reset() clears all logical map contents (tiles, cells,
 *   observations) while retaining container capacity for efficient real-time reuse.
 */
class UniformMap {
 public:
  explicit UniformMap(double radius_m = 100.0);

  /**
   * @brief Integrates a batch of XYZ points representing one LiDAR frame.
   *
   * Rejection precedence:
   * 1. non-finite x/y/z coordinates -> rejected_non_finite
   * 2. XY canonical grid representability -> rejected_out_of_range
   * 3. local-map extent radius -> rejected_outside_extent
   * 4. elevation numeric arithmetic failure -> rejected_numeric
   * 5. accepted point -> accepted_points
   */
  FrameUpdateStats integrateFrame(
      std::span<const PointXYZ> points,
      uint64_t frame_seq);

  /**
   * @brief Updates the map-window center for local extent filtering.
   *
   * Fails and returns false if x or y is non-finite, leaving the previous center unchanged.
   * Window center updates do NOT shift the underlying canonical grid or alter existing CellKeys.
   */
  bool setWindowCenter(double x, double y) noexcept;

  [[nodiscard]] std::pair<double, double> windowCenter() const noexcept;

  /**
   * @brief Configures the acceptance radius in meters.
   *        Fails and returns false if radius is negative or non-finite, preserving old radius.
   *        radius == 0.0 is valid.
   */
  bool setRadius(double radius_m) noexcept;

  [[nodiscard]] double radius() const noexcept;

  /**
   * @brief Pre-reserves hash map buckets for anticipated tile count.
   */
  void reserveTiles(size_t tile_count);

  /**
   * @brief Read-only lookup by CellKey.
   *        Returns nullptr if key is invalid, non-L0, or not active.
   */
  [[nodiscard]] const UniformCell* findCell(const avr::spatial::CellKey& key) const noexcept;

  /**
   * @brief Read-only lookup by continuous world coordinates (odom frame).
   */
  [[nodiscard]] const UniformCell* findCell(double x, double y) const noexcept;
  [[nodiscard]] const UniformCell* findCell(const avr::spatial::WorldPoint2D& pt) const noexcept;

  /**
   * @brief Produces a deterministic snapshot of all active cells sorted by CellKey.
   */
  [[nodiscard]] std::vector<SnapshotRecord> snapshot() const;

  /**
   * @brief Clears all allocated logical tiles, cells, and observations.
   *        Retains container capacity for efficient reuse (Policy A).
   */
  void reset() noexcept;

  /**
   * @brief Returns exact structural and payload statistics.
   */
  [[nodiscard]] MapStats statistics() const noexcept;
  [[nodiscard]] MapStats mapStats() const noexcept { return statistics(); }

  [[nodiscard]] size_t tileCount() const noexcept { return tiles_.size(); }
  [[nodiscard]] size_t activeCellCount() const noexcept { return active_cell_count_; }
  [[nodiscard]] size_t totalAcceptedObservations() const noexcept { return total_observations_; }

 private:
  std::unordered_map<avr::spatial::TileKey, UniformTile> tiles_{};
  double window_center_x_{0.0};
  double window_center_y_{0.0};
  double radius_{100.0};
  double radius_sq_{10000.0};

  size_t active_cell_count_{0};
  size_t total_observations_{0};
  uint64_t integration_generation_{0};
};

}  // namespace avr::mapping

namespace AVR {
namespace mapping = ::avr::mapping;
}

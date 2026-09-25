#pragma once

#include <cstdint>
#include <optional>
#include <span>
#include <unordered_map>
#include <vector>

#include "avr_mapping/evidence_summary.hpp"
#include "avr_mapping/evidence_tile.hpp"
#include "avr_mapping/point_xyz.hpp"
#include "avr_mapping/resolution_decision.hpp"
#include "avr_spatial/cell_key.hpp"
#include "avr_spatial/coordinate.hpp"
#include "avr_spatial/tile_key.hpp"

namespace avr::mapping {

/**
 * @brief Statistics resulting from processing a single frame of LiDAR returns.
 */
struct FrameEvidenceStats {
  uint64_t frame_seq{0};
  uint64_t input_points{0};
  uint64_t accepted_points{0};
  uint64_t rejected_non_finite{0};
  uint64_t rejected_out_of_range{0};
  uint64_t rejected_numeric{0};
  uint32_t active_base_cells{0};
  uint32_t tile_count{0};
  avr::spatial::WorldPoint2D sensor_origin{0.0, 0.0};

  [[nodiscard]] constexpr bool isConsistent() const noexcept {
    return input_points == (accepted_points + rejected_non_finite + rejected_out_of_range + rejected_numeric);
  }

  constexpr bool operator==(const FrameEvidenceStats&) const noexcept = default;
};

/**
 * @brief Ordered snapshot entry for deterministic inspection and replay support.
 */
struct BaseCellSnapshotEntry {
  avr::spatial::CellKey key{};
  EvidenceCell cell{};

  constexpr bool operator==(const BaseCellSnapshotEntry&) const noexcept = default;
  constexpr auto operator<=>(const BaseCellSnapshotEntry& other) const noexcept {
    return key <=> other.key;
  }
};

/**
 * @brief Sparse, frame-local evidence aggregator and resolution-signal provider.
 *
 * CRITICAL ARCHITECTURAL CONTRACT:
 * FrameEvidence is TEMPORARY measurement support for ONE logical sensor frame.
 * It is NOT:
 * - UniformMap
 * - AdaptiveHierarchy
 * - persistent map state
 * - replay history
 * - map revision state
 *
 * It uses canonical L0 (5 cm) bins internally without defeating AVR's persistent
 * memory-saving objective because only occupied frame-local evidence is represented
 * and the structure is replaced each frame.
 *
 * Characteristics:
 * - Frame-Local Lifecycle: Ingests one frame; subsequent buildFrame replaces previous evidence.
 * - Transactional Sensor Origin: If sensor_origin is non-finite, buildFrame fails immediately
 *   and retains previous valid frame evidence unmodified.
 * - Numerically Stable: Welford accumulator per cell; parallel Welford aggregation across regions.
 * - Strictly Read-Only to Hierarchy: FrameEvidence does NOT mutate or own AdaptiveHierarchy.
 */
class FrameEvidence {
 public:
  FrameEvidence() = default;
  ~FrameEvidence() = default;

  FrameEvidence(const FrameEvidence&) = delete;
  FrameEvidence& operator=(const FrameEvidence&) = delete;
  FrameEvidence(FrameEvidence&&) noexcept = default;
  FrameEvidence& operator=(FrameEvidence&&) noexcept = default;

  /**
   * @brief Replaces current evidence with newly aggregated evidence from a single sensor frame.
   *
   * Transactional Contract:
   * - Validates sensor_origin (must be finite in both axes) BEFORE modifying existing state.
   * - If sensor_origin is invalid, returns std::nullopt and leaves existing evidence unchanged.
   * - If sensor_origin is valid, clears previous logical frame evidence and bins accepted points.
   * - Valid individual points are accepted; individual invalid points are classified and rejected
   *   without aborting the frame.
   *
   * @param points Span of input LiDAR returns in odom frame.
   * @param frame_seq Logical sequence number of this frame.
   * @param sensor_origin Continuous 2D position of sensor in odom frame (meters).
   * @return FrameEvidenceStats on success, std::nullopt if sensor origin is invalid.
   */
  std::optional<FrameEvidenceStats> buildFrame(
      std::span<const PointXYZ> points,
      uint64_t frame_seq,
      avr::spatial::WorldPoint2D sensor_origin);

  /**
   * @brief Returns true if a valid logical frame is currently loaded.
   */
  [[nodiscard]] bool hasFrame() const noexcept {
    return has_frame_;
  }

  /**
   * @brief Accesses statistics for the current frame.
   */
  [[nodiscard]] const FrameEvidenceStats& stats() const noexcept {
    return stats_;
  }

  /**
   * @brief Sequence number of current frame (0 if no frame).
   */
  [[nodiscard]] uint64_t frameSeq() const noexcept {
    return stats_.frame_seq;
  }

  /**
   * @brief Continuous sensor origin in odom coordinates for current frame.
   */
  [[nodiscard]] avr::spatial::WorldPoint2D sensorOrigin() const noexcept {
    return stats_.sensor_origin;
  }

  /**
   * @brief Count of allocated Level 0 evidence tiles in the sparse map.
   */
  [[nodiscard]] size_t tileCount() const noexcept {
    return tiles_.size();
  }

  /**
   * @brief Total count of active Level 0 base cells in current frame.
   */
  [[nodiscard]] uint32_t activeBaseCellCount() const noexcept {
    return stats_.active_base_cells;
  }

  /**
   * @brief Read-only aggregation over any valid CellKey from L0 through L3.
   *
   * Queries exact integer base-cell footprint (at most 64 cells for L3; O(1) bounded).
   * Empty regions return observation_count = 0, occupied_base_cells = 0, density_score = 0.0.
   *
   * @param region Target CellKey at level L0, L1, L2, or L3.
   * @return Aggregated evidence summary.
   */
  [[nodiscard]] EvidenceSummary summarize(avr::spatial::CellKey region) const noexcept;

  /**
   * @brief Constructs ResolutionSignals for a target region to feed ResolutionController.
   *
   * Derives distance_m, observation_count (saturated to UINT32_MAX), density_score,
   * and geometry_complexity. Passes through external upstream cues.
   *
   * @param region Target CellKey footprint.
   * @param semantic External semantic relevance (defaults to NONE).
   * @param dynamic External dynamic relevance (defaults to NONE).
   * @param uncertainty External upstream model uncertainty (defaults to std::nullopt).
   * @return Populated ResolutionSignals, or std::nullopt if frame is absent or inputs invalid.
   */
  [[nodiscard]] std::optional<ResolutionSignals> makeResolutionSignals(
      avr::spatial::CellKey region,
      SemanticRelevance semantic = SemanticRelevance::NONE,
      DynamicRelevance dynamic = DynamicRelevance::NONE,
      std::optional<double> uncertainty = std::nullopt) const noexcept;

  /**
   * @brief Produces a deterministic sorted list of all active L0 cells in the current frame.
   *
   * Sorted strictly by CellKey (Morton code order), ensuring container-independent order.
   */
  [[nodiscard]] std::vector<BaseCellSnapshotEntry> baseCellSnapshot() const;

  /**
   * @brief Explicitly clears all frame evidence and resets to uninitialized state.
   *
   * Container bucket allocations may be retained for reuse.
   */
  void reset() noexcept;

 private:
  std::unordered_map<avr::spatial::TileKey, EvidenceTile> tiles_{};
  FrameEvidenceStats stats_{};
  bool has_frame_{false};
};

}  // namespace avr::mapping

namespace AVR {
namespace mapping = ::avr::mapping;
namespace mapping_core = ::avr::mapping;
}

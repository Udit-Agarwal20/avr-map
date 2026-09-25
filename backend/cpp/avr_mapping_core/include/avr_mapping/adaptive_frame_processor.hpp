#pragma once

#include <memory>
#include <optional>
#include <span>
#include <vector>

#include "avr_mapping/adaptive_frame_stats.hpp"
#include "avr_mapping/adaptive_hierarchy.hpp"
#include "avr_mapping/frame_evidence.hpp"
#include "avr_mapping/point_xyz.hpp"
#include "avr_mapping/region_decision_record.hpp"
#include "avr_mapping/resolution_controller.hpp"
#include "avr_mapping/resolution_policy.hpp"
#include "avr_spatial/coordinate.hpp"

namespace avr::mapping {

/**
 * @brief Single-writer geometry-only end-to-end adaptive frame processing pipeline.
 *
 * Connects:
 *   FrameEvidence -> ResolutionSignals -> ResolutionController -> AdaptiveHierarchy
 *
 * Core Architecture & Lifecycle:
 * - Single-Writer Contract: processFrame() must not be called concurrently on the same instance.
 * - Frame-Derived Topology: Rebuilds adaptive hierarchy per valid frame; no cross-frame hysteresis.
 * - Transactional Sensor Origin: If sensor_origin is invalid, processFrame fails with std::nullopt
 *   and preserves previous FrameEvidence, AdaptiveHierarchy, and diagnostics unmodified.
 * - Sparse Root Seeding: L3 roots are allocated exclusively where frame evidence exists.
 * - Monotonic Refinement: Ancestor effective resolution target propagates down observed branches.
 * - Structural Empty Sibling Rule: Children created by a split that contain zero observations
 *   remain structural coverage leaves and are never recursively refined from distance alone.
 */
class AdaptiveFrameProcessor {
 public:
  /**
   * @brief Factory creating a validated AdaptiveFrameProcessor instance.
   *
   * @param config Policy configuration. Must be valid according to ResolutionPolicyConfig::isValid().
   * @param pool_block_capacity Initial block capacity for AdaptiveHierarchy TilePool.
   * @return Unique pointer to processor, or nullptr if config is invalid.
   */
  [[nodiscard]] static std::unique_ptr<AdaptiveFrameProcessor> create(
      const ResolutionPolicyConfig& config = ResolutionPolicyConfig{},
      size_t pool_block_capacity = TilePool::DEFAULT_BLOCK_CAPACITY);

  ~AdaptiveFrameProcessor() = default;

  AdaptiveFrameProcessor(const AdaptiveFrameProcessor&) = delete;
  AdaptiveFrameProcessor& operator=(const AdaptiveFrameProcessor&) = delete;
  AdaptiveFrameProcessor(AdaptiveFrameProcessor&&) = delete;
  AdaptiveFrameProcessor& operator=(AdaptiveFrameProcessor&&) = delete;

  /**
   * @brief Processes a single LiDAR sensor frame and generates frame-local adaptive topology.
   *
   * @param points Span of input points in odom frame.
   * @param frame_seq Logical sequence number of the frame.
   * @param sensor_origin Continuous 2D position of sensor in odom frame.
   * @return AdaptiveFrameStats on success, std::nullopt if global frame parameters are invalid.
   */
  std::optional<AdaptiveFrameStats> processFrame(
      std::span<const PointXYZ> points,
      uint64_t frame_seq,
      avr::spatial::WorldPoint2D sensor_origin);

  /**
   * @brief Read-only view of current frame evidence.
   */
  [[nodiscard]] const FrameEvidence& frameEvidence() const noexcept {
    return frame_evidence_;
  }

  /**
   * @brief Read-only view of current frame adaptive hierarchy topology.
   */
  [[nodiscard]] const AdaptiveHierarchy& hierarchy() const noexcept {
    return hierarchy_;
  }

  /**
   * @brief Read-only view of resolution decision controller.
   */
  [[nodiscard]] const ResolutionController& controller() const noexcept {
    return controller_;
  }

  /**
   * @brief Read-only view of last completed frame stats.
   */
  [[nodiscard]] const std::optional<AdaptiveFrameStats>& lastStats() const noexcept {
    return last_stats_;
  }

  /**
   * @brief Returns deterministic diagnostic records sorted by region CellKey.
   */
  [[nodiscard]] std::vector<RegionDecisionRecord> decisionSnapshot() const;

 private:
  explicit AdaptiveFrameProcessor(
      ResolutionController controller,
      size_t pool_block_capacity);

  ResolutionController controller_;
  FrameEvidence frame_evidence_{};
  AdaptiveHierarchy hierarchy_;
  std::vector<RegionDecisionRecord> decision_records_{};
  std::optional<AdaptiveFrameStats> last_stats_{std::nullopt};
};

}  // namespace avr::mapping

namespace AVR {
namespace mapping = ::avr::mapping;
namespace mapping_core = ::avr::mapping;
}  // namespace AVR

#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <vector>

#include "avr_mapping/frame_evidence.hpp"
#include "avr_mapping/persistent_adaptive_map.hpp"
#include "avr_mapping/persistent_adaptive_pipeline_types.hpp"
#include "avr_mapping/recent_observation_cache.hpp"
#include "avr_mapping/resolution_controller.hpp"
#include "avr_mapping/resolution_hysteresis.hpp"
#include "avr_spatial/cell_key.hpp"
#include "avr_spatial/coordinate.hpp"
#include "avr_mapping/point_xyz.hpp"

namespace avr::mapping {

/**
 * @brief Autonomous coordinator for the persistent multi-frame adaptive mapping pipeline.
 *
 * Implements M9B3F1 Persistent Adaptive Frame Pipeline V1:
 * - Single-writer coordinator connecting FrameEvidence, ResolutionController,
 *   ResolutionHysteresisController, PersistentAdaptiveMap, and RecentObservationCache.
 * - Staged multi-phase processing per raw LiDAR frame:
 *   1. Sequence and generation preflight checks.
 *   2. FrameEvidence building and staged raw-cache preparation (old cache window preserved).
 *   3. Pre-existing region and prospective L3 root discovery from active base cells.
 *   4. Pre-structural M4 resolution decisions and non-mutating M9B1 hysteresis preview.
 *   5. Prospective root allocation and localized recursive evidence-bearing refinement.
 *   6. Immediate direct-parent raw historical replay before deeper child splitting.
 *   7. Full sensor frame fusion into finalized adaptive topology.
 *   8. Post-fusion surviving hysteresis batch commit and structurally removed leaf cleanup.
 *   9. Advancing processing generation and committing staged raw observation frame to cache.
 *
 * Concurrency & Threading:
 * - Strictly single-writer, non-copyable, and non-movable.
 * - No internal locks; designed for exclusive execution on the dedicated map worker thread.
 */
class PersistentAdaptiveFramePipeline {
 public:
  /**
   * @brief Factory constructor that validates all configuration sub-bundles.
   *
   * @param config Pipeline configuration parameters.
   * @return Populated unique_ptr to PersistentAdaptiveFramePipeline, or nullptr if configuration is invalid.
   */
  [[nodiscard]] static std::unique_ptr<PersistentAdaptiveFramePipeline> create(
      const PersistentAdaptivePipelineConfig& config = PersistentAdaptivePipelineConfig{});

  ~PersistentAdaptiveFramePipeline() = default;

  PersistentAdaptiveFramePipeline(const PersistentAdaptiveFramePipeline&) = delete;
  PersistentAdaptiveFramePipeline& operator=(const PersistentAdaptiveFramePipeline&) = delete;
  PersistentAdaptiveFramePipeline(PersistentAdaptiveFramePipeline&&) = delete;
  PersistentAdaptiveFramePipeline& operator=(PersistentAdaptiveFramePipeline&&) = delete;

  /**
   * @brief Ingests and processes a single raw sensor observation frame through the pipeline.
   *
   * @param raw_points Unfiltered raw 3D LiDAR point returns in odom coordinates.
   * @param frame_seq Monotonically increasing logical sensor frame sequence number.
   * @param sensor_origin Continuous 2D position of sensor in odom coordinates.
   * @return AdaptivePipelineFrameResult detailing status, accounting counters, decisions, and structural actions.
   */
  AdaptivePipelineFrameResult processFrame(
      std::span<const PointXYZ> raw_points,
      uint64_t frame_seq,
      avr::spatial::WorldPoint2D sensor_origin);

  // Read-only inspection accessors
  [[nodiscard]] const PersistentAdaptiveMap& map() const noexcept {
    return persistent_map_;
  }

  [[nodiscard]] const RecentObservationCache& replayCache() const noexcept {
    return replay_cache_;
  }

  [[nodiscard]] const ResolutionHysteresisController& hysteresis() const noexcept {
    return hysteresis_;
  }

  [[nodiscard]] const ResolutionController& resolutionController() const noexcept {
    return resolution_controller_;
  }

  /**
   * @brief Read-only accessor for pipeline FrameEvidence scratch state.
   *
   * Note: Returns the last attempted successfully-built FrameEvidence scratch state,
   * which may represent an uncommitted or rejected frame. It is not persistent map state.
   */
  [[nodiscard]] const FrameEvidence& frameEvidence() const noexcept {
    return frame_evidence_;
  }

  [[nodiscard]] uint64_t processingGeneration() const noexcept {
    return processing_generation_;
  }

  [[nodiscard]] bool fatalErrorLatched() const noexcept {
    return fatal_error_latched_;
  }

 private:
  PersistentAdaptiveFramePipeline(
      ResolutionController controller,
      ResolutionHysteresisController hysteresis,
      RecentObservationCache cache,
      size_t pool_block_capacity);

  // Helper context for recursive localized refinement
  struct RefinementContext {
    AdaptivePipelineFrameResult& result;
    std::vector<avr::spatial::CellKey>& structurally_removed_preexisting_leaves;
    const std::vector<avr::spatial::CellKey>& preexisting_leaves_lookup;
    bool fatal_replay_failure{false};
  };

  /**
   * @brief Pipeline-private recursive localized refinement helper.
   *
   * Splits one level at a time, replays direct children if historical parent,
   * inspects child-local evidence, bounds child decisions, and recurses only into
   * evidence-bearing children requesting finer resolution.
   */
  bool refineRegionFromEvidence(
      avr::spatial::CellKey region,
      uint8_t authorized_target,
      bool is_historical_parent,
      RefinementContext& ctx);

  FrameEvidence frame_evidence_{};
  ResolutionController resolution_controller_;
  ResolutionHysteresisController hysteresis_;
  PersistentAdaptiveMap persistent_map_;
  RecentObservationCache replay_cache_;
  uint64_t processing_generation_{0};
  bool fatal_error_latched_{false};

  // Test-only fault injection flags
  bool test_simulate_fusion_failure_{false};
  bool test_simulate_cache_commit_bad_alloc_{false};
  bool test_simulate_cache_commit_failure_{false};
  bool test_simulate_policy_commit_revision_mismatch_{false};
  bool test_simulate_policy_commit_bad_alloc_{false};
  bool test_simulate_invariant_failure_{false};

  friend struct PersistentAdaptivePipelineTestAccess;
};

}  // namespace avr::mapping

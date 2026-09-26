#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <unordered_map>
#include <utility>
#include <vector>

#include "avr_mapping/frame_evidence.hpp"
#include "avr_mapping/persistent_adaptive_map.hpp"
#include "avr_mapping/persistent_adaptive_pipeline_types.hpp"
#include "avr_mapping/recent_observation_cache.hpp"
#include "avr_mapping/resolution_controller.hpp"
#include "avr_mapping/resolution_hysteresis.hpp"
#include "avr_mapping/semantic_frame_evidence.hpp"
#include "avr_mapping/semantic_provider.hpp"
#include "avr_spatial/cell_key.hpp"
#include "avr_spatial/coordinate.hpp"
#include "avr_mapping/point_xyz.hpp"

namespace avr::mapping {

/**
 * @brief Autonomous coordinator for the persistent multi-frame adaptive mapping pipeline.
 *
 * Implements M9B3F1/M9B3F2 Persistent Adaptive Frame Pipeline:
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
 *   9. Advancing processing generation and updating persistent leaf policy metadata.
 *   10. Proactive stale readiness expiration (W-generation window).
 *   11. Committing staged raw observation frame to cache.
 *   12. Post-fusion automatic coarsening (direct sibling merges and complete-shadow recovery).
 *   13. Cross-subsystem policy and map invariant validation.
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
   * Equivalent to create(config, nullptr).
   *
   * @param config Pipeline configuration parameters.
   * @return Populated unique_ptr to PersistentAdaptiveFramePipeline, or nullptr if configuration is invalid.
   */
  [[nodiscard]] static std::unique_ptr<PersistentAdaptiveFramePipeline> create(
      const PersistentAdaptivePipelineConfig& config = PersistentAdaptivePipelineConfig{});

  /**
   * @brief Factory constructor with custom replaceable SemanticProvider.
   *
   * @param config Pipeline configuration parameters.
   * @param provider Unique pointer to custom SemanticProvider (or nullptr for geometry-only).
   * @return Populated unique_ptr to PersistentAdaptiveFramePipeline, or nullptr if configuration is invalid.
   */
  [[nodiscard]] static std::unique_ptr<PersistentAdaptiveFramePipeline> create(
      const PersistentAdaptivePipelineConfig& config,
      std::unique_ptr<SemanticProvider> provider);

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

  [[nodiscard]] const SemanticFrameEvidence& semanticFrameEvidence() const noexcept {
    return semantic_frame_evidence_;
  }

  [[nodiscard]] uint64_t processingGeneration() const noexcept {
    return processing_generation_;
  }

  [[nodiscard]] bool fatalErrorLatched() const noexcept {
    return fatal_error_latched_;
  }

  /**
   * @brief Returns a deterministic, canonically CellKey-sorted snapshot of all active leaf policy metadata.
   */
  [[nodiscard]] std::vector<std::pair<avr::spatial::CellKey, LeafPolicyMetadata>>
  leafPolicyMetadataSnapshot() const;

 private:
  PersistentAdaptiveFramePipeline(
      ResolutionController controller,
      ResolutionHysteresisController hysteresis,
      RecentObservationCache cache,
      size_t pool_block_capacity,
      uint64_t coarsen_group_max_age_generations,
      SemanticPolicyConfig semantic_config,
      std::unique_ptr<SemanticProvider> provider);

  // Helper context for recursive localized refinement
  struct RefinementContext {
    AdaptivePipelineFrameResult& result;
    std::vector<avr::spatial::CellKey>& structurally_removed_preexisting_leaves;
    const std::vector<avr::spatial::CellKey>& preexisting_leaves_lookup;
    bool fatal_replay_failure{false};
  };

  enum class CoarsenCandidateType : uint8_t {
    DIRECT_MERGE = 0,
    RECOVERY_COLLAPSE = 1,
  };

  struct CoarsenCandidate {
    CoarsenCandidateType type{CoarsenCandidateType::DIRECT_MERGE};
    avr::spatial::CellKey root{};
    uint8_t level{0};
    ResolutionReasonBits reasons{};
  };

  /**
   * @brief Pipeline-private recursive localized refinement helper.
   */
  bool refineRegionFromEvidence(
      avr::spatial::CellKey region,
      uint8_t authorized_target,
      bool is_historical_parent,
      RefinementContext& ctx);

  /**
   * @brief Helper combining full geometry FrameEvidence with localized SemanticFrameEvidence.
   */
  [[nodiscard]] std::optional<ResolutionSignals> makeResolutionSignalsForRegion(
      const avr::spatial::CellKey& region) const noexcept;

  /**
   * @brief Expires stale READY metadata where (processing_generation - latest_ready_generation) > W.
   *        Acknowledges hysteresis state with CANCELLED. Returns false if fatal corruption occurs.
   */
  bool expireStaleReadyEntries(AdaptivePipelineFrameResult& result);

  /**
   * @brief Validates all active leaf policy metadata consistency invariants.
   */
  [[nodiscard]] bool validatePolicyMetadataInvariants() const noexcept;

  FrameEvidence frame_evidence_{};
  ResolutionController resolution_controller_;
  ResolutionHysteresisController hysteresis_;
  PersistentAdaptiveMap persistent_map_;
  RecentObservationCache replay_cache_;
  uint64_t processing_generation_{0};
  uint64_t coarsen_group_max_age_generations_{2};
  bool fatal_error_latched_{false};

  SemanticPolicyConfig semantic_config_{};
  std::unique_ptr<SemanticProvider> provider_{nullptr};
  SemanticFrameEvidence semantic_frame_evidence_{};
  FrameEvidence static_fusion_evidence_{};
  std::vector<PointXYZ> static_points_scratch_{};

  std::unordered_map<avr::spatial::CellKey, LeafPolicyMetadata> leaf_policy_metadata_{};

  // Test-only fault injection flags
  bool test_simulate_fusion_failure_{false};
  bool test_simulate_cache_commit_bad_alloc_{false};
  bool test_simulate_cache_commit_failure_{false};
  bool test_simulate_policy_commit_revision_mismatch_{false};
  bool test_simulate_policy_commit_bad_alloc_{false};
  bool test_simulate_invariant_failure_{false};
  bool test_simulate_metadata_bad_alloc_{false};
  bool test_simulate_ack_failure_{false};
  bool test_simulate_coarsen_internal_error_{false};

  friend struct PersistentAdaptivePipelineTestAccess;
};

}  // namespace avr::mapping

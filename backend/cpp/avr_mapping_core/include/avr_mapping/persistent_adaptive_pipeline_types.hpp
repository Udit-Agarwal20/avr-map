#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "avr_mapping/recent_observation_cache.hpp"
#include "avr_mapping/resolution_controller.hpp"
#include "avr_mapping/resolution_decision.hpp"
#include "avr_mapping/resolution_hysteresis.hpp"
#include "avr_mapping/resolution_hysteresis_config.hpp"
#include "avr_mapping/resolution_policy.hpp"
#include "avr_mapping/tile_pool.hpp"
#include "avr_spatial/cell_key.hpp"

namespace avr::mapping {

/**
 * @brief Categorical execution outcome of processing a sensor frame through the persistent pipeline.
 */
enum class AdaptivePipelineStatus : uint8_t {
  /// Frame successfully processed and fused with all descendant replays complete.
  SUCCESS = 0,

  /// Frame successfully processed and fused, but one or more localized refinement splits
  /// have historical replay pending due to raw cache eviction.
  SUCCESS_REPLAY_PENDING = 1,

  /// Frame successfully fused and hysteresis committed, but raw observation cache commit
  /// degraded (e.g. allocation failure during cache push). Map and policy state remain valid.
  SUCCESS_CACHE_DEGRADED = 2,

  /// Frame rejected during input validation (e.g. non-finite sensor origin or raw cache prepare failure).
  FRAME_REJECTED_INPUT = 3,

  /// Frame rejected during sequence preflight (frame_seq <= last_fused_frame_seq). Zero mutation.
  FRAME_REJECTED_SEQUENCE = 4,

  /// Signal extraction or resolution controller decision failed unexpectedly. Zero structural mutation.
  FRAME_REJECTED_DECISION = 5,

  /// Pre-existing region hysteresis preview returned invalid observation or non-committable state.
  FRAME_REJECTED_POLICY = 6,

  /// Hierarchy split or child evaluation encountered an internal invariant error.
  REFINEMENT_INTERNAL_ERROR = 7,

  /// Direct historical replay encountered measurement/temporal corruption or conservation failure.
  REPLAY_INTERNAL_ERROR = 8,

  /// Persistent map fusion failed without any structural topology changes. Zero map mutation.
  FUSION_FAILED = 9,

  /// Persistent map fusion failed after structural changes (root creation, splits, or replays) were committed.
  /// Structural topology is conservatively preserved; prepared hysteresis and raw cache are discarded.
  FUSION_FAILED_AFTER_STRUCTURAL_CHANGE = 10,

  /// Staged hysteresis batch commit failed unexpectedly after map fusion. Fatal policy desync.
  POLICY_COMMIT_INTERNAL_ERROR = 11,

  /// Staged hysteresis batch commit threw std::bad_alloc after map fusion. Fatal policy desync.
  POLICY_COMMIT_FATAL = 12,

  /// Raw cache commit failed logically after successful map fusion and policy commit.
  CACHE_COMMIT_INTERNAL_ERROR = 13,

  /// Pipeline processing generation reached UINT64_MAX. Terminal freeze with zero mutation.
  GENERATION_EXHAUSTED = 14,

  /// Resolution hysteresis controller revision reached UINT64_MAX. Terminal freeze with zero mutation.
  POLICY_REVISION_EXHAUSTED = 15,

  /// Post-processing cross-subsystem invariant validation failed.
  INVARIANT_FAILURE = 16,

  /// Pipeline is permanently latched in fatal error state from an earlier failure. Zero mutation.
  PIPELINE_FATAL_LATCHED = 17,

  /// Automatic coarsening or complete-shadow recovery encountered internal corruption or fatal failure.
  COARSENING_INTERNAL_ERROR = 18,
};

[[nodiscard]] constexpr const char* adaptivePipelineStatusToString(
    AdaptivePipelineStatus status) noexcept {
  switch (status) {
    case AdaptivePipelineStatus::SUCCESS:
      return "SUCCESS";
    case AdaptivePipelineStatus::SUCCESS_REPLAY_PENDING:
      return "SUCCESS_REPLAY_PENDING";
    case AdaptivePipelineStatus::SUCCESS_CACHE_DEGRADED:
      return "SUCCESS_CACHE_DEGRADED";
    case AdaptivePipelineStatus::FRAME_REJECTED_INPUT:
      return "FRAME_REJECTED_INPUT";
    case AdaptivePipelineStatus::FRAME_REJECTED_SEQUENCE:
      return "FRAME_REJECTED_SEQUENCE";
    case AdaptivePipelineStatus::FRAME_REJECTED_DECISION:
      return "FRAME_REJECTED_DECISION";
    case AdaptivePipelineStatus::FRAME_REJECTED_POLICY:
      return "FRAME_REJECTED_POLICY";
    case AdaptivePipelineStatus::REFINEMENT_INTERNAL_ERROR:
      return "REFINEMENT_INTERNAL_ERROR";
    case AdaptivePipelineStatus::REPLAY_INTERNAL_ERROR:
      return "REPLAY_INTERNAL_ERROR";
    case AdaptivePipelineStatus::FUSION_FAILED:
      return "FUSION_FAILED";
    case AdaptivePipelineStatus::FUSION_FAILED_AFTER_STRUCTURAL_CHANGE:
      return "FUSION_FAILED_AFTER_STRUCTURAL_CHANGE";
    case AdaptivePipelineStatus::POLICY_COMMIT_INTERNAL_ERROR:
      return "POLICY_COMMIT_INTERNAL_ERROR";
    case AdaptivePipelineStatus::POLICY_COMMIT_FATAL:
      return "POLICY_COMMIT_FATAL";
    case AdaptivePipelineStatus::CACHE_COMMIT_INTERNAL_ERROR:
      return "CACHE_COMMIT_INTERNAL_ERROR";
    case AdaptivePipelineStatus::GENERATION_EXHAUSTED:
      return "GENERATION_EXHAUSTED";
    case AdaptivePipelineStatus::POLICY_REVISION_EXHAUSTED:
      return "POLICY_REVISION_EXHAUSTED";
    case AdaptivePipelineStatus::INVARIANT_FAILURE:
      return "INVARIANT_FAILURE";
    case AdaptivePipelineStatus::PIPELINE_FATAL_LATCHED:
      return "PIPELINE_FATAL_LATCHED";
    case AdaptivePipelineStatus::COARSENING_INTERNAL_ERROR:
      return "COARSENING_INTERNAL_ERROR";
  }
  return "UNKNOWN";
}

/**
 * @brief Categorical type of structural topology action executed during a frame cycle.
 */
enum class StructuralActionType : uint8_t {
  ROOT_CREATE = 0,
  SPLIT = 1,
  REPLAY_COMPLETE = 2,
  REPLAY_PENDING = 3,
  DIRECT_MERGE = 4,
  SUBTREE_RECOVERY_COLLAPSE = 5,
};

[[nodiscard]] constexpr const char* structuralActionTypeToString(
    StructuralActionType action) noexcept {
  switch (action) {
    case StructuralActionType::ROOT_CREATE:
      return "ROOT_CREATE";
    case StructuralActionType::SPLIT:
      return "SPLIT";
    case StructuralActionType::REPLAY_COMPLETE:
      return "REPLAY_COMPLETE";
    case StructuralActionType::REPLAY_PENDING:
      return "REPLAY_PENDING";
    case StructuralActionType::DIRECT_MERGE:
      return "DIRECT_MERGE";
    case StructuralActionType::SUBTREE_RECOVERY_COLLAPSE:
      return "SUBTREE_RECOVERY_COLLAPSE";
  }
  return "UNKNOWN";
}

/**
 * @brief Record of a structural topology mutation or replay event.
 */
struct StructuralActionRecord {
  avr::spatial::CellKey region{};
  uint8_t from_level{0};
  uint8_t to_level{0};
  ResolutionReasonBits reasons{};
  StructuralActionType action_type{StructuralActionType::ROOT_CREATE};

  constexpr bool operator==(const StructuralActionRecord&) const noexcept = default;
};

/**
 * @brief Persistent leaf policy metadata tracking latest temporal decision and readiness.
 */
struct LeafPolicyMetadata {
  uint64_t latest_decision_generation{0};
  uint64_t latest_ready_generation{0};
  uint8_t latest_requested_level{avr::spatial::L0};
  ResolutionReasonBits latest_reasons{};

  constexpr bool operator==(const LeafPolicyMetadata&) const noexcept = default;
};

/**
 * @brief Record of a resolution policy evaluation for a touched or prospective region.
 */
struct PipelineDecisionRecord {
  avr::spatial::CellKey region{};
  uint8_t current_level{0};
  uint8_t requested_level{0};
  uint8_t baseline_level{0};
  ResolutionReasonBits reasons{};
  HysteresisAction hysteresis_action{HysteresisAction::INVALID_OBSERVATION};
  bool had_current_evidence{false};
  bool existed_pre_frame{false};
  bool used_hysteresis{false};

  constexpr bool operator==(const PipelineDecisionRecord&) const noexcept = default;
};

/**
 * @brief Detailed accounting and status report returned after processing a single sensor frame.
 */
struct AdaptivePipelineFrameResult {
  uint64_t frame_seq{0};
  uint64_t processing_generation_before{0};
  uint64_t processing_generation_after{0};
  size_t input_points{0};
  size_t accepted_points{0};
  bool frame_fused{false};
  bool structure_changed{false};
  bool cache_prepared{false};
  bool cache_committed{false};
  bool replay_pending{false};

  uint32_t regions_evaluated{0};
  uint32_t prospective_roots{0};
  uint32_t refine_requests{0};
  uint32_t splits_applied{0};
  uint32_t replay_attempts{0};
  uint32_t replay_complete{0};
  uint32_t replay_insufficient{0};
  uint32_t hysteresis_previews{0};
  uint32_t hysteresis_committed{0};
  uint32_t hysteresis_erased{0};

  uint32_t coarsen_groups_considered{0};
  uint32_t direct_merge_attempts{0};
  uint32_t direct_merges_applied{0};
  uint32_t direct_merges_blocked{0};
  uint32_t stale_ready_cancelled{0};
  uint32_t recovery_candidates{0};
  uint32_t recovery_attempts{0};
  uint32_t recovery_applied{0};
  uint32_t recovery_blocked{0};

  AdaptivePipelineStatus status{AdaptivePipelineStatus::FRAME_REJECTED_INPUT};

  std::vector<PipelineDecisionRecord> decision_records{};
  std::vector<StructuralActionRecord> structural_action_records{};

  [[nodiscard]] bool isSuccess() const noexcept {
    return status == AdaptivePipelineStatus::SUCCESS ||
           status == AdaptivePipelineStatus::SUCCESS_REPLAY_PENDING ||
           status == AdaptivePipelineStatus::SUCCESS_CACHE_DEGRADED;
  }
};

/**
 * @brief Configuration bundle for PersistentAdaptiveFramePipeline V1.
 */
struct PersistentAdaptivePipelineConfig {
  ResolutionPolicyConfig policy_config{};
  ResolutionHysteresisConfig hysteresis_config{};
  ReplayCacheConfig replay_cache_config{};
  size_t pool_block_capacity{TilePool::DEFAULT_BLOCK_CAPACITY};
  uint64_t coarsen_group_max_age_generations{2};

  [[nodiscard]] bool isValid() const noexcept {
    return policy_config.validate() &&
           hysteresis_config.isValid() &&
           replay_cache_config.isValid() &&
           pool_block_capacity > 0;
  }
};

}  // namespace avr::mapping

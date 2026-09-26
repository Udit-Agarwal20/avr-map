#include "avr_mapping/persistent_adaptive_frame_pipeline.hpp"

#include <algorithm>
#include <new>
#include <utility>
#include <vector>

#include "avr_spatial/constants.hpp"

namespace avr::mapping {

std::unique_ptr<PersistentAdaptiveFramePipeline> PersistentAdaptiveFramePipeline::create(
    const PersistentAdaptivePipelineConfig& config) {
  if (!config.isValid()) {
    return nullptr;
  }
  auto controller_opt = ResolutionController::create(config.policy_config);
  if (!controller_opt.has_value()) {
    return nullptr;
  }
  auto hysteresis_opt = ResolutionHysteresisController::create(config.hysteresis_config);
  if (!hysteresis_opt.has_value()) {
    return nullptr;
  }
  auto cache_opt = RecentObservationCache::create(config.replay_cache_config);
  if (!cache_opt.has_value()) {
    return nullptr;
  }

  return std::unique_ptr<PersistentAdaptiveFramePipeline>(
      new PersistentAdaptiveFramePipeline(
          std::move(*controller_opt),
          std::move(*hysteresis_opt),
          std::move(*cache_opt),
          config.pool_block_capacity));
}

PersistentAdaptiveFramePipeline::PersistentAdaptiveFramePipeline(
    ResolutionController controller,
    ResolutionHysteresisController hysteresis,
    RecentObservationCache cache,
    size_t pool_block_capacity)
    : resolution_controller_(std::move(controller)),
      hysteresis_(std::move(hysteresis)),
      persistent_map_(pool_block_capacity),
      replay_cache_(std::move(cache)) {}

AdaptivePipelineFrameResult PersistentAdaptiveFramePipeline::processFrame(
    std::span<const PointXYZ> raw_points,
    uint64_t frame_seq,
    avr::spatial::WorldPoint2D sensor_origin) {
  AdaptivePipelineFrameResult result{};
  result.frame_seq = frame_seq;
  result.processing_generation_before = processing_generation_;
  result.processing_generation_after = processing_generation_;
  result.input_points = raw_points.size();

  // 1. Fatal-state latch check
  if (fatal_error_latched_) {
    result.status = AdaptivePipelineStatus::PIPELINE_FATAL_LATCHED;
    return result;
  }

  // 2. Processing-generation overflow precheck
  if (processing_generation_ == UINT64_MAX) {
    result.status = AdaptivePipelineStatus::GENERATION_EXHAUSTED;
    return result;
  }

  // 3. Resolution-hysteresis revision exhaustion precheck
  if (hysteresis_.stateRevision() == UINT64_MAX) {
    result.status = AdaptivePipelineStatus::POLICY_REVISION_EXHAUSTED;
    return result;
  }

  // 4. Pre-flight frame-sequence check
  if (persistent_map_.hasFusedFrames()) {
    if (frame_seq <= persistent_map_.lastFusedFrameSeq()) {
      result.status = AdaptivePipelineStatus::FRAME_REJECTED_SEQUENCE;
      return result;
    }
  }

  // 5. Phase 1: Build frame evidence
  auto ev_stats_opt = frame_evidence_.buildFrame(raw_points, frame_seq, sensor_origin);
  if (!ev_stats_opt.has_value()) {
    result.status = AdaptivePipelineStatus::FRAME_REJECTED_INPUT;
    return result;
  }
  result.accepted_points = ev_stats_opt->accepted_points;

  // 6. Phase 1: Stage raw cache
  auto prep_frame_opt = replay_cache_.prepareFrame(raw_points, frame_seq, sensor_origin);
  if (!prep_frame_opt.has_value()) {
    result.status = AdaptivePipelineStatus::FRAME_REJECTED_INPUT;
    return result;
  }
  result.cache_prepared = true;

  // If no accepted observations, frame is valid empty.
  if (ev_stats_opt->active_base_cells == 0) {
    auto fusion_opt = persistent_map_.fuseFrameEvidence(frame_evidence_);
    if (!fusion_opt.has_value()) {
      result.status = AdaptivePipelineStatus::FUSION_FAILED;
      auto map_val = persistent_map_.validateInvariants();
      if (!map_val.is_valid) {
        fatal_error_latched_ = true;
        result.status = AdaptivePipelineStatus::INVARIANT_FAILURE;
      }
      return result;
    }
    result.frame_fused = true;
    ++processing_generation_;
    result.processing_generation_after = processing_generation_;

    auto cache_stats = replay_cache_.commitPreparedFrame(std::move(*prep_frame_opt));
    if (!cache_stats.has_value()) {
      fatal_error_latched_ = true;
      result.status = AdaptivePipelineStatus::CACHE_COMMIT_INTERNAL_ERROR;
      return result;
    }
    result.cache_committed = true;

    auto map_val = persistent_map_.validateInvariants();
    if (!map_val.is_valid || test_simulate_invariant_failure_) {
      fatal_error_latched_ = true;
      result.status = AdaptivePipelineStatus::INVARIANT_FAILURE;
      return result;
    }

    result.status = AdaptivePipelineStatus::SUCCESS;
    return result;
  }

  // 6. Discover active base cells, touched pre-existing leaves, and prospective roots (Requirements 13-15)
  const auto base_snapshot = frame_evidence_.baseCellSnapshot();
  std::vector<avr::spatial::CellKey> pre_existing_touched_leaves;
  std::vector<avr::spatial::CellKey> prospective_l3_roots;
  pre_existing_touched_leaves.reserve(base_snapshot.size());
  prospective_l3_roots.reserve(base_snapshot.size());

  for (const auto& entry : base_snapshot) {
    const auto coord = entry.key.decode();
    const avr::spatial::GridCoord2D base_coord{.x = coord.x, .y = coord.y};

    auto leaf_opt = persistent_map_.hierarchy().findLeafContainingBase(base_coord);
    if (leaf_opt.has_value()) {
      pre_existing_touched_leaves.push_back(*leaf_opt);
    } else {
      auto root_opt = AdaptiveHierarchy::rootKeyForBase(base_coord);
      if (root_opt.has_value()) {
        prospective_l3_roots.push_back(*root_opt);
      }
    }
  }

  std::sort(pre_existing_touched_leaves.begin(), pre_existing_touched_leaves.end());
  pre_existing_touched_leaves.erase(
      std::unique(pre_existing_touched_leaves.begin(), pre_existing_touched_leaves.end()),
      pre_existing_touched_leaves.end());

  std::sort(prospective_l3_roots.begin(), prospective_l3_roots.end());
  prospective_l3_roots.erase(
      std::unique(prospective_l3_roots.begin(), prospective_l3_roots.end()),
      prospective_l3_roots.end());

  result.prospective_roots = static_cast<uint32_t>(prospective_l3_roots.size());

  // 7. Pre-existing leaves M4 decisions and staged M9B1 hysteresis preview (Requirements 16, 18, 19, 20)
  std::vector<HysteresisPreparedObservation> prepared_observations;
  prepared_observations.reserve(pre_existing_touched_leaves.size());

  for (const auto& leaf : pre_existing_touched_leaves) {
    ++result.regions_evaluated;
    auto signals_opt = frame_evidence_.makeResolutionSignals(
        leaf, SemanticRelevance::NONE, DynamicRelevance::NONE, std::nullopt);
    if (!signals_opt.has_value()) {
      result.status = AdaptivePipelineStatus::FRAME_REJECTED_DECISION;
      return result;
    }
    auto decision_opt = resolution_controller_.decide(*signals_opt);
    if (!decision_opt.has_value()) {
      result.status = AdaptivePipelineStatus::FRAME_REJECTED_DECISION;
      return result;
    }
    const auto decision = *decision_opt;

    HysteresisObservation obs{
        .region = leaf,
        .frame_seq = frame_seq,
        .decision = decision,
    };
    auto prep = hysteresis_.previewObservation(obs);
    ++result.hysteresis_previews;

    if (!prep.result.isSuccess() || !prep.isCommittable()) {
      result.status = AdaptivePipelineStatus::FRAME_REJECTED_POLICY;
      return result;
    }

    PipelineDecisionRecord dec_rec{
        .region = leaf,
        .current_level = leaf.level,
        .requested_level = decision.requested_level,
        .baseline_level = decision.baseline_level,
        .reasons = decision.reasons,
        .hysteresis_action = prep.result.action,
        .had_current_evidence = true,
        .existed_pre_frame = true,
        .used_hysteresis = true,
    };
    result.decision_records.push_back(dec_rec);
    prepared_observations.push_back(std::move(prep));
  }

  // 8. Prospective roots M4 decisions (Requirements 17, 19)
  std::vector<ResolutionDecision> prospective_root_decisions;
  prospective_root_decisions.reserve(prospective_l3_roots.size());

  for (const auto& root_key : prospective_l3_roots) {
    ++result.regions_evaluated;
    auto signals_opt = frame_evidence_.makeResolutionSignals(
        root_key, SemanticRelevance::NONE, DynamicRelevance::NONE, std::nullopt);
    if (!signals_opt.has_value()) {
      result.status = AdaptivePipelineStatus::FRAME_REJECTED_DECISION;
      return result;
    }
    auto decision_opt = resolution_controller_.decide(*signals_opt);
    if (!decision_opt.has_value()) {
      result.status = AdaptivePipelineStatus::FRAME_REJECTED_DECISION;
      return result;
    }
    const auto decision = *decision_opt;

    PipelineDecisionRecord dec_rec{
        .region = root_key,
        .current_level = avr::spatial::L3,
        .requested_level = decision.requested_level,
        .baseline_level = decision.baseline_level,
        .reasons = decision.reasons,
        .hysteresis_action = HysteresisAction::INVALID_OBSERVATION,  // Prospective roots bypass hysteresis initially
        .had_current_evidence = true,
        .existed_pre_frame = false,
        .used_hysteresis = false,
    };
    result.decision_records.push_back(dec_rec);
    prospective_root_decisions.push_back(decision);
  }

  // 9. Structural Refinement Phase (Requirements 21-37)
  std::vector<avr::spatial::CellKey> structurally_removed_preexisting_leaves;
  RefinementContext ref_ctx{
      .result = result,
      .structurally_removed_preexisting_leaves = structurally_removed_preexisting_leaves,
      .preexisting_leaves_lookup = pre_existing_touched_leaves,
      .fatal_replay_failure = false,
  };

  // Process prospective roots in canonical CellKey order
  for (size_t i = 0; i < prospective_l3_roots.size(); ++i) {
    const auto& root_key = prospective_l3_roots[i];
    const auto& decision = prospective_root_decisions[i];
    const auto coord = root_key.decode();
    const avr::spatial::GridCoord2D base_coord{
        .x = coord.x * static_cast<int32_t>(avr::spatial::levelScale(avr::spatial::L3)),
        .y = coord.y * static_cast<int32_t>(avr::spatial::levelScale(avr::spatial::L3)),
    };

    auto root_created = persistent_map_.ensureRootForBase(base_coord);
    if (!root_created.has_value()) {
      result.status = AdaptivePipelineStatus::REFINEMENT_INTERNAL_ERROR;
      return result;
    }
    result.structure_changed = true;
    result.structural_action_records.push_back(StructuralActionRecord{
        .region = root_key,
        .from_level = avr::spatial::L3,
        .to_level = avr::spatial::L3,
        .reasons = decision.reasons,
        .action_type = StructuralActionType::ROOT_CREATE,
    });

    if (decision.requested_level < avr::spatial::L3) {
      ++result.refine_requests;
      if (!refineRegionFromEvidence(root_key, decision.requested_level, false, ref_ctx)) {
        if (ref_ctx.fatal_replay_failure) {
          fatal_error_latched_ = true;
          for (const auto& removed_key : structurally_removed_preexisting_leaves) {
            if (hysteresis_.eraseRegion(removed_key)) {
              ++result.hysteresis_erased;
            }
          }
          result.status = AdaptivePipelineStatus::REPLAY_INTERNAL_ERROR;
        } else {
          result.status = AdaptivePipelineStatus::REFINEMENT_INTERNAL_ERROR;
        }
        return result;
      }
    }
  }

  // Process pre-existing leaves in canonical CellKey order
  for (size_t i = 0; i < pre_existing_touched_leaves.size(); ++i) {
    const auto& leaf = pre_existing_touched_leaves[i];
    const auto& prep = prepared_observations[i];

    if (prep.result.action == HysteresisAction::REFINE_NOW) {
      ++result.refine_requests;
      const uint8_t authorized_target = prep.result.target_level;
      if (!refineRegionFromEvidence(leaf, authorized_target, true, ref_ctx)) {
        if (ref_ctx.fatal_replay_failure) {
          fatal_error_latched_ = true;
          result.status = AdaptivePipelineStatus::REPLAY_INTERNAL_ERROR;
        } else {
          result.status = AdaptivePipelineStatus::REFINEMENT_INTERNAL_ERROR;
        }
        // Erase old hysteresis states for any pre-existing leaves already converted to INTERNAL
        for (const auto& removed_key : structurally_removed_preexisting_leaves) {
          if (hysteresis_.eraseRegion(removed_key)) {
            ++result.hysteresis_erased;
          }
        }
        return result;
      }
    }
  }

  // 10. Phase 4: Persistent Map Frame Fusion (Requirements 40-43)
  bool simulate_fusion_fail = test_simulate_fusion_failure_;
  std::optional<PersistentFusionStats> fusion_opt = std::nullopt;
  if (!simulate_fusion_fail) {
    fusion_opt = persistent_map_.fuseFrameEvidence(frame_evidence_);
  }

  if (!fusion_opt.has_value()) {
    if (result.structure_changed) {
      // Conservative structural commit: topology remains, prepared hysteresis discarded.
      // Erase old hysteresis states for structurally removed leaves (Requirement 42).
      for (const auto& removed_key : structurally_removed_preexisting_leaves) {
        if (hysteresis_.eraseRegion(removed_key)) {
          ++result.hysteresis_erased;
        }
      }
      result.status = AdaptivePipelineStatus::FUSION_FAILED_AFTER_STRUCTURAL_CHANGE;
      auto map_val = persistent_map_.validateInvariants();
      if (!map_val.is_valid) {
        fatal_error_latched_ = true;
        result.status = AdaptivePipelineStatus::INVARIANT_FAILURE;
      }
      return result;
    } else {
      result.status = AdaptivePipelineStatus::FUSION_FAILED;
      auto map_val = persistent_map_.validateInvariants();
      if (!map_val.is_valid) {
        fatal_error_latched_ = true;
        result.status = AdaptivePipelineStatus::INVARIANT_FAILURE;
      }
      return result;
    }
  }

  // Fusion succeeded!
  result.frame_fused = true;
  ++processing_generation_;
  result.processing_generation_after = processing_generation_;

  // 11. Phase 5: Surviving Hysteresis Batch Commit & Cleanup (Requirements 45-49)
  std::vector<HysteresisPreparedObservation> surviving_batch;
  surviving_batch.reserve(prepared_observations.size());

  for (const auto& prep : prepared_observations) {
    if (persistent_map_.hierarchy().isLeaf(prep.region)) {
      surviving_batch.push_back(prep);
    }
  }

  if (test_simulate_policy_commit_revision_mismatch_) {
    // Simulate unexpected revision desync
    hysteresis_.clear();
  }

  try {
    if (test_simulate_policy_commit_bad_alloc_) {
      throw std::bad_alloc();
    }
    auto batch_res = hysteresis_.commitPreparedBatch(surviving_batch);
    if (!batch_res.isSuccess()) {
      fatal_error_latched_ = true;
      result.status = AdaptivePipelineStatus::POLICY_COMMIT_INTERNAL_ERROR;
      return result;
    }
    result.hysteresis_committed = batch_res.committed_count;

    // ONLY AFTER batch commit succeeds: erase structurally removed old leaves (Requirement 46, 49)
    for (const auto& removed_key : structurally_removed_preexisting_leaves) {
      if (hysteresis_.eraseRegion(removed_key)) {
        ++result.hysteresis_erased;
      }
    }
  } catch (const std::bad_alloc&) {
    fatal_error_latched_ = true;
    result.status = AdaptivePipelineStatus::POLICY_COMMIT_FATAL;
    return result;
  }

  // 12. Phase 6: Raw Cache Commit (Requirements 54-58)
  bool cache_degraded = false;
  if (test_simulate_cache_commit_bad_alloc_) {
    cache_degraded = true;
    result.cache_committed = false;
  } else if (test_simulate_cache_commit_failure_) {
    fatal_error_latched_ = true;
    result.status = AdaptivePipelineStatus::CACHE_COMMIT_INTERNAL_ERROR;
    return result;
  } else {
    try {
      auto cache_stats_opt = replay_cache_.commitPreparedFrame(std::move(*prep_frame_opt));
      if (!cache_stats_opt.has_value()) {
        fatal_error_latched_ = true;
        result.status = AdaptivePipelineStatus::CACHE_COMMIT_INTERNAL_ERROR;
        return result;
      }
      result.cache_committed = true;
    } catch (const std::bad_alloc&) {
      cache_degraded = true;
      result.cache_committed = false;
    }
  }

  // 13. Final Invariant Validation (Requirement 61, 62)
  auto map_val = persistent_map_.validateInvariants();
  if (!map_val.is_valid || test_simulate_invariant_failure_) {
    fatal_error_latched_ = true;
    result.status = AdaptivePipelineStatus::INVARIANT_FAILURE;
    return result;
  }

  if (cache_degraded) {
    result.status = AdaptivePipelineStatus::SUCCESS_CACHE_DEGRADED;
  } else if (result.replay_pending) {
    result.status = AdaptivePipelineStatus::SUCCESS_REPLAY_PENDING;
  } else {
    result.status = AdaptivePipelineStatus::SUCCESS;
  }

  return result;
}

bool PersistentAdaptiveFramePipeline::refineRegionFromEvidence(
    avr::spatial::CellKey region,
    uint8_t authorized_target,
    bool is_historical_parent,
    RefinementContext& ctx) {
  if (region.level <= authorized_target || region.level == avr::spatial::L0) {
    return true;
  }

  // Record if region is pre-existing leaf
  bool is_preexisting = std::binary_search(
      ctx.preexisting_leaves_lookup.begin(),
      ctx.preexisting_leaves_lookup.end(),
      region);
  if (is_preexisting) {
    if (std::find(ctx.structurally_removed_preexisting_leaves.begin(),
                  ctx.structurally_removed_preexisting_leaves.end(),
                  region) == ctx.structurally_removed_preexisting_leaves.end()) {
      ctx.structurally_removed_preexisting_leaves.push_back(region);
    }
  }

  // Check if region has active payload before split
  bool has_historical_payload = (persistent_map_.findActivePayload(region) != nullptr);

  // Split one level
  bool ok = persistent_map_.splitLeaf(region);
  if (!ok) {
    return false;
  }
  ctx.result.structure_changed = true;
  ++ctx.result.splits_applied;
  ctx.result.structural_action_records.push_back(StructuralActionRecord{
      .region = region,
      .from_level = region.level,
      .to_level = static_cast<uint8_t>(region.level - 1),
      .reasons = ResolutionReasonBits{},
      .action_type = StructuralActionType::SPLIT,
  });

  // Direct Replay before child split (Requirement 26)
  if (is_historical_parent || has_historical_payload || persistent_map_.findRetiredCache(region) != nullptr) {
    ++ctx.result.replay_attempts;
    auto replay_res = persistent_map_.replayDirectChildren(region, replay_cache_);
    if (replay_res.status == ReplayStatus::APPLIED_COMPLETE) {
      ++ctx.result.replay_complete;
      ctx.result.structural_action_records.push_back(StructuralActionRecord{
          .region = region,
          .from_level = static_cast<uint8_t>(region.level - 1),
          .to_level = static_cast<uint8_t>(region.level - 1),
          .reasons = ResolutionReasonBits{},
          .action_type = StructuralActionType::REPLAY_COMPLETE,
      });
    } else if (replay_res.status == ReplayStatus::ALREADY_COMPLETE ||
               replay_res.status == ReplayStatus::NO_RETIRED_PARENT_CACHE) {
      // Clean no-op / already complete
    } else if (replay_res.status == ReplayStatus::CACHE_HISTORY_INSUFFICIENT ||
               replay_res.status == ReplayStatus::PARENT_CACHE_INCOMPLETE) {
      ++ctx.result.replay_insufficient;
      ctx.result.replay_pending = true;
      ctx.result.structural_action_records.push_back(StructuralActionRecord{
          .region = region,
          .from_level = static_cast<uint8_t>(region.level - 1),
          .to_level = static_cast<uint8_t>(region.level - 1),
          .reasons = ResolutionReasonBits{},
          .action_type = StructuralActionType::REPLAY_PENDING,
      });
    } else {
      // Replay corruption/invariant failure (Requirement 30)
      ctx.fatal_replay_failure = true;
      return false;
    }
  }

  // Inspect the four children in canonical Morton order
  auto children_opt = avr::spatial::childrenOf(region);
  if (!children_opt.has_value()) {
    return false;
  }
  auto children = *children_opt;
  std::sort(children.begin(), children.end());

  for (const auto& child : children) {
    const auto summary = frame_evidence_.summarize(child);
    if (summary.observation_count == 0) {
      // Unobserved child remains coarse coverage leaf (Requirement 31)
      continue;
    }

    ++ctx.result.regions_evaluated;
    auto signals_opt = frame_evidence_.makeResolutionSignals(
        child, SemanticRelevance::NONE, DynamicRelevance::NONE, std::nullopt);
    if (!signals_opt.has_value()) {
      return false;
    }
    auto child_decision_opt = resolution_controller_.decide(*signals_opt);
    if (!child_decision_opt.has_value()) {
      return false;
    }
    const auto child_decision = *child_decision_opt;

    ctx.result.decision_records.push_back(PipelineDecisionRecord{
        .region = child,
        .current_level = child.level,
        .requested_level = child_decision.requested_level,
        .baseline_level = child_decision.baseline_level,
        .reasons = child_decision.reasons,
        .hysteresis_action = HysteresisAction::INVALID_OBSERVATION,
        .had_current_evidence = true,
        .existed_pre_frame = false,
        .used_hysteresis = false,
    });

    if (child_decision.requested_level < child.level) {
      ++ctx.result.refine_requests;
    }

    if (child_decision.requested_level >= child.level) {
      // Child M4 stops earlier (Requirement 32, 34)
      continue;
    }

    // Child requests finer: bound by authorized_target (Requirement 33)
    uint8_t effective_child_target = std::max(child_decision.requested_level, authorized_target);
    if (child.level > effective_child_target && child.level > avr::spatial::L0) {
      if (!refineRegionFromEvidence(child, effective_child_target, false, ctx)) {
        return false;
      }
    }
  }

  return true;
}

}  // namespace avr::mapping

#include "avr_mapping/persistent_adaptive_frame_pipeline.hpp"

#include <algorithm>
#include <new>
#include <utility>
#include <vector>

#include "avr_spatial/constants.hpp"

namespace avr::mapping {

namespace {

bool isAncestorOrDescendant(const avr::spatial::CellKey& a, const avr::spatial::CellKey& b) noexcept {
  if (a == b) return true;
  if (a.level > b.level) {
    auto curr = avr::spatial::parentOf(b);
    while (curr.has_value()) {
      if (*curr == a) return true;
      if (curr->level >= a.level) break;
      curr = avr::spatial::parentOf(*curr);
    }
  } else if (b.level > a.level) {
    auto curr = avr::spatial::parentOf(a);
    while (curr.has_value()) {
      if (*curr == b) return true;
      if (curr->level >= b.level) break;
      curr = avr::spatial::parentOf(*curr);
    }
  }
  return false;
}

void enumerateDescendantLeaves(
    const PersistentAdaptiveMap& map,
    const avr::spatial::CellKey& root,
    std::vector<avr::spatial::CellKey>& leaves) {
  auto children_opt = avr::spatial::childrenOf(root);
  if (!children_opt.has_value()) return;
  for (const auto& child : *children_opt) {
    if (map.hierarchy().isLeaf(child)) {
      leaves.push_back(child);
    } else if (map.hierarchy().isInternal(child)) {
      enumerateDescendantLeaves(map, child, leaves);
    }
  }
}

bool subtreeHasProvenanceGap(
    const PersistentAdaptiveMap& map,
    const avr::spatial::CellKey& root) {
  auto children_opt = avr::spatial::childrenOf(root);
  if (!children_opt.has_value()) return false;
  for (const auto& child : *children_opt) {
    if (map.hierarchy().isLeaf(child)) {
      if (map.requiresReplay(child)) return true;
    } else if (map.hierarchy().isInternal(child)) {
      const auto* coarsening_state = map.findInternalCoarseningState(child);
      if (coarsening_state != nullptr &&
          coarsening_state->base_state == CoarseningBaseState::INCOMPLETE) {
        return true;
      }
      if (subtreeHasProvenanceGap(map, child)) return true;
    }
  }
  return false;
}

}  // namespace

std::unique_ptr<PersistentAdaptiveFramePipeline> PersistentAdaptiveFramePipeline::create(
    const PersistentAdaptivePipelineConfig& config) {
  return create(config, nullptr);
}

std::unique_ptr<PersistentAdaptiveFramePipeline> PersistentAdaptiveFramePipeline::create(
    const PersistentAdaptivePipelineConfig& config,
    std::unique_ptr<SemanticProvider> provider) {
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
          config.pool_block_capacity,
          config.coarsen_group_max_age_generations,
          config.semantic_config,
          std::move(provider)));
}

PersistentAdaptiveFramePipeline::PersistentAdaptiveFramePipeline(
    ResolutionController controller,
    ResolutionHysteresisController hysteresis,
    RecentObservationCache cache,
    size_t pool_block_capacity,
    uint64_t coarsen_group_max_age_generations,
    SemanticPolicyConfig semantic_config,
    std::unique_ptr<SemanticProvider> provider)
    : resolution_controller_(std::move(controller)),
      hysteresis_(std::move(hysteresis)),
      persistent_map_(pool_block_capacity),
      replay_cache_(std::move(cache)),
      coarsen_group_max_age_generations_(coarsen_group_max_age_generations),
      semantic_config_(semantic_config),
      provider_(std::move(provider)) {}

std::optional<ResolutionSignals> PersistentAdaptiveFramePipeline::makeResolutionSignalsForRegion(
    const avr::spatial::CellKey& region) const noexcept {
  return frame_evidence_.makeResolutionSignals(
      region,
      semantic_frame_evidence_.evaluateSemanticRelevance(region),
      semantic_frame_evidence_.evaluateDynamicRelevance(region),
      std::nullopt);
}

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
  semantic_frame_evidence_.reset();
  auto ev_stats_opt = frame_evidence_.buildFrame(raw_points, frame_seq, sensor_origin);
  if (!ev_stats_opt.has_value()) {
    result.status = AdaptivePipelineStatus::FRAME_REJECTED_INPUT;
    return result;
  }
  result.accepted_points = ev_stats_opt->accepted_points;

  // Semantic provider execution and validation
  result.semantic_diagnostics.provider_configured = (provider_ != nullptr);
  std::vector<PointRouting> routing(raw_points.size(), PointRouting::STATIC_FUSABLE);
  DynamicPointSet extracted_dynamic_points{};
  extracted_dynamic_points.frame_seq = frame_seq;

  if (provider_ != nullptr) {
    if (!raw_points.empty()) {
      result.semantic_diagnostics.provider_invoked = true;
      std::optional<SemanticFrameResult> provider_result = std::nullopt;
      try {
        provider_result = provider_->infer(raw_points, frame_seq, sensor_origin);
      } catch (const std::bad_alloc&) {
        result.semantic_diagnostics.provider_status = SemanticProviderStatus::RESOURCE_ERROR;
        result.status = AdaptivePipelineStatus::FRAME_REJECTED_SEMANTIC_RESOURCE;
        return result;
      } catch (const std::exception&) {
        result.semantic_diagnostics.provider_status = SemanticProviderStatus::ERROR;
        result.status = AdaptivePipelineStatus::FRAME_REJECTED_SEMANTIC_ERROR;
        return result;
      } catch (...) {
        result.semantic_diagnostics.provider_status = SemanticProviderStatus::ERROR;
        result.status = AdaptivePipelineStatus::FRAME_REJECTED_SEMANTIC_ERROR;
        return result;
      }

      if (!provider_result.has_value()) {
        result.semantic_diagnostics.provider_degraded = true;
        result.semantic_diagnostics.provider_status = SemanticProviderStatus::UNAVAILABLE;
      } else {
        result.semantic_diagnostics.provider_status = SemanticProviderStatus::OK;
        if (provider_result->frame_seq != frame_seq) {
          result.status = AdaptivePipelineStatus::FRAME_REJECTED_SEMANTIC_MALFORMED;
          return result;
        }

        result.semantic_diagnostics.observations_returned =
            static_cast<uint32_t>(provider_result->observations.size());

        try {
          std::vector<bool> seen_indices(raw_points.size(), false);
          for (const auto& obs : provider_result->observations) {
            if (obs.source_point_index >= raw_points.size()) {
              result.status = AdaptivePipelineStatus::FRAME_REJECTED_SEMANTIC_MALFORMED;
              return result;
            }
            if (seen_indices[obs.source_point_index]) {
              result.status = AdaptivePipelineStatus::FRAME_REJECTED_SEMANTIC_MALFORMED;
              return result;
            }
            seen_indices[obs.source_point_index] = true;

            if (!std::isfinite(obs.semantic_confidence) ||
                obs.semantic_confidence < 0.0f || obs.semantic_confidence > 1.0f) {
              result.status = AdaptivePipelineStatus::FRAME_REJECTED_SEMANTIC_MALFORMED;
              return result;
            }
            if (!std::isfinite(obs.motion_confidence) ||
                obs.motion_confidence < 0.0f || obs.motion_confidence > 1.0f) {
              result.status = AdaptivePipelineStatus::FRAME_REJECTED_SEMANTIC_MALFORMED;
              return result;
            }

            if (!isValidSemanticClass(obs.semantic_class) ||
                !isValidMotionClassification(obs.motion_state)) {
              result.status = AdaptivePipelineStatus::FRAME_REJECTED_SEMANTIC_MALFORMED;
              return result;
            }
          }

          for (const auto& obs : provider_result->observations) {
            const bool is_dynamic =
                (obs.motion_state == MotionClassification::CONFIRMED_DYNAMIC &&
                 obs.motion_confidence >= semantic_config_.min_motion_confidence);

            if (is_dynamic) {
              routing[obs.source_point_index] = PointRouting::DYNAMIC_EXCLUDED;
              ++result.semantic_diagnostics.confirmed_dynamic_points;
              ++result.semantic_diagnostics.dynamic_excluded_points;
            }

            const auto& pt = raw_points[obs.source_point_index];
            if (pt.isFinite()) {
              auto gx = avr::spatial::quantizeToBaseGridSafe(pt.x);
              auto gy = avr::spatial::quantizeToBaseGridSafe(pt.y);
              if (gx.has_value() && gy.has_value()) {
                semantic_frame_evidence_.addObservation(
                    obs.source_point_index,
                    avr::spatial::GridCoord2D{.x = *gx, .y = *gy},
                    obs.semantic_class,
                    obs.semantic_confidence,
                    obs.motion_state,
                    obs.motion_confidence,
                    semantic_config_);
                ++result.semantic_diagnostics.semantic_observations_used;

                if (is_dynamic) {
                  extracted_dynamic_points.points.push_back(DynamicPoint{
                      .source_point_index = obs.source_point_index,
                      .position = pt,
                      .semantic_class = obs.semantic_class,
                      .semantic_confidence = obs.semantic_confidence,
                      .motion_confidence = obs.motion_confidence,
                  });
                }
              }
            }
          }

          std::sort(extracted_dynamic_points.points.begin(),
                    extracted_dynamic_points.points.end(),
                    [](const auto& a, const auto& b) {
                      return a.source_point_index < b.source_point_index;
                    });
        } catch (const std::bad_alloc&) {
          result.status = AdaptivePipelineStatus::FRAME_REJECTED_SEMANTIC_RESOURCE;
          return result;
        }
      }
    }
  }

  if (result.semantic_diagnostics.dynamic_excluded_points > 0) {
    try {
      static_points_scratch_.clear();
      static_points_scratch_.reserve(raw_points.size() - result.semantic_diagnostics.dynamic_excluded_points);
      for (size_t i = 0; i < raw_points.size(); ++i) {
        if (routing[i] == PointRouting::STATIC_FUSABLE) {
          static_points_scratch_.push_back(raw_points[i]);
        }
      }
      auto static_build_opt = static_fusion_evidence_.buildFrame(
          static_points_scratch_, frame_seq, sensor_origin);
      if (!static_build_opt.has_value()) {
        result.status = AdaptivePipelineStatus::FRAME_REJECTED_INPUT;
        return result;
      }
    } catch (const std::bad_alloc&) {
      result.status = AdaptivePipelineStatus::FRAME_REJECTED_SEMANTIC_RESOURCE;
      return result;
    }
  }

  const FrameEvidence& fusion_evidence =
      (result.semantic_diagnostics.dynamic_excluded_points > 0)
          ? static_fusion_evidence_
          : frame_evidence_;

  // 6. Phase 1: Stage raw cache
  std::optional<PreparedCachedObservationFrame> prep_frame_opt;
  if (result.semantic_diagnostics.provider_configured &&
      !result.semantic_diagnostics.provider_degraded) {
    prep_frame_opt = replay_cache_.prepareFrame(raw_points, routing, frame_seq, sensor_origin);
  } else {
    prep_frame_opt = replay_cache_.prepareFrame(raw_points, frame_seq, sensor_origin);
  }
  if (!prep_frame_opt.has_value()) {
    result.status = AdaptivePipelineStatus::FRAME_REJECTED_INPUT;
    return result;
  }
  result.cache_prepared = true;

  // If no accepted observations, frame is valid empty.
  if (ev_stats_opt->active_base_cells == 0) {
    auto fusion_opt = persistent_map_.fuseFrameEvidence(fusion_evidence);
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
        auto cache_stats = replay_cache_.commitPreparedFrame(std::move(*prep_frame_opt));
        if (!cache_stats.has_value()) {
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

    if (!expireStaleReadyEntries(result)) {
      return result;
    }

    if (!validatePolicyMetadataInvariants()) {
      fatal_error_latched_ = true;
      result.status = AdaptivePipelineStatus::COARSENING_INTERNAL_ERROR;
      return result;
    }

    auto map_val = persistent_map_.validateInvariants();
    if (!map_val.is_valid || test_simulate_invariant_failure_) {
      fatal_error_latched_ = true;
      result.status = AdaptivePipelineStatus::INVARIANT_FAILURE;
      return result;
    }

    if (cache_degraded) {
      result.status = AdaptivePipelineStatus::SUCCESS_CACHE_DEGRADED;
    } else {
      result.status = AdaptivePipelineStatus::SUCCESS;
    }
    result.dynamic_points = std::move(extracted_dynamic_points);
    result.semantic_diagnostics.dynamic_points_extracted =
        result.dynamic_points.points.size();
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
    auto signals_opt = makeResolutionSignalsForRegion(leaf);
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
    auto signals_opt = makeResolutionSignalsForRegion(root_key);
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
    fusion_opt = persistent_map_.fuseFrameEvidence(fusion_evidence);
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
      leaf_policy_metadata_.erase(removed_key);
    }
  } catch (const std::bad_alloc&) {
    fatal_error_latched_ = true;
    result.status = AdaptivePipelineStatus::POLICY_COMMIT_FATAL;
    return result;
  }

  // Update LeafPolicyMetadata for surviving pre-existing active leaves
  try {
    if (test_simulate_metadata_bad_alloc_) {
      throw std::bad_alloc();
    }
    for (const auto& prep : surviving_batch) {
      auto& meta = leaf_policy_metadata_[prep.region];
      meta.latest_decision_generation = processing_generation_;
      meta.latest_requested_level = prep.result.requested_level;
      meta.latest_reasons = prep.result.trigger_reasons;
      if (prep.result.action == HysteresisAction::COARSEN_READY ||
          prep.result.action == HysteresisAction::COARSEN_READY_PENDING_ACK) {
        meta.latest_ready_generation = processing_generation_;
      } else {
        meta.latest_ready_generation = 0;
      }
    }
  } catch (const std::bad_alloc&) {
    fatal_error_latched_ = true;
    result.status = AdaptivePipelineStatus::COARSENING_INTERNAL_ERROR;
    return result;
  }

  // Stale readiness expiration (age > W)
  if (!expireStaleReadyEntries(result)) {
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

  if (cache_degraded) {
    if (!validatePolicyMetadataInvariants()) {
      fatal_error_latched_ = true;
      result.status = AdaptivePipelineStatus::COARSENING_INTERNAL_ERROR;
      return result;
    }
    auto map_val = persistent_map_.validateInvariants();
    if (!map_val.is_valid || test_simulate_invariant_failure_) {
      fatal_error_latched_ = true;
      result.status = AdaptivePipelineStatus::INVARIANT_FAILURE;
      return result;
    }
    result.status = AdaptivePipelineStatus::SUCCESS_CACHE_DEGRADED;
    result.dynamic_points = std::move(extracted_dynamic_points);
    result.semantic_diagnostics.dynamic_points_extracted =
        result.dynamic_points.points.size();
    return result;
  }

  // 13. Phase 7: Post-Fusion Automatic Coarsening & Complete-Shadow Recovery
  std::vector<avr::spatial::CellKey> split_regions;
  for (const auto& sa : result.structural_action_records) {
    if (sa.action_type == StructuralActionType::SPLIT) {
      split_regions.push_back(sa.region);
    }
  }

  std::unordered_map<avr::spatial::CellKey, HysteresisAction> current_committed_actions;
  for (const auto& prep : surviving_batch) {
    current_committed_actions[prep.region] = prep.result.action;
  }

  std::vector<CoarsenCandidate> candidates;

  // Direct Merge Seeding
  std::vector<avr::spatial::CellKey> seed_parents;
  for (const auto& prep : surviving_batch) {
    if (prep.result.action == HysteresisAction::COARSEN_READY ||
        prep.result.action == HysteresisAction::COARSEN_READY_PENDING_ACK) {
      auto parent_opt = avr::spatial::parentOf(prep.region);
      if (parent_opt.has_value()) {
        seed_parents.push_back(*parent_opt);
      }
    }
  }
  std::sort(seed_parents.begin(), seed_parents.end());
  seed_parents.erase(std::unique(seed_parents.begin(), seed_parents.end()), seed_parents.end());
  result.coarsen_groups_considered += static_cast<uint32_t>(seed_parents.size());

  for (const auto& P : seed_parents) {
    if (!persistent_map_.hierarchy().isInternal(P)) {
      continue;
    }
    bool split_conflict = false;
    for (const auto& s : split_regions) {
      if (isAncestorOrDescendant(P, s)) {
        split_conflict = true;
        break;
      }
    }
    if (split_conflict) {
      continue;
    }

    auto children_opt = avr::spatial::childrenOf(P);
    if (!children_opt.has_value()) {
      continue;
    }

    bool all_children_eligible = true;
    ResolutionReasonBits merged_reasons{};

    for (const auto& C : *children_opt) {
      if (!persistent_map_.hierarchy().isLeaf(C)) {
        all_children_eligible = false;
        break;
      }
      if (persistent_map_.requiresReplay(C)) {
        all_children_eligible = false;
        break;
      }
      auto meta_it = leaf_policy_metadata_.find(C);
      if (meta_it == leaf_policy_metadata_.end()) {
        all_children_eligible = false;
        break;
      }
      if (meta_it->second.latest_decision_generation != processing_generation_) {
        all_children_eligible = false;
        break;
      }
      if (meta_it->second.latest_ready_generation != processing_generation_) {
        all_children_eligible = false;
        break;
      }
      auto act_it = current_committed_actions.find(C);
      if (act_it == current_committed_actions.end()) {
        all_children_eligible = false;
        break;
      }
      if (act_it->second != HysteresisAction::COARSEN_READY &&
          act_it->second != HysteresisAction::COARSEN_READY_PENDING_ACK) {
        all_children_eligible = false;
        break;
      }
      const auto* h_state = hysteresis_.findRegionState(C);
      if (h_state == nullptr || !h_state->coarsen_action_pending ||
          h_state->pending_target_level != P.level) {
        all_children_eligible = false;
        break;
      }
      if (meta_it->second.latest_requested_level < P.level) {
        all_children_eligible = false;
        break;
      }
      for (const auto& s : split_regions) {
        if (isAncestorOrDescendant(C, s)) {
          all_children_eligible = false;
          break;
        }
      }
      if (!all_children_eligible) {
        break;
      }

      merged_reasons = merged_reasons | meta_it->second.latest_reasons;
    }

    if (all_children_eligible) {
      candidates.push_back(CoarsenCandidate{
          .type = CoarsenCandidateType::DIRECT_MERGE,
          .root = P,
          .level = P.level,
          .reasons = merged_reasons,
      });
    }
  }

  // Recovery Seeding
  std::vector<avr::spatial::CellKey> seed_ancestors;
  for (const auto& prep : surviving_batch) {
    if (prep.result.action != HysteresisAction::COARSEN_READY &&
        prep.result.action != HysteresisAction::COARSEN_READY_PENDING_ACK) {
      continue;
    }
    bool is_gap_leaf = persistent_map_.requiresReplay(prep.region);
    if (!is_gap_leaf) {
      auto curr = avr::spatial::parentOf(prep.region);
      while (curr.has_value()) {
        const auto* cs = persistent_map_.findInternalCoarseningState(*curr);
        if (cs != nullptr && cs->base_state == CoarseningBaseState::INCOMPLETE) {
          is_gap_leaf = true;
          break;
        }
        curr = avr::spatial::parentOf(*curr);
      }
    }

    if (is_gap_leaf) {
      auto curr = avr::spatial::parentOf(prep.region);
      while (curr.has_value()) {
        const auto* cs = persistent_map_.findInternalCoarseningState(*curr);
        if (cs != nullptr &&
            (cs->base_state == CoarseningBaseState::COMPLETE_EMPTY ||
             cs->base_state == CoarseningBaseState::COMPLETE_RETIRED_CACHE)) {
          seed_ancestors.push_back(*curr);
          break;
        }
        curr = avr::spatial::parentOf(*curr);
      }
    }
  }

  std::sort(seed_ancestors.begin(), seed_ancestors.end());
  seed_ancestors.erase(std::unique(seed_ancestors.begin(), seed_ancestors.end()), seed_ancestors.end());
  result.recovery_candidates += static_cast<uint32_t>(seed_ancestors.size());

  for (const auto& A : seed_ancestors) {
    if (!persistent_map_.hierarchy().isInternal(A)) {
      ++result.recovery_blocked;
      continue;
    }
    if (!subtreeHasProvenanceGap(persistent_map_, A)) {
      ++result.recovery_blocked;
      continue;
    }
    bool split_conflict = false;
    for (const auto& s : split_regions) {
      if (isAncestorOrDescendant(A, s)) {
        split_conflict = true;
        break;
      }
    }
    if (split_conflict) {
      ++result.recovery_blocked;
      continue;
    }

    std::vector<avr::spatial::CellKey> frontier;
    enumerateDescendantLeaves(persistent_map_, A, frontier);
    if (frontier.empty()) {
      ++result.recovery_blocked;
      continue;
    }

    bool frontier_eligible = true;
    ResolutionReasonBits recovery_reasons{};

    for (const auto& leaf : frontier) {
      if (!persistent_map_.hierarchy().isLeaf(leaf)) {
        frontier_eligible = false;
        break;
      }
      auto meta_it = leaf_policy_metadata_.find(leaf);
      if (meta_it == leaf_policy_metadata_.end()) {
        frontier_eligible = false;
        break;
      }
      if (meta_it->second.latest_decision_generation != processing_generation_) {
        frontier_eligible = false;
        break;
      }
      if (meta_it->second.latest_ready_generation != processing_generation_) {
        frontier_eligible = false;
        break;
      }
      auto act_it = current_committed_actions.find(leaf);
      if (act_it == current_committed_actions.end()) {
        frontier_eligible = false;
        break;
      }
      if (act_it->second != HysteresisAction::COARSEN_READY &&
          act_it->second != HysteresisAction::COARSEN_READY_PENDING_ACK) {
        frontier_eligible = false;
        break;
      }
      const auto* h_state = hysteresis_.findRegionState(leaf);
      if (h_state == nullptr || !h_state->coarsen_action_pending) {
        frontier_eligible = false;
        break;
      }
      if (meta_it->second.latest_requested_level < A.level) {
        frontier_eligible = false;
        break;
      }
      if (act_it->second == HysteresisAction::HOLD_CURRENT ||
          act_it->second == HysteresisAction::REFINE_NOW) {
        frontier_eligible = false;
        break;
      }
      for (const auto& s : split_regions) {
        if (isAncestorOrDescendant(leaf, s)) {
          frontier_eligible = false;
          break;
        }
      }
      if (!frontier_eligible) {
        break;
      }

      recovery_reasons = recovery_reasons | meta_it->second.latest_reasons;
    }

    if (frontier_eligible) {
      candidates.push_back(CoarsenCandidate{
          .type = CoarsenCandidateType::RECOVERY_COLLAPSE,
          .root = A,
          .level = A.level,
          .reasons = recovery_reasons,
      });
    } else {
      ++result.recovery_blocked;
    }
  }

  // Conflict & Overlap Resolution
  std::sort(candidates.begin(), candidates.end(), [](const CoarsenCandidate& a, const CoarsenCandidate& b) {
    if (a.level != b.level) {
      return a.level < b.level;
    }
    if (a.type != b.type) {
      return static_cast<uint8_t>(a.type) < static_cast<uint8_t>(b.type);
    }
    return a.root < b.root;
  });

  std::vector<CoarsenCandidate> accepted_operations;
  for (const auto& cand : candidates) {
    bool conflicts = false;
    for (const auto& accepted : accepted_operations) {
      if (isAncestorOrDescendant(cand.root, accepted.root)) {
        conflicts = true;
        break;
      }
    }
    if (!conflicts) {
      accepted_operations.push_back(cand);
    } else {
      if (cand.type == CoarsenCandidateType::RECOVERY_COLLAPSE) {
        ++result.recovery_blocked;
      }
    }
  }

  // Execute Accepted Operations
  for (const auto& cand : accepted_operations) {
    if (cand.type == CoarsenCandidateType::DIRECT_MERGE) {
      ++result.direct_merge_attempts;

      auto children_opt = avr::spatial::childrenOf(cand.root);
      if (!children_opt.has_value()) {
        fatal_error_latched_ = true;
        result.status = AdaptivePipelineStatus::COARSENING_INTERNAL_ERROR;
        return result;
      }
      std::vector<avr::spatial::CellKey> children(children_opt->begin(), children_opt->end());
      std::sort(children.begin(), children.end());

      for (const auto& C : children) {
        const auto* h_state = hysteresis_.findRegionState(C);
        if (h_state == nullptr || !h_state->coarsen_action_pending) {
          fatal_error_latched_ = true;
          result.status = AdaptivePipelineStatus::COARSENING_INTERNAL_ERROR;
          return result;
        }
      }

      auto merge_res = persistent_map_.mergeDirectChildren(cand.root, cand.reasons);
      if (test_simulate_coarsen_internal_error_) {
        merge_res.status = MergeStatus::INTERNAL_ERROR;
      }

      if (merge_res.status == MergeStatus::APPLIED) {
        for (const auto& C : children) {
          bool ack_ok = hysteresis_.acknowledgeCoarsenResult(C, CoarsenAttemptOutcome::APPLIED);
          if (test_simulate_ack_failure_) {
            ack_ok = false;
          }
          if (!ack_ok) {
            fatal_error_latched_ = true;
            result.status = AdaptivePipelineStatus::COARSENING_INTERNAL_ERROR;
            return result;
          }
          leaf_policy_metadata_.erase(C);
        }
        hysteresis_.eraseRegion(cand.root);
        leaf_policy_metadata_.erase(cand.root);

        result.structural_action_records.push_back(StructuralActionRecord{
            .region = cand.root,
            .from_level = static_cast<uint8_t>(cand.root.level - 1),
            .to_level = cand.root.level,
            .reasons = cand.reasons,
            .action_type = StructuralActionType::DIRECT_MERGE,
        });
        result.structure_changed = true;
        ++result.direct_merges_applied;
      } else if (merge_res.status == MergeStatus::CHILD_NOT_LEAF ||
                 merge_res.status == MergeStatus::REPLAY_INCOMPLETE ||
                 merge_res.status == MergeStatus::CACHE_HISTORY_INSUFFICIENT) {
        for (const auto& C : children) {
          bool ack_ok = hysteresis_.acknowledgeCoarsenResult(C, CoarsenAttemptOutcome::BLOCKED_HOLD_FINE);
          if (test_simulate_ack_failure_) {
            ack_ok = false;
          }
          if (!ack_ok) {
            fatal_error_latched_ = true;
            result.status = AdaptivePipelineStatus::COARSENING_INTERNAL_ERROR;
            return result;
          }
          auto it = leaf_policy_metadata_.find(C);
          if (it != leaf_policy_metadata_.end()) {
            it->second.latest_ready_generation = 0;
          }
        }
        ++result.direct_merges_blocked;
      } else {
        fatal_error_latched_ = true;
        result.status = AdaptivePipelineStatus::COARSENING_INTERNAL_ERROR;
        return result;
      }
    } else if (cand.type == CoarsenCandidateType::RECOVERY_COLLAPSE) {
      ++result.recovery_attempts;

      auto collapse_res = persistent_map_.collapseSubtreeFromCompleteShadow(cand.root, cand.reasons);
      if (test_simulate_coarsen_internal_error_) {
        collapse_res.status = SubtreeCollapseStatus::INTERNAL_ERROR;
      }

      if (collapse_res.status == SubtreeCollapseStatus::APPLIED_GAP_CLOSED) {
        for (const auto& key : collapse_res.removed_descendant_keys) {
          hysteresis_.eraseRegion(key);
          leaf_policy_metadata_.erase(key);
        }
        hysteresis_.eraseRegion(cand.root);
        leaf_policy_metadata_.erase(cand.root);

        result.structural_action_records.push_back(StructuralActionRecord{
            .region = cand.root,
            .from_level = collapse_res.finest_removed_level,
            .to_level = cand.root.level,
            .reasons = cand.reasons,
            .action_type = StructuralActionType::SUBTREE_RECOVERY_COLLAPSE,
        });
        result.structure_changed = true;
        ++result.recovery_applied;
      } else {
        fatal_error_latched_ = true;
        result.status = AdaptivePipelineStatus::COARSENING_INTERNAL_ERROR;
        return result;
      }
    }
  }

  // 14. Validation and Final Result Status Assignment
  if (!validatePolicyMetadataInvariants()) {
    fatal_error_latched_ = true;
    result.status = AdaptivePipelineStatus::COARSENING_INTERNAL_ERROR;
    return result;
  }

  auto map_val = persistent_map_.validateInvariants();
  if (!map_val.is_valid || test_simulate_invariant_failure_) {
    fatal_error_latched_ = true;
    result.status = AdaptivePipelineStatus::INVARIANT_FAILURE;
    return result;
  }

  if (result.replay_pending) {
    result.status = AdaptivePipelineStatus::SUCCESS_REPLAY_PENDING;
  } else {
    result.status = AdaptivePipelineStatus::SUCCESS;
  }
  result.dynamic_points = std::move(extracted_dynamic_points);
  result.semantic_diagnostics.dynamic_points_extracted =
      result.dynamic_points.points.size();

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
    auto signals_opt = makeResolutionSignalsForRegion(child);
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

std::vector<std::pair<avr::spatial::CellKey, LeafPolicyMetadata>>
PersistentAdaptiveFramePipeline::leafPolicyMetadataSnapshot() const {
  std::vector<std::pair<avr::spatial::CellKey, LeafPolicyMetadata>> snapshot;
  snapshot.reserve(leaf_policy_metadata_.size());
  for (const auto& entry : leaf_policy_metadata_) {
    snapshot.push_back(entry);
  }
  std::sort(snapshot.begin(), snapshot.end(), [](const auto& a, const auto& b) {
    return a.first < b.first;
  });
  return snapshot;
}

bool PersistentAdaptiveFramePipeline::expireStaleReadyEntries(AdaptivePipelineFrameResult& result) {
  std::vector<avr::spatial::CellKey> ready_keys;
  for (const auto& [key, meta] : leaf_policy_metadata_) {
    if (meta.latest_decision_generation > processing_generation_ ||
        meta.latest_ready_generation > processing_generation_) {
      fatal_error_latched_ = true;
      result.status = AdaptivePipelineStatus::COARSENING_INTERNAL_ERROR;
      return false;
    }
    if (meta.latest_ready_generation > 0) {
      ready_keys.push_back(key);
    }
  }
  std::sort(ready_keys.begin(), ready_keys.end());

  for (const auto& key : ready_keys) {
    auto it = leaf_policy_metadata_.find(key);
    if (it == leaf_policy_metadata_.end()) {
      continue;
    }
    const uint64_t age = processing_generation_ - it->second.latest_ready_generation;
    if (age > coarsen_group_max_age_generations_) {
      const auto* reg_state = hysteresis_.findRegionState(key);
      if (reg_state == nullptr || !reg_state->coarsen_action_pending) {
        fatal_error_latched_ = true;
        result.status = AdaptivePipelineStatus::COARSENING_INTERNAL_ERROR;
        return false;
      }
      bool ack_ok = hysteresis_.acknowledgeCoarsenResult(key, CoarsenAttemptOutcome::CANCELLED);
      if (test_simulate_ack_failure_) {
        ack_ok = false;
      }
      if (!ack_ok) {
        fatal_error_latched_ = true;
        result.status = AdaptivePipelineStatus::COARSENING_INTERNAL_ERROR;
        return false;
      }
      it->second.latest_ready_generation = 0;
      ++result.stale_ready_cancelled;
    }
  }
  return true;
}

bool PersistentAdaptiveFramePipeline::validatePolicyMetadataInvariants() const noexcept {
  for (const auto& [key, meta] : leaf_policy_metadata_) {
    if (!persistent_map_.hierarchy().isLeaf(key)) {
      return false;
    }
    if (meta.latest_decision_generation > processing_generation_) {
      return false;
    }
    if (meta.latest_ready_generation > processing_generation_) {
      return false;
    }
    if (!avr::spatial::isValidLevel(meta.latest_requested_level)) {
      return false;
    }
    if (meta.latest_ready_generation > 0) {
      const auto* h_state = hysteresis_.findRegionState(key);
      if (h_state == nullptr || !h_state->coarsen_action_pending) {
        return false;
      }
    }
  }
  return true;
}

}  // namespace avr::mapping

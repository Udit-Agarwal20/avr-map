#include "avr_mapping/adaptive_frame_processor.hpp"

#include <algorithm>
#include <queue>
#include <vector>

namespace avr::mapping {

std::unique_ptr<AdaptiveFrameProcessor> AdaptiveFrameProcessor::create(
    const ResolutionPolicyConfig& config, size_t pool_block_capacity) {
  auto controller_opt = ResolutionController::create(config);
  if (!controller_opt.has_value()) {
    return nullptr;
  }
  return std::unique_ptr<AdaptiveFrameProcessor>(
      new AdaptiveFrameProcessor(std::move(*controller_opt), pool_block_capacity));
}

AdaptiveFrameProcessor::AdaptiveFrameProcessor(
    ResolutionController controller, size_t pool_block_capacity)
    : controller_(std::move(controller)), hierarchy_(pool_block_capacity) {}

std::optional<AdaptiveFrameStats> AdaptiveFrameProcessor::processFrame(
    std::span<const PointXYZ> points,
    uint64_t frame_seq,
    avr::spatial::WorldPoint2D sensor_origin) {
  // 1. Transactional ingestion in FrameEvidence.
  // If sensor_origin is invalid, buildFrame returns std::nullopt and preserves old state.
  auto ev_stats_opt = frame_evidence_.buildFrame(points, frame_seq, sensor_origin);
  if (!ev_stats_opt.has_value()) {
    return std::nullopt;
  }

  // 2. Global frame accepted: clear previous logical adaptive topology and diagnostics.
  hierarchy_.reset();
  decision_records_.clear();

  AdaptiveFrameStats stats{};
  stats.frame_seq = ev_stats_opt->frame_seq;
  stats.input_points = ev_stats_opt->input_points;
  stats.accepted_points = ev_stats_opt->accepted_points;
  stats.rejected_non_finite = ev_stats_opt->rejected_non_finite;
  stats.rejected_out_of_range = ev_stats_opt->rejected_out_of_range;
  stats.rejected_numeric = ev_stats_opt->rejected_numeric;
  stats.evidence_tile_count = ev_stats_opt->tile_count;
  stats.active_base_cells = ev_stats_opt->active_base_cells;

  // If no accepted observations, frame is valid empty. Topology remains empty.
  if (ev_stats_opt->active_base_cells == 0) {
    last_stats_ = stats;
    return stats;
  }

  // 3. Seed root discovery exclusively from active evidence snapshot.
  const auto base_snapshot = frame_evidence_.baseCellSnapshot();
  std::vector<avr::spatial::CellKey> seed_roots;
  seed_roots.reserve(base_snapshot.size());

  for (const auto& entry : base_snapshot) {
    auto p1 = avr::spatial::parentOf(entry.key);
    auto p2 = p1 ? avr::spatial::parentOf(*p1) : std::nullopt;
    auto p3 = p2 ? avr::spatial::parentOf(*p2) : std::nullopt;
    if (p3.has_value()) {
      seed_roots.push_back(*p3);
    }
  }

  // Deterministic root ordering: sort and unique by CellKey (Morton code order)
  std::sort(seed_roots.begin(), seed_roots.end());
  seed_roots.erase(std::unique(seed_roots.begin(), seed_roots.end()), seed_roots.end());
  stats.seed_root_count = static_cast<uint32_t>(seed_roots.size());

  // 4. Ensure each evidence-backed L3 root exists as an L3 LEAF in AdaptiveHierarchy.
  for (const auto& root_key : seed_roots) {
    const auto coord = root_key.decode();
    const avr::spatial::GridCoord2D base_coord{
        .x = coord.x * static_cast<int32_t>(avr::spatial::levelScale(avr::spatial::L3)),
        .y = coord.y * static_cast<int32_t>(avr::spatial::levelScale(avr::spatial::L3)),
    };
    hierarchy_.ensureRootForBase(base_coord);
  }

  // 5. Region Evaluation & Refinement Queue
  struct EvaluationTask {
    avr::spatial::CellKey region;
    uint8_t inherited_target{avr::spatial::L3};
    ResolutionReasonBits inherited_reasons{};
  };

  std::queue<EvaluationTask> task_queue;
  for (const auto& root_key : seed_roots) {
    task_queue.push(EvaluationTask{
        .region = root_key,
        .inherited_target = avr::spatial::L3,
        .inherited_reasons = ResolutionReasonBits{},
    });
  }

  while (!task_queue.empty()) {
    const auto task = task_queue.front();
    task_queue.pop();

    const auto summary = frame_evidence_.summarize(task.region);
    // Empty-Region Rule: Regions with 0 observations must not be evaluated/refined.
    if (summary.observation_count == 0) {
      continue;
    }

    ++stats.regions_evaluated;

    // Build signals with fixed geometry-only upstream defaults
    auto signals_opt = frame_evidence_.makeResolutionSignals(
        task.region,
        SemanticRelevance::NONE,
        DynamicRelevance::NONE,
        std::nullopt);
    if (!signals_opt.has_value()) {
      hierarchy_.reset();
      decision_records_.clear();
      last_stats_ = std::nullopt;
      return std::nullopt;
    }

    auto decision_opt = controller_.decide(*signals_opt);
    if (!decision_opt.has_value()) {
      hierarchy_.reset();
      decision_records_.clear();
      last_stats_ = std::nullopt;
      return std::nullopt;
    }

    const auto& local_decision = *decision_opt;

    // Reason counts for diagnostic statistics
    if (local_decision.reasons.has(ResolutionReason::INSUFFICIENT_EVIDENCE)) {
      ++stats.regions_with_insufficient_evidence;
    }
    if (local_decision.reasons.has(ResolutionReason::GEOMETRY_COMPLEXITY)) {
      ++stats.regions_with_geometry_reason;
    }
    if (local_decision.reasons.mask == static_cast<uint16_t>(ResolutionReason::DISTANCE_BASELINE)) {
      ++stats.regions_with_distance_only;
    }

    // Target and reason propagation:
    // effective_target = min(local_target, inherited_target) (finer level since L0 < L1 < L2 < L3)
    const uint8_t local_target = local_decision.requested_level;
    const uint8_t effective_target = std::min(local_target, task.inherited_target);

    const ResolutionReasonBits local_reasons = local_decision.reasons;
    const ResolutionReasonBits inherited_reasons = task.inherited_reasons;
    const ResolutionReasonBits effective_reasons = local_reasons | inherited_reasons;

    bool split_applied = false;
    if (task.region.level > effective_target && task.region.level > avr::spatial::L0) {
      if (hierarchy_.splitLeaf(task.region)) {
        split_applied = true;
        ++stats.regions_split;

        auto children_opt = avr::spatial::childrenOf(task.region);
        if (children_opt.has_value()) {
          for (const auto& child_key : *children_opt) {
            const auto child_summary = frame_evidence_.summarize(child_key);
            if (child_summary.observation_count > 0) {
              task_queue.push(EvaluationTask{
                  .region = child_key,
                  .inherited_target = effective_target,
                  .inherited_reasons = effective_reasons,
              });
            } else {
              // Structural coverage sibling with zero evidence: leave as LEAF
              ++stats.empty_children_held;
            }
          }
        }
      } else {
        // Internal structural split failure: fail-closed
        hierarchy_.reset();
        decision_records_.clear();
        last_stats_ = std::nullopt;
        return std::nullopt;
      }
    }

    decision_records_.push_back(RegionDecisionRecord{
        .region = task.region,
        .local_decision = local_decision,
        .inherited_target = task.inherited_target,
        .effective_target = effective_target,
        .local_reasons = local_reasons,
        .inherited_reasons = inherited_reasons,
        .effective_reasons = effective_reasons,
        .observation_count = summary.observation_count,
        .occupied_base_cells = summary.occupied_base_cells,
        .density_score = summary.density_score,
        .geometry_complexity = summary.geometry_complexity,
        .split_applied = split_applied,
    });
  }

  // 6. Validate topology invariants before publishing success
  const auto validation = hierarchy_.validateInvariants();
  if (!validation.valid) {
    hierarchy_.reset();
    decision_records_.clear();
    last_stats_ = std::nullopt;
    return std::nullopt;
  }

  // 7. Finalize hierarchy statistics and publish lastStats()
  const auto h_stats = hierarchy_.statistics();
  stats.final_hierarchy_tile_count = h_stats.tile_count;
  stats.final_node_count = h_stats.node_count;
  stats.final_leaf_count = h_stats.leaf_count;
  stats.final_internal_count = h_stats.internal_count;
  stats.leaf_count_by_level = h_stats.leaf_count_by_level;

  last_stats_ = stats;
  return stats;
}

std::vector<RegionDecisionRecord> AdaptiveFrameProcessor::decisionSnapshot() const {
  auto snapshot = decision_records_;
  std::sort(snapshot.begin(), snapshot.end());
  return snapshot;
}

}  // namespace avr::mapping

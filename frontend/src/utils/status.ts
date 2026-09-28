import {
  AdaptivePipelineStatus,
  AdaptivePipelineStatusName,
} from '../types/perception';

export type PipelineStatusSeverity = 'healthy' | 'degraded' | 'rejected' | 'error' | 'terminal';

/**
 * @brief Map status enum code or name to human-facing display label.
 */
export function pipelineStatusLabel(
  status: AdaptivePipelineStatus | AdaptivePipelineStatusName
): string {
  const name: AdaptivePipelineStatusName =
    typeof status === 'number'
      ? (AdaptivePipelineStatus[status] as AdaptivePipelineStatusName)
      : status;

  switch (name) {
    case 'SUCCESS':
      return 'Success (Nominal)';
    case 'SUCCESS_REPLAY_PENDING':
      return 'Replay Pending';
    case 'SUCCESS_CACHE_DEGRADED':
      return 'Cache Degraded';
    case 'FRAME_REJECTED_INPUT':
      return 'Rejected: Input Invalidation';
    case 'FRAME_REJECTED_SEQUENCE':
      return 'Rejected: Sequence Stale';
    case 'FRAME_REJECTED_DECISION':
      return 'Rejected: Controller Decision';
    case 'FRAME_REJECTED_POLICY':
      return 'Rejected: Policy Hysteresis';
    case 'REFINEMENT_INTERNAL_ERROR':
      return 'Internal Error: Refinement';
    case 'REPLAY_INTERNAL_ERROR':
      return 'Internal Error: Historical Replay';
    case 'FUSION_FAILED':
      return 'Fusion Failed';
    case 'FUSION_FAILED_AFTER_STRUCTURAL_CHANGE':
      return 'Fusion Failed Post-Split';
    case 'POLICY_COMMIT_INTERNAL_ERROR':
      return 'Policy Commit Error';
    case 'POLICY_COMMIT_FATAL':
      return 'Fatal: Policy Allocation Failure';
    case 'CACHE_COMMIT_INTERNAL_ERROR':
      return 'Cache Commit Error';
    case 'GENERATION_EXHAUSTED':
      return 'Terminal: Generation Exhausted';
    case 'POLICY_REVISION_EXHAUSTED':
      return 'Terminal: Policy Revision Exhausted';
    case 'INVARIANT_FAILURE':
      return 'Fatal: Invariant Failure';
    case 'PIPELINE_FATAL_LATCHED':
      return 'Fatal: Pipeline Latched';
    case 'COARSENING_INTERNAL_ERROR':
      return 'Internal Error: Coarsening';
    case 'FRAME_REJECTED_SEMANTIC_MALFORMED':
      return 'Rejected: Semantic Frame Malformed';
    case 'FRAME_REJECTED_SEMANTIC_ERROR':
      return 'Rejected: Semantic Runtime Exception';
    case 'FRAME_REJECTED_SEMANTIC_RESOURCE':
      return 'Rejected: Semantic Resource Exhaustion';
    default:
      return String(status);
  }
}

/**
 * @brief Determine deterministic severity classification for status based on frozen C++ control flow.
 */
export function pipelineStatusSeverity(
  status: AdaptivePipelineStatus | AdaptivePipelineStatusName
): PipelineStatusSeverity {
  const name: AdaptivePipelineStatusName =
    typeof status === 'number'
      ? (AdaptivePipelineStatus[status] as AdaptivePipelineStatusName)
      : status;

  switch (name) {
    case 'SUCCESS':
      return 'healthy';

    case 'SUCCESS_REPLAY_PENDING':
    case 'SUCCESS_CACHE_DEGRADED':
      return 'degraded';

    case 'FRAME_REJECTED_INPUT':
    case 'FRAME_REJECTED_SEQUENCE':
    case 'FRAME_REJECTED_DECISION':
    case 'FRAME_REJECTED_POLICY':
    case 'FRAME_REJECTED_SEMANTIC_MALFORMED':
    case 'FRAME_REJECTED_SEMANTIC_ERROR':
    case 'FRAME_REJECTED_SEMANTIC_RESOURCE':
      return 'rejected';

    case 'REFINEMENT_INTERNAL_ERROR':
    case 'REPLAY_INTERNAL_ERROR':
    case 'FUSION_FAILED':
    case 'FUSION_FAILED_AFTER_STRUCTURAL_CHANGE':
    case 'CACHE_COMMIT_INTERNAL_ERROR':
    case 'COARSENING_INTERNAL_ERROR':
      return 'error';

    case 'POLICY_COMMIT_INTERNAL_ERROR':
    case 'POLICY_COMMIT_FATAL':
    case 'GENERATION_EXHAUSTED':
    case 'POLICY_REVISION_EXHAUSTED':
    case 'INVARIANT_FAILURE':
    case 'PIPELINE_FATAL_LATCHED':
    default:
      return 'terminal';
  }
}

/**
 * @brief Convert numeric status code from binary/websocket transport to canonical name.
 */
export function statusCodeToName(code: number): AdaptivePipelineStatusName {
  const name = AdaptivePipelineStatus[code] as AdaptivePipelineStatusName | undefined;
  if (!name || typeof code !== 'number' || code < 0 || code > 21) {
    throw new Error(`Unknown AdaptivePipelineStatus code: ${code}`);
  }
  return name;
}

/**
 * @brief Convert status name to frozen backend numeric code.
 */
export function statusNameToCode(name: AdaptivePipelineStatusName): number {
  const code = AdaptivePipelineStatus[name];
  if (typeof code !== 'number') {
    throw new Error(`Unknown AdaptivePipelineStatus name: ${name}`);
  }
  return code;
}

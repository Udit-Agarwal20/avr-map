#pragma once

#include <optional>

#include "avr_mapping/frame_evidence.hpp"
#include "avr_mapping/resolution_decision.hpp"
#include "avr_mapping/resolution_policy.hpp"
#include "avr_spatial/cell_key.hpp"

namespace avr::mapping {

/**
 * @brief Standalone adapter constructing ResolutionSignals from FrameEvidence for a target region.
 *
 * Derives:
 * - distance_m: metric distance from sensor origin to region center (canonical odom frame).
 * - observation_count: saturated from uint64_t to uint32_t (clamped to UINT32_MAX without wrapping).
 * - density_score: frame-local spatial coverage proxy in [0.0, 1.0].
 * - geometry_complexity: vertical relief proxy in [0.0, 1.0] if support is met, or nullopt.
 *
 * Passes through external upstream cues:
 * - semantic_relevance (defaults to NONE)
 * - dynamic_relevance (defaults to NONE)
 * - uncertainty (defaults to nullopt)
 *
 * Validates:
 * - Frame must be active in FrameEvidence.
 * - Region level must be valid (L0 - L3).
 * - Distance arithmetic must be finite and >= 0.0.
 * - External cues must be within valid enum / range definitions.
 * - Resulting signals must satisfy signals.isValid().
 *
 * @return Populated ResolutionSignals, or std::nullopt on any validation or numeric failure.
 */
[[nodiscard]] std::optional<ResolutionSignals> makeResolutionSignals(
    const FrameEvidence& evidence,
    avr::spatial::CellKey region,
    SemanticRelevance semantic = SemanticRelevance::NONE,
    DynamicRelevance dynamic = DynamicRelevance::NONE,
    std::optional<double> uncertainty = std::nullopt) noexcept;

}  // namespace avr::mapping

namespace AVR {
namespace mapping = ::avr::mapping;
namespace mapping_core = ::avr::mapping;
}

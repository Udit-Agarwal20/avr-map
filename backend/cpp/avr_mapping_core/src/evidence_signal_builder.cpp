#include "avr_mapping/evidence_signal_builder.hpp"

#include <cmath>
#include <limits>

#include "avr_spatial/bounds.hpp"
#include "avr_spatial/constants.hpp"

namespace avr::mapping {

std::optional<ResolutionSignals> makeResolutionSignals(
    const FrameEvidence& evidence,
    avr::spatial::CellKey region,
    SemanticRelevance semantic,
    DynamicRelevance dynamic,
    std::optional<double> uncertainty) noexcept {
  // 1. Evidence frame must be active
  if (!evidence.hasFrame()) {
    return std::nullopt;
  }

  // 2. Target region level must be valid [L0, L3]
  if (!avr::spatial::isValidLevel(region.level)) {
    return std::nullopt;
  }

  // 3. Validate upstream external cues
  if (static_cast<uint8_t>(semantic) > static_cast<uint8_t>(SemanticRelevance::SAFETY_CRITICAL)) {
    return std::nullopt;
  }
  if (static_cast<uint8_t>(dynamic) > static_cast<uint8_t>(DynamicRelevance::CONFIRMED)) {
    return std::nullopt;
  }
  if (uncertainty.has_value()) {
    if (!std::isfinite(*uncertainty) || *uncertainty < 0.0 || *uncertainty > 1.0) {
      return std::nullopt;
    }
  }

  // 4. Calculate continuous metric distance from sensor origin to region center
  const auto bounds = region.bounds();  // Canonical GridOrigin2D{0, 0}
  const double center_x = (bounds.min_x + bounds.max_x) * 0.5;
  const double center_y = (bounds.min_y + bounds.max_y) * 0.5;
  if (!std::isfinite(center_x) || !std::isfinite(center_y)) {
    return std::nullopt;
  }

  const auto sensor_origin = evidence.sensorOrigin();
  const double dx = center_x - sensor_origin.x;
  const double dy = center_y - sensor_origin.y;
  if (!std::isfinite(dx) || !std::isfinite(dy)) {
    return std::nullopt;
  }

  const double distance_m = std::hypot(dx, dy);
  if (!std::isfinite(distance_m) || distance_m < 0.0) {
    return std::nullopt;
  }

  // 5. Aggregate evidence over region
  const EvidenceSummary summary = evidence.summarize(region);

  // 6. Convert observation_count with explicit saturation to UINT32_MAX (never wraps)
  const uint32_t obs_count_32 =
      (summary.observation_count > static_cast<uint64_t>(std::numeric_limits<uint32_t>::max()))
          ? std::numeric_limits<uint32_t>::max()
          : static_cast<uint32_t>(summary.observation_count);

  ResolutionSignals signals{
      .distance_m = distance_m,
      .semantic_relevance = semantic,
      .dynamic_relevance = dynamic,
      .observation_count = obs_count_32,
      .density_score = summary.density_score,
      .uncertainty = uncertainty,
      .geometry_complexity = summary.geometry_complexity,
  };

  if (!signals.isValid()) {
    return std::nullopt;
  }

  return signals;
}

}  // namespace avr::mapping

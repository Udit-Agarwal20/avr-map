#pragma once

#include "avr_mapping/replay_reconstruction_result.hpp"

namespace avr::mapping {

/**
 * @brief Diagnostic statistics alias for replay reconstruction results.
 */
using ReplayReconstructionStats = ReplayReconstructionResult;

}  // namespace avr::mapping

namespace AVR {
namespace mapping = ::avr::mapping;
}

#include "avr_spatial/morton.hpp"

namespace avr::spatial {

// Explicit instantiations / non-inline symbols to ensure robust linkage across translation units
uint64_t mortonEncodeExport(int32_t x, int32_t y) noexcept {
  return morton2DEncode(x, y);
}

void mortonDecodeExport(uint64_t code, int32_t& x, int32_t& y) noexcept {
  morton2DDecode(code, x, y);
}

}  // namespace avr::spatial

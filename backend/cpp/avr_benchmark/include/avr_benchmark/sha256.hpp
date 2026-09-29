#pragma once
#include <array>
#include <cstdint>
#include <string>
#include <string_view>

namespace avr::benchmark {
// Incremental SHA-256; hashes canonical bytes, never native object padding.
class Sha256 {
 public:
  void update(std::string_view bytes);
  [[nodiscard]] std::string hex() const;
 private:
  void compress();
  std::array<uint32_t, 8> state_{0x6a09e667,0xbb67ae85,0x3c6ef372,0xa54ff53a,
                                0x510e527f,0x9b05688c,0x1f83d9ab,0x5be0cd19};
  std::array<uint8_t, 64> block_{};
  uint64_t bytes_{0};
  size_t used_{0};
};
[[nodiscard]] std::string sha256(std::string_view bytes);
// Unambiguous decimal-length-prefixed UTF-8/ASCII field framing.
void hashField(Sha256& hash, std::string_view bytes);
}

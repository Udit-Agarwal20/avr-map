#include "avr_benchmark/sha256.hpp"
#include <bit>
#include <iomanip>
#include <sstream>

namespace avr::benchmark {
namespace {
constexpr std::array<uint32_t, 64> k{
  0x428a2f98,0x71374491,0xb5c0fbcf,0xe9b5dba5,0x3956c25b,0x59f111f1,0x923f82a4,0xab1c5ed5,
  0xd807aa98,0x12835b01,0x243185be,0x550c7dc3,0x72be5d74,0x80deb1fe,0x9bdc06a7,0xc19bf174,
  0xe49b69c1,0xefbe4786,0x0fc19dc6,0x240ca1cc,0x2de92c6f,0x4a7484aa,0x5cb0a9dc,0x76f988da,
  0x983e5152,0xa831c66d,0xb00327c8,0xbf597fc7,0xc6e00bf3,0xd5a79147,0x06ca6351,0x14292967,
  0x27b70a85,0x2e1b2138,0x4d2c6dfc,0x53380d13,0x650a7354,0x766a0abb,0x81c2c92e,0x92722c85,
  0xa2bfe8a1,0xa81a664b,0xc24b8b70,0xc76c51a3,0xd192e819,0xd6990624,0xf40e3585,0x106aa070,
  0x19a4c116,0x1e376c08,0x2748774c,0x34b0bcb5,0x391c0cb3,0x4ed8aa4a,0x5b9cca4f,0x682e6ff3,
  0x748f82ee,0x78a5636f,0x84c87814,0x8cc70208,0x90befffa,0xa4506ceb,0xbef9a3f7,0xc67178f2};
}
void Sha256::compress() {
  std::array<uint32_t,64> w{};
  for (size_t i=0;i<16;++i) for (size_t j=0;j<4;++j) w[i]=(w[i]<<8)|block_[4*i+j];
  for (size_t i=16;i<64;++i) {
    const auto x=w[i-15], y=w[i-2];
    w[i]=w[i-16]+(std::rotr(x,7)^std::rotr(x,18)^(x>>3))+w[i-7]+
         (std::rotr(y,17)^std::rotr(y,19)^(y>>10));
  }
  auto [a,b,c,d,e,f,g,h]=state_;
  for (size_t i=0;i<64;++i) {
    const auto t1=h+(std::rotr(e,6)^std::rotr(e,11)^std::rotr(e,25))+((e&f)^(~e&g))+k[i]+w[i];
    const auto t2=(std::rotr(a,2)^std::rotr(a,13)^std::rotr(a,22))+((a&b)^(a&c)^(b&c));
    h=g; g=f; f=e; e=d+t1; d=c; c=b; b=a; a=t1+t2;
  }
  state_[0]+=a;state_[1]+=b;state_[2]+=c;state_[3]+=d;
  state_[4]+=e;state_[5]+=f;state_[6]+=g;state_[7]+=h;
}
void Sha256::update(std::string_view bytes) {
  bytes_+=bytes.size();
  for (unsigned char c:bytes) { block_[used_++]=c; if(used_==64){compress();used_=0;} }
}
std::string Sha256::hex() const {
  auto copy=*this;
  const uint64_t bits=bytes_*8;
  copy.update(std::string(1,static_cast<char>(0x80)));
  while(copy.used_!=56) copy.update(std::string(1,'\0'));
  for(int i=7;i>=0;--i) copy.update(std::string(1,static_cast<char>(bits>>(i*8))));
  std::ostringstream out;
  out<<std::hex<<std::setfill('0');
  for(auto word:copy.state_) out<<std::setw(8)<<word;
  return out.str();
}
std::string sha256(std::string_view bytes){Sha256 h;h.update(bytes);return h.hex();}
void hashField(Sha256& h,std::string_view bytes){h.update(std::to_string(bytes.size()));h.update(":");h.update(bytes);}
}

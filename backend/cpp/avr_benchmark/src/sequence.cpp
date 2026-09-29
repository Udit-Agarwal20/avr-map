#include "avr_benchmark/sequence.hpp"
#include "avr_benchmark/sha256.hpp"
#include "avr_spatial/cell_key.hpp"
#include <algorithm>
#include <charconv>
#include <cmath>
#include <fstream>
#include <limits>
#include <locale>
#include <map>
#include <sstream>
#include <stdexcept>

namespace avr::benchmark {
namespace {
[[noreturn]] void bad(const std::string& why){throw std::runtime_error("sequence: "+why);}
std::string read(const std::filesystem::path& path) {
  std::ifstream in(path,std::ios::binary);
  if(!in) bad("cannot open "+path.filename().string());
  std::string bytes{std::istreambuf_iterator<char>(in),{}};
  if(in.bad()) bad("read failure");
  return bytes;
}
std::filesystem::path resolve(const std::filesystem::path& root,const std::string& relative) {
  const std::filesystem::path p(relative);
  if(p.empty()||p.is_absolute()) bad("path must be relative");
  for(const auto& component:p) if(component=="..") bad("parent path forbidden");
  auto full=std::filesystem::canonical(root/p);
  auto rel=full.lexically_relative(root);
  if(rel.empty()||*rel.begin()==".."||!std::filesystem::is_regular_file(full)) bad("path escapes sequence root");
  return full;
}
std::vector<std::string> fields(const std::string& line,size_t count) {
  if(line.find_first_of("\"\r\0",0,3)!=std::string::npos) bad("unsupported CSV quoting/control character");
  std::vector<std::string> out;
  size_t start=0;
  do { auto end=line.find(',',start); out.push_back(line.substr(start,end-start));
    if(end==std::string::npos) break;
    start=end+1;
  }while(true);
  if(out.size()!=count) bad("wrong CSV field count");
  return out;
}
double number(const std::string& s) {
  std::istringstream in(s); in.imbue(std::locale::classic()); in>>std::noskipws;
  double value{};
  if(!(in>>value)||in.peek()!=EOF||!std::isfinite(value)) bad("invalid finite number: "+s);
  return value;
}
float confidence(const std::string& s) {
  auto value=number(s);
  if(value<0||value>1) bad("confidence outside [0,1]");
  return static_cast<float>(value);
}
LabeledFrame parseFrame(const Sequence& seq,const FrameIndex& index,const std::string& bytes) {
  LabeledFrame frame;frame.index=index;frame.semantics.frame_seq=index.frame_seq;
  std::istringstream in(bytes);std::string line;
  if(!std::getline(in,line)||line!="source_index,x,y,z,intensity,semantic_class,semantic_confidence,motion_state,motion_confidence")
    bad("required point/semantic columns absent");
  while(std::getline(in,line)) {
    auto f=fields(line,9);
    auto source=parseUnsigned(f[0]);
    if(source!=frame.points.size()||source>std::numeric_limits<uint32_t>::max()) bad("source_index must preserve consecutive original ordinals");
    mapping::PointXYZ p{number(f[1]),number(f[2]),number(f[3])};
    if(p.x<seq.bounds[0]||p.x>=seq.bounds[1]||p.y<seq.bounds[2]||p.y>=seq.bounds[3]||
       !spatial::CellKey::fromWorldPoint({p.x,p.y},spatial::L0)) bad("point outside bounds or quantizable domain");
    auto semantic=parseUnsigned(f[5]), motion=parseUnsigned(f[7]);
    if(semantic>7||motion>3) bad("unknown semantic/motion enum");
    frame.points.push_back(p); frame.intensities.push_back(number(f[4]));
    frame.semantics.observations.push_back({static_cast<uint32_t>(source),
        static_cast<mapping::SemanticClass>(semantic),confidence(f[6]),
        static_cast<mapping::MotionClassification>(motion),confidence(f[8])});
  }
  return frame;
}
}
uint64_t parseUnsigned(const std::string& s) {
  uint64_t value{};
  const auto [end,ec]=std::from_chars(s.data(),s.data()+s.size(),value);
  if(ec!=std::errc{}||end!=s.data()+s.size()||s.empty()) bad("invalid unsigned integer: "+s);
  return value;
}
std::string Sequence::dataTruth() const {
  return isFixture()?"HARNESS VALIDATION FIXTURE ONLY / NOT REAL PERFORMANCE DATA":"REAL RECORDED DATA / OFFLINE BENCHMARK";
}
Sequence loadSequence(const std::filesystem::path& directory) {
  Sequence seq;seq.root=std::filesystem::canonical(directory);
  auto manifest=read(resolve(seq.root,"sequence.avr"));
  std::istringstream in(manifest);std::string line;
  if(!std::getline(in,line)||line!="AVR_SEQUENCE_V1") bad("unsupported manifest schema");
  std::map<std::string,std::string> meta;
  while(std::getline(in,line)) {
    auto equal=line.find('=');
    if(equal==std::string::npos||equal==0||equal+1==line.size()||line.find('\r')!=std::string::npos) bad("malformed manifest field");
    if(!meta.emplace(line.substr(0,equal),line.substr(equal+1)).second) bad("duplicate manifest field");
  }
  const std::array<std::string,9> keys{"name","kind","semantic_provenance","source_description","license","pose_provenance","coordinate_frame","bounds","frames"};
  if(meta.size()!=keys.size()) bad("unexpected manifest fields");
  for(const auto& key:keys) if(!meta.contains(key)) bad("missing manifest field: "+key);
  seq.name=meta.at("name");seq.kind=meta.at("kind");seq.provenance=meta.at("semantic_provenance");
  seq.source_description=meta.at("source_description");seq.license=meta.at("license");seq.pose_provenance=meta.at("pose_provenance");
  if(seq.kind!="HARNESS_VALIDATION_FIXTURE"&&seq.kind!="REAL_RECORDED") bad("unknown data kind");
  if(seq.provenance!="GROUND_TRUTH_LABELS"&&seq.provenance!="PRECOMPUTED_LABELS"&&seq.provenance!="SYNTHETIC_VALIDATION_LABELS") bad("missing/unknown label provenance");
  if(seq.isFixture()!=(seq.provenance=="SYNTHETIC_VALIDATION_LABELS")) bad("synthetic/real provenance conflict");
  if(meta.at("coordinate_frame")!="FIXED_MAP_FRAME") bad("transform points/origins into FIXED_MAP_FRAME before replay");
  auto bounds=fields(meta.at("bounds"),4);
  for(size_t i=0;i<4;++i) seq.bounds[i]=number(bounds[i]);
  if(seq.bounds[0]>=seq.bounds[1]||seq.bounds[2]>=seq.bounds[3]) bad("invalid bounds");
  const auto index_bytes=read(resolve(seq.root,meta.at("frames")));
  Sha256 hash;hashField(hash,manifest);hashField(hash,index_bytes);
  std::istringstream index_stream(index_bytes);
  if(!std::getline(index_stream,line)||line!="frame_seq,timestamp_ns,sensor_x,sensor_y,sensor_z,points_file") bad("invalid frame index header");
  while(std::getline(index_stream,line)) {
    auto f=fields(line,6);FrameIndex index;
    index.frame_seq=parseUnsigned(f[0]);index.timestamp_ns=parseUnsigned(f[1]);
    index.sensor_origin={number(f[2]),number(f[3]),number(f[4])};index.points_file=f[5];
    if(!seq.frames.empty()&&(index.frame_seq<=seq.frames.back().frame_seq||index.timestamp_ns<=seq.frames.back().timestamp_ns)) bad("frame sequence and timestamps must strictly increase");
    auto bytes=read(resolve(seq.root,index.points_file));index.content_hash=sha256(bytes);
    auto frame=parseFrame(seq,index,bytes);index.point_count=frame.points.size();
    if(index.point_count>std::numeric_limits<uint64_t>::max()-seq.point_count) bad("point count overflow");
    seq.point_count+=index.point_count;hashField(hash,bytes);seq.frames.push_back(index);
  }
  if(seq.frames.empty()) bad("empty sequence index");
  seq.hash=hash.hex();return seq;
}
LabeledFrame loadFrame(const Sequence& seq,size_t ordinal) {
  const auto& index=seq.frames.at(ordinal);
  auto bytes=read(resolve(seq.root,index.points_file));
  if(sha256(bytes)!=index.content_hash) bad("point file changed since preflight");
  return parseFrame(seq,index,bytes);
}
std::optional<mapping::SemanticFrameResult> OfflineLabeledSemanticProvider::infer(
    std::span<const mapping::PointXYZ> points,uint64_t frame_seq,spatial::WorldPoint2D origin) {
  if(!frame_||frame_seq!=frame_->index.frame_seq||origin.x!=frame_->index.sensor_origin.x||origin.y!=frame_->index.sensor_origin.y||
     points.size()!=frame_->points.size()||!std::equal(points.begin(),points.end(),frame_->points.begin())||
     frame_->semantics.frame_seq!=frame_seq||frame_->semantics.observations.size()!=points.size())
    throw std::runtime_error("offline provider input/evidence alignment failure");
  return frame_->semantics;
}
}

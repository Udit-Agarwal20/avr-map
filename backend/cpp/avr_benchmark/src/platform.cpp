#include "avr_benchmark/benchmark.hpp"
#include <fstream>
#include <stdexcept>
#include <sys/resource.h>
#if defined(__APPLE__)
#include <mach/mach.h>
#elif defined(__linux__)
#include <unistd.h>
#else
#error "avr_benchmark supports real RSS measurement on macOS and Linux only"
#endif
namespace avr::benchmark {
uint64_t currentRssBytes() {
#if defined(__APPLE__)
  mach_task_basic_info_data_t info{};
  mach_msg_type_number_t count=MACH_TASK_BASIC_INFO_COUNT;
  if(task_info(mach_task_self(),MACH_TASK_BASIC_INFO,reinterpret_cast<task_info_t>(&info),&count)!=KERN_SUCCESS)
    throw std::runtime_error("mach task_info RSS unavailable");
  return info.resident_size;
#else
  std::ifstream in("/proc/self/statm");uint64_t total{},resident{};
  const long page=sysconf(_SC_PAGESIZE);
  if(!(in>>total>>resident)||page<=0) throw std::runtime_error("proc RSS unavailable");
  return resident*static_cast<uint64_t>(page);
#endif
}
uint64_t peakRssBytes() {
  rusage usage{};
  if(getrusage(RUSAGE_SELF,&usage)!=0||usage.ru_maxrss<0) throw std::runtime_error("getrusage high-water RSS unavailable");
#if defined(__APPLE__)
  return static_cast<uint64_t>(usage.ru_maxrss);
#else
  return static_cast<uint64_t>(usage.ru_maxrss)*1024;
#endif
}
}

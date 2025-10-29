#include "perf.h"

#include "log.h"

using std::string;

void Timer::Start() { start_time_ = std::chrono::high_resolution_clock::now(); }

uint64_t Timer::End(const string &msg) {
  end_time_ = std::chrono::high_resolution_clock::now();
  auto elapsed_time = std::chrono::duration_cast<std::chrono::nanoseconds>(
                          end_time_ - start_time_)
                          .count();
  AINFO_F("TIMER [{}] elapsed_time: {} ms", msg,
          static_cast<double>(elapsed_time) / 1000000.0);

  // start new timer.
  start_time_ = end_time_;
  return elapsed_time;
}

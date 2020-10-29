/*
***********************************************************************
* timecounter.h: to give the precise elapsed time in milliseconds, or
* in microseconds. And it can also give the UTC time.
* This header file can be read by C++ compilers
*
*  by Hu.ZH(CrossOcean.ai)
***********************************************************************
*/

#ifndef _TIMECOUNTER_H_
#define _TIMECOUNTER_H_

#include <chrono>
#include <ctime>

namespace ASV::common {

class timecounter {
  using PTIMER = std::chrono::steady_clock;

 public:
  timecounter() : pt_start(PTIMER::now()){};

  // return the elapsed duration in milliseconds
  long int timeelapsed() {
    auto pt_now = PTIMER::now();
    long int milliseconds =
        std::chrono::duration_cast<std::chrono::milliseconds>(pt_now - pt_start)
            .count();
    pt_start = pt_now;
    return milliseconds;
  }

  // return the elapsed duration in microseconds
  long long micro_timeelapsed() {
    auto pt_now = PTIMER::now();
    long long microseconds =
        std::chrono::duration_cast<std::chrono::microseconds>(pt_now - pt_start)
            .count();
    pt_start = pt_now;
    return microseconds;
  }

  // return the UTC time (ISO)
  // TODO: C++20 support utc_time
  std::string getUTCtime() {
    std::time_t result = std::time(nullptr);
    std::string _utc = std::asctime(std::localtime(&result));
    _utc.pop_back();
    return _utc;
  }
  ~timecounter() {}

 private:
  PTIMER::time_point pt_start;

};  // end class timecounter

}  // namespace ASV::common

#endif /*_TIMECOUNTER_H_*/
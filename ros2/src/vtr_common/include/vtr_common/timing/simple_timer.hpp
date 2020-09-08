#pragma once

// system timer
#include <chrono>
#include <ostream>

#include <vtr_common/timing/time_utils.hpp>

namespace vtr {
namespace common {
namespace timing {

class SimpleTimer {
 public:
  SimpleTimer() { reset(); }
  ~SimpleTimer() {}
  void reset() { start_time_ = get_time(); }
  double elapsedMs() const {
    return duration_ms(get_time() - start_time_).count();
  }

  friend std::ostream& operator<<(std::ostream& os, const SimpleTimer& st) {
    return os << st.elapsedMs() << " ms";
  }

 private:
  static time_point get_time() { return clock::now(); }

  time_point start_time_;
};

}  // namespace timing
}  // namespace common
}  // namespace vtr

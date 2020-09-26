#include <algorithm>
#include <sstream>

#include <vtr_common/timing/time_utils.hpp>

namespace vtr {
namespace common {
namespace timing {

time_point toChrono(uint64_t nano_since_epoch) {
  // First order approximation: add the epoch
  date::sys_days unix_epoch = date::day(1) / date::jan / 1970;
  return time_point(unix_epoch + nanoseconds(nano_since_epoch));
}

uint64_t toUnix(const time_point& time) {
  // Beginning of today
  date::sys_days today = date::floor<days>(time);
  time_point this_morning = today;

  // Use day-based arithmetic for the majority of the calculation
  date::sys_days unix_epoch = date::day(1) / date::jan / 1970;
  days days_since_epoch = today - unix_epoch;

  // Time elapsed since the day started
  auto s = time - this_morning;

  // No timezone offset: use UTC
  auto tz_offset = hours(0);
  return (days_since_epoch + s + tz_offset) / nanoseconds(1);
}

std::string toIsoString(const time_point& time) {
  std::stringstream ss;
  ss << datePart(time) << " " << timePart(time) << "Z";
  return ss.str();
}

/// \brief Generate a string representation of a chrono time point that can be
/// used in a file name
std::string toIsoFilename(const time_point& time) {
  std::string name = toIsoString(time);
  std::replace(name.begin(), name.end(), ':', '-');
  std::replace(name.begin(), name.end(), '.', '_');
  std::replace(name.begin(), name.end(), ' ', '_');
  return name;
}

}  // namespace timing
}  // namespace common
}  // namespace vtr

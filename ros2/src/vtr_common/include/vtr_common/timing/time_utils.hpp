#pragma once

#include <chrono>

#include <rclcpp/time.hpp>

#include <vtr_common/timing/date.h>  /// \todo remove this when moving to c++20
#include <vtr_messages/msg/time_stamp.hpp>

namespace vtr {
namespace common {
namespace timing {

// Bring in all of the typedefs so the prefix is consistent
using date::days;
using date::months;
using date::years;
using std::chrono::hours;
using std::chrono::microseconds;
using std::chrono::milliseconds;
using std::chrono::minutes;
using std::chrono::nanoseconds;
using std::chrono::seconds;

using clock = std::chrono::high_resolution_clock;
using time_point = std::chrono::time_point<clock>;
using duration_ms = std::chrono::duration<double, std::milli>;

/** \brief Return the time of day (since midnight, UTC) of a chrono time point
 */
inline date::time_of_day<nanoseconds> timePart(const time_point& time) {
  return date::make_time(time - date::floor<days>(time));
}

/** \brief Return the date (day, month, year) of a chrono time point */
inline date::year_month_day datePart(const time_point& time) {
  return date::year_month_day(date::floor<days>(time));
}

/**
 * \brief Converts a unix timestamp (ns since epoch in UTC) to a chrono time
 * point
 */
time_point toChrono(uint64_t nano_since_epoch);

/** \brief Converts a Robochunk TimeStamp into a chrono time point */
inline time_point toChrono(const vtr_messages::msg::TimeStamp& time) {
  return toChrono(time.nanoseconds_since_epoch);
}

/**
 * \brief Converts a chrono time point into a unix timestamp (ns since epoch,
 * UTC)
 */
uint64_t toUnix(const time_point& time);

/** \brief Converts a chrono time point to a new VTR3 TimeStamp */
inline vtr_messages::msg::TimeStamp toRosTimestamp(const time_point& time) {
  vtr_messages::msg::TimeStamp rtime;
  rtime.nanoseconds_since_epoch = toUnix(time);
  return rtime;
}

/** \brief Fills in an exsting Robochunk TimeStamp froma chrono time point */
inline void setRosTimestamp(vtr_messages::msg::TimeStamp& rtime,
                            const time_point& time) {
  rtime.nanoseconds_since_epoch = toUnix(time);
}

/// \brief Convert a chrono time point into a ROS Time
inline rclcpp::Time toRosTime(const time_point& time) {
  auto stamp = common::timing::nanoseconds(common::timing::toUnix(time));
  auto seconds = std::chrono::duration_cast<common::timing::seconds>(stamp);
  return rclcpp::Time(uint32_t(seconds.count()),
                      uint32_t((stamp - seconds).count()), RCL_ROS_TIME);
}

/**
 * \brief Generate a human-readable string representation of a chrono time
 * point
 */
std::string toIsoString(const time_point& time);

/**
 * \brief Generate a string representation of a chrono time point that can be
 * used in a file name
 */
std::string toIsoFilename(const time_point& time);

}  // namespace timing
}  // namespace common
}  // namespace vtr
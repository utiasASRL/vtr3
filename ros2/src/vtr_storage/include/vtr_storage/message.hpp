#pragma once

#include <rcutils/time.h>
#include <any>
#include <iostream>
#include <optional>

namespace vtr {
namespace storage {

using Index = int32_t;
using TimeStamp = rcutils_time_point_value_t;

constexpr TimeStamp NO_TIMESTAMP_VALUE =
    -1;  // timestamp value stored in sqlite database if message has no
         // timestamps

class VTRMessage {
 public:
  VTRMessage() = default;
  template <class T>
  VTRMessage(T message) : message_{message} {
    static_assert(!std::is_same_v<std::any, T>,
                  "Attempted to initialize a VTRMessage with an std::any!");
  }

  template <class T>
  VTRMessage& operator=(const T& message) {
    message_ = std::make_any<T>(message);
    return *this;
  }

  template <class T>
  void set(T message) {
    message_ = std::make_any<T>(message);
  }

  template <class T>
  T get() const {
    try {
      return std::any_cast<T>(message_);
    } catch (const std::bad_any_cast& e) {
      throw std::runtime_error(
          "Any cast failed in retrieving data in VTR Storage");
    }
  }

  TimeStamp get_timestamp() const {
    if (!timestamp_.has_value()) {
      throw std::runtime_error(
          "Attempted to get uninitialized timestamp of a VTRMessage");
    } else {
      return timestamp_.value();
    }
  }

  bool has_timestamp() const { return timestamp_.has_value(); }

  void set_timestamp(TimeStamp new_timestamp) { timestamp_ = new_timestamp; }

  Index get_index() const {
    if (!database_index_.has_value()) {
      throw std::runtime_error(
          "Attempted to get uninitialized timestamp of a VTRMessage");
    } else {
      return database_index_.value();
    }
  }

  bool has_index() const { return database_index_.has_value(); }

  void set_index(Index new_index) { database_index_ = new_index; }

 private:
  std::any message_;
  std::optional<Index> database_index_;
  std::optional<TimeStamp> timestamp_;
};

}  // namespace storage
}  // namespace vtr
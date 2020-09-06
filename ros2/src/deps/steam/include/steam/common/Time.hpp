//////////////////////////////////////////////////////////////////////////////////////////////
/// \file Time.hpp
/// \brief Implements a basic time class, wrapping an int64, which gives
///        nanosecond precision since epoch (+/- ~9.2e18)...
///
/// \author Sean Anderson, ASRL
//////////////////////////////////////////////////////////////////////////////////////////////

#ifndef STEAM_TIME_HPP
#define STEAM_TIME_HPP

#include <iostream>
#include <boost/cstdint.hpp>

namespace steam {

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Time class
//////////////////////////////////////////////////////////////////////////////////////////////
class Time {
 public:
  Time() : nsecs_(0) {}
  Time(boost::int64_t nsecs) : nsecs_(nsecs) {}
  Time(double secs) : nsecs_(secs*1e9) {}
  Time(boost::int32_t secs, boost::int32_t nsec) {
    boost::int64_t t1 = (boost::int64_t) secs;
    boost::int64_t t2 = (boost::int64_t) nsec;
    this->nsecs_ = t1*1000000000 + t2;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Get number of seconds in double format. Note depending on scale of time, this may
  ///        cause a loss in precious (for example, if nanosecond since epoch are stored).
  ///        Usually it makes sense to use this method after *this* has been reduced to a
  ///        duration between two times.
  //////////////////////////////////////////////////////////////////////////////////////////////
  double seconds() const {
    return static_cast<double>(nsecs_)*1e-9;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Get number of seconds in double format. Note depending on scale of time, this may
  ///        cause a loss in precious (for example, if nanosecond since epoch are stored).
  ///        Usually it makes sense to use this method after *this* has been reduced to a
  ///        duration between two times.
  //////////////////////////////////////////////////////////////////////////////////////////////
  const boost::int64_t& nanosecs() const {
    return nsecs_;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief In-place add
  //////////////////////////////////////////////////////////////////////////////////////////////
  Time& operator+=(const Time& other) {
    nsecs_ += other.nsecs_;
    return *this;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Add
  //////////////////////////////////////////////////////////////////////////////////////////////
  Time operator+(const Time& other) const {
    Time temp(*this);
    temp += other;
    return temp;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief In-place subtract
  //////////////////////////////////////////////////////////////////////////////////////////////
  Time& operator-=(const Time& other) {
    nsecs_ -= other.nsecs_;
    return *this;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Subtract
  //////////////////////////////////////////////////////////////////////////////////////////////
  Time operator-(const Time& other) const {
    Time temp(*this);
    temp -= other;
    return temp;
  }

  bool operator ==(const Time& other) const {
    return this->nsecs_ == other.nsecs_;
  }
  bool operator !=(const Time& other) const {
    return this->nsecs_ != other.nsecs_;
  }
  bool operator <(const Time& other) const {
    return this->nsecs_ < other.nsecs_;
  }
  bool operator >(const Time& other) const {
    return this->nsecs_ > other.nsecs_;
  }
  bool operator <=(const Time& other) const {
    return this->nsecs_ <= other.nsecs_;
  }
  bool operator >=(const Time& other) const {
    return this->nsecs_ >= other.nsecs_;
  }

 private:

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief int64 gives nanosecond precision since epoch (+/- ~9.2e18), which covers the ~1.5e9
  ///        seconds since epoch and 1e9 nsecs. Furthermore, a single base type, rather than
  ///        two combined unsigned int32s to allow nsecs to be used as a key in a std::map.
  //////////////////////////////////////////////////////////////////////////////////////////////
  boost::int64_t nsecs_;

};

} // steam

#endif // STEAM_TIME_HPP

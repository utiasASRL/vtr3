// Copyright 2021, Autonomous Space Robotics Lab (ASRL)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * \file lockable.hpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <mutex>
#include <type_traits>

namespace vtr {
namespace common {

/// A lockable type that requires being locked for access
template <typename T>
struct Lockable {
 public:
  /// A locked thread-safe reference to the value
  template <typename R>
  struct LockedRef : public std::reference_wrapper<R> {
    LockedRef(std::recursive_mutex& mutex, R& ref)
        : lock(mutex, std::defer_lock), std::reference_wrapper<R>(ref) {
      lock.lock();
    }
    LockedRef(LockedRef&& other) = default;

   private:
    std::unique_lock<std::recursive_mutex> lock;
  };

  /// Constructors
  template <bool IDC = std::is_default_constructible<T>::value,
            typename std::enable_if<IDC, bool>::type = false>
  Lockable() {}
  Lockable(T&& val) { val_ = val; }
  Lockable(const Lockable& other) : val_(other.val_) {}
  Lockable(Lockable&& other) = default;

  Lockable& operator=(const Lockable& other) {
    this->val_ = other.val_;
    return *this;
  }
  Lockable& operator=(Lockable&& other) = default;

  /// Get a locked const reference to the value
  LockedRef<const T> locked() const { return {mutex_, val_}; }
  /// Get a locked reference to the value
  LockedRef<T> locked() { return {mutex_, val_}; }

 private:
  /// The value with controlled access
  T val_;
  /// The mutex that controls access to the value
  mutable std::recursive_mutex mutex_;
};

}  // namespace common
}  // namespace vtr

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
 * \file semaphore.cpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#include <vtr_common/utils/semaphore.hpp>

namespace vtr {
namespace common {

/////////////////////////////////////////////////////////////////////////////
/// @brief Default constructor
/// @param count The initial semaphore count
/////////////////////////////////////////////////////////////////////////////
semaphore::semaphore(size_t count) : count_(count) {}

/////////////////////////////////////////////////////////////////////////////
/// @brief Release the semaphore, allowing another thread to acquire it
/////////////////////////////////////////////////////////////////////////////
void semaphore::release() {
  lock_t lck(mtx_);
  ++count_;
  cv_.notify_one();
}

/////////////////////////////////////////////////////////////////////////////
/// @brief Acquire the semaphore.  Blocks if it is not available (count zero)
/////////////////////////////////////////////////////////////////////////////
void semaphore::acquire() {
  lock_t lck(mtx_);
  while (count_ == 0) {
    cv_.wait(lck);
  }

  --count_;
}

/////////////////////////////////////////////////////////////////////////////
/// @brief Try to acquire the semaphore.  Returns false immediately if the
///        semaphore is not available.
/////////////////////////////////////////////////////////////////////////////
bool semaphore::try_acquire() {
  lock_t lck(mtx_);
  if (count_ == 0) {
    return false;
  } else {
    --count_;
    return true;
  }
}

/////////////////////////////////////////////////////////////////////////////
/// @brief Default constructor
/// @param count The initial semaphore count
/// @param bound The maximum semaphore count
/////////////////////////////////////////////////////////////////////////////
bounded_semaphore::bounded_semaphore(size_t count, size_t bound)
    : semaphore(count), bound_(bound) {}

/////////////////////////////////////////////////////////////////////////////
/// @brief Release the semaphore, allowing another thread to acquire it. This
///        blocks until acquire() is called elsewhere when count == bound.
/////////////////////////////////////////////////////////////////////////////
void bounded_semaphore::release() {
  lock_t lck(mtx_);
  while (count_ == bound_) {
    cv_reverse_.wait(lck);
  }
  ++count_;
  cv_.notify_one();
}

/////////////////////////////////////////////////////////////////////////////
/// @brief Acquire the semaphore.  Blocks if it is not available (count zero)
/////////////////////////////////////////////////////////////////////////////
void bounded_semaphore::acquire() {
  lock_t lck(mtx_);
  while (count_ == 0) {
    cv_.wait(lck);
  }

  --count_;
  cv_reverse_.notify_one();
}

/////////////////////////////////////////////////////////////////////////////
/// @brief Try to acquire the semaphore.  Returns false immediately if the
///        semaphore is not available.
/////////////////////////////////////////////////////////////////////////////
bool bounded_semaphore::try_acquire() {
  lock_t lck(mtx_);
  if (count_ == 0) {
    return false;
  } else {
    --count_;
    return true;
  }
}

/////////////////////////////////////////////////////////////////////////////
/// @brief Try to release the semaphore.  Returns false immediately if the
///        semaphore is at the bound.
/////////////////////////////////////////////////////////////////////////////
bool bounded_semaphore::try_release() {
  lock_t lck(mtx_);
  if (count_ == bound_) {
    return false;
  } else {
    ++count_;
    return true;
  }
}

/////////////////////////////////////////////////////////////////////////////
/// @brief Default constructor
/// @param count The initial semaphore count
/////////////////////////////////////////////////////////////////////////////
joinable_semaphore::joinable_semaphore(size_t count) : semaphore(count) {}

/////////////////////////////////////////////////////////////////////////////
/// @brief Release the semaphore, allowing another thread to acquire it
/////////////////////////////////////////////////////////////////////////////
void joinable_semaphore::release() {
  lock_t lck(mtx_);
  ++count_;
  wait_.notify_all();
  cv_.notify_one();
}

/////////////////////////////////////////////////////////////////////////////
/// @brief Acquire the semaphore.  Blocks if it is not available (count zero)
/////////////////////////////////////////////////////////////////////////////
void joinable_semaphore::acquire() {
  lock_t lck(mtx_);
  while (count_ == 0) {
    cv_.wait(lck);
  }

  --count_;
  wait_.notify_all();
}

/////////////////////////////////////////////////////////////////////////////
/// @brief Try to acquire the semaphore.  Returns false immediately if the
///        semaphore is not available.
/////////////////////////////////////////////////////////////////////////////
bool joinable_semaphore::try_acquire() {
  lock_t lck(mtx_);
  if (count_ == 0) {
    return false;
  } else {
    --count_;
    wait_.notify_all();
    return true;
  }
}

/////////////////////////////////////////////////////////////////////////////
/// @brief Wait for the semaphore to achieve a specific value
/////////////////////////////////////////////////////////////////////////////
void joinable_semaphore::wait(size_t val) {
  // TODO: size_t is unsigned, which means calling wait(-1) will wait
  // forerver...
  lock_t lck(mtx_);
  while (count_ != val) {
    wait_.wait(lck);
  }
}

/////////////////////////////////////////////////////////////////////////////
/// @brief Default constructor
/////////////////////////////////////////////////////////////////////////////
bounded_joinable_semaphore::bounded_joinable_semaphore(size_t count,
                                                       size_t bound)
    : semaphore(count),
      bounded_semaphore(count, bound),
      joinable_semaphore(count) {}

/////////////////////////////////////////////////////////////////////////////
/// @brief Release the semaphore, allowing another thread to acquire it
/////////////////////////////////////////////////////////////////////////////
void bounded_joinable_semaphore::release() {
  lock_t lck(mtx_);
  while (count_ == bound_) {
    cv_reverse_.wait(lck);
  }
  ++count_;
  wait_.notify_all();
  cv_.notify_one();
}

/////////////////////////////////////////////////////////////////////////////
/// @brief Acquire the semaphore.  Blocks if it is not available (count zero)
/////////////////////////////////////////////////////////////////////////////
void bounded_joinable_semaphore::acquire() {
  lock_t lck(mtx_);
  while (count_ == 0) {
    cv_.wait(lck);
  }

  --count_;
  wait_.notify_all();
  cv_reverse_.notify_one();
}

/////////////////////////////////////////////////////////////////////////////
/// @brief Try to acquire the semaphore.  Returns false immediately if the
///        semaphore is not available.
/////////////////////////////////////////////////////////////////////////////
bool bounded_joinable_semaphore::try_acquire() {
  lock_t lck(mtx_);
  if (count_ == 0) {
    return false;
  } else {
    --count_;
    wait_.notify_all();
    return true;
  }
}

/////////////////////////////////////////////////////////////////////////////
/// @brief Try to release the semaphore.  Returns false immediately if the
///        semaphore is at the bound.
/////////////////////////////////////////////////////////////////////////////
bool bounded_joinable_semaphore::try_release() {
  lock_t lck(mtx_);
  if (count_ == bound_) {
    return false;
  } else {
    ++count_;
    wait_.notify_all();
    return true;
  }
}

/////////////////////////////////////////////////////////////////////////////
/// @brief Wait for the semaphore to achieve a specific value
/////////////////////////////////////////////////////////////////////////////
void bounded_joinable_semaphore::wait(size_t val) {
  // TODO: size_t is unsigned, which means calling wait(-1) will wait
  // forerver...
  lock_t lck(mtx_);
  while (count_ != val) {
    wait_.wait(lck);
  }
}

}  // namespace common
}  // namespace vtr
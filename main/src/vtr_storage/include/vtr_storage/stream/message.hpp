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
 * \file message.hpp
 * \brief
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <memory>
#include <mutex>
#include <shared_mutex>

#include <rcutils/time.h>

namespace vtr {
namespace storage {

using Index = int;
using Timestamp = rcutils_time_point_value_t;

// index is 0 before the messages has been stored into database, otherwise >0
constexpr Index NO_INDEX_VALUE = 0;
// timestamp value stored in sqlite database if message has no timestamps
constexpr Timestamp NO_TIMESTAMP_VALUE = -1;

/**
 * \brief A simple, generic data container that uses std::shared_ptr<void> to
 * store arbitrary data. No type check is performed when getting/setting data!!
 */
class MessageBase {
 public:
  MessageBase() = delete;
  MessageBase(const MessageBase&) = delete;
  MessageBase(MessageBase&&) = delete;
  MessageBase& operator=(const MessageBase&) = delete;
  MessageBase& operator=(MessageBase&& other) = delete;

  explicit MessageBase(const Timestamp& timestamp = NO_TIMESTAMP_VALUE,
                       const Index& index = NO_INDEX_VALUE)
      : timestamp_{timestamp},
        index_{index},
        saved_{index == NO_INDEX_VALUE ? false : true} {}

  Timestamp getTimestamp() const { return timestamp_; }
  void setTimestamp(const Timestamp& timestamp) {
    timestamp_ = timestamp;
    saved_ = false;
  }

  Index getIndex() const { return index_; }
  void setIndex(const Index& index) {
    if (index_ != NO_INDEX_VALUE && index_ != index)
      throw std::runtime_error{
          "Setting index of a message that already has a different index."};
    index_ = index;
    saved_ = true;
  }

  bool getSaved() const { return saved_; }
  void setSaved(bool saved = true) { saved_ = saved; }

 protected:
  Timestamp timestamp_;
  Index index_;
  bool saved_;
};

template <typename DataType>
class Message : public MessageBase {
 public:
  explicit Message(const std::shared_ptr<DataType>& data,
                   const Timestamp& timestamp = NO_TIMESTAMP_VALUE,
                   const Index& index = NO_INDEX_VALUE)
      : MessageBase{timestamp, index}, data_{data} {}

  const DataType& getData() const { return *data_; }

  const std::shared_ptr<DataType>& getDataPtr() const {return data_;} 

  void setData(const DataType& data) {
    *data_ = data;
    saved_ = false;
  }

 private:
  std::shared_ptr<DataType> data_;
};

/** A lockable message that requires being locked for access. */
template <typename DataType>
class LockableMessage {
 public:
  using Ptr = std::shared_ptr<LockableMessage<DataType>>;
  using MutexType = std::shared_mutex;

  /// A locked thread-safe reference to the value
  template <typename R>
  struct LockedRef : public std::reference_wrapper<R> {
    LockedRef(MutexType& mutex, R& ref)
        : std::reference_wrapper<R>(ref), lock(mutex, std::defer_lock) {
      lock.lock();
    }

   private:
    std::unique_lock<MutexType> lock;
  };

  /// A shared locked thread-safe reference to the value
  template <typename R>
  struct SharedLockedRef : public std::reference_wrapper<R> {
    SharedLockedRef(MutexType& mutex, R& ref)
        : std::reference_wrapper<R>(ref), lock(mutex, std::defer_lock) {
      lock.lock();
    }

   private:
    std::shared_lock<MutexType> lock;
  };

  /// Constructors
  template <class... Args>
  LockableMessage(Args&&... args) : message_(std::forward<Args>(args)...) {}

  /// Disable copy and move
  LockableMessage(const LockableMessage& other) = delete;
  LockableMessage(LockableMessage&& other) = delete;
  LockableMessage& operator=(const LockableMessage& other) = delete;
  LockableMessage& operator=(LockableMessage&& other) = delete;

  /** \brief Gets a locked const reference to the value. */
  LockedRef<const Message<DataType>> locked() const {
    return {mutex_, message_};
  }
  /** \brief Gets a locked reference to the value. */
  LockedRef<Message<DataType>> locked() { return {mutex_, message_}; }

  /** \brief Gets a shared locked const reference to the value. */
  SharedLockedRef<const Message<DataType>> sharedLocked() const {
    return {mutex_, message_};
  }
  /** \brief Gets a shared locked reference to the value. */
  SharedLockedRef<Message<DataType>> sharedLocked() {
    return {mutex_, message_};
  }

  /** \brief Gets an unlocked const reference to the value. */
  std::reference_wrapper<const Message<DataType>> unlocked() const {
    return message_;
  }
  std::reference_wrapper<Message<DataType>> unlocked() { return message_; }

  /** \brief Gets a reference to the mutex. */
  MutexType& mutex() const { return std::ref(mutex_); }

  /** \brief Manually locks the chain. */
  void lock() const { mutex_.lock(); }

  /** \brief Manually unlocks the chain. */
  void unlock() const { mutex_.unlock(); }

  /** \brief Manually locks the chain. */
  void lockShared() const { mutex_.lock_shared(); }

  /** \brief Manually unlocks the chain. */
  void unlockShared() const { mutex_.unlock_shared(); }

 private:
  /// The value with controlled access
  Message<DataType> message_;
  /// The mutex that controls access to the value
  mutable MutexType mutex_;
};

}  // namespace storage
}  // namespace vtr
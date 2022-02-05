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
 * \file simple_queue.hpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <functional>
#include <queue>
#include <stack>

#include "vtr_common/utils/macros.hpp"

namespace vtr {
namespace pose_graph {
namespace simple {

/** \brief Base queue wrapper to standardize interface across queue types */
template <class T>
class QueueBase {
 public:
  PTR_TYPEDEFS(QueueBase);

  virtual ~QueueBase() = default;

  virtual const T &top() const = 0;
  virtual void pop() = 0;

  virtual bool empty() const = 0;

  virtual void push(const T &) = 0;
  virtual void push(T &&) = 0;

  virtual Ptr clone() const = 0;
};

/** \brief Wrapper for a std::queue */
template <class T>
class SimpleQueue : public QueueBase<T> {
 public:
  PTR_TYPEDEFS(SimpleQueue);

  using BasePtr = typename QueueBase<T>::Ptr;

  const T &top() const override { return queue_.front(); }
  void pop() override { queue_.pop(); }

  bool empty() const override { return queue_.empty(); }

  void push(const T &elem) override { queue_.push(elem); }
  void push(T &&elem) override { queue_.push(elem); }

  BasePtr clone() const override {
    return std::make_shared<SimpleQueue>(*this);
  }

 private:
  std::queue<T> queue_;
};

/** \brief Wrapper for a std::stack */
template <class T>
class SimpleStack : public QueueBase<T> {
 public:
  PTR_TYPEDEFS(SimpleStack)

  using BasePtr = typename QueueBase<T>::Ptr;

  const T &top() const override { return queue_.top(); }
  void pop() override { queue_.pop(); }

  bool empty() const override { return queue_.empty(); }

  void push(const T &elem) override { queue_.push(elem); }
  void push(T &&elem) override { queue_.push(elem); }

  BasePtr clone() const override {
    return std::make_shared<SimpleStack>(*this);
  }

 private:
  std::stack<T> queue_;
};

/** \brief Wrapper for a std::priority_queue */
template <class T>
class PriorityQueue : public QueueBase<T> {
 public:
  PTR_TYPEDEFS(PriorityQueue)

  using BasePtr = typename QueueBase<T>::Ptr;

  const T &top() const override { return queue_.top(); }
  void pop() override { queue_.pop(); }

  bool empty() const override { return queue_.empty(); }

  void push(const T &elem) override { queue_.push(elem); }
  void push(T &&elem) override { queue_.push(elem); }

  BasePtr clone() const override {
    return std::make_shared<PriorityQueue>(*this);
  }

 private:
  std::priority_queue<T, std::vector<T>, std::greater<T>> queue_;
};

}  // namespace simple
}  // namespace pose_graph
}  // namespace vtr

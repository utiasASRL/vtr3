#pragma once

#include <functional>
#include <queue>
#include <stack>

#include <vtr_common/utils/macros.hpp>

namespace vtr {
namespace pose_graph {
namespace simple {

/** \brief Base queue wrapper to standardize interface across queue types */
template <class T>
class QueueBase {
 public:
  PTR_TYPEDEFS(QueueBase)

  virtual ~QueueBase(){};

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
  PTR_TYPEDEFS(SimpleQueue)

  using BasePtr = typename QueueBase<T>::Ptr;

  SimpleQueue(){};
  SimpleQueue(const SimpleQueue &) = default;
  SimpleQueue(SimpleQueue &&) = default;
  ~SimpleQueue(){};

  SimpleQueue &operator=(const SimpleQueue &) = default;
  SimpleQueue &operator=(SimpleQueue &&) = default;

  virtual inline const T &top() const {
    return queue_.front();
  }
  virtual inline void pop() {
    queue_.pop();
  }

  virtual inline bool empty() const {
    return queue_.empty();
  }

  virtual inline void push(const T &elem) {
    queue_.push(elem);
  }
  virtual inline void push(T &&elem) {
    queue_.push(elem);
  }

  virtual inline BasePtr clone() const {
    return BasePtr(new SimpleQueue(*this));
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

  SimpleStack(){};
  SimpleStack(const SimpleStack &) = default;
  SimpleStack(SimpleStack &&) = default;
  ~SimpleStack(){};

  SimpleStack &operator=(const SimpleStack &) = default;
  SimpleStack &operator=(SimpleStack &&) = default;

  virtual inline const T &top() const {
    return queue_.top();
  }
  virtual inline void pop() {
    queue_.pop();
  }

  virtual inline bool empty() const {
    return queue_.empty();
  }

  virtual inline void push(const T &elem) {
    queue_.push(elem);
  }
  virtual inline void push(T &&elem) {
    queue_.push(elem);
  }

  virtual inline BasePtr clone() const {
    return BasePtr(new SimpleStack(*this));
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

  PriorityQueue(){};
  PriorityQueue(const PriorityQueue &) = default;
  PriorityQueue(PriorityQueue &&) = default;
  ~PriorityQueue(){};

  PriorityQueue &operator=(const PriorityQueue &) = default;
  PriorityQueue &operator=(PriorityQueue &&) = default;

  virtual inline const T &top() const {
    return queue_.top();
  }
  virtual inline void pop() {
    queue_.pop();
  }

  virtual inline bool empty() const {
    return queue_.empty();
  }

  virtual inline void push(const T &elem) {
    queue_.push(elem);
  }
  virtual inline void push(T &&elem) {
    queue_.push(elem);
  }

  virtual inline BasePtr clone() const {
    return BasePtr(new PriorityQueue(*this));
  }

 private:
  std::priority_queue<T, std::vector<T>, std::greater<T>> queue_;
};

}  // namespace simple
}  // namespace pose_graph
}  // namespace vtr

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
 * \file test_task_queues.cpp
 * \brief
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include <gtest/gtest.h>

#include <chrono>

#include "vtr_logging/logging_init.hpp"
#include "vtr_tactic/cache.hpp"
#include "vtr_tactic/task_queues/async_task_queue.hpp"

using namespace ::testing;
using namespace vtr;
using namespace vtr::logging;
using namespace vtr::tactic;

class SimpleTask : public BaseTask {
 public:
  using Ptr = std::shared_ptr<SimpleTask>;

  SimpleTask(const int& duration, const int& id, const unsigned& priority = 0)
      : BaseTask(priority), duration_(duration), id_(id) {}

  void run(const AsyncTaskExecutor::Ptr&, const Graph::Ptr&) override {
    LOG(INFO) << "Start simple task with duration " << duration_
              << "s; and priority " << priority;
    std::this_thread::sleep_for(std::chrono::seconds(duration_));
    LOG(INFO) << "Finish simple task with duration " << duration_
              << "s; and priority " << priority;
  }

 private:
  const int duration_;
  const int id_;
};

TEST(AsyncTaskExecutor, async_task_queue_basics) {
  // create a task queue with 2 threads and queue length of 6
  auto task_queue = std::make_shared<AsyncTaskExecutor>(nullptr, 2, 6);

  // dispatch some tasks with equal priority
  for (int i = 0; i < 10; i++) {
    int priority = 0, id = i, duration = 2;
    task_queue->dispatch(std::make_shared<SimpleTask>(duration, id, priority));
    LOG(INFO) << "Dispatched task of priority: " << priority << ", id: " << id
              << ", duration: " << duration << " seconds.";
  }

  // wait until all tasks are done
  task_queue->wait();
  LOG(INFO) << "All tasks have finished!" << std::endl;

  // dispatch some tasks with unequal priority
  for (int i = 0; i < 2; i++) {
    int priority = 0, id = i, duration = 3;
    task_queue->dispatch(std::make_shared<SimpleTask>(duration, id, priority));
    LOG(INFO) << "Dispatched task of priority: " << priority << ", id: " << id
              << ", duration: " << duration << " seconds.";
  }
  for (int i = 2; i < 8; i++) {
    int priority = (i < 5 ? 0 : 9);
    int id = i, duration = 1;
    task_queue->dispatch(std::make_shared<SimpleTask>(duration, id, priority));
    LOG(INFO) << "Dispatched task of priority: " << priority << ", id: " << id
              << ", duration: " << duration << " seconds.";
  }

  // wait until all tasks are done
  task_queue->wait();
  LOG(INFO) << "All tasks have finished!";

  // destructor will call join, which clears the queue stops all threads
}

class SimpleTask1 : public BaseTask {
 public:
  SimpleTask1(const LockableCache<std::tuple<int, int, int>>::Ptr& data,
              const unsigned& priority = 0)
      : BaseTask(priority), data_(data) {}

  void run(const AsyncTaskExecutor::Ptr&, const Graph::Ptr&) override {
    /// This task has no dependency, simply wait and update data.
    LOG(INFO) << "Start simple task 1 with duration " << duration_
              << "s; and priority " << priority;
    std::this_thread::sleep_for(std::chrono::seconds(duration_));
    /// perform update to data
    auto locked = data_->locked();
    auto& dataref = locked.get();
    std::get<0>(*dataref)++;
    LOG(INFO) << "Finish simple task 1 with duration " << duration_
              << "s; and priority " << priority;
  }

 private:
  int duration_ = 3;
  const LockableCache<std::tuple<int, int, int>>::Ptr data_;
};

class SimpleTask2 : public BaseTask {
 public:
  SimpleTask2(const LockableCache<std::tuple<int, int, int>>::Ptr& data,
              const unsigned& priority = 0)
      : BaseTask(priority), data_(data) {}

  void run(const AsyncTaskExecutor::Ptr& executor, const Graph::Ptr&) override {
    /// This task depends on task 1, so first check dependency
    if (!std::get<0>(*data_->locked().get())) {
      /// dependency not met (simple task 1 has not been run even once)
      /// launch simple task 1 with 1 higher priority
      /// \note IMPORTANT: always launch the dependent task with a higher
      /// priority, otherwise you may end up launching *this task infinite
      /// times.
      /// \note one downside of this is that you may end up launching multiple
      /// instances of the dependent task - have a look at the result of this
      /// test.
      executor->tryDispatch(std::make_shared<SimpleTask1>(data_, priority + 1));
      /// then relaunch this task
      executor->tryDispatch(shared_from_this());
      /// \note IMPORTANT: always use tryDispatch instead of dispatch in this
      /// case because dispatching a task with dependency in this case can hang
      /// the system, think about the extreme case where you have an executor
      /// with 1 thread and 0 queue length.
      return;
    }
    LOG(INFO) << "Start simple task 2 with duration " << duration_
              << "s; and priority " << priority;
    std::this_thread::sleep_for(std::chrono::seconds(duration_));
    /// perform update to data
    auto locked = data_->locked();
    auto& dataref = locked.get();
    std::get<1>(*dataref)++;
    LOG(INFO) << "Finish simple task 2 with duration " << duration_
              << "s; and priority " << priority;
  }

 private:
  int duration_ = 3;
  const LockableCache<std::tuple<int, int, int>>::Ptr data_;
};

class SimpleTask3 : public BaseTask {
 public:
  SimpleTask3(const LockableCache<std::tuple<int, int, int>>::Ptr& data,
              const unsigned& priority = 0)
      : BaseTask(priority), data_(data) {}

  void run(const AsyncTaskExecutor::Ptr& executor, const Graph::Ptr&) override {
    /// This task depends on task 1, so first check dependency
    if (!std::get<1>(*data_->locked().get())) {
      /// dependency not met (simple task 2 has not been run even once)
      /// launch simple task 1 with 1 higher priority
      /// \note IMPORTANT: always launch the dependent task with a higher
      /// priority, otherwise you may end up launching *this task infinite
      /// times.
      /// \note one downside of this is that you may end up launching multiple
      /// instances of the dependent task - have a look at the result of this
      /// test.
      executor->tryDispatch(std::make_shared<SimpleTask2>(data_, priority + 1));
      /// then relaunch this task
      executor->tryDispatch(shared_from_this());
      /// \note IMPORTANT: always use tryDispatch instead of dispatch in this
      /// case because dispatching a task with dependency in this case can hang
      /// the system, think about the extreme case where you have an executor
      /// with 1 thread and 0 queue length.
      return;
    }
    LOG(INFO) << "Start simple task 3 with duration " << duration_
              << "s; and priority " << priority;
    std::this_thread::sleep_for(std::chrono::seconds(duration_));
    /// perform update to data
    auto locked = data_->locked();
    auto& dataref = locked.get();
    std::get<2>(*dataref)++;
    LOG(INFO) << "Finish simple task 3 with duration " << duration_
              << "s; and priority " << priority;
  }

 private:
  int duration_ = 3;
  const LockableCache<std::tuple<int, int, int>>::Ptr data_;
};

TEST(AsyncTaskExecutor, async_task_queue_dependency_handling) {
  /// Create tasks with dependencies and see how this execution model reacts.
  /// \note to get more sense: try the following and check the result
  ///   executor config:      result:
  ///  - (nullptr, 1, 0)  ->    <0, 0, 0>
  ///  - (nullptr, 2, 0)  ->    <1, 0, 0>
  ///  - (nullptr, 3, 0)  ->    <2, 0, 0>
  ///  - (nullptr, 4, 0)  ->    <3, 0, 0>
  ///  - (nullptr, 1, 1)  ->    <1, 0, 0>
  ///  - (nullptr, 1, 2)  ->    <1, 1, 1>
  ///  - (nullptr, 1, 3)  ->    <1, 1, 1>
  ///  - (nullptr, 2, 1)  ->    <2, 0, 0>
  ///  - (nullptr, 3, 1)  ->    <2, 0, 0>
  ///  - (nullptr, 3, 10)  ->   <3, 3, 1>
  ///  - (nullptr, 3, 1000)  ->   <3, 3, 1>
  ///  - (nullptr, 8, 10)  ->   <8, 8, 1>
  /// Can see the trend here: number of repetitive task execution depends on
  /// number of threads but not queue length - this behavior is acceptable since
  /// we normally do not use many threads in the executor, but the queue length
  /// can be infinity.
  auto task_queue = std::make_shared<AsyncTaskExecutor>(nullptr, 8, 1000);
  auto data = std::make_shared<LockableCache<std::tuple<int, int, int>>>();
  data->unlocked().get().emplace(0, 0, 0);
  task_queue->dispatch(std::make_shared<SimpleTask3>(data));

  task_queue->wait();

  auto result = *data->unlocked().get();
  LOG(INFO) << "Result: <" << std::get<0>(result) << ", " << std::get<1>(result)
            << ", " << std::get<2>(result) << ">";

  // destructor will call join, which clears the queue stops all threads
}

int main(int argc, char** argv) {
  configureLogging("", true);
  InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
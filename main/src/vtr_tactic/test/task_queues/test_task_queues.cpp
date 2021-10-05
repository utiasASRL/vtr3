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

#include <vtr_logging/logging_init.hpp>
#include <vtr_tactic/task_queues/priority_task_queue.hpp>

using namespace ::testing;
using namespace vtr::logging;
using namespace vtr::tactic;

TEST(PriorityTaskQueue, priority_task_queue_basics) {
  // defines some task to run
  auto task = [](int priority, int id, int duration) {
    LOG(INFO) << "  Starting task of priority: " << priority << ", id: " << id
              << ", duration: " << duration << " seconds.";
    std::this_thread::sleep_for(std::chrono::seconds(duration));
    LOG(INFO) << "    Finishing task of priority: " << priority
              << ", id: " << id << ", duration: " << duration << " seconds.";
  };

  // create a task queue with 2 threads and queue length of 5
  PriorityTaskQueue task_queue(2, 6);

  // dispatch some tasks with equal priority
  for (int i = 0; i < 10; i++) {
    int priority = 0, id = i, duration = 2;
    task_queue.dispatch(priority, task, priority, id, duration);
    LOG(INFO) << "Dispatched task of priority: " << priority << ", id: " << id
              << ", duration: " << duration << " seconds.";
  }

  // wait until all tasks are done
  task_queue.wait();
  LOG(INFO) << "All tasks have finished!" << std::endl;

  // dispatch some tasks with unequal priority
  for (int i = 0; i < 2; i++) {
    int priority = 0, id = i, duration = 3;
    task_queue.dispatch(priority, task, priority, id, duration);
    LOG(INFO) << "Dispatched task of priority: " << priority << ", id: " << id
              << ", duration: " << duration << " seconds.";
  }
  for (int i = 2; i < 8; i++) {
    int priority = (i < 5 ? 0 : 9);
    int id = i, duration = 1;
    task_queue.dispatch(priority, task, priority, id, duration);
    LOG(INFO) << "Dispatched task of priority: " << priority << ", id: " << id
              << ", duration: " << duration << " seconds.";
  }

  // wait until all tasks are done
  task_queue.wait();
  LOG(INFO) << "All tasks have finished!";

  // destructor will call join, which clears the queue stops all threads
}

int main(int argc, char** argv) {
  configureLogging("", true);
  InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
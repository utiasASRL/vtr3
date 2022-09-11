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
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include <gtest/gtest.h>

#include <chrono>

#include "vtr_logging/logging_init.hpp"
#include "vtr_tactic/task_queue.hpp"

using namespace ::testing;
using namespace vtr;
using namespace vtr::logging;
using namespace vtr::tactic;

TEST(TaskQueue, task_queue_basic) {
  // create a task queue
  TaskQueue task_queue(4);
  std::vector<unsigned> task_ids;
  // create some test tasks with no dependencies
  for (unsigned i = 0; i < 4; ++i) {
    auto task = std::make_shared<Task>(nullptr, nullptr);
    task_queue.push(task);
    task_ids.push_back(task->id);
  }
  // default order is from oldest to latest
  for (unsigned i = 0; i < 4; ++i) {
    ASSERT_TRUE(task_queue.hasNext());
    auto task = task_queue.pop();
    EXPECT_EQ(task->id, task_ids.at(i));
    task_queue.updateDeps(task);  // must call this after pop
  }

  ASSERT_FALSE(task_queue.hasNext());
}

TEST(TaskQueue, task_queue_basic_priority) {
  // create a task queue
  TaskQueue task_queue(4);
  std::vector<unsigned> task_ids;
  // create some test tasks with no dependencies
  for (unsigned i = 0; i < 4; ++i) {
    auto task = std::make_shared<Task>(nullptr, nullptr, i);
    task_queue.push(task);
    task_ids.push_back(task->id);
  }
  // discard the oldest task
  for (int i = 3; i >= 0; --i) {
    ASSERT_TRUE(task_queue.hasNext());
    auto task = task_queue.pop();
    EXPECT_EQ(task->id, task_ids.at(i));
    task_queue.updateDeps(task);  // must call this after pop
  }

  ASSERT_FALSE(task_queue.hasNext());
}

TEST(TaskQueue, task_queue_basic_exceeded_capacity) {
  // create a task queue
  TaskQueue task_queue(2);
  std::vector<unsigned> task_ids;
  // create some test tasks with no dependencies
  for (unsigned i = 0; i < 4; ++i) {
    auto task = std::make_shared<Task>(nullptr, nullptr);
    task_queue.push(task);
    task_ids.push_back(task->id);
  }
  // discard the oldest task
  for (unsigned i = 2; i < 4; ++i) {
    ASSERT_TRUE(task_queue.hasNext());
    auto task = task_queue.pop();
    EXPECT_EQ(task->id, task_ids.at(i));
    task_queue.updateDeps(task);  // must call this after pop
  }

  ASSERT_FALSE(task_queue.hasNext());
}

TEST(TaskQueue, task_queue_basic_dependency) {
  // create a task queue
  TaskQueue task_queue(4);
  std::vector<unsigned> task_ids;

  // create test tasks with dependency
  auto task0 = std::make_shared<Task>(nullptr, nullptr);
  task_queue.push(task0);
  task_ids.push_back(task0->id);

  auto task1 = std::make_shared<Task>(
      nullptr, nullptr, 0, std::initializer_list<Task::DepId>{task0->dep_id});
  task_queue.push(task1);
  task_ids.push_back(task1->id);

  auto task2 = std::make_shared<Task>(
      nullptr, nullptr, 0, std::initializer_list<Task::DepId>{task1->dep_id});
  task_queue.push(task2);
  task_ids.push_back(task2->id);

  auto task3 = std::make_shared<Task>(
      nullptr, nullptr, 0, std::initializer_list<Task::DepId>{task2->dep_id});
  task_queue.push(task3);
  task_ids.push_back(task3->id);

  // dependency is respected
  for (unsigned i = 0; i < 4; ++i) {
    ASSERT_TRUE(task_queue.hasNext());
    auto task = task_queue.pop();
    EXPECT_EQ(task->id, task_ids.at(i));
    task_queue.updateDeps(task);  // must call this after pop
  }

  ASSERT_FALSE(task_queue.hasNext());
}

TEST(TaskQueue, task_queue_basic_dependency_priority) {
  // create a task queue
  TaskQueue task_queue(4);
  std::vector<unsigned> task_ids;

  // create test tasks with dependency and priority
  // 0 <- 1, 2 <- 3
  std::vector<size_t> deps{0, 2, 1, 3};
  auto task0 = std::make_shared<Task>(nullptr, nullptr);
  task_queue.push(task0);
  task_ids.push_back(task0->id);

  auto task1 = std::make_shared<Task>(
      nullptr, nullptr, 1, std::initializer_list<Task::DepId>{task0->dep_id});
  task_queue.push(task1);
  task_ids.push_back(task1->id);

  auto task2 = std::make_shared<Task>(
      nullptr, nullptr, 2, std::initializer_list<Task::DepId>{task0->dep_id});
  task_queue.push(task2);
  task_ids.push_back(task2->id);

  auto task3 = std::make_shared<Task>(
      nullptr, nullptr, 0,
      std::initializer_list<Task::DepId>{task1->dep_id, task2->dep_id});
  task_queue.push(task3);
  task_ids.push_back(task3->id);

  // dependency is respected
  for (unsigned i = 0; i < 4; ++i) {
    ASSERT_TRUE(task_queue.hasNext());
    auto task = task_queue.pop();
    EXPECT_EQ(task->id, task_ids.at(deps[i]));
    task_queue.updateDeps(task);  // must call this after pop
  }

  ASSERT_FALSE(task_queue.hasNext());
}

TEST(TaskQueue, task_queue_basic_dependency_priority_capacity) {
  // create a task queue
  TaskQueue task_queue(3);
  std::vector<unsigned> task_ids;

  // create test tasks with dependency and priority
  // 0 <- 1, 2 <- 3
  auto task0 = std::make_shared<Task>(nullptr, nullptr, 1);
  task_queue.push(task0);

  auto task1 = std::make_shared<Task>(
      nullptr, nullptr, 1, std::initializer_list<Task::DepId>{task0->dep_id});
  task_queue.push(task1);

  auto task2 = std::make_shared<Task>(
      nullptr, nullptr, 2, std::initializer_list<Task::DepId>{task0->dep_id});
  task_queue.push(task2);

  // discarded immediately (due to dependency)
  auto task3 = std::make_shared<Task>(
      nullptr, nullptr, 4,
      std::initializer_list<Task::DepId>{task1->dep_id, task2->dep_id});
  task_queue.push(task3);

  // discarded immediately (due to priority)
  auto task4 = std::make_shared<Task>(nullptr, nullptr, 0);
  task_queue.push(task4);

  // discard task2
  auto task5 = std::make_shared<Task>(nullptr, nullptr, 1);
  task_queue.push(task5);
  task_ids.push_back(task5->id);

  // discard task1
  auto task6 = std::make_shared<Task>(nullptr, nullptr, 1);
  task_queue.push(task6);
  task_ids.push_back(task6->id);

  // discard task0
  auto task7 = std::make_shared<Task>(nullptr, nullptr, 1);
  task_queue.push(task7);
  task_ids.push_back(task7->id);

  // dependency is respected
  for (unsigned i = 0; i < 3; ++i) {
    ASSERT_TRUE(task_queue.hasNext());
    auto task = task_queue.pop();
    EXPECT_EQ(task->id, task_ids.at(i));
    task_queue.updateDeps(task);  // must call this after pop
  }

  ASSERT_FALSE(task_queue.hasNext());
}

TEST(TaskQueue, task_queue_basic_dependency_priority_capacity_with_pop) {
  // create a task queue
  TaskQueue task_queue(3);
  std::vector<unsigned> task_ids;

  // create test tasks with dependency and priority
  // 0 <- 1, 2 <- 3
  auto task0 = std::make_shared<Task>(nullptr, nullptr, 1);
  task_queue.push(task0);

  auto task1 = std::make_shared<Task>(
      nullptr, nullptr, 1, std::initializer_list<Task::DepId>{task0->dep_id});
  task_queue.push(task1);

  auto task2 = std::make_shared<Task>(
      nullptr, nullptr, 2, std::initializer_list<Task::DepId>{task0->dep_id});
  task_queue.push(task2);

  // pop task0 (running)
  auto poped = task_queue.pop();

  // not discarded
  auto task3 = std::make_shared<Task>(
      nullptr, nullptr, 4,
      std::initializer_list<Task::DepId>{task1->dep_id, task2->dep_id});
  task_queue.push(task3);

  // discarded immediately (due to priority)
  auto task4 = std::make_shared<Task>(nullptr, nullptr, 0);
  task_queue.push(task4);

  // discard task2
  auto task5 = std::make_shared<Task>(nullptr, nullptr, 2);
  task_queue.push(task5);
  task_ids.push_back(task5->id);

  // discard task2
  auto task6 = std::make_shared<Task>(nullptr, nullptr, 2);
  task_queue.push(task6);
  task_ids.push_back(task6->id);

  // discard task1
  auto task7 = std::make_shared<Task>(nullptr, nullptr, 2);
  task_queue.push(task7);
  task_ids.push_back(task7->id);

  task_queue.updateDeps(poped);  // must call this after pop

  // dependency is respected
  for (unsigned i = 0; i < 3; ++i) {
    ASSERT_TRUE(task_queue.hasNext());
    auto task = task_queue.pop();
    EXPECT_EQ(task->id, task_ids.at(i));
    task_queue.updateDeps(task);  // must call this after pop
  }

  ASSERT_FALSE(task_queue.hasNext());
  ASSERT_TRUE(task_queue.empty());  // consistency check
}

TEST(TaskQueue, task_queue_basic_stop) {
  // create a task queue
  TaskQueue task_queue(10);

  // create test tasks with complex dependency
  // 0, 1 <- 2 <- 4 <- 6
  //      <- 3 <- 5
  auto task0 = std::make_shared<Task>(nullptr, nullptr, 1);
  task_queue.push(task0);
  auto task1 = std::make_shared<Task>(nullptr, nullptr, 1);
  task_queue.push(task1);
  auto task2 = std::make_shared<Task>(
      nullptr, nullptr, 1,
      std::initializer_list<Task::DepId>{task0->dep_id, task1->dep_id});
  task_queue.push(task2);
  auto task3 = std::make_shared<Task>(
      nullptr, nullptr, 1,
      std::initializer_list<Task::DepId>{task0->dep_id, task1->dep_id});
  task_queue.push(task3);
  auto task4 = std::make_shared<Task>(
      nullptr, nullptr, 1, std::initializer_list<Task::DepId>{task2->dep_id});
  task_queue.push(task4);
  auto task5 = std::make_shared<Task>(
      nullptr, nullptr, 1, std::initializer_list<Task::DepId>{task3->dep_id});
  task_queue.push(task5);
  auto task6 = std::make_shared<Task>(
      nullptr, nullptr, 1,
      std::initializer_list<Task::DepId>{task4->dep_id, task5->dep_id});
  task_queue.push(task6);

  task_queue.clear();
  ASSERT_FALSE(task_queue.hasNext());
  ASSERT_TRUE(task_queue.empty());
}

TEST(TaskQueue, task_queue_basic_stop_with_pop) {
  // create a task queue
  TaskQueue task_queue(10);

  // create test tasks with complex dependency
  // 0, 1 <- 2 <- 4 <- 6
  //      <- 3 <- 5
  auto task0 = std::make_shared<Task>(nullptr, nullptr, 1);
  task_queue.push(task0);
  auto task1 = std::make_shared<Task>(nullptr, nullptr, 1);
  task_queue.push(task1);
  auto task2 = std::make_shared<Task>(
      nullptr, nullptr, 1,
      std::initializer_list<Task::DepId>{task0->dep_id, task1->dep_id});
  task_queue.push(task2);
  auto task3 = std::make_shared<Task>(
      nullptr, nullptr, 1,
      std::initializer_list<Task::DepId>{task0->dep_id, task1->dep_id});
  task_queue.push(task3);
  auto task4 = std::make_shared<Task>(
      nullptr, nullptr, 1, std::initializer_list<Task::DepId>{task2->dep_id});
  task_queue.push(task4);
  auto task5 = std::make_shared<Task>(
      nullptr, nullptr, 1, std::initializer_list<Task::DepId>{task3->dep_id});
  task_queue.push(task5);
  auto task6 = std::make_shared<Task>(
      nullptr, nullptr, 1,
      std::initializer_list<Task::DepId>{task4->dep_id, task5->dep_id});
  task_queue.push(task6);

  // some tasks start running
  auto poped1 = task_queue.pop();
  auto poped2 = task_queue.pop();
  ASSERT_FALSE(task_queue.hasNext());

  // clear the pending tasks
  task_queue.clear();

  // running tasks have finished
  task_queue.updateDeps(poped1);
  task_queue.updateDeps(poped2);

  ASSERT_FALSE(task_queue.hasNext());
  ASSERT_TRUE(task_queue.empty());
}

int main(int argc, char** argv) {
  configureLogging("", true);
  InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
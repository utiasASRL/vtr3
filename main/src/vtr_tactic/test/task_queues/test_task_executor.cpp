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
#include "vtr_tactic/modules/base_module.hpp"
#include "vtr_tactic/modules/factory.hpp"
#include "vtr_tactic/task_queue.hpp"

using namespace ::testing;
using namespace vtr;
using namespace vtr::logging;
using namespace vtr::tactic;

std::mutex g_mutex;  // for qdata thread safety

class TestAsyncModule : public BaseModule {
 public:
  static constexpr auto static_name = "test_async";

  struct Config : public BaseModule::Config {
    using Ptr = std::shared_ptr<Config>;
    using ConstPtr = std::shared_ptr<const Config>;

    static ConstPtr fromROS(const rclcpp::Node::SharedPtr&,
                            const std::string&) {
      return std::make_shared<Config>();
    }
  };

  TestAsyncModule(
      const Config::ConstPtr& config,
      const std::shared_ptr<ModuleFactoryV2>& module_factory = nullptr,
      const std::string& name = static_name)
      : BaseModule{module_factory, name}, config_(config) {}

 private:
  void runImpl(QueryCache& qdata, OutputCache&, const Graph::Ptr&,
               const TaskExecutor::Ptr& executor) override {
    executor->dispatch(
        std::make_shared<Task>(shared_from_this(), qdata.shared_from_this()));
  }

  void runAsyncImpl(QueryCache& qdata, OutputCache&, const Graph::Ptr&,
                    const TaskExecutor::Ptr&, const Task::Priority&,
                    const Task::DepId&) override {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    std::lock_guard<std::mutex> guard(g_mutex);
    ++(*qdata.stamp);
  }

  Config::ConstPtr config_;

  VTR_REGISTER_MODULE_DEC_TYPE(TestAsyncModule);
};

TEST(TaskExecutor, async_task_queue_basic) {
  auto output = std::make_shared<OutputCache>();

  // create a task queue with 2 threads and queue length of 6
  auto executor = std::make_shared<TaskExecutor>(output, nullptr, 2, 2);

  // create a query cache
  auto qdata = std::make_shared<QueryCache>();
  qdata->stamp.emplace(0);

  // create modules that will be dispatched to the task queue
  std::vector<BaseModule::Ptr> modules;
  for (int i = 0; i < 6; ++i) {
    auto module = std::make_shared<TestAsyncModule>(
        nullptr, nullptr, TestAsyncModule::static_name + std::to_string(i + 1));
    modules.emplace_back(module);
  }

  // dispatch all modules to the task queue
  for (auto& module : modules) module->run(*qdata, *output, nullptr, executor);

  // wait until all tasks are done
  executor->wait();
  LOG(INFO) << "All tasks have finished!";
  LOG(INFO) << "Final qdata stamp: " << *qdata->stamp;
  EXPECT_EQ(*qdata->stamp, 4);

  // destructor will call join, which clears the queue stops all threads
}

class TestAsyncModuleDep0 : public BaseModule {
 public:
  static constexpr auto static_name = "test_async_dep0";

  struct Config : public BaseModule::Config {
    using Ptr = std::shared_ptr<Config>;
    using ConstPtr = std::shared_ptr<const Config>;

    static ConstPtr fromROS(const rclcpp::Node::SharedPtr&,
                            const std::string&) {
      return std::make_shared<Config>();
    }
  };

  TestAsyncModuleDep0(
      const Config::ConstPtr& config,
      const std::shared_ptr<ModuleFactoryV2>& module_factory = nullptr,
      const std::string& name = static_name)
      : BaseModule{module_factory, name}, config_(config) {}

 private:
  void runImpl(QueryCache& qdata, OutputCache&, const Graph::Ptr&,
               const TaskExecutor::Ptr& executor) override {
    executor->dispatch(
        std::make_shared<Task>(shared_from_this(), qdata.shared_from_this()));
  }

  void runAsyncImpl(QueryCache& qdata, OutputCache&, const Graph::Ptr&,
                    const TaskExecutor::Ptr& executor,
                    const Task::Priority& priority,
                    const Task::DepId& dep_id) override {
    {
      std::lock_guard<std::mutex> guard(g_mutex);
      if (*qdata.stamp < 1) {
        // launch the dependent task with higher priority
        auto dep_module = factory()->get(TestAsyncModule::static_name);
        auto dep_task = std::make_shared<Task>(
            dep_module, qdata.shared_from_this(), priority + 1);
        executor->dispatch(dep_task);
        // launch this task again with the same task and dep id
        auto task = std::make_shared<Task>(
            shared_from_this(), qdata.shared_from_this(), priority,
            std::initializer_list<Task::DepId>{dep_task->dep_id}, dep_id);
        executor->dispatch(task);
        return;
      }

      // does the work of this module
      ++(*qdata.stamp);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  Config::ConstPtr config_;

  VTR_REGISTER_MODULE_DEC_TYPE(TestAsyncModuleDep0);
};

TEST(TaskExecutor, async_task_queue_dep0) {
  auto output = std::make_shared<OutputCache>();

  // create a task queue with 2 threads and queue length of 2
  auto executor = std::make_shared<TaskExecutor>(output, nullptr, 2, 2);

  // create a query cache
  auto qdata = std::make_shared<QueryCache>();
  qdata->stamp.emplace(0);

  // create a module factory to get module
  auto factory = std::make_shared<ModuleFactoryV2>();

  // get and run the module
  auto module = factory->get(TestAsyncModuleDep0::static_name);
  module->run(*qdata, *output, nullptr, executor);

  // wait until all tasks are done
  executor->wait();
  LOG(INFO) << "All tasks have finished!";
  LOG(INFO) << "Final qdata stamp: " << *qdata->stamp;
  EXPECT_EQ(*qdata->stamp, 2);

  // destructor will call join, which clears the queue stops all threads
}

class TestAsyncModuleDep1 : public BaseModule {
 public:
  static constexpr auto static_name = "test_async_dep1";

  struct Config : public BaseModule::Config {
    using Ptr = std::shared_ptr<Config>;
    using ConstPtr = std::shared_ptr<const Config>;

    static ConstPtr fromROS(const rclcpp::Node::SharedPtr&,
                            const std::string&) {
      return std::make_shared<Config>();
    }
  };

  TestAsyncModuleDep1(
      const Config::ConstPtr& config,
      const std::shared_ptr<ModuleFactoryV2>& module_factory = nullptr,
      const std::string& name = static_name)
      : BaseModule{module_factory, name}, config_(config) {}

 private:
  void runImpl(QueryCache& qdata, OutputCache&, const Graph::Ptr&,
               const TaskExecutor::Ptr& executor) override {
    executor->dispatch(
        std::make_shared<Task>(shared_from_this(), qdata.shared_from_this()));
  }

  void runAsyncImpl(QueryCache& qdata, OutputCache&, const Graph::Ptr&,
                    const TaskExecutor::Ptr& executor,
                    const Task::Priority& priority,
                    const Task::DepId& dep_id) override {
    {
      std::lock_guard<std::mutex> guard(g_mutex);
      if (*qdata.stamp < 2) {
        // launch the dependent task with higher priority
        auto dep_module = factory()->get(TestAsyncModuleDep0::static_name);
        auto dep_task = std::make_shared<Task>(
            dep_module, qdata.shared_from_this(), priority + 1);
        executor->dispatch(dep_task);
        // launch this task again with the same task and dep id
        auto task = std::make_shared<Task>(
            shared_from_this(), qdata.shared_from_this(), priority,
            std::initializer_list<Task::DepId>{dep_task->dep_id}, dep_id);
        executor->dispatch(task);
        return;
      }

      // does the work of this module
      ++(*qdata.stamp);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  Config::ConstPtr config_;

  VTR_REGISTER_MODULE_DEC_TYPE(TestAsyncModuleDep1);
};

TEST(TaskExecutor, async_task_queue_dep1_multi_thread) {
  auto output = std::make_shared<OutputCache>();

  // create a task queue with 2 threads and queue length of 3
  auto executor = std::make_shared<TaskExecutor>(output, nullptr, 2, 3);

  // create a query cache
  auto qdata = std::make_shared<QueryCache>();
  qdata->stamp.emplace(0);

  // create a module factory to get module
  auto factory = std::make_shared<ModuleFactoryV2>();

  // get and run the module
  auto module = factory->get(TestAsyncModuleDep1::static_name);
  module->run(*qdata, *output, nullptr, executor);

  // wait until all tasks are done
  executor->wait();
  LOG(INFO) << "All tasks have finished!";
  LOG(INFO) << "Final qdata stamp: " << *qdata->stamp;
  EXPECT_EQ(*qdata->stamp, 3);

  // destructor will call join, which clears the queue stops all threads
}

TEST(TaskExecutor, async_task_queue_dep1_queue_full) {
  auto output = std::make_shared<OutputCache>();

  // create a task queue with 1 threads and queue length of 2
  auto executor = std::make_shared<TaskExecutor>(output, nullptr, 1, 2);

  // create a query cache
  auto qdata = std::make_shared<QueryCache>();
  qdata->stamp.emplace(0);

  // create a module factory to get module
  auto factory = std::make_shared<ModuleFactoryV2>();

  // get and run the module
  auto module = factory->get(TestAsyncModuleDep1::static_name);
  module->run(*qdata, *output, nullptr, executor);

  // wait until all tasks are done
  executor->wait();
  LOG(INFO) << "All tasks have finished!";
  LOG(INFO) << "Final qdata stamp: " << *qdata->stamp;
  // only 2 tasks are actually run, 1 discarded due to queue full
  EXPECT_EQ(*qdata->stamp, 2);

  // destructor will call join, which clears the queue stops all threads
}

TEST(TaskExecutor, async_task_queue_dep1_queue_full_stop) {
  auto output = std::make_shared<OutputCache>();

  // create a task queue with 2 threads and queue length of 4
  auto executor = std::make_shared<TaskExecutor>(output, nullptr, 2, 4);

  // create a query cache
  auto qdata = std::make_shared<QueryCache>();
  qdata->stamp.emplace(0);

  // create a module factory to get module
  auto factory = std::make_shared<ModuleFactoryV2>();

  // get and run the module
  auto module = factory->get(TestAsyncModuleDep1::static_name);
  module->run(*qdata, *output, nullptr, executor);

  // wait until all tasks are done
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  executor->stop();  // stop immediately to check if job can be discarded
  LOG(INFO) << "All tasks have finished!";
  LOG(INFO) << "Final qdata stamp: " << *qdata->stamp;

  // destructor will call join, which clears the queue stops all threads
}

int main(int argc, char** argv) {
  configureLogging("", true);
  InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
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
 * \file tactic_v2.cpp
 * \brief Tactic class method definition
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */

#include "vtr_tactic/tactic_v2.hpp"

namespace vtr {
namespace tactic {

PipelineInterface::PipelineInterface(const Graph::Ptr& graph,
                                     const size_t& num_async_threads,
                                     const size_t& async_queue_size)
    : task_queue_(std::make_shared<AsyncTaskExecutor>(graph, num_async_threads,
                                                      async_queue_size)) {
  // clang-format off
  preprocessing_thread_ = std::thread(&PipelineInterface::preprocess, this);
  odometry_mapping_thread_ = std::thread(&PipelineInterface::runOdometryMapping, this);
  localization_thread_ = std::thread(&PipelineInterface::runLocalization, this);
  // clang-format on
  task_queue_->start();
}

void PipelineInterface::join() {
  auto lck = lockPipeline();
  if (preprocessing_thread_.joinable()) {
    preprocessing_buffer_.push(nullptr, false);
    preprocessing_thread_.join();
  }
  if (odometry_mapping_thread_.joinable()) {
    odometry_mapping_buffer_.push(nullptr, false);
    odometry_mapping_thread_.join();
  }
  if (localization_thread_.joinable()) {
    localization_buffer_.push(nullptr, false);
    localization_thread_.join();
  }
  task_queue_->stop();
}

auto PipelineInterface::lockPipeline() -> PipelineLock {
  // Lock so that no more data are passed into the pipeline
  PipelineLock lock(pipeline_mutex_);
  // Wait for the pipeline to be empty
  pipeline_semaphore_.wait();
  return lock;
}

/** \brief Changes the pipeine behavior based on current operation mode */
void PipelineInterface::setPipeline(const PipelineMode& pipeline_mode) {
  auto lock = lockPipeline();
  pipeline_mode_ = pipeline_mode;
}

/** \brief Pipline entrypoint, gets query input from navigator */
void PipelineInterface::input(const QueryCache::Ptr& qdata) {
  PipelineLock lock(pipeline_mutex_, std::defer_lock_t());
  if (lock.try_lock_for(std::chrono::milliseconds(30))) {
    pipeline_semaphore_.release();
    CLOG(INFO, "tactic.concurrency") << "Accepting a new frame input.";
    const bool discarded = preprocessing_buffer_.push(qdata, true);
    CLOG_IF(discarded, INFO, "tactic.concurrency")
        << "[input] Buffer is full, one frame discarded.";
    if (discarded) pipeline_semaphore_.acquire();
  } else {
    CLOG(WARNING, "tactic.concurrency")
        << "Dropping frame due to unavailable pipeline mutex.";
  }
}

/** \brief Data preprocessing thread, input->preprocess->odo&mapping */
void PipelineInterface::preprocess() {
  el::Helpers::setThreadName("tactic.preprocessing");
  while (true) {
    auto qdata = preprocessing_buffer_.pop();
    if (qdata == nullptr) return;
    CLOG(INFO, "tactic.concurrency")
        << "Start running preprocessing: " << *qdata->stamp;
    const bool discardable = preprocess_(qdata);
    const bool discarded = odometry_mapping_buffer_.push(qdata, discardable);
    CLOG_IF(discarded, INFO, "tactic.concurrency")
        << "[preprocess] Buffer is full, one frame discarded.";
    if (discarded) pipeline_semaphore_.acquire();
    CLOG(INFO, "tactic.concurrency")
        << "Finish running preprocessing: " << *qdata->stamp;
  }
}

/** \brief Odometry & mapping thread, preprocess->odo&mapping->localization */
void PipelineInterface::runOdometryMapping() {
  el::Helpers::setThreadName("tactic.odometry_mapping");
  while (true) {
    auto qdata = odometry_mapping_buffer_.pop();
    if (qdata == nullptr) return;
    CLOG(INFO, "tactic.concurrency")
        << "Start running odometry mapping, timestamp: " << *qdata->stamp;
    const bool discardable = runOdometryMapping_(qdata);
    const bool discarded = localization_buffer_.push(qdata, discardable);
    CLOG_IF(discarded, INFO, "tactic.concurrency")
        << "[odometry_mapping] Buffer is full, one frame discarded.";
    if (discarded) pipeline_semaphore_.acquire();
    CLOG(INFO, "tactic.concurrency")
        << "Finish running odometry mapping, timestamp: " << *qdata->stamp;
  }
}

/** \brief Localization thread, odomtry&mapping->localization */
void PipelineInterface::runLocalization() {
  el::Helpers::setThreadName("tactic.localization");
  while (true) {
    auto qdata = localization_buffer_.pop();
    if (qdata == nullptr) return;
    CLOG(INFO, "tactic.concurrency")
        << "Start running localization, timestamp: " << *qdata->stamp;
    runLocalization_(qdata);
    CLOG(INFO, "tactic.concurrency")
        << "Finish running localization, timestamp: " << *qdata->stamp;
    pipeline_semaphore_.acquire();
  }
}

}  // namespace tactic
}  // namespace vtr
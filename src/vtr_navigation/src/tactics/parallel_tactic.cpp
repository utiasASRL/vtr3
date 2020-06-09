#include <vtr/navigation/tactics/parallel_tactic.h>

// #define _GLIBCXX_USE_NANOSLEEP 1

// #include <pthread.h>
// #include <condition_variable>
// #include <future>

// #include <asrl/messages/VOStatus.pb.h>
// #include <asrl/common/emotions.hpp>
// #include <asrl/navigation/pipelines/BasePipeline.hpp>
// #include <asrl/navigation/pipelines/PipelineFactory.hpp>

namespace asrl {
namespace navigation {

ParallelTactic::ParallelTactic(
    TacticConfig& config, const unsigned num_threads,
    const std::shared_ptr<ConverterAssembly>& converter,
    const std::shared_ptr<QuickVoAssembly>& quick_vo,
    const std::shared_ptr<RefinedVoAssembly>& refined_vo,
    // const std::shared_ptr<LocalizerAssembly>& localizer,
    // const std::shared_ptr<TerrainAssessmentAssembly>& terrain_assessment,
    std::shared_ptr<pose_graph::RCGraph /*Graph*/> graph)
    : BasicTactic::BasicTactic(config, converter, quick_vo,
                               refined_vo, /* localizer,
            terrain_assessment,*/
                               graph) {
#if 0
  // make these equivalent
  max_concurrent_jobs_ = num_threads;

  // make a new thread pool
  pool_.reset(new asrl::common::thread_pool(max_concurrent_jobs_, 0));

  // make sure the processing thread won't quit
  quit_flag_ = false;

  // default is that the process thread is not busy
  busy_ = false;

  // launch the processing thread to wait for new data from the converter
  process_thread_ = std::thread(&ParallelTactic::process, this);

  // update the scheduler priority
  sched_param sch_params;
  sch_params.sched_priority = sched_get_priority_max(SCHED_BATCH);
  if (pthread_setschedparam(process_thread_.native_handle(), SCHED_BATCH,
                            &sch_params)) {
    LOG(ERROR) << "Failed to set thread scheduling : " << std::strerror(errno);
  }
#endif
}

ParallelTactic::~ParallelTactic() {
  // this->halt();
}

#if 0
void ParallelTactic::halt() {
  // tell the processing thread to stop
  quit_flag_ = true;

  // notify the condition variables
  run_cv_.notify_all();
  process_cv_.notify_all();

  // wait for the pipeline to clear
  auto lck = lockPipeline();

  // wait for everything to join
  if (pool_ != nullptr) {
    pool_->wait();
    pool_->join();
    pool_.reset();
  }

  if (process_thread_.joinable()) {
    process_thread_.join();
  }

  if (!queue_.empty()) {
    LOG(WARNING) << __func__ << " quitting with " << queue_.size()
                 << " caches left in the queue";
  }

  // call the base class halt()
  BasicTactic::halt();

  return;
}

/// @brief Set the operational mode (which pipeline to use)
void ParallelTactic::setPipeline(const planning::PipelineType& pipeline) {
  // Lock to make sure all frames clear the pipeline
  LOG(DEBUG) << "[Lock Requested] setPipeline";
  auto lck = lockPipeline();
  LOG(DEBUG) << "[Lock Acquired] setPipeline";

  // We have to join the pool to prevent a race condition where setPipeline and
  // runPipeline are called at the same time
  if (pool_ != nullptr) {
    pool_->wait();
    pool_->join();
  }

  // Change the pipeline after we're sure everything is clear
  pipeline_ = PipelineFactory::make(pipeline, this);

  // start the pool
  if (pool_ == nullptr) {
    pool_.reset(new asrl::common::thread_pool(max_concurrent_jobs_, 0));
  }
  pool_->start();

  return;
}

void ParallelTactic::runPipeline(QueryCachePtr query_data) {
  // lock the pipeline so that it won't get changed before we process necessary
  // data through the queue
  {
    LockType pipelinelock(pipeline_mutex_);

    // make a new map cache
    MapCachePtr map_data(new MapCache);

    // add initial data to the cache
    setupCaches(query_data, map_data);

    // setup a thread to run the image converter on the thread pool
    std::function<void()> bound = std::bind(
        &BasePipeline::convertData, pipeline_.get(), query_data, map_data);
    std::future<void> res;
    try {
      res = std::move(pool_->dispatch(bound));
    } catch (const std::runtime_error& e) {
      return;
    }

    // Only put the jobs on the queue when we aren't processing too many jobs
    {
      std::unique_lock<std::mutex> mylock(queue_lock_);

      while (queue_.size() >= max_concurrent_jobs_) {
        run_cv_.wait(mylock);
      }

      // put the promise and caches on the queue to be processed
      queue_.emplace(std::make_tuple(std::move(res), query_data, map_data));
      // notify that there is data to be processed
      process_cv_.notify_one();
    }
  }

  return;
}

void ParallelTactic::wait() {
  while (busy_ || !queue_.empty()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  };

  if (keyframe_thread_future_.valid()) {
    keyframe_thread_future_.wait();
  }
  return;
}

void ParallelTactic::process(void) {
  // make sure the tactic instance hasn't yet been destroyed
  while (!quit_flag_ || !queue_.empty()) {
    // easy reference
    std::future<void> converter_future;
    QueryCachePtr query_data;
    MapCachePtr map_data;

    {
      std::unique_lock<std::mutex> mylock(queue_lock_);

      // wait for the data to be added to the queues
      while (queue_.empty() && quit_flag_ == false) {
        process_cv_.wait(mylock);
      }

      // check if we don't need to quit
      if (queue_.empty()) {
        continue;
      }

      // copy
      converter_future = std::move(std::get<0>(queue_.front()));
      query_data = std::get<1>(queue_.front());
      map_data = std::get<2>(queue_.front());

      // make sure to set busy flag before the queue is empty
      busy_ = true;
      // pop the data off the front because we don't need them now
      queue_.pop();
      run_cv_.notify_one();
    }

    // wait for the future to be given (or possibly destructed)
    try {
      converter_future.wait();

      if (!converter_future.valid()) {
        // the future is invalid, and this frame is no good
        busy_ = false;
        continue;
      } else {
        converter_future.get();
      }
    } catch (const std::future_error& e) {
      // the promise was broken or the future is invalid, and this frame is no
      // good
      busy_ = false;
      continue;
    }

    // check if we don't need to quit

    // setup and run the pipeline processData();
    processData(query_data, map_data);
    busy_ = false;
  }
}
#endif

}  // namespace navigation
}  // namespace asrl

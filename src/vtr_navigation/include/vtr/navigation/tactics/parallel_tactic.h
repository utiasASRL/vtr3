#pragma once

#include <condition_variable>
#include <tuple>

#include <vtr/navigation/tactics/basic_tactic.h>

#include <asrl/common/utils/thread_pool.hpp>

namespace vtr {
namespace navigation {

class ParallelTactic : public BasicTactic {
 public:
  ParallelTactic(
      TacticConfig& config, const unsigned num_threads,
      const std::shared_ptr<ConverterAssembly>& converter,
      const std::shared_ptr<QuickVoAssembly>& quick_vo,
      const std::shared_ptr<RefinedVoAssembly>& refined_vo,
      const std::shared_ptr<LocalizerAssembly>& localizer,
      // const std::shared_ptr<TerrainAssessmentAssembly>& terrain_assessment,
      std::shared_ptr<Graph> graph = nullptr);

  virtual ~ParallelTactic();

  /** \brief Calling halt stops all associated processes/threads
   */
  virtual void halt();

  /** \brief Run the pipeline on the data
   */
  virtual void runPipeline(QueryCachePtr query_data);

  /** \brief Set the operational mode (which pipeline to use)
   */
  virtual void setPipeline(const asrl::planning::PipelineType& pipeline);

  /** \brief Set the path being followed
   */
  virtual void setPath(const asrl::planning::PathType& path,
                       bool follow = false) {
    BasicTactic::setPath(path, follow);
  }

#if 0
  /** \brief Set the current privileged vertex (topological localization)
   */
  virtual void setTrunk(const VertexId& v) { BasicTactic::setTrunk(v); }

  /** \brief Add a new run to the graph and reset localization flags
   */
  virtual void addRun(bool ephemeral = false, bool extend = false) {
    BasicTactic::addRun(ephemeral, extend);
  }

  /** \brief Trigger a graph relaxation
   */
  virtual void relaxGraph() { BasicTactic::relaxGraph(); }

  /** \brief Save the graph
   */
  virtual void saveGraph() { BasicTactic::saveGraph(); }
#endif

  /** \brief A seperate thread to process data after pipeline convertData()
   */
  void process(void);

 protected:
  virtual void wait(void);

  /** \brief a flag to indicate to the process() thread when to quit
   */
  std::atomic<bool> quit_flag_;
  /** \brief a flag to say the processing thread is busy
   */
  std::atomic<bool> busy_;
  /** \brief a lock to coordinate adding/removing jobs from the queue
   */
  std::mutex queue_lock_;
  /** \brief notifies when new data is available on the queue
   */
  std::condition_variable process_cv_;
  /** \brief notifies when queue has room for new data
   */
  std::condition_variable run_cv_;
  /** \brief the thread to process data using the pipeline processData()
   */
  std::thread process_thread_;
  /** \brief the futures and caches returned by the parallel convertData()'s
   */
  std::queue<std::tuple<std::future<void>, QueryCachePtr, MapCachePtr>> queue_;
  /** \brief The pool of threads to process data in convertData()
   */
  std::unique_ptr<asrl::common::thread_pool> pool_;
  /** \brief the maximum number of jobs allowed on the queue
   */
  unsigned max_concurrent_jobs_;
};

}  // namespace navigation
}  // namespace vtr

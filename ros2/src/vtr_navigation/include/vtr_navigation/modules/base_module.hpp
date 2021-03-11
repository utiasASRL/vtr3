#pragma once

#include <mutex>

#include <vtr_logging/logging.hpp>  //  for debugging only
#include <vtr_navigation/caches.hpp>
#include <vtr_navigation/types.hpp>

namespace vtr {
namespace navigation {

class BaseModule {
 public:
  /** An unique identifier. Subclass should overwrite this. */
  static constexpr auto type_str_ = "module";

  BaseModule(std::string name = type_str_) : name_{name} {};

  virtual ~BaseModule(){};

  /**
   * \brief Get the identifier of the module instance at runtime.
   * \details The identifier is the string passed to the BaseModule constructor.
   */
  const std::string &getName() const {
    return name_;
  };

  /**
   * \brief Runs the module with necessary timing or other type of monitoring.
   */
  void runWrapper(QueryCache &qdata, MapCache &mdata,
                  const std::shared_ptr<const Graph> &graph) {
    LOG(DEBUG) << "\033[1;31mRunning module: " << getName() << "\033[0m";
    run(qdata, mdata, graph);
    LOG(DEBUG) << "Finished running module: " << getName();
  }

  /**
   * \brief Update the graph with the frame data for the live vertex
   * \details \todo This function should replace the actual updateGraph
   * function.
   */
  void updateGraphWrapper(QueryCache &qdata, MapCache &mdata,
                          const std::shared_ptr<Graph> &graph,
                          VertexId live_id) {
    LOG(DEBUG) << "\033[1;32mUpdating graph module: " << getName() << "\033[0m";
    updateGraph(qdata, mdata, graph, live_id);
    LOG(DEBUG) << "Finished updating graph module: " << getName();
  }

  /** \brief Visualize data in this module. */
  void visualizeWrapper(QueryCache &qdata, MapCache &mdata,
                        const std::shared_ptr<const Graph> &graph) {
    LOG(DEBUG) << "\033[1;33mVisualizing module: " << getName() << "\033[0m";
    visualize(qdata, mdata, graph, vis_mtx_);
    LOG(DEBUG) << "Finished visualizing module: " << getName();
  }

 protected:
  /**
   * \brief Localize the frame data against the map vertex using the (sub)graph
   */
  virtual void run(QueryCache &qdata, MapCache &mdata,
                   const std::shared_ptr<const Graph> &graph) = 0;

  /**
   * \brief Updates the graph with the frame data for the live vertex. Subclass
   * should override this method.
   */
  virtual void updateGraph(QueryCache &, MapCache &,
                           const std::shared_ptr<Graph> &, VertexId){};

  /** \brief Visualization */
  virtual void visualize(QueryCache &, MapCache &,
                         const std::shared_ptr<const Graph> &, std::mutex &){};

 private:
  const std::string name_;

  /** \brief mutex to ensure thread safety with OpenCV HighGui calls */
  static std::mutex vis_mtx_;
};

}  // namespace navigation
}  // namespace vtr

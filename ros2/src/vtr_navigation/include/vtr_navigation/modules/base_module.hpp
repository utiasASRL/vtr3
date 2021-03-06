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

  BaseModule(std::string name = type_str_) : name_{name} {}

  virtual ~BaseModule() {}

  /**
   * \brief Get the identifier of the module instance at runtime.
   * \details The identifier is the string passed to the BaseModule constructor.
   */
  const std::string &getName() const { return name_; };

  /**
   * \brief Runs the module with necessary timing or other type of monitoring.
   * \todo This function should replace the actual run function, and the current
   * run function should be changed to runImpl. This allows us to do something
   * that's needed by all module runs, such as dumping out debug messages.
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
  void visualize(QueryCache &qdata, MapCache &mdata,
                 const std::shared_ptr<const Graph> &graph) {
    LOG(DEBUG) << "\033[1;33mVisualizing module: " << getName() << "\033[0m";
    visualizeImpl(qdata, mdata, graph, vis_mtx_);
    LOG(DEBUG) << "Finished visualizing module: " << getName();
  }

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
                           const std::shared_ptr<Graph> &, VertexId) {}

 private:
  const std::string name_;

  /** \brief mutex to ensure thread safety with OpenCV HighGui calls */
  static std::mutex vis_mtx_;

  /** \brief Visualization implementation */
  virtual void visualizeImpl(QueryCache &, MapCache &,
                             const std::shared_ptr<const Graph> &,
                             std::mutex &){};

  // /// Assemblies built by assembly builders
  // friend class BaseModuleFactory;  // Note sure if this is needed.
};

}  // namespace navigation
}  // namespace vtr

#pragma once

#include <vtr/navigation/caches.h>
#include <vtr/navigation/types.h>

#include <asrl/common/logging.hpp>  //  for debugging only
#include <mutex>

// namespace cv {
// class Mat;
// }

namespace asrl {
namespace navigation {

class BaseModule {
 public:
  /** An unique identifier. Subclass should overwrite this.
   */
  static constexpr auto type_str_ = "module";

  BaseModule(std::string name = type_str_) : name_{name} {}

  virtual ~BaseModule() {}

  /** \brief Get the identifier of the module instance at runtime.
   *
   * The identifier is the string passed to the BaseModule constructor.
   */
  const std::string &getName() const { return name_; };

  /** \brief Run the module.
   *
   * \todo This function should replace the actual run function, and the current
   * run function should be changed to runImpl. This allows us to do something
   * that's needed by all module runs, such as dumping out debug messages.
   */
  void runWrapper(QueryCache &qdata, MapCache &mdata,
                  const std::shared_ptr<const Graph> &graph) {
    LOG(DEBUG) << "Running module: " << getName();
    run(qdata, mdata, graph);
    LOG(DEBUG) << "Finished running module: " << getName();
  }

  /** \brief Update the graph with the frame data for the live vertex
   *
   * \todo This function should replace the actual updateGraph function.
   */
  void updateGraphWrapper(QueryCache &qdata, MapCache &mdata,
                          const std::shared_ptr<Graph> &graph,
                          VertexId live_id) {
    LOG(DEBUG) << "Updating graph module: " << getName();
    updateGraph(qdata, mdata, graph, live_id);
    LOG(DEBUG) << "Finished updating graph module: " << getName();
  }

  /** \brief Visualize data in this module.
   */
  void visualize(QueryCache &qdata, MapCache &mdata,
                 const std::shared_ptr<const Graph> &graph) {
    LOG(DEBUG) << "Visualizing module: " << getName();
    visualizeImpl(qdata, mdata, graph, vis_mtx_);
    LOG(DEBUG) << "Finished visualizing module: " << getName();
    return;
  }

  /** \brief Localize the frame data against the map vertex using the
   * (sub)graph
   */
  virtual void run(QueryCache &qdata, MapCache &mdata,
                   const std::shared_ptr<const Graph> &graph) = 0;

  /** \brief Update the graph with the frame data for the live vertex */
  virtual void updateGraph(QueryCache &qdata, MapCache &mdata,
                           const std::shared_ptr<Graph> &graph,
                           VertexId live_id) = 0;

 private:
  const std::string name_;

  /** \brief mutex to ensure thread safety with OpenCV HighGui calls
   */
  static std::mutex vis_mtx_;

  /** \brief Visualization implementation
   */
  virtual void visualizeImpl(QueryCache &, MapCache &,
                             const std::shared_ptr<const Graph> &,
                             std::mutex &){};

  // /// Assemblies built by assembly builders
  // friend class BaseModuleFactory;  // Note sure if this is needed.
};

#if 0
class NoopModule : public BaseModule {
 public:
  static constexpr auto type_str_ = "noop";

  virtual void run(QueryCache &qdata, MapCache &mdata,
                   const std::shared_ptr<const Graph> &graph) {
    (void)&qdata;
    (void)&mdata, (void)&graph;
  }

  virtual void updateGraph(QueryCache &qdata, MapCache &mdata,
                           const std::shared_ptr<Graph> &graph,
                           VertexId live_id) {
    (void)&qdata;
    (void)&mdata, (void)&graph;
    (void)&live_id;
  }
};

class VisualizerModule : public BaseModule {
  /** \brief Visualize the vo/localization data
   */
  virtual void run(QueryCache &, MapCache &,
                   const std::shared_ptr<const Graph> &) {
    // TODO visualize
  }
  virtual void updateGraph(QueryCache &, MapCache &,
                           const std::shared_ptr<Graph> &, VertexId) {}
};

#endif

}  // namespace navigation
}  // namespace asrl

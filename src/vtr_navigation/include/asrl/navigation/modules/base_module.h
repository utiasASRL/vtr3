#pragma once

#include <mutex>

#include <asrl/navigation/caches.h>
#include <asrl/navigation/types.h>

// namespace cv {
// class Mat;
// }

namespace asrl {
namespace navigation {

class BaseModule {
 public:
  BaseModule(void) {}
  virtual ~BaseModule() {}

  /** \brief Localize the frame data against the map vertex using the (sub)graph
   */
  virtual void run(QueryCache &qdata, MapCache &mdata,
                   const std::shared_ptr<const Graph> &graph) = 0;

  /** \brief Update the graph with the frame data for the live vertex */
  virtual void updateGraph(QueryCache &qdata, MapCache &mdata,
                           const std::shared_ptr<Graph> &graph,
                           VertexId live_id) = 0;

  /** \brief Visualize data in this module.
   */
  void visualize(QueryCache &qdata, MapCache &mdata,
                 const std::shared_ptr<const Graph> &graph) {
    // run the derived class visualizer
    visualizeImpl(qdata, mdata, graph, vis_mtx_);

    return;
  }

 private:
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

// class NoopModule : public BaseModule {
//  public:
//   static constexpr auto type_str_ = "noop";

//   /// @brief Visualize the vo/localization data
//   virtual void run(QueryCache &qdata, MapCache &mdata,
//                    const std::shared_ptr<const Graph> &graph) {
//     (void)&qdata;
//     (void)&mdata, (void)&graph;
//   }
//   virtual void updateGraph(QueryCache &qdata, MapCache &mdata,
//                            const std::shared_ptr<Graph> &graph,
//                            VertexId live_id) {
//     (void)&qdata;
//     (void)&mdata, (void)&graph;
//     (void)&live_id;
//   }
// };

// class VisualizerModule : public BaseModule {
//   /// @brief Visualize the vo/localization data
//   virtual void run(QueryCache &, MapCache &,
//                    const std::shared_ptr<const Graph> &) {
//     // TODO visualize
//   }
//   virtual void updateGraph(QueryCache &, MapCache &,
//                            const std::shared_ptr<Graph> &, VertexId) {}
// };

}  // namespace navigation
}  // namespace asrl

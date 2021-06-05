#pragma once

#include <mutex>

#include <vtr_logging/logging.hpp>  // for debugging only
#include <vtr_tactic/caches.hpp>
#include <vtr_tactic/types.hpp>

namespace vtr {
namespace tactic {

class BaseModule {
 public:
  using Ptr = std::shared_ptr<BaseModule>;

  /** An unique identifier. Subclass should overwrite this. */
  static constexpr auto static_name = "module";

  BaseModule(const std::string &name = static_name) : name_{name} {};

  virtual ~BaseModule(){};

  /**
   * \brief Get the identifier of the module instance at runtime.
   * \details The identifier is the string passed to the BaseModule constructor.
   */
  const std::string &getName() const { return name_; };

  /**
   * \brief Runs the module with necessary timing or other type of monitoring.
   */
  void initialize(MapCache &mdata, const Graph::ConstPtr &graph) {
    LOG(DEBUG) << "\033[1;31mInitializing module: " << getName() << "\033[0m";
    initializeImpl(mdata, graph);
    LOG(DEBUG) << "Finished initializing module: " << getName();
  }

  /**
   * \brief Runs the module with necessary timing or other type of monitoring.
   */
  void run(QueryCache &qdata, MapCache &mdata, const Graph::ConstPtr &graph) {
    LOG(DEBUG) << "\033[1;31mRunning module: " << getName() << "\033[0m";
    runImpl(qdata, mdata, graph);
    LOG(DEBUG) << "Finished running module: " << getName();
  }

  /**
   * \brief Update the graph with the frame data for the live vertex
   * \details \todo This function should replace the actual updateGraph
   * function.
   */
  void updateGraph(QueryCache &qdata, MapCache &mdata, const Graph::Ptr &graph,
                   VertexId live_id) {
    LOG(DEBUG) << "\033[1;32mUpdating graph module: " << getName() << "\033[0m";
    updateGraphImpl(qdata, mdata, graph, live_id);
    LOG(DEBUG) << "Finished updating graph module: " << getName();
  }

  /** \brief Visualize data in this module. */
  void visualize(QueryCache &qdata, MapCache &mdata,
                 const Graph::ConstPtr &graph) {
    LOG(DEBUG) << "\033[1;33mVisualizing module: " << getName() << "\033[0m";
    visualizeImpl(qdata, mdata, graph, vis_mtx_);
    LOG(DEBUG) << "Finished visualizing module: " << getName();
  }

  virtual void configFromROS(const rclcpp::Node::SharedPtr &,
                             const std::string) {}

 private:
  /**
   * \brief Localize the frame data against the map vertex using the
   * (sub)graph
   */
  virtual void initializeImpl(MapCache &, const Graph::ConstPtr &){};

  /**
   * \brief Localize the frame data against the map vertex using the (sub)graph
   */
  virtual void runImpl(QueryCache &qdata, MapCache &mdata,
                       const Graph::ConstPtr &graph) = 0;

  /**
   * \brief Updates the graph with the frame data for the live vertex. Subclass
   * should override this method.
   */
  virtual void updateGraphImpl(QueryCache &, MapCache &, const Graph::Ptr &,
                               VertexId){};

  /** \brief Visualization */
  virtual void visualizeImpl(QueryCache &, MapCache &, const Graph::ConstPtr &,
                             std::mutex &){};

 private:
  /** \brief Name of the module assigned at runtime. */
  const std::string name_;

  /** \brief Mutex to ensure thread safety with OpenCV HighGui calls */
  static std::mutex vis_mtx_;
};

}  // namespace tactic
}  // namespace vtr

#pragma once

#include <asrl/messages/Matches.pb.h>
#include <vtr/navigation/modules/base_module.h>
#include <vtr/navigation/modules/miscellaneous/vertex_creation_test_module.h>
//#include <asrl/vision/matching/SensorModel/SensorModelBase.h>
//#include <asrl/vision/matching/Sampler/BasicSampler.h>

namespace vtr {
namespace navigation {

/** \brief Reject outliers and estimate a preliminary transform
 */
class SimpleVertexTestModule : public VertexCreationModule {
 public:
  static constexpr auto type_str_ = "simple_vertex_creation_test";
  struct Config : VertexCreationModule::Config {
    double min_creation_distance;
    double max_creation_distance;
    double min_distance;
    double rotation_threshold_min;
    double rotation_threshold_max;
    int match_threshold_min_count;
    int match_threshold_fail_count;
  };
  /** \brief TODO Construct with settings...
   */
  SimpleVertexTestModule(std::string name = type_str_)
      : VertexCreationModule{name} {}

  /** \brief Given two frames and matches detects the inliers that fit the given
   * model, and provides an initial guess at transform T_q_m.
   */
  virtual void run(QueryCache &qdata, MapCache &mdata,
                   const std::shared_ptr<const Graph> &graph);

  /** \brief Update the graph with the frame data for the live vertex
   */
  virtual void updateGraph(QueryCache &qdata, MapCache &mdata,
                           const std::shared_ptr<Graph> &graph,
                           VertexId live_id);

  void setConfig(std::shared_ptr<Config> &config);

 protected:
 private:
  /** \brief Algorithm Configuration
   */
  std::shared_ptr<Config> simple_config_;
};

}  // namespace navigation
}  // namespace vtr

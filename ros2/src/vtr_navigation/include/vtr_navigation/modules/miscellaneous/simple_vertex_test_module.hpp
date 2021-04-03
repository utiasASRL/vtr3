#pragma once

#include <vtr_navigation/modules/base_module.hpp>
#include <vtr_navigation/modules/miscellaneous/vertex_creation_test_module.hpp>

namespace vtr {
namespace navigation {

/** 
 * \brief A module that determines whether a new vertex should be created.
 * \details
 * requires: 
 *   qdata.[]
 *   mdata.[ransac_matches]
 * outputs: 
 *   mdata.[new_vertex_flag, success]
 */
class SimpleVertexTestModule : public VertexCreationModule {
 public:
  /**
   * \brief Static module identifier.
   * \todo change this to static_name
   */
  static constexpr auto type_str_ = "simple_vertex_creation_test";

  /** \brief Collection of config parameters */
  struct Config : VertexCreationModule::Config {
    double min_creation_distance;
    double max_creation_distance;
    double min_distance;
    double rotation_threshold_min;
    double rotation_threshold_max;
    int match_threshold_min_count;
    int match_threshold_fail_count;
  };

  SimpleVertexTestModule(std::string name = type_str_)
      : VertexCreationModule{name} {
  }

  void setConfig(std::shared_ptr<Config> &config);

 protected:
  /**
   * \brief Given two frames and matches detects the inliers that fit the given
   * model, and provides an initial guess at transform T_q_m.
   */
  void run(QueryCache &qdata, MapCache &mdata,
           const std::shared_ptr<const Graph> &graph) override;

 private:
  /** \brief Module configuration. */
  std::shared_ptr<Config> simple_config_;
};

}  // namespace navigation
}  // namespace vtr

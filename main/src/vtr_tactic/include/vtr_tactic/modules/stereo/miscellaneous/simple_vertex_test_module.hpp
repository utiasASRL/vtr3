#pragma once

#include <vtr_tactic/modules/base_module.hpp>
#include <vtr_tactic/modules/stereo/miscellaneous/vertex_creation_test_module.hpp>

namespace vtr {
namespace tactic {
namespace stereo {

/**
 * \brief A module that determines whether a new vertex should be created.
 * \details
 * requires:
 *   qdata.[]
 *   mdata.[ransac_matches, steam_failure, T_q_m]
 * outputs:
 *   mdata.[new_vertex_flag, success]
 */
class SimpleVertexTestModule : public VertexCreationModule {
 public:
  /** \brief Static module identifier. */
  static constexpr auto static_name = "simple_vertex_creation_test";

  /** \brief Collection of config parameters */
  struct Config : VertexCreationModule::Config {
    double min_creation_distance;
    double max_creation_distance;
    double rotation_threshold_min;
    double rotation_threshold_max;
    double min_distance;
    int match_threshold_min_count;
    int match_threshold_fail_count;
  };

  SimpleVertexTestModule(std::string name = static_name)
      : VertexCreationModule{name} {}

  void configFromROS(const rclcpp::Node::SharedPtr &node,
                     const std::string param_prefix) override;

 protected:
  /**
   * \brief Given two frames and matches detects the inliers that fit the given
   * model, and provides an initial guess at transform T_q_m.
   */
  void runImpl(QueryCache &qdata, MapCache &mdata,
               const Graph::ConstPtr &graph) override;

 private:
  /** \brief Module configuration. */
  std::shared_ptr<Config> simple_config_;
};

}  // namespace stereo
}  // namespace tactic
}  // namespace vtr
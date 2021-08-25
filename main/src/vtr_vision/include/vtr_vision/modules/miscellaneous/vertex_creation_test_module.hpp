/**
 * \file vertex_creation_test_module.hpp
 * \brief VertexCreationModule class definition
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <vtr_tactic/modules/base_module.hpp>
#include <vtr_vision/cache.hpp>

namespace vtr {
namespace vision {

/** \brief Reject outliers and estimate a preliminary transform */
class VertexCreationModule : public tactic::BaseModule {
 public:
  /**
   * \brief Static module identifier.
   * \todo change this to static_name
   */
  static constexpr auto static_name = "vertex_creation";

  /** \brief Collection of config parameters */
  struct Config {
    double distance_threshold_min;
    double distance_threshold_max;
    double rotation_threshold_min;
    double rotation_threshold_max;
    int match_threshold_min_count;
    int match_threshold_fail_count;
  };

  VertexCreationModule(const std::string &name = static_name)
      : tactic::BaseModule{name}, config_(std::make_shared<Config>()){};

 protected:
  /** \brief Module configuration. */
  std::shared_ptr<Config> config_;
};

}  // namespace vision
}  // namespace vtr

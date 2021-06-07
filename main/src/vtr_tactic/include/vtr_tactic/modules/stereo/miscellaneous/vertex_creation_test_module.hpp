#pragma once

#include <vtr_tactic/modules/base_module.hpp>

namespace vtr {
namespace tactic {
namespace stereo {

/** \brief Reject outliers and estimate a preliminary transform */
class VertexCreationModule : public BaseModule {
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
      : BaseModule{name}, config_(std::make_shared<Config>()){};

 protected:
  /** \brief Module configuration. */
  std::shared_ptr<Config> config_;
};

}  // namespace stereo
}  // namespace tactic
}  // namespace vtr
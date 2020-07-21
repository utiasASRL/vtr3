#pragma once

#include <asrl/messages/Matches.pb.h>
#include <vtr/navigation/modules/base_module.h>

namespace vtr {
namespace navigation {

/** \brief Reject outliers and estimate a preliminary transform
 */
class VertexCreationModule : public BaseModule {
 public:
  /** \brief Static module identifier.
   *
   * \todo change this to static_name
   */
  static constexpr auto type_str_ = "vertex_creation";

  /** \brief Collection of config parameters
   */
  struct Config {
    double distance_threshold_min;
    double distance_threshold_max;
    double rotation_threshold_min;
    double rotation_threshold_max;
    int match_threshold_min_count;
    int match_threshold_fail_count;
  };

  VertexCreationModule(std::string name = type_str_) : BaseModule{name} {}

  void setConfig(std::shared_ptr<Config> &config) { config_ = config; }

 protected:
 private:
  /** \brief Module configuration.
   */
  std::shared_ptr<Config> config_;
};

}  // namespace navigation
}  // namespace vtr

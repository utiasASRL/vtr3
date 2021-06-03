#pragma once

#include <vtr_tactic/modules/base_module.hpp>
#include <vtr_tactic/types.hpp>

namespace vtr {
namespace tactic {

/** \brief Preprocess raw pointcloud points and compute normals */
class KeyframeTestModule : public BaseModule {
 public:
  /** \brief Static module identifier. */
  static constexpr auto static_name = "keyframe_test";

  /** \brief Collection of config parameters */
  struct Config {
    float min_translation = 10;
    float min_rotation = 30;
  };

  KeyframeTestModule(const std::string &name = static_name)
      : BaseModule{name}, config_(std::make_shared<Config>()){};

  void setConfig(std::shared_ptr<Config> &config) {
    config_ = config;
  }

 private:
  void runImpl(QueryCache &qdata, MapCache &mdata,
               const Graph::ConstPtr &graph) override;

  /** \brief Module configuration. */
  std::shared_ptr<Config> config_;
};

}  // namespace tactic
}  // namespace vtr
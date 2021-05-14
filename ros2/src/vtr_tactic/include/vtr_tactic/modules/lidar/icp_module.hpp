#pragma once

#include "vtr_lidar/icp/icp.h"

#include <vtr_tactic/modules/base_module.hpp>

namespace vtr {
namespace tactic {

/** \brief Preprocess raw pointcloud points and compute normals */
class ICPModule : public BaseModule {
 public:
  /** \brief Static module identifier. */
  static constexpr auto static_name = "icp";

  /** \brief Collection of config parameters */
  struct Config : public ICP_params {
    std::string source = "live";
  };

  ICPModule(const std::string &name = static_name)
      : BaseModule{name}, config_(std::make_shared<Config>()){};

  void setConfig(std::shared_ptr<Config> &config) { config_ = config; }

 private:
  void runImpl(QueryCache &qdata, MapCache &mdata,
               const Graph::ConstPtr &graph) override;

  /** \brief Module configuration. */
  std::shared_ptr<Config> config_;
};

}  // namespace tactic
}  // namespace vtr
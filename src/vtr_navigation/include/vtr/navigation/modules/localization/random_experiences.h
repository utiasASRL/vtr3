#pragma once

#include <vtr/navigation/modules/base_module.h>
#include <vtr/navigation/modules/localization/experience_triage.h>

namespace vtr {
namespace navigation {

/// Selects some random experiences to use for localization
class RandomExperiencesModule : public BaseModule {
 public:
  template <class T>
  using Ptr = std::shared_ptr<T>;

  /// Module name
  static constexpr auto type_str_ = "random_experiences";

  /// Module Configuration
  struct Config {
    bool in_the_loop = true;
    bool verbose = false;
    int num_exp = 10;
  };

  RandomExperiencesModule(std::string name = type_str_) : BaseModule{name} {}

  /// Masks the localization map by the experience mask generated from upstream
  /// recommenders
  virtual void run(QueryCache& qdata,  ///< Information about the live image
                   MapCache& mdata,    ///< Information about the map
                   const std::shared_ptr<const Graph>&
                       graph);  ///< The spatio-temporal pose graph

  /// Saves the status message to the pose graph
  virtual void updateGraph(
      QueryCache& qdata,  ///< Information about the live image
      MapCache& mdata,    ///< Information about the map
      const std::shared_ptr<Graph>& graph,  ///< The pose grpah
      VertexId live_id);                    ///< The live vertex id

  /// Sets the module configuration.
  void setConfig(const Config& config) {  ///< The new config
    config_ = config;
  }

 private:
  /// The status message to save to the graph
  asrl::status_msgs::ExpRecogStatus status_msg_;

  /// The module configuration
  Config config_;
};

}  // namespace navigation
}  // namespace vtr

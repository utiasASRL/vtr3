#pragma once

#include <vtr_tactic/modules/base_module.hpp>

namespace vtr {
namespace tactic {

/** \brief Reject outliers and estimate a preliminary transform */
class TemplateModule : public BaseModule {
 public:
  /** \brief Static module identifier. */
  static constexpr auto static_name = "template";

  /** \brief Collection of config parameters */
  struct Config {
    std::string parameter = "default value";
  };

  TemplateModule(const std::string &name = static_name)
      : BaseModule{name}, config_(std::make_shared<Config>()){};

  void configFromROS(const rclcpp::Node::SharedPtr &node,
                     const std::string param_prefix) override {
    /// Configure your module from ROS
    config_ = std::make_shared<Config>();
    // clang-format off
    config_->parameter = node->declare_parameter<std::string>(param_prefix + ".parameter", config_->parameter);
    // clang-format on
    LOG(INFO) << "Template module parameter set to: " << config_->parameter;
  }

 private:
  void runImpl(QueryCache &, MapCache &, const Graph::ConstPtr &) override {
    /// Pure virtual method that must be overriden.
    /// Do the actual work of your module. Load data from and store data to
    /// QueryCache.
    /// \todo remove MapCache
    LOG(INFO) << "Running the template module...";
  }

  void updateGraphImpl(QueryCache &, MapCache &, const Graph::Ptr &,
                       VertexId) override {
    /// Override this method if your module needs to store data into the graph.
    LOG(INFO) << "Template module is updating the pose graph...";
  }

  void visualizeImpl(QueryCache &, MapCache &, const Graph::ConstPtr &,
                     std::mutex &) {
    /// Override thsi method if you module produces visualization. The mutex is
    /// for OpenCV.
    LOG(INFO) << "Template module is being visualized...";
  }

  /** \brief Module configuration. */
  std::shared_ptr<Config> config_;
};

}  // namespace tactic
}  // namespace vtr
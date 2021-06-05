#pragma once

#include <vtr_tactic/pipelines/base_pipeline.hpp>

namespace vtr {
namespace tactic {

class TemplatePipeline : public BasePipeline {
 public:
  using Ptr = std::shared_ptr<TemplatePipeline>;

  /** \brief Static pipeline identifier. */
  static constexpr auto static_name = "template";

  /** \brief Collection of config parameters */
  struct Config {
    std::string parameter = "default value";
  };

  TemplatePipeline(const std::string &name = static_name)
      : BasePipeline{name} {}

  virtual ~TemplatePipeline() {}

  void configFromROS(const rclcpp::Node::SharedPtr &node,
                     const std::string &param_prefix) override {
    /// Sets up pipeline config using parameters from the ROS parameter server.
    config_ = std::make_shared<Config>();
    // clang-format off
    config_->parameter = node->declare_parameter<std::string>(param_prefix + ".parameter", config_->parameter);
    // clang-format on
  }

  /** \brief initializes the pipeline data */
  void initialize(const Graph::Ptr &graph) override {
    /// Perform necessary initialization of the pipeline, e.g., create and
    /// initialize modules.
    /// Pose-graph is given but may be an empty graph.
    (void)graph;
  }

  /** \brief Preprocesses input data */
  void preprocess(QueryCache::Ptr &qdata, const Graph::Ptr &) override {
    /// This method is called on every input data.
    /// The following will be in qdata:
    ///   - input data (raw)
    ///   - stamp: time stamp of this data.
    ///   - node: a shared pointer to ROS node that can be used to create
    ///   publishers for visualization
    /// Any processed data (e.g. features) should be put in qdata.
    /// This method should not touch the pose graph.
    /// Any data preprocessing module should not touch the pose graph.
    (void)qdata;
  }

  void runOdometry(QueryCache::Ptr &, const Graph::Ptr &) override {
    /// This method is called on every preprocessed input data.
    /// The following will be in qdata:
    ///   - everything from preprocessing.
    ///   - first_frame: the first data received for the current run (teach or
    ///   repeat).
    ///   - live_id: the current vertex being localized against for odometry, or
    ///   invalid if first frame.
    ///   - T_r_m_odo: odometry estimation from last input, or identity if this
    ///   is the first frame or a keyframe has just been created.
    ///   - odo_success: whether or not odometry estimation is successful
    ///   - keyframe_test_result: whether or not to create a new keyframe,
    ///   always default to NO.
    /// This method should update the following:
    ///   - T_r_m_odo, odo_success, keyframe_test_result
    /// This method should only read from the graph.
    /// Any debug info, extra stuff can be put in qdata.
  }
  void visualizeOdometry(QueryCache::Ptr &, const Graph::Ptr &) override {
    /// This method is always called right after runOdometry.
  }

  void runLocalization(QueryCache::Ptr &, const Graph::Ptr &) override {
    /// This method is called in the following cases:
    ///   - first keyframe of a teach that branches from existing path to
    ///   localize against the existing path (i.e., trunk)
    ///   - every frame when merging into existing graph to create a loop
    ///   - every frame when doing metric localization (e.g. before path
    ///   following)
    ///   - every keyframe when repeating a path
    /// The following will be in qdata:
    ///   - everything from odometry and processKeyframe.
    ///   - map_id: the vertex to be localized against by this method.
    ///   - T_r_m_loc: prior estimate from localization chain based on odometry.
    ///   - loc_success: whether or not localization is successful.
    /// This method should update the following:
    ///   - T_r_m_loc, loc_success
    /// This method may read from or write to the graph.
  }

  void visualizeLocalization(QueryCache::Ptr &, const Graph::Ptr &) override {
    /// This method is always called right after runLocalization.
  }

  void processKeyframe(QueryCache::Ptr &, const Graph::Ptr &,
                       VertexId) override {
    /// This method is called whenever is keyframe is created.
    /// The following will be in qdata:
    ///   - everything from odometry
    ///   - live_id: always the vertex corresponding to the keyframe just
    ///   created
    /// This method may read from or write to the graph.
  }

 private:
  /** \brief Pipeline configuration */
  std::shared_ptr<Config> config_ = std::make_shared<Config>();
};

}  // namespace tactic
}  // namespace vtr

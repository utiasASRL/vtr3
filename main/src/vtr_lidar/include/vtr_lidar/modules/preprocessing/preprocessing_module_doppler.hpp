// Copyright 2021, Autonomous Space Robotics Lab (ASRL)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * \file preprocessing_module.hpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "sensor_msgs/msg/point_cloud2.hpp"

#include "vtr_lidar/cache.hpp"
#include "vtr_tactic/modules/base_module.hpp"
#include "vtr_tactic/task_queue.hpp"

#include <fstream>
#include <iostream>

namespace vtr {
namespace lidar {

/** \brief Preprocess raw pointcloud points and compute normals */
class PreprocessingDopplerModule : public tactic::BaseModule {
 public:
  using PointCloudMsg = sensor_msgs::msg::PointCloud2;

  /** \brief Static module identifier. */
  static constexpr auto static_name = "lidar.preprocessing_doppler";

  /** \brief Config parameters. */
  struct Config : public tactic::BaseModule::Config {
    PTR_TYPEDEFS(Config);

    // DOPPLER
    std::string root_path;
    std::string model_name;
    int downsample_steps = 1;
    //
    std::vector<std::string> bias_input_feat;
    std::vector<std::string> var_input_feat;
    int bias_polyorder;
    int var_polyorder;
    //
    double azimuth_res;   // rad
    double azimuth_start;
    double azimuth_end;
    mutable int num_rows = 80;
    mutable int num_cols = 501;   
    double min_dist = 20.0;
    double max_dist = 150.0;
    //
    mutable bool calc_median = false;
    mutable bool calc_pseudovar = false;

    bool visualize = false;

    static ConstPtr fromROS(const rclcpp::Node::SharedPtr &node,
                            const std::string &param_prefix);
  };
  
  PreprocessingDopplerModule(
      const Config::ConstPtr &config,
      const std::shared_ptr<tactic::ModuleFactory> &module_factory = nullptr,
      const std::string &name = static_name);

 protected:
  // calibration model weights
  using ImgWeight = std::vector<std::vector<Eigen::VectorXd>>; // (elevation) x (azimuth) x (weight dim)
  std::vector<std::vector<ImgWeight>> bias_weights_; // (# sensors) x (# face ids)
  std::vector<std::vector<ImgWeight>> var_weights_; // (# sensors) x (# face ids)
  std::vector<std::string> bias_features_;
  std::vector<std::string> var_features_;
  int bias_porder_;
  int var_porder_;
  bool calc_dop_median_ = false;
  bool calc_pseudo_var_ = false;
  int pseudo_var_hwidth_ = 5;

  void initImgWeight(bool set_dims, const Config::ConstPtr &config, const std::string& dim_txt, const std::string& binary, std::vector<std::vector<ImgWeight>>& weights);
  void buildFeatVec(Eigen::VectorXd& feat, const PointWithInfo& point, const std::vector<std::string>& feat_string, 
                    double dop_median_, double dop_pseudovar) const;
  double computeModel(const Eigen::VectorXd& feat, const Eigen::VectorXd& weights, int polyorder) const;
  bool computePseudovar(double& pseudovar, const std::vector<const PointWithInfo*>& img_row, int c, int hwidth, double tol) const;

  std::vector<std::vector<Eigen::MatrixXd>> weights;

 private:
  void run_(tactic::QueryCache &qdata, tactic::OutputCache &output,
            const tactic::Graph::Ptr &graph,
            const tactic::TaskExecutor::Ptr &executor) override;

  Config::ConstPtr config_;

  /** \brief for visualization only */
  bool publisher_initialized_ = false;
  rclcpp::Publisher<PointCloudMsg>::SharedPtr filtered_pub_;

  VTR_REGISTER_MODULE_DEC_TYPE(PreprocessingDopplerModule);
};

}  // namespace lidar
}  // namespace vtr
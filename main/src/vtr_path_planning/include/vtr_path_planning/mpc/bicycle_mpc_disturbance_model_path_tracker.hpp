// Copyright 2026, Autonomous Space Robotics Lab (ASRL)
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
 * \file unicycle_mpc_path_tracker.hpp
 * \author Luka Antonyshyn Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <vtr_logging/logging.hpp>
#include <vtr_tactic/types.hpp>
#include <vtr_path_planning/mpc/bicycle_mpc_path_tracker.hpp>
#include <vtr_path_planning/mpc/casadi_path_planners.hpp>
#include <vtr_path_planning/mpc/speed_scheduler.hpp>
#include <iostream>
#include <fstream>

#include <vtr_path_planning/cbit/visualization_utils.hpp>

namespace vtr {
namespace path_planning {

namespace{
// Data sample structure necessary to fit / evaluate a GP-based disturbance model
struct DisturbanceSample {
  std::array<double, 2> v_k_m1;
  std::array<double, 3> x_k;
  std::array<double, 2> u_k;
  std::array<double, 2> u_k_m1;
  std::array<double, 3> x_k_p1;
  // Bookkeeping for binning and FIFO bins
  int vertex_id;
  int trial_id;
};
}

class BicycleMPCDisturbanceModelPathTracker : public BicycleMPCPathTracker {
 public:
  PTR_TYPEDEFS(BicycleMPCDisturbanceModelPathTracker);

  static constexpr auto static_name = "bicycle_mpc_disturbance_model";

  // Note all rosparams that are in the config yaml file need to be declared here first, though they can be then changes using the declareparam function for ros in the cpp file
  struct Config : public BicycleMPCPathTracker::Config {
    PTR_TYPEDEFS(Config);
    std::string csv_path="";
    double bin_resolution = 0.1;
    int bin_size = 10;
    int vertex_range = 3;
    int velocity_bin_range = 2;
    int n_gp_inputs = 350;

    std::string gp_hyperparam_path = "";

    static void loadConfig(Config::Ptr config,
                   const rclcpp::Node::SharedPtr& node,
                   const std::string& prefix = "path_planning.mpc");

    static Ptr fromROS(const rclcpp::Node::SharedPtr& node,
                       const std::string& prefix = "path_planning.mpc");
  };



  struct GPHyperParams {
    std::vector<double> length_scales;
    double noise_variance;
    double output_variance;
  };

  BicycleMPCDisturbanceModelPathTracker(const Config::ConstPtr& config,
                 const RobotState::Ptr& robot_state,
                 const tactic::GraphBase::Ptr& graph,
                 const Callback::Ptr& callback);

  ~BicycleMPCDisturbanceModelPathTracker() override;

 protected:
 
  void loadMPCConfig(
      CasadiBicycleMPC::Config::Ptr mpc_config, Eigen::Matrix<double, 6, 1> w_p_r_in_r, Eigen::Vector2d applied_vel,
                            const bool isReversing);

  virtual CasadiMPC::Config::Ptr getMPCConfig(Eigen::Matrix<double, 6, 1> w_p_r_in_r, Eigen::Vector2d applied_vel,
                            const bool isReversing) override;
  
  void loadMPCPath(CasadiMPC::Config::Ptr mpcConfig, const lgmath::se3::Transformation& T_w_p,
                            const lgmath::se3::Transformation& T_p_r_extp,
                            const double state_p,
                            RobotState& robot_state, 
                            const tactic::Timestamp& curr_time) override;

  virtual std::map<std::string, casadi::DM> callSolver(CasadiMPC::Config::Ptr config) override;

 private: 
  VTR_REGISTER_PATH_PLANNER_DEC_TYPE(BicycleMPCDisturbanceModelPathTracker);

  void loadDisturbanceSamples();
  DisturbanceSample parseDisturbanceSample(const std::string& line) const;

  void writeDisturbanceSample(const DisturbanceSample& sample);

  int velocityBin(double v_cmd) const;

  // CSV to dump / read previous experiences from
  std::ofstream csv_writer_; 

  int failure_count = 0;
  int success_count = 0;
  Config::ConstPtr config_;
  CasadiBicycleMPC solver_;
  // One hyperparam set for each dimension
  std::array<GPHyperParams, 3> gp_hyperparams_;
  std::vector<DisturbanceSample> disturbance_samples_;
  // Bin experiences according to Ostafew, 2014, by vertex id and velocity
  using BinKey = std::pair<int, int>;  // (vertex_id, velocity_bin)
  struct BinKeyHash {
  size_t operator()(const BinKey& k) const {
    return std::hash<int>()(k.first) ^ (std::hash<int>()(k.second) << 1);
  }
  };
  std::unordered_map<BinKey, std::deque<DisturbanceSample>, BinKeyHash> bins_;
  

  void getLocalDataset(int vertex_id, double v_cmd,
                        Eigen::MatrixXd& A_local,
                        Eigen::MatrixXd& G_local) const;

  void recordDisturbanceSample(lgmath::se3::Transformation T_p_r_extp, Eigen::Matrix<double, 6, 1> w_p_r_in_r);

  std::array<double,2> casadiResToControlVec(std::map<std::string, casadi::DM> result);

  double last_v_ = 0.0;
  std::array<double, 3> x_k_ = {0.0, 0.0, 0.0};
  std::array<double, 2> v_k_m1_ = {0.0, 0.0};
  std::array<double, 2> u_k_ = {0.0, 0.0};
  std::array<double, 2> u_k_m1_ = {0.0, 0.0};
  int sid_ = 0;
  int trial_id_ = 0;
};

}  // namespace path_planning
}  // namespace vtr

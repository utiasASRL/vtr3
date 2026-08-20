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
 * \file bicycle_mpc_path_tracker_disturbance_model.cpp
 * \author Luka Antonyshyn, Autonomous Space Robotics Lab (ASRL)
 */

#include "vtr_path_planning/mpc/bicycle_mpc_disturbance_model_path_tracker.hpp"

#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <vtr_path_planning/cbit/utils.hpp>

namespace vtr::path_planning {

// Configure the class as a ROS2 node, get configurations from the ros parameter server

auto BicycleMPCDisturbanceModelPathTracker::Config::loadConfig(BicycleMPCDisturbanceModelPathTracker::Config::Ptr config, 
		           const rclcpp::Node::SharedPtr& node,
                           const std::string& prefix)->void{
  config->csv_path = node->declare_parameter<std::string>(prefix + ".dist_model.csv_path", config->csv_path);
  config->gp_hyperparam_path = node->declare_parameter<std::string>(prefix + ".dist_model.gp_hyperparam_path", config->gp_hyperparam_path);
  config->bin_resolution = node->declare_parameter<double>(prefix + ".dist_model.bin_resolution", config->bin_resolution);
  config->bin_size = node->declare_parameter<int>(prefix + ".dist_model.bin_size", config->bin_size);
  config->vertex_range = node->declare_parameter<int>(prefix + ".dist_model.vertex_range", config->vertex_range);
  config->velocity_bin_range = node->declare_parameter<int>(prefix + ".dist_model.velocity_bin_range", config->velocity_bin_range);
  config->n_gp_inputs = node->declare_parameter<int>(prefix + ".dist_model.n_gp_inputs", config->n_gp_inputs);

  // MISC
  config->r_racc2 = node->declare_parameter<double>(prefix + ".mpc.reverse.racc2", config->r_racc2);
  config->command_history_length = node->declare_parameter<int>(prefix + ".mpc.command_history_length", config->command_history_length);
}

auto BicycleMPCDisturbanceModelPathTracker::Config::fromROS(const rclcpp::Node::SharedPtr& node, const std::string& prefix) -> Ptr {
  auto config = std::make_shared<Config>();

  auto base_config = std::static_pointer_cast<BicycleMPCPathTracker::Config>(config);
  *base_config =  *BicycleMPCPathTracker::Config::fromROS(node, prefix);
  loadConfig(config, node, prefix);

  CLOG(INFO, "cbit.control") << "Loaded BicycleMPCDisturbanceModelPathTracker config from ROS parameters";
  CLOG(INFO, "cbit.control") << "CSV path: " << config->csv_path;
  CLOG(INFO, "cbit.control") << "GP hyperparam path: " << config->gp_hyperparam_path;
  CLOG(INFO, "cbit.control") << "Bin resolution: " << config->bin_resolution;
  CLOG(INFO, "cbit.control") << "Bin size: " << config->bin_size;
  CLOG(INFO, "cbit.control") << "Vertex range: " << config->vertex_range;
  CLOG(INFO, "cbit.control") << "Velocity bin range: " << config->velocity_bin_range;
  CLOG(INFO, "cbit.control") << "Number of GP inputs: " << config->n_gp_inputs;

  return config;
}

BicycleMPCDisturbanceModelPathTracker::BicycleMPCDisturbanceModelPathTracker(const Config::ConstPtr& config,
                               const RobotState::Ptr& robot_state,
                               const tactic::GraphBase::Ptr& graph,
                               const Callback::Ptr& callback)
    : BicycleMPCPathTracker(config, robot_state, graph, callback),
    config_(config), 
    solver_{config_->mpc_verbosity} {
      CLOG(DEBUG, "cbit.control") << "Constructed Bicycle tracker";

    std::ifstream file(config->csv_path);
    std::string line;
    int line_count = 0;
    if (!std::getline(file, line)){
      CLOG(INFO, "cbit.control") << "CSV file is empty, writing header";
      csv_writer_.open(config->csv_path, std::ios::app);
      csv_writer_ << "trial_id,vertex_id,"
            << "v_k_m1[0],v_k_m1[1],"
            << "x_k[0],x_k[1],x_k[2],"
            << "u_k[0],u_k[1],"
            << "u_k_m1[0],u_k_m1[1],"
            << "x_k_p1[0],x_k_p1[1],x_k_p1[2]\n";
    }
    else{
      CLOG(INFO, "cbit.control") << "CSV file already has header, not writing header";
      csv_writer_.open(config->csv_path, std::ios::app);
      loadDisturbanceSamples();
    }
}

BicycleMPCDisturbanceModelPathTracker::~BicycleMPCDisturbanceModelPathTracker() {}

void BicycleMPCDisturbanceModelPathTracker::writeDisturbanceSample(const DisturbanceSample& sample) {
  // Write the disturbance sample to the CSV file
  csv_writer_ << sample.trial_id << ","
              << sample.vertex_id << ","
              << sample.v_k_m1[0] << ","
              << sample.v_k_m1[1] << ","
              << sample.x_k[0] << ","
              << sample.x_k[1] << ","
              << sample.x_k[2] << ","
              << sample.u_k[0] << ","
              << sample.u_k[1] << ","
              << sample.u_k_m1[0] << ","
              << sample.u_k_m1[1] << ","
              << sample.u_k_m1[2] << ","
              << sample.x_k_p1[0] << ","
              << sample.x_k_p1[1] << ","
              << sample.x_k_p1[2]
              << "\n";
}

void BicycleMPCDisturbanceModelPathTracker::loadDisturbanceSamples() {
  std::ifstream file(config_->csv_path);
  if (!file.is_open()) {
    CLOG(WARNING, "cbit.control") << "Could not open CSV for reading: " << config_->csv_path;
    return;
  }

  std::string line;
  // Skip header row
  if (!std::getline(file, line)) {
    CLOG(INFO, "cbit.control") << "CSV file empty, nothing to load.";
    return;
  }

  int max_trial_id = -1;
  int loaded_count = 0;

  while (std::getline(file, line)) {
    if (line.empty()) continue;

    DisturbanceSample sample;
    try {
      sample = parseDisturbanceSample(line);
    } catch (const std::exception& e) {
      CLOG(WARNING, "cbit.control")
          << "Skipping malformed CSV row: " << e.what() << " | line: " << line;
      continue;
    }

    disturbance_samples_.push_back(sample);

    // Populate bins with FIFO eviction, same as recordDisturbanceSample should do
    BinKey key{sample.vertex_id, velocityBin(sample.v_k_m1[0])};
    auto& bin = bins_[key];
    bin.push_back(sample);
    if (static_cast<int>(bin.size()) > config_->bin_size) {
      bin.pop_front();
    }

    max_trial_id = std::max(max_trial_id, sample.trial_id);
    ++loaded_count;
  }

  trial_id_ = max_trial_id + 1;  // continue from the next fresh trial id

  CLOG(INFO, "cbit.control")
      << "Loaded " << loaded_count << " disturbance samples from "
      << config_->csv_path << ". Next trial_id = " << trial_id_;
}

DisturbanceSample BicycleMPCDisturbanceModelPathTracker::parseDisturbanceSample(const std::string& line) const {
  std::stringstream ss(line);
  std::vector<double> vals;
  std::string field;

  while (std::getline(ss, field, ',')) {
    if (field.empty()) {
      throw std::runtime_error("empty field in CSV row");
    }
    vals.push_back(std::stod(field));
  }

  constexpr size_t kExpectedFields = 14;  // matches the fixed header above
  if (vals.size() != kExpectedFields) {
    throw std::runtime_error(
        "expected " + std::to_string(kExpectedFields) +
        " fields, got " + std::to_string(vals.size()));
  }

  DisturbanceSample sample;
  size_t i = 0;
  sample.trial_id  = static_cast<int>(vals[i++]);
  sample.vertex_id = static_cast<int>(vals[i++]);
  sample.v_k_m1    = {vals[i], vals[i+1]}; i += 2;
  sample.x_k       = {vals[i], vals[i+1], vals[i+2]}; i += 3;
  sample.u_k       = {vals[i], vals[i+1]}; i += 2;
  sample.u_k_m1    = {vals[i], vals[i+1]}; i += 2;
  sample.x_k_p1    = {vals[i], vals[i+1], vals[i+2]}; i += 3;

  return sample;
}

int BicycleMPCDisturbanceModelPathTracker::velocityBin(double v_cmd) const {
  // Calculate the bin index for the given commanded velocity
  int bin_index = static_cast<int>(std::floor(v_cmd / config_->bin_resolution));
  return bin_index;
}

void BicycleMPCDisturbanceModelPathTracker::loadMPCConfig(
    CasadiBicycleMPC::Config::Ptr mpc_config, Eigen::Matrix<double, 6, 1> w_p_r_in_r, Eigen::Vector2d applied_vel, const bool isReversing) {
  // Set the MPC parameters based on the configuration
  mpc_config->VF           = config_->forward_vel;
  mpc_config->vel_max(0)   = isReversing ? 0.0 : config_->max_lin_vel;
  mpc_config->vel_min(0)   = isReversing ? -config_->max_lin_vel : 0.0;
  mpc_config->vel_max(1)   = config_->max_ang_vel;
  mpc_config->vel_min(1)   = -config_->max_ang_vel;
  mpc_config->previous_vel = {-w_p_r_in_r(0, 0), applied_vel(1)};
  mpc_config->wheelbase    = config_->wheelbase;

  // Set the MPC costs based on if we're reversing or not
  mpc_config->Q_lat  = isReversing ? config_->r_q_lat : config_->f_q_lat;
  mpc_config->Q_lon  = isReversing ? config_->r_q_lon : config_->f_q_lon;
  mpc_config->Q_th   = isReversing ? config_->r_q_th : config_->f_q_th;
  mpc_config->R1     = isReversing ? config_->r_r1 : config_->f_r1; 
  mpc_config->R2     = isReversing ? config_->r_r2 : config_->f_r2;
  mpc_config->Acc_R1 = isReversing ? config_->r_racc1 : config_->f_racc1;
  mpc_config->Acc_R2 = isReversing ? config_->r_racc2 : config_->f_racc2; 

  if (failure_count >= config_->failure_threshold) {
    CLOG(WARNING, "cbit.control") << "Failure count exceeded threshold. Enabling recovery mode.";
    mpc_config->recovery = true;
    // If we're recovering, the only thing that matters is getting back to the path
    mpc_config->Q_lon      = 0.4;
    // TODO: make these parameters configurable
    mpc_config->vel_max(0) = 0.5*config_->max_lin_vel;
    mpc_config->vel_min(0) = -0.5*config_->max_lin_vel;
    mpc_config->R1         = 0.0;
    mpc_config->R2         = 0.0;
    mpc_config->Acc_R1     = 0.0;
    mpc_config->Acc_R2     = 0.0;
  }
}

CasadiMPC::Config::Ptr BicycleMPCDisturbanceModelPathTracker::getMPCConfig(Eigen::Matrix<double, 6, 1> w_p_r_in_r, Eigen::Vector2d applied_vel, const bool isReversing) {
  auto mpc_config = std::make_shared<CasadiBicycleMPC::Config>();
  loadMPCConfig(mpc_config, w_p_r_in_r, applied_vel, isReversing);
  return mpc_config;
}

void BicycleMPCDisturbanceModelPathTracker::loadMPCPath(CasadiMPC::Config::Ptr mpcConfig, const lgmath::se3::Transformation& T_w_p,
                         const lgmath::se3::Transformation& T_p_r_extp,
                         const double state_p,
                         RobotState& robot_state,
                         const tactic::Timestamp& t) {
  auto mpc_config = std::static_pointer_cast<CasadiBicycleMPC::Config>(mpcConfig);
  BaseReferenceAdjustmentMPCPathTracker::loadMPCPath(mpcConfig, T_w_p, T_p_r_extp, state_p, robot_state, t);

  const auto [stamp, w_p_r_in_r, T_p_r, tmp, T_w_v_odo, T_r_v_odo, curr_sid] =
      getChainInfo(*chain);
  
  // We have not yet recorded any of the new info, so x is x_{k+1} - as such, we record a sample and progress one step
  recordDisturbanceSample(T_p_r_extp, w_p_r_in_r);

  sid_ = curr_sid;
}

void BicycleMPCDisturbanceModelPathTracker::recordDisturbanceSample(lgmath::se3::Transformation T_p_r, Eigen::Matrix<double, 6, 1> w_p_r_in_r){
  auto lin_vel = w_p_r_in_r.head(3).norm();
  //Yaw rate
  auto ang_vel = w_p_r_in_r(5);
  std::array<double, 2> v_k(lin_vel, ang_vel);
  // SE3->vector space representation
  auto x_k_p1 = tf_to_global(T_p_r);
  // Construct sample from cached values
  DisturbanceSample sample;
  sample.v_k_m1 = v_k_m1_;
  sample.u_k = u_k_;
  sample.u_k_m1 = u_k_m1_;
  sample.x_k = x_k_;
  sample.x_k_p1 = x_k_p1;
  sample.vertex_id = sid_;
  sample.trial_id = trial_id_;
  disturbance_samples_.push_back(sample);
  // Move window
  v_k_m1_ = v_k_;
  v_k_ = v_k;
  x_k_ = x_k_p1;
  // Control inputs are handled in that loop
  // Append to CSV
  writeDisturbanceSample(sample);
  // Write to bin - pop FIFO if bin is at capacity
  BinKey key{sample.vertex_id, velocityBin(sample.v_k_m1[0])};
  auto& bin = bins_[key];
  bin.push_back(sample);
  if (static_cast<int>(bin.size()) > config_->bin_size) {
    bin.pop_front();
  }
}

std::map<std::string, casadi::DM> BicycleMPCDisturbanceModelPathTracker::callSolver(CasadiMPC::Config::Ptr config) {
  std::map<std::string, casadi::DM> result;
  u_k_m1_ = u_k_;

  try {
    CLOG(INFO, "cbit.control") << "Attempting to solve the MPC problem";
    result = solver_.solve(*config);
    CLOG(INFO, "cbit.control") << "Solver called";
    if (std::abs(result["vel"](casadi::Slice(),0).get_elements()[0]) < 0.0001){
      throw std::invalid_argument("Velocity too low.");
    }
    success_count += 1;
    if(success_count > config_->recovery_steps){
        failure_count = 0;
    }
  } catch (std::exception& e) {
      CLOG(WARNING, "cbit.control")
          << "casadi failed! " << e.what() << ". Incrementing failure count. Current count: " << failure_count;
      failure_count += 1;
      // Only reset success count if we are not actively trying to recover
      // If we are failing consistently when recovering, we should just keep trying.
      if (failure_count < config_->failure_threshold){
          success_count = 0;
      }
      u_k_ = std::array<double, 2> (0.0,0.0);
      throw e;
  }
  
  u_k_ = casadiResToControlVec(result);
  return result;
}

std::array<double, 2> BicycleMPCDisturbanceModelPathTracker::casadiResToControlVec(std::map<std::string, casadi::DM> result){
  const auto inputs = result["vel"](casadi::Slice(), 0).get_elements();
  std::array<double, 2> u(inputs[0], inputs[1]);
  return u;
}

}  // namespace vtr::path_planning

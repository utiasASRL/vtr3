// This module essentially just do radar_gp_state_estimation.cpp but populate the qdata in the end for the correct values
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <filesystem>
#include <opencv2/opencv.hpp>
#include "yaml-cpp/yaml.h"
#include "vtr_radar/modules/odometry/odometry_dense_module.hpp"
#include "vtr_radar/modules/odometry/dense_utils/gp_doppler.hpp"


namespace vtr {
namespace radar {

using namespace tactic;
using namespace steam;
using namespace steam::se3;
using namespace steam::traj;
using namespace steam::vspace;
// using vtr::radar::GPStateEstimator;

auto OdometryDenseModule::Config::fromROS(const rclcpp::Node::SharedPtr &node,
                                        const std::string &param_prefix)
    -> ConstPtr {

    auto config = std::make_shared<Config>();
    // clang-format off
    CLOG(DEBUG, "radar.odometry_dense") << "Loading dense odometry parameters";
    // I want to know what param_prefix is
    CLOG(DEBUG, "radar.odometry_dense") << "Param prefix is: " << param_prefix;
    // lets construct the config object here by reading the fields of the config, later on the config_ object will be passed to the gp_doppler object
    // 1. estimation
    // boolean flag doppler_cost
    config->doppler_cost = node->declare_parameter<bool>(param_prefix + ".estimation" + ".doppler_cost", config->doppler_cost);
    CLOG(DEBUG, "radar.odometry_dense") << "The config param: doppler_cost is: " << config->doppler_cost;
    // boolean flag direct_cost
    config->direct_cost = node->declare_parameter<bool>(param_prefix + ".estimation" + ".direct_cost", config->direct_cost);
    CLOG(DEBUG, "radar.odometry_dense") << "The config param: direct_cost is: " << config->direct_cost;
    // string motion model
    config->motion_model = node->declare_parameter<std::string>(param_prefix + ".estimation" + ".motion_model", config->motion_model);
    CLOG(DEBUG, "radar.odometry_dense") << "The config param: motion_model is: " << config->motion_model;
    // boolean flag gyro_bias_estimation
    config->gyro_bias_estimation = node->declare_parameter<bool>(param_prefix + ".estimation" + ".gyro_bias_estimation", config->gyro_bias_estimation);
    CLOG(DEBUG, "radar.odometry_dense") << "The config param: gyro_bias_estimation is: " << config->gyro_bias_estimation;
    // boolean flag estimate_gyro_bias 
    config->estimate_gyro_bias = node->declare_parameter<bool>(param_prefix + ".estimation" + ".estimate_gyro_bias", config->estimate_gyro_bias);
    // max_acceleration
    config->max_acceleration = node->declare_parameter<float>(param_prefix + ".estimation" + ".max_acceleration", config->max_acceleration);
    CLOG(DEBUG, "radar.odometry_dense") << "The config param: max_acceleration is: " << config->max_acceleration;
    // optimization_first_step
    config->optimization_first_step = node->declare_parameter<float>(param_prefix + ".estimation" + ".optimization_first_step", config->optimization_first_step);
    CLOG(DEBUG, "radar.odometry_dense") << "The config param: optimization_first_step is: " << config->optimization_first_step;
    // vy_bias_prior
    config->vy_bias_prior = node->declare_parameter<double>(param_prefix + ".estimation" + ".vy_bias_prior", config->vy_bias_prior);
    CLOG(DEBUG, "radar.odometry_dense") << "The config param: vy_bias_prior is: " << config->vy_bias_prior;
    // boolean flag estimate_doppler_vy_bias
    config->estimate_doppler_vy_bias = node->declare_parameter<bool>(param_prefix + ".estimation" + ".estimate_doppler_vy_bias", config->estimate_doppler_vy_bias);
    CLOG(DEBUG, "radar.odometry_dense") << "The config param: estimate_doppler_vy_bias is: " << config->estimate_doppler_vy_bias;
    // boolean flag use_gyro
    config->use_gyro = node->declare_parameter<bool>(param_prefix + ".estimation" + ".use_gyro", config->use_gyro);
    CLOG(DEBUG, "radar.odometry_dense") << "The config param: use_gyro is: " << config->use_gyro;
    // float ang_vel_bias
    config->ang_vel_bias = node->declare_parameter<float>(param_prefix + ".estimation" + ".ang_vel_bias", config->ang_vel_bias);
    CLOG(DEBUG, "radar.odometry_dense") << "The config param: ang_vel_bias is: " << config->ang_vel_bias;

    
    // 2. gp
    // float lengthscale_az
    config->lengthscale_az = node->declare_parameter<float>(param_prefix + ".gp" + ".lengthscale_az", config->lengthscale_az);
    CLOG(DEBUG, "radar.odometry_dense") << "The config param: lengthscale_az is: " << config->lengthscale_az;
    // float lengthscale_range
    config->lengthscale_range = node->declare_parameter<float>(param_prefix + ".gp" + ".lengthscale_range", config->lengthscale_range);
    CLOG(DEBUG, "radar.odometry_dense") << "The config param: lengthscale_range is: " << config->lengthscale_range;
    // sz
    config->sz = node->declare_parameter<double>(param_prefix + ".gp" + ".sz", config->sz);
    CLOG(DEBUG, "radar.odometry_dense") << "The config param: sz is: " << config->sz;

    // 3. radar
    // radar resolution
    config->radar_resolution = node->declare_parameter<double>(param_prefix + ".radar" + ".radar_resolution", config->radar_resolution);
    CLOG(DEBUG, "radar.odometry_dense") << "The config param: radar_resolution is: " << config->radar_resolution;
    // radar range offset
    config->range_offset = node->declare_parameter<double>(param_prefix + ".radar" + ".range_offset", config->range_offset);
    CLOG(DEBUG, "radar.odometry_dense") << "The config param: range_offset is: " << config->range_offset;
    // boolean flag doppler_enabled
    config->doppler_enabled = node->declare_parameter<bool>(param_prefix + ".radar" + ".doppler_enabled", config->doppler_enabled);
    CLOG(DEBUG, "radar.odometry_dense") << "The config param: doppler_enabled is: " << config->doppler_enabled;
    // boolean flag chirp_up
    config->chirp_up = node->declare_parameter<bool>(param_prefix + ".radar" + ".chirp_up", config->chirp_up);
    CLOG(DEBUG, "radar.odometry_dense") << "The config param: chirp_up is: " << config->chirp_up;
    // ft parameter
    config->ft = node->declare_parameter<double>(param_prefix + ".radar" + ".ft", config->ft);
    CLOG(DEBUG, "radar.odometry_dense") << "The config param: ft is: " << config->ft;
    // meas_freq parameter
    config->meas_freq = node->declare_parameter<double>(param_prefix + ".radar" + ".meas_freq", config->meas_freq);
    CLOG(DEBUG, "radar.odometry_dense") << "The config param: meas_freq is: " << config->meas_freq;
    // del_f parameter
    config->del_f = node->declare_parameter<double>(param_prefix + ".radar" + ".del_f", config->del_f);
    CLOG(DEBUG, "radar.odometry_dense") << "The config param: del_f is: " << config->del_f;
    // beta_corr_fact parameter
    config->beta_corr_fact = node->declare_parameter<double>(param_prefix + ".radar" + ".beta_corr_fact", config->beta_corr_fact);
    CLOG(DEBUG, "radar.odometry_dense") << "The config param: beta_corr_fact is: " << config->beta_corr_fact;

    // 4. direct
    // min_range
    config->min_range = node->declare_parameter<float>(param_prefix + ".direct" + ".min_range", config->min_range);
    CLOG(DEBUG, "radar.odometry_dense") << "The config param: min_range is: " << config->min_range;
    // max_range
    config->max_range = node->declare_parameter<float>(param_prefix + ".direct" + ".max_range", config->max_range);
    CLOG(DEBUG, "radar.odometry_dense") << "The config param: max_range is: " << config->max_range;
    // max_local_map_range
    config->max_local_map_range = node->declare_parameter<double>(param_prefix + ".direct" + ".max_local_map_range", config->max_local_map_range);
    CLOG(DEBUG, "radar.odometry_dense") << "The config param: max_local_map_range is: " << config->max_local_map_range;
    // local_map_resolution
    config->local_map_res = node->declare_parameter<double>(param_prefix + ".direct" + ".local_map_res", config->local_map_res);
    CLOG(DEBUG, "radar.odometry_dense") << "The config param: local_map_res is: " << config->local_map_res;
    // local_map_update_alpha
    config->local_map_update_alpha = node->declare_parameter<double>(param_prefix + ".direct" + ".local_map_update_alpha", config->local_map_update_alpha);
    CLOG(DEBUG, "radar.odometry_dense") << "The config param: local_map_update_alpha is: " << config->local_map_update_alpha;


    return config;

    }

void OdometryDenseModule::run_(QueryCache &qdata0, OutputCache &, const Graph::Ptr &, const TaskExecutor::Ptr &) {
  auto &qdata = dynamic_cast<RadarQueryCache &>(qdata0);


  CLOG(DEBUG, "radar.odometry_dense") << "Running dense odometry module";

  // Do nothing if qdata does not contain any radar data (was populated by gyro)
  if(!qdata.radar_data)
  {
    CLOG(WARNING, "radar.odometry_dense") << "No radar data available";
    return;
  }

  // do what we do for the first frame, nitic that for first frame there is no gyro msgs
  if (!initialized) { //change it later
    // Initialize all variables
    CLOG(INFO, "radar.odometry_dense") << "First frame, simply return.";

    // clang-format off
    //
    // qdata.timestamp_odo.emplace(*qdata.stamp);
    // qdata.T_r_m_odo.emplace(EdgeTransform(true));
    // qdata.w_m_r_in_r_odo.emplace(Eigen::Matrix<double, 6, 1>::Zero());

    // qdata.timestamp_odo_radar.emplace(*qdata.stamp);
    // qdata.T_r_m_odo_radar.emplace(EdgeTransform(true));
    // qdata.w_m_r_in_r_odo_radar.emplace(Eigen::Matrix<double, 6, 1>::Zero());
    // // Initialize prior values
    // qdata.T_r_m_odo_prior.emplace(lgmath::se3::Transformation());
    // qdata.w_m_r_in_r_odo_prior.emplace(Eigen::Matrix<double, 6, 1>::Zero());
    
    // Initialize timestamp equal to the end of the first frame

    // intialize the gp_doppler GP stateEstimator object and pass in the config 
    if (!state_estimator_) {
      state_estimator_ = std::make_shared<GPStateEstimator>(config_);
    }
    CLOG(DEBUG, "radar.odometry_dense") << "Initialized GPStateEstimator";
   
    *qdata.odo_success = true;
    // clang-format on

    initialized = true;
    // This is the first odomety frame
    if(qdata.first_frame)
      *qdata.first_frame = true;
    else
      qdata.first_frame.emplace(true); // reset first frame - this is the first frame! Gyro could have run before though

    return; // return later
  }

  CLOG(DEBUG, "radar.odometry_dense")
      << "Retrieve input data and setup GPStateEstimator.";

  // here is the game plan: I will intialize the gpstateestimator with the config above and radar resolution
  // input
  const auto &T_s_r = *qdata.T_s_r; // radar to baselink transform
  const auto &radar_data = *qdata.radar_data; // radar data
  // Create robot to sensor transform variable, fixed.
  const auto T_s_r_var = SE3StateVar::MakeShared(T_s_r);
  T_s_r_var->locked() = true;

  // auto &sliding_map_odo = *qdata.sliding_map_odo;

  torch::Device device = state_estimator_->getDevice();

  // print which device I am running it on here
  std::cout << "Sam: Running on device: " << device << std::endl;


  // need to have a local map here
  // &local_dense_map == sliding_map_odo.local_map()




  // I need to convert the tensor to torch tensor qdata.radar_data
  // auto radar_tensor = torch::from_blob(qdata.radar_data->data(), {1, 1, 1, 1}, torch::kFloat32);

  // I want to see the gyro msg size
  // the brain of the code goes here I need gyro data as well how do I approach this
  

  // i intialize the state_estimator and then set the data and get the output and set vtr qdata variable
  // so i need to import the gp_doppler class and pass in the config_

   CLOG(DEBUG, "radar.odometry_dense") << "Finished odometry step and exiting Odometry dense module: ";

  return;



  // need to check the result and store them in vtr as a lgmath::se3::Transformation

}

}  // namespace radar
}  // namespace vtr
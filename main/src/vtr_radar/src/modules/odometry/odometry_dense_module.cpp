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
#include "vtr_radar/types.hpp"
#include "vtr_radar/utils/utils.hpp"

// dumping functions start
#include <iomanip>
#include <filesystem>
namespace fs = std::filesystem;

static void dump_vector_csv(const std::string& path, const std::vector<double>& v) {
  fs::create_directories(fs::path(path).parent_path());
  std::ofstream f(path);
  f << std::setprecision(17);
  for (auto x : v) f << x << "\n";
}

static void dump_tensor_csv(const std::string& path, const torch::Tensor& t_cpu_1d) {
  fs::create_directories(fs::path(path).parent_path());
  auto t = t_cpu_1d.contiguous().to(torch::kCPU);
  std::ofstream f(path);
  f << std::setprecision(17);
  for (int64_t i = 0; i < t.numel(); ++i) f << t[i].item<double>() << "\n";
}

static void dump_mat_png(const std::string& path, const cv::Mat& m) {
  fs::create_directories(fs::path(path).parent_path());
  cv::Mat out;
  cv::imwrite(path, out);
}

static void dump_polar_tensor_png(const std::string& path, const torch::Tensor& polar) {
  fs::create_directories(fs::path(path).parent_path());
  // (H,W) float32 on any device -> CPU CV_8U PNG
  auto p = polar.detach().to(torch::kCPU).contiguous();
  cv::Mat m(p.size(0), p.size(1), CV_32F, (void*)p.data_ptr<float>());
  dump_mat_png(path, m);
}

// Dumping functions

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

  bool DUMP_2_DEBUG = true;

  // frame index
  // Do nothing if qdata does not contain any radar data (was populated by gyro)
  if(!qdata.radar_data)
  {
    CLOG(WARNING, "radar.odometry_dense") << "No radar data available";
    return;
  }

  CLOG(DEBUG, "radar.odometry_dense") << "-----------------------------------------Running dense odometry module with frame idx: " << frame_idx;

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
    CLOG(DEBUG, "radar.odometry_dense") << "0.Initialized GPStateEstimator";
   
    *qdata.odo_success = true;
    // clang-format on

    initialized = true;
    // This is the first odomety frame
    if(qdata.first_frame)
      *qdata.first_frame = true;
    else
      qdata.first_frame.emplace(true); // reset first frame - this is the first frame! Gyro could have run before though

    frame_idx++;
    return; // return later
  }

  CLOG(DEBUG, "radar.odometry_dense")
      << "1. ------------------ Retrieve input radar data and to set up GPStateEstimator. ------------------ ";

  // here is the game plan: I will intialize the gpstateestimator with the config above and radar resolution
  // need a radar data frame index
  // as well as imu times
  // beginning radar time stamps and ending radar time stamps
  // input
  const auto &T_s_r = *qdata.T_s_r; // radar to baselink transform
  const auto &radar_data = *qdata.radar_data; // radar data
  // Create robot to sensor transform variable, fixed.
  const auto T_s_r_var = SE3StateVar::MakeShared(T_s_r);
  T_s_r_var->locked() = true;

  int64_t frame_start = radar_data.azimuth_times.front();// convert to microsecs
  int64_t frame_end = radar_data.azimuth_times.back(); // convert to microsecs
  CLOG(DEBUG, "radar.odometry_dense") << "Radar frame start time: " << frame_start;
  CLOG(DEBUG, "radar.odometry_dense") << "Radar frame end time: " << frame_end;
  double duration = frame_end - frame_start;
  CLOG(DEBUG, "radar.odometry_dense") << "Radar frame duration: " << duration;

  // now go find the imu data corrsonpond to those radar 
  // auto &sliding_map_odo = *qdata.sliding_map_odo;

  torch::Device device = state_estimator_->getDevice();
  // print which device I am running it on here
  // std::cout << "Sam: Running on device: " << device << std::endl;

  // learn from odometry icp and do a timer
  using Stopwatch = common::timing::Stopwatch<>;
  std::vector<std::unique_ptr<Stopwatch>> timer;
  std::vector<std::string> clock_str;
  clock_str.push_back("Dense odometry step ");
  timer.emplace_back(std::make_unique<Stopwatch>(false));

  // // gyrooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooo
  // // need to set imu data first
  // // we need to construct an array for the imu vector or tensor....
  // CLOG_IF(qdata.gyro_msgs, DEBUG, "radar.odometry_dense") << "2. ------------------ Gyro messages are available so we process here ------------------ ";
  // if(qdata.gyro_msgs){
  //   const auto &T_s_r_gyro = *qdata.T_s_r_gyro;
  //   // I want to print the first and last gyro timestamps
  //   const auto &first_gyro_msg = qdata.gyro_msgs->front();
  //   const auto &last_gyro_msg = qdata.gyro_msgs->back();

  //   int64_t first_gyro_time = first_gyro_msg.header.stamp.sec*1e9 + first_gyro_msg.header.stamp.nanosec;
  //   int64_t last_gyro_time = last_gyro_msg.header.stamp.sec*1e9 + last_gyro_msg.header.stamp.nanosec;
  //   int64_t gyro_duration = last_gyro_time - first_gyro_time;

  //   CLOG(DEBUG, "radar.odometry_dense") << "First gyro timestamp in nano: " << first_gyro_time;
  //   CLOG(DEBUG, "radar.odometry_dense") << "Last gyro timestamp in nano: " << last_gyro_time;
  //   CLOG(DEBUG, "radar.odometry_dense") << "Gyro duration in secs: " << gyro_duration/1e9;

  //   bool pad_gyro_msg = false;
  //    // we need to make sure imu times spread more than the radar scan time frames otherwise we pad the gyro msgs
  //   if (qdata.gyro_msgs && ((first_gyro_time > frame_start) || (last_gyro_time < frame_end))) {
  //       CLOG(WARNING, "radar.odometry_dense") << "Skipping frame: IMU messages are not properly aligned with radar scan, as it does not cover the entirity of the radar scan.";
  //       pad_gyro_msg = true;
  //       // return;
  //       // I can pad the imu with same data-> use the same as the radar gyro 
  //   }

  //   // if it does not return, we are in the clear as the gyro msg duration covers the radar scan duration
  //   float gyro_bias = 0.0;
  //   int gyro_bias_counter = 0;
  //   bool gyro_bias_initialised = false;
  //   bool previous_vel_null = false;
  //   std::vector<double> imu_time;
  //   std::vector<double> imu_yaw;
  //   for (const auto &gyro_msg : *qdata.gyro_msgs) {
  //       // Load in gyro measurement and timestamp
  //       const auto gyro_meas = Eigen::Vector3d(gyro_msg.angular_velocity.x, gyro_msg.angular_velocity.y, gyro_msg.angular_velocity.z);
  //       const rclcpp::Time gyro_stamp(gyro_msg.header.stamp);

  //       const auto gyro_stamp_time = static_cast<int64_t>(gyro_stamp.nanoseconds());

  //       imu_time.push_back(gyro_stamp_time * 1e-9); // in seconds for some reason
  //       // CLOG(DEBUG, "radar.odometry_dense") << "Adding gyro timestamp in microseconds: " << imu_time.back();
       
  //       // Transform gyro measurement into robot frame
  //       Eigen::VectorXd gyro_meas_g(3);
  //       gyro_meas_g << 0, 0, gyro_meas(2);
  //       const Eigen::Matrix<double, 3, 1> gyro_meas_robot =  T_s_r_gyro.matrix().block<3, 3>(0, 0).transpose() * gyro_meas_g; // in the robot frame
  //       // we need it in the radar frame tbh
  //       const Eigen::Matrix<double, 3, 1> gyro_meas_radar = T_s_r.matrix().block<3, 3>(0, 0) * gyro_meas_robot; //T_radar_robot * gyro_in_robot
  //       // I need the yaw member
  //       imu_yaw.push_back(gyro_meas_radar(2));
  //       // print out the gyro yaw values
  //       // CLOG(DEBUG, "radar.odometry_dense") << "Gyro measurement in radar frame: " << gyro_meas_radar.transpose();
  //       // and also the third element just so I am correct in yaw
  //       // CLOG(DEBUG, "radar.odometry_dense") << "Gyro yaw in radar frame: " << imu_yaw.back();
  //     }
  //     // if padding is needed, do it here
  //     if (pad_gyro_msg) {
  //         // Pad the IMU messages with the last known values with the last radar azimuth time
  //         CLOG(WARNING, "radar.odometry_dense") << "Padding IMU messages with last known values!";
  //         imu_time.push_back(frame_end*1e-9); // in seconds
  //         imu_yaw.push_back(imu_yaw.back()); // last known value

  //         // // also in the front as well
  //         // imu_time.push_back(frame_start*1e-9); // in seconds
  //         // imu_yaw.push_back(imu_yaw.front()); // first known value
  //     }

  //     // Pass IMU data into the state estimator I want to log the shape of imu_time and imu_yaw
  //     CLOG(DEBUG, "radar.odometry_dense") << "IMU time vector shape: " << imu_time.size();
  //     CLOG(DEBUG, "radar.odometry_dense") << "IMU yaw vector shape: " << imu_yaw.size();
  // //     // imu_yaw mean is
  // //     if (!imu_yaw.empty()) {
  // //         double imu_yaw_mean = std::accumulate(imu_yaw.begin(), imu_yaw.end(), 0.0) / imu_yaw.size();
  // //         CLOG(DEBUG, "radar.odometry_dense") << "IMU yaw vector mean: " << imu_yaw_mean;
  // //     }
  // //     CLOG(DEBUG, "radar.odometry_dense") << "Finished processing and setting gyro messages.";

  // //     if(DUMP_2_DEBUG){
              
  // //         const std::string base = "/home/samqiao/ASRL/vtr3/src/main/src/vtr_radar/src/modules/odometry/dump" + std::to_string(frame_idx);
  // //         // fs::create_directories(base);
  // //         dump_vector_csv(base + "_imu_time_s.csv", imu_time);  // seconds
  // //         dump_vector_csv(base + "_imu_yaw_rad.csv",  imu_yaw); // yaw rate (or yaw), whatever you pass
  // //         CLOG(DEBUG, "radar.odometry_dense") << "[ONLINE dump f" << frame_idx << "] imu N=" << imu_time.size();

  // //     }

  //     state_estimator_->setGyroData(imu_time, imu_yaw); // we can zero out the gyro to focus on images
  // }

  // // radarrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrr
  // prepare the radar torch tensors for OdoemtryStep note that all these tensor have timestamps in microsecs
  CLOG(DEBUG, "radar.odometry_dense") << "3. ------------------ Converting radar data to torch tensors ---------------------------- " ;

    // remove the mid id which corresponds to the first couple of cols of polar image
  int min_id = static_cast<int>(std::round(config_->min_range / config_->radar_resolution));
  auto radar_torch = toTorch(radar_data, device, min_id);

//   if(DUMP_2_DEBUG){
//     // --- DEBUG DUMP OF ONLINE INPUTS ---
// const std::string base = "/home/samqiao/ASRL/vtr3/src/main/src/vtr_radar/src/modules/odometry/dump" + std::to_string(frame_idx);

// // raw ROS/radar data (ns, doubles, cv::Mat)
// {
//   // azimuth_times are int64 ns -> also dump as µs for direct compare with offline
//   std::vector<double> az_us;
//   az_us.reserve(radar_data.azimuth_times.size());
//   for (auto ns : radar_data.azimuth_times) az_us.push_back(double(ns)/1e3);

//   dump_vector_csv(base + "_azimuth_times_us.csv", az_us);
//   // angles
//   dump_vector_csv(base + "_azimuth_angles_rad.csv", radar_data.azimuth_angles);
//   // fft image
//   dump_mat_png(base + "_fft_scan.png", radar_data.fft_scan);
// }

// // tensors we feed to GP (already min_id-masked, on device)
// {
//   dump_tensor_csv(base + "_timestamps_us_tensor.csv", radar_torch.timestamps.to(torch::kCPU));
//   dump_tensor_csv(base + "_azimuths_rad_tensor.csv", radar_torch.azimuths.to(torch::kCPU));
//   dump_polar_tensor_png(base + "_polar_tensor.png", radar_torch.polar);
// }

// // quick sanity logs (units!)
// {
//   auto ts0 = radar_torch.timestamps[0].item<double>();
//   auto ts1 = radar_torch.timestamps[radar_torch.timestamps.size(0)-1].item<double>();
//   auto az0 = radar_torch.azimuths[0].item<double>();
//   auto az1 = radar_torch.azimuths[radar_torch.azimuths.size(0)-1].item<double>();
//   auto polar_sum = radar_torch.polar.sum().item<double>();

//   CLOG(DEBUG, "radar.odometry_dense")
//       << "[ONLINE dump f" << frame_idx << "] "
//       << "ts(us): " << ts0 << " -> " << ts1
//       << " (N=" << radar_torch.timestamps.size(0) << "), "
//       << "az(rad): " << az0 << " -> " << az1
//       << " (N=" << radar_torch.azimuths.size(0) << "), "
//       << "polar(HxW)=" << radar_torch.polar.size(0) << "x" << radar_torch.polar.size(1)
//       << " sum=" << polar_sum
//       << " chirp_up=" << config_->chirp_up;
// }

// // invariants that often bite (log warnings instead of hard failing)
// {
//   bool ts_monotonic = true;
//   auto tcpu = radar_torch.timestamps.to(torch::kCPU).contiguous();
//   for (int64_t i=1; i<tcpu.numel(); ++i)
//     if (tcpu[i].item<double>() < tcpu[i-1].item<double>()) { ts_monotonic = false; break; }
//   if (!ts_monotonic) CLOG(WARNING, "radar.debug") << "Timestamps not monotonic (µs).";

//   if (std::abs(radar_torch.azimuths[0].item<double>() - radar_torch.azimuths[-1].item<double>()) < 1e-6)
//     CLOG(WARNING, "radar.debug") << "Azimuth first==last; check encoder unwrap.";

//   // dimensions consistency
//   if (radar_torch.azimuths.size(0) != radar_torch.polar.size(0))
//     CLOG(WARNING, "radar.debug") << "H mismatch: azimuths(" << radar_torch.azimuths.size(0)
//                                  << ") vs polar.H(" << radar_torch.polar.size(0) << ")";
// }

//   }

  // I want to log the shape of its field
  CLOG(DEBUG, "radar.odometry_dense") << "Radar data torch azimuths shape: " << radar_torch.azimuths.sizes();
  CLOG(DEBUG, "radar.odometry_dense") << "Radar data torch azimuths device: " << radar_torch.azimuths.device();
  CLOG(DEBUG, "radar.odometry_dense") << "Radar data torch timestamps shape: " << radar_torch.timestamps.sizes();
  // CLOG(DEBUG, "radar.odometry_dense") << "Radar data torch timestamps device: " << radar_torch.timestamps.device();
  // can i print a few timestamps they are on cuda
  // CLOG(DEBUG, "radar.odometry_dense") << "Radar data torch timestamps (first 5): " << radar_torch.timestamps.slice(/*dim=*/0, /*start=*/0, /*end=*/5);

  CLOG(DEBUG, "radar.odometry_dense") << "Radar data torch polar shape: " << radar_torch.polar.sizes();
  // I also like to know the type of the polar image like is it float?
  CLOG(DEBUG, "radar.odometry_dense") << "Radar data torch polar type: " << radar_torch.polar.dtype();

  // can I print out the range of polar image
  CLOG(DEBUG, "radar.odometry_dense") << "Radar data torch polar min: " << radar_torch.polar.min().item<float>();
  CLOG(DEBUG, "radar.odometry_dense") << "Radar data torch polar max: " << radar_torch.polar.max().item<float>();
  // // and printout the timestamp of the scan
  // CLOG(DEBUG, "radar.odometry_dense") << "Radar data torch timestamp: " << radar_torch.timestamp;
  // CLOG(DEBUG, "radar.odometry_dense") << "Radar data torch timestamp device: " << radar_torch.timestamp.device();
  // controlled input -> control output

  // // Run odometry so this is the odometry step call I need to check all the input before that
  //   if (VISUALIZE) {
  //     auto polar_intensity_cpu = radar_torch.polar.to(torch::kCPU);
  //     auto polar_azimuths_cpu = radar_torch.azimuths.to(torch::kCPU);
  //     auto polar_timestamps_cpu = radar_torch.timestamps.to(torch::kCPU);

  //     // duration is equal to the last subtracts first
  //     auto duration = polar_timestamps_cpu.index({-1}) - polar_timestamps_cpu.index({0});
  //     CLOG(DEBUG, "radar.odometry_dense") << "Radar data torch duration: " << duration;

  //     // azimuth difference is last subtracts first
  //     auto azimuth_diff = polar_azimuths_cpu.index({-1}) - polar_azimuths_cpu.index({0});
  //     CLOG(DEBUG, "radar.odometry_dense") << "Radar data torch azimuth difference: " << azimuth_diff;

  //   }
    CLOG(DEBUG, "radar.odometry_dense") << "4. ------------------ Call Odometry step and log the output state ---------------------------- " ;
    timer[0]->start();
    auto state = state_estimator_->odometryStep(
        radar_torch.polar,
        radar_torch.azimuths,
        radar_torch.timestamps,
        config_->chirp_up
    );
    timer[0]->stop();

    
    // // the state here is a torch tensor we can log the size
    // CLOG(DEBUG, "radar.odometry_dense") << "Sam!!!!! Odometry step output state size: " << state.sizes();
    // // I also like to know the type of the state tensor
    // CLOG(DEBUG, "radar.odometry_dense") << "Sam!!!!! Odometry step output state type: " << state.dtype();
    // also the contents it is a [2] tensor vx,vy and I want to print them one at a time
    CLOG(DEBUG, "radar.odometry_dense") << "Sam!!!!! Odometry step output state contents: vx: " << state.index({0});
    CLOG(DEBUG, "radar.odometry_dense") << "Sam!!!!! Odometry step output state contents: vy: " << state.index({1});
    // why is there no wz? bc just the integration of gyro

    // Save velocity data for plotting
    static std::vector<double> velocity_timestamps, vx_values, vy_values;
    static std::string velocity_data_path = "/home/samqiao/ASRL/vtr3/src/main/src/vtr_radar/src/modules/odometry/output/velocity_data.csv";
    
    // Extract velocity values
    double vx = state.index({0}).item<double>();
    double vy = state.index({1}).item<double>();
    double timestamp = radar_data.timestamp / 1e9; // Convert to seconds
    
    // Store velocity values
    velocity_timestamps.push_back(timestamp);
    vx_values.push_back(vx);
    vy_values.push_back(vy);
    
    // Write velocity data to CSV file
    std::ofstream velocity_csv(velocity_data_path);
    velocity_csv << "timestamp,vx,vy\n";
    for (size_t i = 0; i < velocity_timestamps.size(); ++i) {
        velocity_csv << velocity_timestamps[i] << "," << vx_values[i] << "," << vy_values[i] << "\n";
    }
    velocity_csv.close();
    
    CLOG(DEBUG, "radar.odometry_dense") << "Saved velocity data to: " << velocity_data_path << " (total samples: " << velocity_timestamps.size() << ")";

  /// Dump timing info
  CLOG(DEBUG, "radar.odometry_dense") << "Dump timing info inside loop: ";
  for (size_t i = 0; i < clock_str.size(); i++) {
    CLOG(DEBUG, "radar.odometry_dense") << "  " << clock_str[i] << timer[i]->count();
  }


  // after the odometry step
   // Extract 2D velocity
      auto vel_tensor = state.index({torch::indexing::Slice(torch::indexing::None,2)}).to(torch::kCPU);
      Eigen::Vector2d velocity{
          vel_tensor[0].item<double>(),
          vel_tensor[1].item<double>()
      };


    // need to have a local map here
  // &local_dense_map == sliding_map_odo.local_map()
  // one is pose and the other is velocity
  CLOG(DEBUG, "radar.odometry_dense") << "5. ------------------ Get the transformation matrix from the odometry state ---------------------------- " ;
  auto [pos, rot] = state_estimator_->getAzPosRot(); // get the transform 

  if ( pos.has_value() && rot.has_value()) {
      auto current_pos = pos.value();
      auto current_rot = rot.value();

      // Eigen::MatrixXd current_pos_eig = Eigen::Map<Eigen::MatrixXd>(current_pos.squeeze().to(torch::kCPU).data_ptr<double>(), current_pos.size(0), current_pos.size(1));
      // Eigen::VectorXd current_rot_eig = Eigen::Map<Eigen::VectorXd>(current_rot.squeeze().to(torch::kCPU).data_ptr<double>(), current_rot.size(0));

      double radar_timestamp_sec = radar_data.timestamp/1e-9; // in secs
      double min_diff = std::numeric_limits<double>::max();
      int mid_id = 0;
      for (int idx = 0; idx < radar_torch.timestamps.size(0); ++idx) {
          double diff = std::abs(radar_torch.timestamps[idx].item<double>() * 1e-6 - radar_timestamp_sec);
          if (diff < min_diff) {
              min_diff = diff;
              mid_id = idx;
          }
      }

      // Create transformation matrix
      Eigen::Matrix4d trans_mat = Eigen::Matrix4d::Identity();
      double cos_theta = std::cos(current_rot[mid_id].item<double>());
      double sin_theta = std::sin(current_rot[mid_id].item<double>());
      trans_mat(0, 0) = cos_theta;
      trans_mat(0, 1) = -sin_theta;
      trans_mat(1, 0) = sin_theta;
      trans_mat(1, 1) = cos_theta;
      trans_mat(0, 3) = current_pos[mid_id][0].item<double>();
      trans_mat(1, 3) = current_pos[mid_id][1].item<double>();

      // Save position data for plotting
      static std::vector<double> position_timestamps, x_values, y_values;
      static std::string position_data_path = "/home/samqiao/ASRL/vtr3/src/main/src/vtr_radar/src/modules/odometry/output/odometry_estimate_xy.csv";
      
      // Store position values
      double position_timestamp = radar_data.timestamp / 1e9; // Convert to seconds
      position_timestamps.push_back(position_timestamp);
      x_values.push_back(trans_mat(0, 3));
      y_values.push_back(trans_mat(1, 3));
      
      // Write position data to CSV file
      std::ofstream position_csv(position_data_path);
      position_csv << "timestamp,x,y\n";
      for (size_t i = 0; i < position_timestamps.size(); ++i) {
          position_csv << position_timestamps[i] << "," << x_values[i] << "," << y_values[i] << "\n";
      }
      position_csv.close();
      
      CLOG(DEBUG, "radar.odometry_dense") << "Saved position data to: " << position_data_path << " (total samples: " << position_timestamps.size() << ")";

  // save result in q data
      CLOG(DEBUG, "radar.odometry_dense") << "odometry result: x:" << trans_mat(0, 3) << " y:" << trans_mat(1, 3);
  }



  CLOG(DEBUG, "radar.odometry_dense") << "Finished odometry step and exiting Odometry dense module: Bye bye!";
  frame_idx++;
  return;

    

}

}  // namespace radar
}  // namespace vtr
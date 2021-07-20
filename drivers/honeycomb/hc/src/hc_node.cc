/*
 * Unpublished Work Copyright 2019 Waymo LLC.  All rights reserved.
 * Waymo Proprietary and Confidential - Contains Trade Secrets
 *
 * This is the proprietary software of Waymo LLC ("Waymo") and/or its licensors,
 * and may only be used, duplicated, modified or distributed pursuant to the
 * terms and conditions of a separate, written license agreement executed
 * between you and Waymo (an "Authorized License"). Except as set forth in an
 * Authorized License, Waymo grants no license (express or implied), right to
 * use, or waiver of any kind with respect to the Software, and Waymo expressly
 * reserves all rights in and to the Software and all intellectual property
 * rights therein. IF YOU HAVE NO AUTHORIZED LICENSE, THEN YOU HAVE NO RIGHT TO
 * USE THIS SOFTWARE IN ANY WAY, AND SHOULD IMMEDIATELY NOTIFY WAYMO AND
 * DISCONTINUE ALL USE OF THE SOFTWARE.
 *
 * Except as expressly set forth in the Authorized License:
 *
 * 1. This software includes trade secrets of Waymo, and you shall use all
 * reasonable efforts to protect the confidentiality thereof.  You shall use
 * this software only in connection with your authorized use of Waymo products.
 *
 * 2. TO THE MAXIMUM EXTENT PERMITTED BY APPLICABLE LAW, WAYMO PROVIDES THE
 * WAYMO SOFTWARE ON AN “AS IS” AND “AS AVAILABLE” BASIS WITH ALL FAULTS AND
 * WITHOUT ANY REPRESENTATIONS OR WARRANTIES OF ANY KIND.  WAYMO EXPRESSLY
 * DISCLAIMS ALL WARRANTIES, WHETHER IMPLIED, STATUTORY OR OTHERWISE, INCLUDING
 * IMPLIED WARRANTIES OF MERCHANTABILITY, NON-INFRINGEMENT AND FITNESS FOR
 * PARTICULAR PURPOSE OR ARISING FROM COURSE OF PERFORMANCE, COURSE OF DEALING
 * OR USAGE OF TRADE.  SOME JURISDICTIONS DO NOT ALLOW THE EXCLUSION OF CERTAIN
 * WARRANTIES, REPRESENTATIONS OR CONDITIONS, THE LIMITATION OR EXCLUSION OF
 * IMPLIED WARRANTIES, LIMITATIONS ON HOW LONG AN IMPLIED WARRANTY MAY LAST OR
 * EXCLUSIONS OR LIMITATIONS FOR CONSEQUENTIAL OR INCIDENTAL DAMAGES, SO SOME OF
 * THE ABOVE LIMITATIONS MAY NOT APPLY IN FULL TO YOU, AND WAYMO’S LIABILITY
 * SHALL BE LIMITED TO THE EXTENT SUCH LIMITATIONS ARE PERMITTED BY LAW.
 *
 * 3. TO THE MAXIMUM EXTENT PERMITTED UNDER APPLICABLE LAW, IN NO EVENT WILL
 * WAYMO OR ITS LICENSORS BE LIABLE FOR ANY DIRECT, INDIRECT, SPECIAL,
 * INCIDENTAL, EXEMPLARY, PUNITIVE OR CONSEQUENTIAL DAMAGES OF ANY KIND ARISING
 * OUT OF OR IN CONNECTION WITH THE WAYMO SOFTWARE, REGARDLESS OF THE FORM OF
 * ACTION, WHETHER IN CONTRACT, TORT (INCLUDING NEGLIGENCE), STRICT LIABILITY OR
 * OTHERWISE, EVEN IF WAYMO HAS BEEN ADVISED OR IS OTHERWISE AWARE OF THE
 * POSSIBILITY OF SUCH DAMAGES. THE FOREGOING LIMITATIONS, EXCLUSIONS, AND
 * DISCLAIMERS WILL APPLY TO THE MAXIMUM EXTENT PERMITTED BY APPLICABLE LAW,
 * EVEN IF ANY REMEDY FAILS ITS ESSENTIAL PURPOSE.
 */
#include "hc/hc_node.h"

#include <getopt.h>
#include <sensor_msgs/Imu.h>

#include <algorithm>
#include <cmath>

#include "hc/pitch_table_utils.h"
#include "hc/scans.h"

namespace honeycomb {

HcRosNode::~HcRosNode() {
  config_cv_.notify_all();
  if (points_publisher_thread_.joinable()) {
    points_publisher_thread_.join();
  }
  if (imu_publisher_thread_.joinable()) {
    imu_publisher_thread_.join();
  }
}

bool HcRosNode::Init() {
  waymo::Status status;
  std::string hostname;
  std::string mac_address;

  pnh_ = ::ros::NodeHandle("~");
  pnh_.param<std::string>("hostname", hostname, "");
  pnh_.param<std::string>("mac_address", mac_address, "");

  lidar_ = std::unique_ptr<waymo::Honeycomb>(new waymo::Honeycomb());
  if (!lidar_) {
    ROS_FATAL("Unable to create Waymo Lidar");
    return false;
  }

  if ((status = lidar_->Init(hostname, mac_address)) != waymo::Status::kOk) {
    ROS_FATAL_STREAM("Unable to initialize lidar: " << status);
    return false;
  }

  // Generate demo pitch tables
  GeneratePitchTables();

  points_publisher_ = pnh_.advertise<sensor_msgs::PointCloud2>("/points", 100);
  range_image_publisher_ =
      pnh_.advertise<sensor_msgs::Image>("/range_image", 100);
  intensity_image_publisher_ =
      pnh_.advertise<sensor_msgs::Image>("/intensity_image", 100);
  imu_publisher_ = pnh_.advertise<sensor_msgs::Imu>("/imu/data_raw", 200);
  normals_publisher_ =
      pnh_.advertise<visualization_msgs::Marker>("/normals", 100);

  dynamic_reconfigure_server_ =
      std::unique_ptr<dynamic_reconfigure::Server<hc::HcNodeConfig>>(
          new dynamic_reconfigure::Server<hc::HcNodeConfig>());
  dynamic_reconfigure_server_->setCallback(
      boost::bind(&HcRosNode::dynamic_reconfigureCB, this, _1, _2));

  return ::ros::ok();
}

void HcRosNode::dynamic_reconfigureCB(const hc::HcNodeConfig &new_config,
                                      uint32_t level) {
  waymo::Status status = waymo::Status::kOk;

  if ((status = lidar_->SetSpinFrequency(new_config.spin_frequency)) !=
      waymo::Status::kOk) {
    ROS_ERROR_STREAM("Failed to update spin frequency: " << status);
  }

  if ((status = lidar_->SetVerticalScanFrequency(
           new_config.vertical_scan_frequency)) != waymo::Status::kOk) {
    ROS_ERROR_STREAM("Failed to update vertical scan frequency: " << status);
  }

  if ((status = lidar_->SetFieldOfView(new_config.fov, new_config.direction)) !=
      waymo::Status::kOk) {
    ROS_ERROR_STREAM("Failed to update field of view: " << status);
  }

  if ((status = lidar_->SetSides(static_cast<waymo::Sides>(
           new_config.sides))) != waymo::Status::kOk) {
    ROS_ERROR_STREAM("Failed to update sides: " << status);
  }

  if ((status = lidar_->SetComputeNormals(static_cast<waymo::Sides>(
           new_config.compute_normals))) != waymo::Status::kOk) {
    ROS_ERROR_STREAM("Failed to update normal computation enable: " << status);
  }

  if ((status = lidar_->SetScansPerShard(new_config.scans_per_point_cloud)) !=
      waymo::Status::kOk) {
    ROS_ERROR_STREAM("Failed to update scans per point cloud: " << status);
  }

  if ((status = lidar_->SetDropNoiseReturns(new_config.drop_noise_returns)) !=
      waymo::Status::kOk) {
    ROS_ERROR_STREAM("Failed to set drop noise returns: " << status);
  }

  if (level & hc::HcNode_Pitch_Table_Change) {
    std::vector<double> pitch_table;
    std::string error_str;
    if (new_config.pitch_table == hc::HcNode_Custom) {
      pitch_table =
          ParsePitchTableString(new_config.custom_pitch_table, &error_str);

    } else {
      pitch_table = pitch_tables_[new_config.pitch_table];
    }
    if (pitch_table.empty()) {
      ROS_ERROR("Unable to parse custom pitch table '%s'.", error_str.c_str());
    } else if ((status = lidar_->SetPitchTable(pitch_table,
                                               new_config.interlaced)) !=
               waymo::Status::kOk) {
      ROS_ERROR_STREAM("Failed to update pitch table: " << status);
    } else {
      GetCurrentPitchTable();
    }
  }

  if (level & hc::HcNode_Calibration_Change) {
    waymo::WL_Calibration cal;
    // Read persistent storage for factory calibration.
    if ((status = lidar_->GetCalibrationParameters(true, &cal)) !=
        waymo::Status::kOk) {
      ROS_ERROR_STREAM(
          "Unable to retrieve calibration from device: " << status);
    }
    cal.azimuth_adjustment += new_config.azimuth_adjustment_delta;
    cal.elevation_adjustment += new_config.elevation_adjustment_delta;
    cal.distance_offset += new_config.distance_offset_delta;
    cal.comparator_threshold += new_config.comparator_threshold_delta;
    // Set current calibration, but don't update persistent storage.
    if ((status = lidar_->SetCalibrationParameters(false, &cal)) !=
        waymo::Status::kOk) {
      ROS_ERROR_STREAM("Failed to update calibration: " << status);
    }
  }

  {
    std::lock_guard<std::mutex> config_lck(config_mtx_);  // NOLINT
    config_ = new_config;
  }  // End config lock.
  config_cv_.notify_all();
}

waymo::Status HcRosNode::Run() {
  waymo::Status status = lidar_->Run();
  if (status != waymo::Status::kOk) {
    return status;
  }

  if ((status = lidar_->GetSystemInfo(&system_info_)) != waymo::Status::kOk) {
    return status;
  }
  ROS_INFO(
      "Honeycomb information:\n"
      "  IP Address: %s\n"
      "  MAC Address: %s\n"
      "  Serial Number: %s\n"
      "  Model Number: %s\n"
      "  Software Version: %s\n"
      "  Software Build Date: %s",
      system_info_.ip_address.c_str(), system_info_.mac_address.c_str(),
      system_info_.serial_number.c_str(), system_info_.model_number.c_str(),
      system_info_.software_build_version.c_str(),
      system_info_.software_build_date.c_str());

  diagnostic_updater::Updater updater;
  updater.setHardwareID(system_info_.serial_number);
  updater.add("Statistics", this, &HcRosNode::UpdateStatistics);
  updater.add("Status", this, &HcRosNode::UpdateStatus);
  updater.add("System Errors", this, &HcRosNode::UpdateSystemErrors);
  updater.add("System Info", this, &HcRosNode::UpdateSystemInfo);

  points_publisher_thread_ = std::thread(&HcRosNode::PublishPoints, this);
  imu_publisher_thread_ = std::thread(&HcRosNode::PublishImu, this);

  ros::Rate r(10);  // 10 hz
  while (ros::ok() && lidar_->IsRunning()) {
    ros::spinOnce();
    updater.update();
    r.sleep();
  }
  return lidar_->Stop();
}

void HcRosNode::UpdateSystemErrors(
    diagnostic_updater::DiagnosticStatusWrapper &stat) {
  waymo::Status status;
  std::vector<waymo::Honeycomb::SystemError> system_errors;
  if ((status = lidar_->GetSystemErrors(&system_errors)) !=
      waymo::Status::kOk) {
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR,
                  "Failed to get system errors: %s",
                  waymo::StatusToString(status).c_str());
    return;
  }

  int8_t level = diagnostic_msgs::DiagnosticStatus::OK;
  int num_errors = 0;
  for (const auto &error : system_errors) {
    std::string msg;
    if (error.code.empty()) {
      msg = error.description;
    } else {
      msg = error.code + ": " + error.description;
    }
    switch (error.severity) {
      case waymo::Severity::kError:
        ++num_errors;
        stat.add(error.code, error.description);
        ROS_ERROR_STREAM_NAMED("diagnostics", msg);
        level = diagnostic_msgs::DiagnosticStatus::ERROR;
        break;
      case waymo::Severity::kWarning:
        ROS_WARN_STREAM_NAMED("diagnostics", msg);
        level = std::max(level, static_cast<int8_t>(
                                    diagnostic_msgs::DiagnosticStatus::WARN));
        break;
      case waymo::Severity::kLog:
        ROS_INFO_STREAM_NAMED("diagnostics", msg);
        break;
      case waymo::Severity::kDebug:
      default:
        ROS_DEBUG_STREAM_NAMED("diagnostics", msg);
        break;
    }
  }
  stat.summaryf(level, "Errors reported: %d", num_errors);
}

void HcRosNode::UpdateSystemInfo(
    diagnostic_updater::DiagnosticStatusWrapper &stat) {
  stat.add("IP Address", system_info_.ip_address);
  stat.add("MAC Address", system_info_.mac_address);
  stat.add("Serial Number", system_info_.serial_number);
  stat.add("Model Number", system_info_.model_number);
  stat.add("Software Version", system_info_.software_build_version);
  stat.add("Software Build Date", system_info_.software_build_date);
  stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "OK");
}

void HcRosNode::UpdateStatus(
    diagnostic_updater::DiagnosticStatusWrapper &stat) {
  waymo::Status status;
  waymo::SystemStatus system_status;
  if ((status = lidar_->GetSystemStatus(&system_status)) !=
      waymo::Status::kOk) {
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR,
                  "Failed to get system status: %s",
                  waymo::StatusToString(status).c_str());
    return;
  }
  int level;
  if (system_status == waymo::SystemStatus::kRunning) {
    level = diagnostic_msgs::DiagnosticStatus::OK;
  } else {
    level = diagnostic_msgs::DiagnosticStatus::WARN;
  }
  stat.summary(level, waymo::SystemStatusToString(system_status));
}

void HcRosNode::UpdateStatistics(
    diagnostic_updater::DiagnosticStatusWrapper &stat) {
  waymo::Status status;
  waymo::Honeycomb::SystemStats system_stats;
  if ((status = lidar_->GetStatsData(0, true, &system_stats)) !=
      waymo::Status::kOk) {
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR,
                  "Failed to get stats data: %s",
                  waymo::StatusToString(status).c_str());
    return;
  }
  for (const auto &p : system_stats) {
    stat.add(p.first, p.second.GetValue());
  }
  stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "OK");
}

void HcRosNode::GetCurrentPitchTable() {
  waymo::Status status;
  if ((status = lidar_->GetPitchTable(&current_pitch_table_,
                                      &is_interlaced_)) != waymo::Status::kOk) {
    ROS_ERROR_STREAM("Failed to fetch default pitch table: " << status);
  }
  if ((status = lidar_->GetPitchTableLimits(&pitch_table_limits_)) !=
      waymo::Status::kOk) {
    ROS_ERROR_STREAM("Failed to fetch pitch table limits: " << status);
  }
}

void HcRosNode::GeneratePitchTables() {
  GetCurrentPitchTable();
  pitch_tables_[hc::HcNode_Default] = current_pitch_table_;

  std::string error_str;
  // Full FoV (high res)
  const double pitch_spacing = -std::max(
      pitch_table_limits_.min_spacing_degrees,
      (pitch_table_limits_.max_degrees - pitch_table_limits_.min_degrees) /
          (pitch_table_limits_.max_entries - 1));
  std::string full_res =
      std::to_string(pitch_table_limits_.max_degrees) + ":" +  // NOLINT
      std::to_string(pitch_table_limits_.min_degrees) + ":" +  // NOLINT
      std::to_string(pitch_spacing);                           // NOLINT
  pitch_tables_[hc::HcNode_Full_Fov_High_Res] =
      ParsePitchTableString(full_res, &error_str);
}

void HcRosNode::PublishPoints() {
  waymo::ScansPtr scans;
  while (::ros::ok() && lidar_->IsRunning()) {
    hc::HcNodeConfig config;
    {
      std::lock_guard<std::mutex> config_lck(config_mtx_);  // NOLINT
      config = config_;
    }  // End config lock.
    if (lidar_->GetLidarData(waymo::kDurationForever,
                             /*latest=*/config.scans_per_point_cloud == 0,
                             &scans) == waymo::Status::kOk) {
      points_publisher_.publish(ScansToPoints(scans.get(), config));
      if (config.publish_normal_markers) {
        normals_publisher_.publish(
            ScansToMarkers(scans.get(), config.frame_id));
      }

      // Generate Scan Image output
      if (range_image_publisher_.getNumSubscribers() > 0) {
        range_image_publisher_.publish(scan_image_producer_.ScansToImage(
            scans.get(), config, current_pitch_table_,
            ScanImageProducer::ImageType::kRange));
      }
      if (intensity_image_publisher_.getNumSubscribers() > 0) {
        intensity_image_publisher_.publish(scan_image_producer_.ScansToImage(
            scans.get(), config, current_pitch_table_,
            ScanImageProducer::ImageType::kIntensity));
      }
    }
  }
}

void HcRosNode::PublishImu() {
  std::vector<waymo::WL_ImuSample> samples;
  constexpr double kDegreesToRadians = M_PI / 180.0;
  constexpr double kGravityConstant = 9.80665;  // [m/s^2]

  while (::ros::ok() && lidar_->IsRunning()) {
    hc::HcNodeConfig config;
    {
      std::unique_lock<std::mutex> config_lck(config_mtx_);  // NOLINT
      if (!config_.publish_imu_data) {
        config_cv_.wait(config_lck);
        continue;
      }
      config = config_;
    }  // End config lock.
    if (lidar_->GetImuData(waymo::kDurationForever, &samples) ==
        waymo::Status::kOk) {
      for (const auto &sample : samples) {
        sensor_msgs::ImuPtr ros_msg(new sensor_msgs::Imu());
        ros_msg->header.stamp.fromSec(sample.timestamp);
        ros_msg->header.frame_id = config.imu_frame_id;
        ros_msg->orientation_covariance[0] = -1;

        // Convert linear acceleration from g to m/s^2
        const auto &la = sample.linear_acceleration;
        ros_msg->linear_acceleration.x = kGravityConstant * la.x;
        ros_msg->linear_acceleration.y = kGravityConstant * la.y;
        ros_msg->linear_acceleration.z = kGravityConstant * la.z;

        // Convert angular velocity from deg/s to rad/s
        const auto &av = sample.angular_velocity;
        ros_msg->angular_velocity.x = kDegreesToRadians * av.x;
        ros_msg->angular_velocity.y = kDegreesToRadians * av.y;
        ros_msg->angular_velocity.z = kDegreesToRadians * av.z;

        imu_publisher_.publish(ros_msg);
      }
    }
  }
}
}  // namespace honeycomb

void usage(const char *progname) {
  std::cout << "Usage: " << basename(progname) << " [options]\n"
            << "-l, --opensourcelicenses  Print open-source license notices.\n"
            << "-h, --help                Show help.\n";
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "hc_node");

  int opt;
  const char *short_opts = "lh";
  const option long_opts[] = {
      {"opensourcelicenses", no_argument, nullptr, 'l'},
      {"help", no_argument, nullptr, 'h'},
      {nullptr, no_argument, nullptr, 0},
  };

  while ((opt = getopt_long(argc, argv, short_opts, long_opts, nullptr)) !=
         -1) {
    switch (opt) {
      case 'l': {
        const char *notices = waymo::internal::WL_GetLicenseNotices();
        std::cout << notices << std::endl;
        return 0;
      }
      case 'h':
      case '?':
      default:
        usage(argv[0]);
        return 0;
    }
  }

  honeycomb::HcRosNode node;
  if (node.Init()) {
    auto status = node.Run();
    return status != waymo::Status::kOk;
  }

  return 1;
}

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
 * \file test_himmelsbach.cpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "vtr_lidar/segmentation/himmelsbach.hpp"
#include "vtr_lidar/types.hpp"
#include "vtr_logging/logging_init.hpp"

// pcl type visualization implementations
// must include after type definition where PCL_NO_PRECOMPILE macro is defined
#include <pcl/visualization/impl/pcl_visualizer.hpp>
#include <pcl/visualization/impl/point_cloud_geometry_handlers.hpp>

using namespace std::chrono_literals;

using namespace vtr;
using namespace vtr::logging;
using namespace vtr::lidar;

namespace {

float getFloatFromByteArray(char *byteArray, uint index) {
  return *((float *)(byteArray + index));
}

int64_t getStampFromPath(const std::string &path) {
  std::vector<std::string> parts;
  boost::split(parts, path, boost::is_any_of("/"));
  std::string stem = parts[parts.size() - 1];
  boost::split(parts, stem, boost::is_any_of("."));
  int64_t time1 = std::stoll(parts[0]);
  return time1;
}

// Input is a .bin binary file.
std::pair<int64_t, Eigen::MatrixXd> load_lidar(const std::string &path) {
  std::ifstream ifs(path, std::ios::binary);
  std::vector<char> buffer(std::istreambuf_iterator<char>(ifs), {});
  uint float_offset = 4;
  uint fields = 6;  // x, y, z, i, r, t
  uint point_step = float_offset * fields;
  uint N = floor(buffer.size() / point_step);
  Eigen::MatrixXd pc(Eigen::MatrixXd::Ones(N, fields));
  for (uint i = 0; i < N; ++i) {
    uint bufpos = i * point_step;
    for (uint j = 0; j < fields; ++j) {
      pc(i, j) =
          getFloatFromByteArray(buffer.data(), bufpos + j * float_offset);
    }
  }
  // Add offset to timestamps
  const auto time_micro = getStampFromPath(path);
  double t = double(time_micro) * 1.0e-6;
  pc.block(0, 5, N, 1).array() += t;

  return std::make_pair<int64_t, Eigen::MatrixXd>(time_micro * 1e3,
                                                  std::move(pc));
}

}  // namespace

int main(int, char **) {
  configureLogging("", true);

  Himmelsbach<PointWithInfo> himmelsbach;

  /// \todo hardcoded path
  std::string binary_path(
      "/ext0/datasets/boreas/sequences/boreas-2020-12-01-13-26/lidar/"
      "1606847188054077.bin");
  const auto [time_micro, points] = load_lidar(binary_path);

  using PointCloud = pcl::PointCloud<PointWithInfo>;
  PointCloud::Ptr point_cloud(new PointCloud(points.rows(), 1));
  for (size_t idx = 0; idx < (size_t)points.rows(); idx++) {
    // cartesian coordinates
    point_cloud->at(idx).x = points(idx, 0);
    point_cloud->at(idx).y = points(idx, 1);
    point_cloud->at(idx).z = points(idx, 2);
  }

  const auto ground_idx = himmelsbach(*point_cloud);
  PointCloud::Ptr ground_cloud(new PointCloud);
  for (const auto &idx : ground_idx)
    ground_cloud->push_back(point_cloud->at(idx));

  // clang-format off
  using namespace pcl::visualization;
  PCLVisualizer::Ptr viewer(new PCLVisualizer("3D Viewer"));
  viewer->setBackgroundColor(0, 0, 0);
  PointCloudColorHandlerCustom<PointWithInfo> color(point_cloud, 0, 255, 0);
  viewer->addPointCloud<PointWithInfo>(point_cloud, color, "original");
  viewer->setPointCloudRenderingProperties(PCL_VISUALIZER_POINT_SIZE, 2, "original");
  PointCloudColorHandlerCustom<PointWithInfo> ground_color(ground_cloud, 255, 0, 0);
  viewer->addPointCloud<PointWithInfo>(ground_cloud, ground_color, "ground");
  viewer->setPointCloudRenderingProperties(PCL_VISUALIZER_POINT_SIZE, 2, "ground");
  viewer->addCoordinateSystem(1.0);
  viewer->initCameraParameters();

  while (!viewer->wasStopped()) {
    viewer->spinOnce(100);
    std::this_thread::sleep_for(100ms);
  }

  return 0;
}
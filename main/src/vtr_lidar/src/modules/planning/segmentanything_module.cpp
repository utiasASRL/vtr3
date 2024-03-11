// Copyright 2024, Autonomous Space Robotics Lab (ASRL)
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
 * \file segmentanything_module.cpp
 * \author Alec Krawciw, Autonomous Space Robotics Lab (ASRL)
 */
#include <vtr_lidar/modules/planning/segmentanything_module.hpp>

#include <opencv2/imgproc.hpp>
#include "cv_bridge/cv_bridge.h"


namespace vtr {
namespace lidar {

using namespace tactic;

auto SegmentAnythingModule::Config::fromROS(const rclcpp::Node::SharedPtr &node,
                                          const std::string &param_prefix)
    -> ConstPtr {
  auto config = std::make_shared<SegmentAnythingModule::Config>();
  auto base_config = std::static_pointer_cast<TorchModule::Config>(config);
  *base_config =  *nn::TorchModule::Config::fromROS(node, param_prefix);


  auto& params = config->perspective_params;

  params.width = node->declare_parameter<int>(param_prefix + ".img_width", 0);
  params.height = node->declare_parameter<int>(param_prefix + ".img_height", 0);
  params.h_fov = node->declare_parameter<double>(param_prefix + ".h_fov", M_PI/2);
  params.v_fov = node->declare_parameter<double>(param_prefix + ".v_fov", M_PI/4);
  params.max_range = node->declare_parameter<double>(param_prefix + ".max_range", params.max_range);
  params.min_range = node->declare_parameter<double>(param_prefix + ".min_range", params.min_range);
  
  config->iou_thresh = node->declare_parameter<float>(param_prefix + ".iou_threshold", config->iou_thresh);
  config->num_prompts = node->declare_parameter<long>(param_prefix + ".num_prompts", config->num_prompts);

  // clang-format off
  config->visualize = node->declare_parameter<bool>(param_prefix + ".visualize", config->visualize);

  // clang-format on
  return config;
}


void SegmentAnythingModule::run_(QueryCache &qdata0, OutputCache &,
                            const Graph::Ptr &graph, const TaskExecutor::Ptr &) {
  auto &qdata = dynamic_cast<LidarQueryCache &>(qdata0);


  // if(!qdata.rendered_images.valid()) {
  //   CLOG(WARNING, "lidar.perspective") << "Rendered perspective images required a map to work!";
  //   return;
  // }

  if(!pub_init_){
    mask_pub_ = qdata.node->create_publisher<ImageMsg>("detection_mask", 5);
    diff_pub_ = qdata.node->create_publisher<ImageMsg>("normed_diff", 5);
    live_img_pub_ = qdata.node->create_publisher<ImageMsg>("live_range_coloured", 5);
    map_img_pub_ = qdata.node->create_publisher<ImageMsg>("map_range_coloured", 5);
    filtered_pub_ = qdata.node->create_publisher<PointCloudMsg>("changed_point_cloud", 5);

    pub_init_ = true;
  }

  auto raw_point_cloud = *qdata.raw_point_cloud;

  if(!qdata.submap_loc.valid()) {
    CLOG(WARNING, "lidar.perspective") << "Perspective image requires a map to work";
    return;
  }

  const auto &T_s_r = *qdata.T_s_r;  
  const auto &T_r_v_loc = *qdata.T_r_v_loc;
  const auto &T_v_m_loc = *qdata.T_v_m_loc;
  const auto &vid_loc = *qdata.vid_loc;

  auto map_vertex = graph->at(vid_loc);
  auto nn_map_scan = [&] {
      auto locked_nn_pc_msg = map_vertex->retrieve<PointScan<PointWithInfo>>(
            "raw_point_cloud", "vtr_lidar_msgs/msg/PointScan");

      if (locked_nn_pc_msg != nullptr) {
        auto locked_msg = locked_nn_pc_msg->sharedLocked();
        return locked_msg.get().getDataPtr();
      }
      return std::make_shared<PointScan<PointWithInfo>>();
  }();

  auto nn_map_point_cloud = nn_map_scan->point_cloud();
  
  auto map_nn_mat = nn_map_point_cloud.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::cartesian_offset());
  const auto T_s_sm = (T_s_r * T_r_v_loc * nn_map_scan->T_vertex_this()).matrix();
  

  auto& sub_map= *qdata.submap_loc;
  auto map_point_cloud = sub_map.point_cloud();
  auto map_points_mat = map_point_cloud.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::cartesian_offset());
  
  const auto T_s_m = (T_s_r * T_r_v_loc * T_v_m_loc).matrix();


  auto live_points_mat = raw_point_cloud.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::cartesian_offset());

  Eigen::Matrix4f T_c_s;

  T_c_s << 0, -1, 0, 0,
           0, 0, 1, 0,
           -1, 0, 0, 0,
           0, 0, 0, 1;

  live_points_mat = T_c_s * live_points_mat;
  map_points_mat = (T_c_s * T_s_m.cast<float>()) * map_points_mat;
  map_nn_mat = (T_c_s * T_s_sm.cast<float>()) * map_nn_mat;

  

  cv::Mat live_index_img = cv::Mat::zeros(config_->perspective_params.height, config_->perspective_params.width, CV_32S);
  cv::Mat live_hsv_img = cv::Mat::zeros(config_->perspective_params.height, config_->perspective_params.width, CV_8UC3);
  cv::Mat live_rgb_img = cv::Mat::zeros(config_->perspective_params.height, config_->perspective_params.width, CV_8UC3);

  generate_depth_image(raw_point_cloud, live_hsv_img, live_index_img, config_->perspective_params);
  interpolate_hsv_image(live_hsv_img);

  cv::cvtColor(live_hsv_img, live_rgb_img, cv::COLOR_HSV2RGB);

  if (config_->visualize) {
    cv_bridge::CvImage live_cv_rgb_img;
    live_cv_rgb_img.header.frame_id = "lidar";
    //cv_rgb_img.header.stamp = qdata.stamp->header.stamp;
    live_cv_rgb_img.encoding = "rgb8";
    live_cv_rgb_img.image = live_rgb_img;
    live_img_pub_->publish(*live_cv_rgb_img.toImageMsg());
  }

  cv::Mat map_index_img = cv::Mat::zeros(config_->perspective_params.height, config_->perspective_params.width, CV_32S);
  cv::Mat map_hsv_img = cv::Mat::zeros(config_->perspective_params.height, config_->perspective_params.width, CV_8UC3);
  cv::Mat map_rgb_img = cv::Mat::zeros(config_->perspective_params.height, config_->perspective_params.width, CV_8UC3);


  // generate_depth_image(map_point_cloud, map_hsv_img, map_index_img, config_->perspective_params);
  generate_depth_image(nn_map_point_cloud, map_hsv_img, map_index_img, config_->perspective_params);
  interpolate_hsv_image(map_hsv_img);

  
  cv::cvtColor(map_hsv_img, map_rgb_img, cv::COLOR_HSV2RGB);

  // qdata.rendered_images.emplace(std::make_pair(live_rgb_img, map_rgb_img));

  if (config_->visualize) {
    cv_bridge::CvImage map_cv_rgb_img;
    map_cv_rgb_img.header.frame_id = "lidar";
    //cv_rgb_img.header.stamp = qdata.stamp->header.stamp;
    map_cv_rgb_img.encoding = "rgb8";
    map_cv_rgb_img.image = map_rgb_img;
    map_img_pub_->publish(*map_cv_rgb_img.toImageMsg());
  }

  CLOG(DEBUG, "lidar.perspective") << "Received images! ";

  cv::Mat big_live;
  cv::Mat big_map;

  cv::resize(live_rgb_img, big_live, cv::Size(1024, 512), cv::INTER_LINEAR);
  cv::resize(map_rgb_img, big_map, cv::Size(1024, 512), cv::INTER_LINEAR);

  if (!big_live.isContinuous()){
    big_live = big_live.clone();
    CLOG(WARNING, "lidar.perspective") << "Live image was not contiguous in memory!";
  }
  if (!big_map.isContinuous()){
    big_map = big_map.clone();
    CLOG(WARNING, "lidar.perspective") << "Map image was not contiguous in memory!";
  }


  torch::Tensor live_tensor = torch::from_blob(big_live.data, {big_live.rows, big_live.cols, big_live.channels()}, torch::kByte).to(torch::kFloat32).permute({2, 0, 1}).contiguous();
  torch::Tensor map_tensor = torch::from_blob(big_map.data, {big_map.rows, big_map.cols, big_map.channels()}, torch::kByte).to(torch::kFloat32).permute({2, 0, 1}).contiguous();

  torch::Tensor diff = live_tensor - map_tensor;
  diff = diff.norm(2, {0});
  const auto [topk_vals, topk_idx] = diff.flatten().topk(config_->num_prompts);

  using namespace torch::indexing;

  // int data[] = { 512, 256,
  //                256, 256,
  //                768, 256 };
  std::vector<torch::Tensor> prompts;

  auto idxs_a = topk_idx.accessor<long, 1>();

  for (size_t i = 0; i < idxs_a.size(0); i++) {
    const long& idx = idxs_a[i];
    int prompt[] = {idx % diff.size(1), idx / diff.size(1)};

    auto& pc_idx = live_index_img.at<uint32_t>(prompt[1] / 4, prompt[0] / 4);

    auto live_point = raw_point_cloud[pc_idx];
    Eigen::Vector4f h_point;
    h_point << live_point.x, live_point.y, live_point.z, 1.0f;
    h_point = T_s_sm.inverse().cast<float>() * h_point;

    double theta = atan2(h_point[1], h_point[0]);
    CLOG(DEBUG, "lidar.perspective") << "Diff Tensor prompt (" << prompt[0] << ", " << prompt[1] << ") Theta: " << theta;


    //Prompts are x, y rather than row, column
    //map_tensor.index({0, prompt[1], prompt[0]}).item().to<float>() > 0  || 
    if (!((theta > 2.06 && theta < 2.18) || (theta > -2.27 && theta < -2.135)))
      prompts.push_back(torch::from_blob(prompt, {2}, torch::kInt).to(device));  
    else
      CLOG(DEBUG, "lidar.perspective") << "Prompt point on an empty map pixel. Try again.";
  }
  
  // prompts.push_back(torch::from_blob(&data[2], {2}, torch::kInt).to(device));  
  // prompts.push_back(torch::from_blob(&data[4], {2}, torch::kInt).to(device));  

  torch::NoGradGuard no_grad;
  std::vector<torch::jit::IValue> jit_inputs;

  jit_inputs.push_back(map_tensor.to(device));
  jit_inputs.push_back(live_tensor.to(device));
  jit_inputs.push_back(prompts);

  auto outputs = network(jit_inputs).toGenericDict();

  CLOG(DEBUG, "lidar.perspective") << "Ran model!";

  auto teach_masks = outputs.at("teach_masks").toTensor();
  auto repeat_masks = outputs.at("repeat_masks").toTensor();

  auto intersection = teach_masks.bitwise_and(repeat_masks);
  auto unions = teach_masks.bitwise_or(repeat_masks);

  auto ious = intersection.sum({1, 2}) / unions.sum({1, 2});

  CLOG(DEBUG, "lidar.perspective") << "IoUS " << ious.cpu();

  std::vector<unsigned int> idxs_to_keep;
  auto ious_a = ious.accessor<float, 1>();


  for (size_t i = 0; i < ious_a.size(0); i++) {
    CLOG(DEBUG, "lidar.perspective") << "Iou VAL " << ious_a[i] << " < " << config_->iou_thresh;

    if (ious_a[i] < config_->iou_thresh) {
      idxs_to_keep.push_back(i);
    }
  }

  CLOG(DEBUG, "lidar.perspective") << "IDs to KEEP " << idxs_to_keep;

  cv::Mat teach_mask = cv::Mat::zeros(teach_masks.size(1), teach_masks.size(2), CV_8UC1);
  cv::Mat repeat_mask = cv::Mat::zeros(repeat_masks.size(1), repeat_masks.size(2), CV_8UC1);

  if (idxs_to_keep.size() > 0){
    auto bg_flat = teach_masks.sum({0});
    bg_flat = bg_flat.div(bg_flat.max()).to(torch::kByte).mul(255);
    teach_mask = cv::Mat{bg_flat.size(0), bg_flat.size(1), CV_8UC1, bg_flat.data_ptr<uint8_t>()};


    auto changes = repeat_masks.index({torch::from_blob(idxs_to_keep.data(), {idxs_to_keep.size()}, torch::kInt), Slice(), Slice()});
    CLOG(DEBUG, "lidar.perspective") << "Repeat Tensor size " << changes.sizes();

    changes = changes.sum({0}).div(changes.max()).to(torch::kByte).mul(255);

    repeat_mask = cv::Mat{changes.size(0), changes.size(1), CV_8UC1, changes.data_ptr<uint8_t>()};


    mask_to_pointcloud(repeat_mask, live_index_img, raw_point_cloud);

    auto filtered_point_cloud =
        std::make_shared<pcl::PointCloud<PointWithInfo>>(raw_point_cloud);

    /// changed cropping
    {
      std::vector<int> indices;
      indices.reserve(filtered_point_cloud->size());
      for (size_t i = 0; i < filtered_point_cloud->size(); ++i) {
        if ((*filtered_point_cloud)[i].flex24 > 0)
          indices.emplace_back(i);
      }
      *filtered_point_cloud =
          pcl::PointCloud<PointWithInfo>(*filtered_point_cloud, indices);
    }

    if (config_->visualize) {
      PointCloudMsg pc2_msg;
      pcl::toROSMsg(*filtered_point_cloud, pc2_msg);
      pc2_msg.header.frame_id = "lidar"; //"lidar" for honeycomb
      pc2_msg.header.stamp = rclcpp::Time(*qdata.stamp);
      filtered_pub_->publish(pc2_msg);
    }

  } else {
    if (config_->visualize) {
      PointCloudMsg pc2_msg;
      pcl::toROSMsg(raw_point_cloud, pc2_msg);
      pc2_msg.header.frame_id = "lidar"; //"lidar" for honeycomb
      pc2_msg.header.stamp = rclcpp::Time(*qdata.stamp);
      filtered_pub_->publish(pc2_msg);
    }
  }

  // /// publish the transformed pointcloud
  if (config_->visualize) {
    cv_bridge::CvImage mask_img_msg;
    mask_img_msg.header.frame_id = "lidar";
    //cv_rgb_img.header.stamp = qdata.stamp->header.stamp;
    mask_img_msg.encoding = "mono8";
    mask_img_msg.image = repeat_mask;
    mask_pub_->publish(*mask_img_msg.toImageMsg());

    diff = diff.div(diff.max()).mul(255).to(torch::kByte).contiguous();
    CLOG(DEBUG, "lidar.perspective") << "Max: " << diff.max() << " Min: " << diff.min();

    cv_bridge::CvImage diff_img_msg;
    diff_img_msg.header.frame_id = "lidar";
    //cv_rgb_img.header.stamp = qdata.stamp->header.stamp;
    diff_img_msg.encoding = "mono8";
    // diff_img_msg.image = cv::Mat{diff.size(0), diff.size(1), CV_8UC1, diff.data_ptr<uint8_t>()};;
    diff_img_msg.image = teach_mask;
    diff_pub_->publish(*diff_img_msg.toImageMsg());
  }
                            
}

}  // namespace nn
}  // namespace vtr
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
#include <pcl/common/common.h>
#include "cv_bridge/cv_bridge.h"
// #include <opencv2/quality/qualityssim.hpp>
#include <vtr_lidar/filters/corridor_filter.hpp>


#include <opencv2/opencv.hpp>

using namespace cv;

namespace {
// size comparison, for list sorting
bool compare_size(KeyPoint first, KeyPoint second)
{
  if (first.size > second.size) return true;
  else return false;
}

std::vector<KeyPoint> detectBlobsSortedBySize(Mat im)
{
	//Invert image
	im = 255 - im;
	
	// Setup SimpleBlobDetector parameters.
	SimpleBlobDetector::Params params;
	
	// Change thresholds
	params.minThreshold = 10;
	params.maxThreshold = 200;
	
	// Filter by Area.
	params.filterByArea = false;
	params.minArea = 25;
	
	// Filter by Circularity
	params.filterByCircularity = false;
	params.minCircularity = 0.1;
	
	// Filter by Convexity
	params.filterByConvexity = false;
	params.minConvexity = 0.87;
	
	// Filter by Inertia
	params.filterByInertia = false;
	params.minInertiaRatio = 0.01;
	
	
	params.minDistBetweenBlobs = 1;
	
	
	// Set up the detector with default parameters.
	Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);
	
	// Detect blobs.
	std::vector<KeyPoint> keypoints;
	detector->detect( im, keypoints);
	
	//Sort by size
	std::sort (keypoints.begin(), keypoints.end(), compare_size); 
	
	return keypoints;

}

}



namespace vtr {
namespace lidar {

using namespace tactic;
using Image_LockMsg = storage::LockableMessage<sensor_msgs::msg::Image>;


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
  config->smooth_size = node->declare_parameter<long>(param_prefix + ".smooth_size", config->smooth_size);
  config->corridor_width = node->declare_parameter<double>(param_prefix + ".corridor_width", config->corridor_width);
  config->max_size = node->declare_parameter<float>(param_prefix + ".max_size", config->max_size);

  // clang-format off
  config->visualize = node->declare_parameter<bool>(param_prefix + ".visualize", config->visualize);

  // clang-format on
  return config;
}

using BGR = typename cv::Scalar;

void SegmentAnythingModule::run_(QueryCache &qdata0, OutputCache &output,
                            const Graph::Ptr &graph, const TaskExecutor::Ptr &) {
  auto &qdata = dynamic_cast<LidarQueryCache &>(qdata0);

  static const std::vector<cv::Scalar> color_options {BGR(230, 25, 75), BGR(60, 180, 75), BGR(255, 225, 25), BGR(0, 130, 200), BGR(245, 130, 48), BGR(145, 30, 180),
  BGR(70, 240, 240), BGR(240, 50, 230), BGR(210, 245, 60), BGR(250, 190, 212), BGR(0, 128, 128), BGR(220, 190, 255), BGR(170, 110, 40), BGR(255, 250, 200), BGR(128, 0, 0),
   BGR(170, 255, 195), BGR(128, 128, 0), BGR(255, 215, 180), BGR(0, 0, 128)};


  if(!pub_init_){
    live_mask_pub_ = qdata.node->create_publisher<ImageMsg>("detection_mask", 5);
    map_mask_pub_ = qdata.node->create_publisher<ImageMsg>("background_mask", 5);
    diff_pub_ = qdata.node->create_publisher<ImageMsg>("normed_diff", 5);
    live_img_pub_ = qdata.node->create_publisher<ImageMsg>("live_range_coloured", 5);
    map_img_pub_ = qdata.node->create_publisher<ImageMsg>("map_range_coloured", 5);
    filtered_pub_ = qdata.node->create_publisher<PointCloudMsg>("changed_point_cloud", 5);

    pub_init_ = true;
  }

  if (!*qdata.run_cd)
    return;

  auto raw_point_cloud = *qdata.raw_point_cloud;

  if(!qdata.submap_loc.valid()) {
    CLOG(WARNING, "lidar.perspective") << "Perspective image requires a map to work";
    return;
  }

  const auto &T_s_r = *qdata.T_s_r;  
  const auto &T_r_v_loc = *qdata.T_r_v_loc;
  const auto &T_v_m_loc = *qdata.T_v_m_loc;
  const auto &vid_loc = *qdata.vid_loc;
  const auto &curr_sid = *qdata.sid_loc;

  auto vertex = graph->at(*qdata.vid_odo);

  
  auto &chain = *output.chain;
  const auto T_w_curr = chain.pose(curr_sid);


  auto map_vertex = graph->at(vid_loc);
  auto nn_map_scan = [&] {
      auto locked_nn_pc_msg = map_vertex->retrieve<PointScan<PointWithInfo>>(
            "raw_point_cloud", "vtr_lidar_msgs/msg/PointScan");

      if (locked_nn_pc_msg != nullptr) {
        auto locked_msg = locked_nn_pc_msg->sharedLocked();
        return locked_msg.get().getDataPtr();
      }
      CLOG(WARNING, "lidar.perspective") << "Could not load raw view from teach";
      return std::make_shared<PointScan<PointWithInfo>>();
  }();

  auto nn_map_point_cloud = nn_map_scan->point_cloud();
  
  auto map_nn_mat = nn_map_point_cloud.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::cartesian_offset());
  const auto T_s_sm = (T_s_r * T_r_v_loc * nn_map_scan->T_vertex_this()).matrix();
  

  auto& sub_map= *qdata.submap_loc;
  auto map_point_cloud = sub_map.point_cloud();
  auto map_points_mat = map_point_cloud.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::cartesian_offset());
  
  const auto T_s_m = T_s_r * T_r_v_loc * T_v_m_loc;


  auto live_points_mat = raw_point_cloud.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::cartesian_offset());

  Eigen::Matrix4d T_c_s_temp;

  T_c_s_temp << 0, -1, 0, 0,
           0, 0, 1, 0,
           -1, 0, 0, 0,
           0, 0, 0, 1;

  const tactic::EdgeTransform T_c_s {T_c_s_temp};
  const tactic::EdgeTransform T_v_loc_c = (T_c_s * T_s_r * T_r_v_loc).inverse();
  const auto T_cam_w = (T_w_curr * T_v_loc_c).inverse();


  live_points_mat = T_c_s.matrix().cast<float>() * live_points_mat;
  map_points_mat = (T_c_s * T_s_m).matrix().cast<float>() * map_points_mat;
  map_nn_mat = (T_c_s * T_s_sm).matrix().cast<float>() * map_nn_mat;

  cv::Mat live_index_img = cv::Mat::zeros(config_->perspective_params.height, config_->perspective_params.width, CV_32S);
  cv::Mat live_hsv_img = cv::Mat::zeros(config_->perspective_params.height, config_->perspective_params.width, CV_8UC3);
  cv::Mat live_rgb_img = cv::Mat::zeros(config_->perspective_params.height, config_->perspective_params.width, CV_8UC3);
  cv::Mat raw_rgb_img = cv::Mat::zeros(config_->perspective_params.height, config_->perspective_params.width, CV_8UC3);

  raw_point_cloud = filter_by_corridor(raw_point_cloud, curr_sid, 20, chain, config_->corridor_width, T_cam_w);
  generate_depth_image(raw_point_cloud, live_hsv_img, live_index_img, config_->perspective_params);

  cv::Mat raw_hsv_img = live_hsv_img.clone();
  interpolate_hsv_image(live_hsv_img);

  cv::cvtColor(live_hsv_img, live_rgb_img, cv::COLOR_HSV2RGB);
  cv::cvtColor(raw_hsv_img, raw_rgb_img, cv::COLOR_HSV2RGB);

  if (config_->visualize) {
    cv_bridge::CvImage live_cv_rgb_img;
    live_cv_rgb_img.header.frame_id = "lidar";
    //cv_rgb_img.header.stamp = qdata.stamp->header.stamp;
    live_cv_rgb_img.encoding = "rgb8";
    live_cv_rgb_img.image = live_rgb_img;
    live_img_pub_->publish(*live_cv_rgb_img.toImageMsg());
    if (*qdata.vertex_test_result == VertexTestResult::CREATE_VERTEX) {
      auto locked_image_msg =
              std::make_shared<Image_LockMsg>(live_cv_rgb_img.toImageMsg(), *qdata.stamp);
      vertex->insert<sensor_msgs::msg::Image>("live_depth_image", "sensor_msgs/msg/Image", locked_image_msg);
    }

    cv_bridge::CvImage raw_cv_rgb_img;
    raw_cv_rgb_img.header.frame_id = "lidar";
    //cv_rgb_img.header.stamp = qdata.stamp->header.stamp;
    raw_cv_rgb_img.encoding = "rgb8";
    raw_cv_rgb_img.image = raw_rgb_img;
    if (*qdata.vertex_test_result == VertexTestResult::CREATE_VERTEX) {
      auto locked_image_msg =
              std::make_shared<Image_LockMsg>(raw_cv_rgb_img.toImageMsg(), *qdata.stamp);
      vertex->insert<sensor_msgs::msg::Image>("live_raw_image", "sensor_msgs/msg/Image", locked_image_msg);
    }
  }

  cv::Mat map_index_img = cv::Mat::zeros(config_->perspective_params.height, config_->perspective_params.width, CV_32S);
  cv::Mat map_hsv_img = cv::Mat::zeros(config_->perspective_params.height, config_->perspective_params.width, CV_8UC3);
  cv::Mat map_rgb_img = cv::Mat::zeros(config_->perspective_params.height, config_->perspective_params.width, CV_8UC3);
  cv::Mat similarity_diff; // = cv::Mat::empty(config_->perspective_params.height, config_->perspective_params.width, CV_32FC1);

  nn_map_point_cloud = filter_by_corridor(nn_map_point_cloud, curr_sid, 20, chain, config_->corridor_width, T_cam_w);
  // generate_depth_image(map_point_cloud, map_hsv_img, map_index_img, config_->perspective_params);
  generate_depth_image(nn_map_point_cloud, map_hsv_img, map_index_img, config_->perspective_params);
  interpolate_hsv_image(map_hsv_img);

  
  cv::cvtColor(map_hsv_img, map_rgb_img, cv::COLOR_HSV2RGB);

  //   {
  //   CLOG(DEBUG, "lidar.perspective") << chain.size();

  //   auto lock = chain.guard();
  //   // compute vertex lookahead
  //   const auto distance = chain.dist(curr_sid);
  //   // forwards
  //   for (auto query_sid = curr_sid;
  //        query_sid < chain.size()
  //             && (chain.dist(query_sid) - distance) < 10;
  //        ++query_sid) {
  //     const auto T_cam_query = T_c_s * T_s_r * T_w_curr.inverse() * chain.pose(query_sid);
  //     const auto p_cam_query = T_cam_query.matrix().block<4, 1>(0, 3);
  //     // T_curr_query_vec.emplace_back(T_curr_query.matrix());
  //     CLOG(DEBUG, "lidar.perspective") << "Pose origin" << p_cam_query;

  //     int lu = (int)round(config_->perspective_params.f_u() * (p_cam_query[0] - 0.75) / p_cam_query[2]) + config_->perspective_params.c_u();
  //     int lv = (int)round(config_->perspective_params.f_v() * (p_cam_query[1] - 0.2) / p_cam_query[2]) + config_->perspective_params.c_v();

  //     int ru = (int)round(config_->perspective_params.f_u() * (p_cam_query[0] + 0.75) / p_cam_query[2]) + config_->perspective_params.c_u();
  //     int rv = (int)round(config_->perspective_params.f_v() * (p_cam_query[1] + 3) / p_cam_query[2]) + config_->perspective_params.c_v();


  //     if ((chain.dist(query_sid) - distance) > 0.75)
  //       cv::rectangle(map_rgb_img, cv::Point{lu, lv}, cv::Point{ru, rv}, color_options[3], cv::FILLED);


  //     CLOG(DEBUG, "lidar.perspective") << "Recatngle " << cv::Point{lu, lv} << " " << cv::Point{ru, rv};

  //     // if (0 <= u && u < config_->perspective_params.width && 0 <= v && v < config_->perspective_params.height) {
  //     //   cv::circle(map_rgb_img, cv::Point{u, v}, 12, color_options[color_options.size() - 1], 1);
  //     // }
  //   }
  // }

  // auto val = cv::quality::QualitySSIM::compute(map_hsv_img, live_hsv_img, similarity_diff);
  // CLOG(DEBUG, "lidar.perspective") << "Sim: " << similarity_diff.type();
  // similarity_diff = similarity_diff;
  // similarity_diff *= 255;
  // similarity_diff.convertTo(similarity_diff, CV_8UC3);

  // cv::Mat hsv_scores[3];   //destination array
  // cv::split(similarity_diff, hsv_scores);//split source  


  // CLOG(DEBUG, "lidar.perspective") << "SSIM " << val;


  if (config_->visualize) {
    cv_bridge::CvImage map_cv_rgb_img;
    map_cv_rgb_img.header.frame_id = "lidar";
    //cv_rgb_img.header.stamp = qdata.stamp->header.stamp;
    map_cv_rgb_img.encoding = "rgb8";
    map_cv_rgb_img.image = map_rgb_img;
    map_img_pub_->publish(*map_cv_rgb_img.toImageMsg());

    if (*qdata.vertex_test_result == VertexTestResult::CREATE_VERTEX) {
      auto locked_image_msg =
              std::make_shared<Image_LockMsg>(map_cv_rgb_img.toImageMsg(), *qdata.stamp);
      vertex->insert<sensor_msgs::msg::Image>("map_depth_image", "sensor_msgs/msg/Image", locked_image_msg);
    }
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

  cv::Mat binary_nearest_img;
  {
    const auto msg = vertex->retrieve<ImageMsg>(
        "detection_mask_nearest", "sensor_msgs/msg/Image");
    if (msg == nullptr) {
      CLOG(WARNING, "lidar.perspective")
          << "This vertex has no 3D diff image.";
      return;
    }
    auto locked_msg = msg->sharedLocked();
    binary_nearest_img = cv_bridge::toCvShare(locked_msg.get().getDataPtr(), "mono8")->image;
  }

  // torch::Tensor small_live_tensor = torch::from_blob(live_rgb_img.data, {raw_rgb_img.rows, raw_rgb_img.cols, raw_rgb_img.channels()}, torch::kByte).to(torch::kFloat32).permute({2, 0, 1});
  // torch::Tensor small_raw_tensor = torch::from_blob(raw_rgb_img.data, {raw_rgb_img.rows, raw_rgb_img.cols, raw_rgb_img.channels()}, torch::kByte).to(torch::kFloat32).permute({2, 0, 1});
  // torch::Tensor small_map_tensor = torch::from_blob(map_rgb_img.data, {map_rgb_img.rows, map_rgb_img.cols, map_rgb_img.channels()}, torch::kByte).to(torch::kFloat32).permute({2, 0, 1});
  // torch::Tensor diff = small_live_tensor - small_map_tensor;
  // diff = diff.norm(2, {0});
  // //.clamp(0, 255.0)

  // namespace F = torch::nn::functional;

  // using namespace torch::indexing;

  // torch::Tensor average = torch::ones({1, 1, 3, 3}) / 9;

  // diff = F::conv2d(diff.unsqueeze(0).unsqueeze(0), average);

  // diff = diff.squeeze().squeeze();
  // diff.index_put_({small_raw_tensor.norm() == 0}, 0);

  std::vector<torch::Tensor> prompts;

  // static const int MAX_ITERS = 40;
  // for (int i = 0; prompts.size() < config_->num_prompts && i < MAX_ITERS; ++i) {
  //   const auto [max_val, idx2] = torch::max(diff.flatten(), 0);

  //   int idx = idx2.item<long>();
  //   int prompt[] = {idx % diff.size(1), idx / diff.size(1)};

  //   auto& pc_idx = live_index_img.at<uint32_t>(prompt[1], prompt[0]);

  //   if (pc_idx == 0) {
  //     // CLOG(DEBUG, "lidar.perspective") << "Prompt point (" << prompt[0] << ", " << prompt[1] << ") <<  on an interpolated live pixel. Skipping.";

  //     diff.index_put_({Slice(idx / diff.size(1), idx / diff.size(1) + 1), 
  //                   Slice(idx % diff.size(1), idx % diff.size(1) + 1) }, 0);
  //     continue;
  //   }
  //   prompt[0] = 4*prompt[0] + 2;
  //   prompt[1] = 4*prompt[1] + 2;

  //   auto live_point = raw_point_cloud[pc_idx - 1];
  //   Eigen::Vector4f h_point;
  //   h_point << live_point.x, live_point.y, live_point.z, 1.0f;
  //   h_point = T_v_loc_c.matrix().cast<float>() * h_point;

  //   double theta = atan2(h_point[1], h_point[0]);
  //   CLOG(DEBUG, "lidar.perspective") << "Diff Tensor prompt (" << prompt[0] << ", " << prompt[1] << ") Theta: " << theta << " DIff norm " << max_val;


  //   //Prompts are x, y rather than row, column
  //   //map_tensor.index({0, prompt[1], prompt[0]}).item().to<float>() > 0  || 
  //   if (!((theta > 0.611 && theta < 0.96) || (theta > -0.96 && theta < -0.611))){
  //     prompts.push_back(torch::from_blob(prompt, {2}, torch::kInt).to(device));  


  //     diff.index_put_({Slice(idx / diff.size(1) - 3, idx / diff.size(1) + 4), 
  //                     Slice(idx % diff.size(1) - 3, idx % diff.size(1) + 4) }, 0);

  //   } else {
  //     // CLOG(DEBUG, "lidar.perspective") << "Prompt point on an empty map pixel. Try again.";
  //     diff.index_put_({Slice(idx / diff.size(1), idx / diff.size(1) + 1), 
  //                   Slice(idx % diff.size(1), idx % diff.size(1) + 1) }, 0);
  //     continue;
  //   }

  // }

  
  // const auto [smaller_diff, idx] = F::max_pool2d_with_indices(diff.squeeze(0), F::MaxPool2dFuncOptions(5));
  // const auto [topk_vals, topk_idx] = smaller_diff.flatten().topk(config_->num_prompts);

  // CLOG(DEBUG, "lidar.perspective") << "Max pooling " << smaller_diff.sizes() << " and " << idx.sizes();

  // diff = diff.squeeze();


  // auto topk_idx_a = topk_idx.accessor<long, 1>();
  // auto topk_val_a = topk_vals.accessor<float, 1>();
  // auto idx_a = idx.accessor<long, 3>();  //shape (x, y, z, dtype=long)

  // for (size_t i = 0; i < topk_idx_a.size(0); i++) {
  //   const long& small_idx = topk_idx_a[i];

  //   const long& idx = idx_a[0][small_idx / smaller_diff.size(2)][small_idx % smaller_diff.size(2)];



  //   //Scale up to whole image

  // }


  std::vector<KeyPoint> keypoints = detectBlobsSortedBySize(binary_nearest_img.clone());

  for (const auto &kp : keypoints) {
    int prompt[] = {kp.pt.x, kp.pt.y};

    prompt[0] = 4*prompt[0] + 2;
    prompt[1] = 4*prompt[1] + 2;
    prompts.push_back(torch::from_blob(prompt, {2}, torch::kInt).to(device)); 

    if(prompts.size() == config_->num_prompts) {
      break;
    }
  }

  torch::NoGradGuard no_grad;
  std::vector<torch::jit::IValue> jit_inputs;

  jit_inputs.push_back(map_tensor.to(device));
  jit_inputs.push_back(live_tensor.to(device));
  jit_inputs.push_back(prompts);

  auto outputs = network(jit_inputs).toGenericDict();


  auto teach_masks = outputs.at("teach_masks").toTensor();
  auto repeat_masks = outputs.at("repeat_masks").toTensor();


  CLOG(DEBUG, "lidar.perspective") << "Ran model! Teach size " << teach_masks.sizes() << " Repeat size " << repeat_masks.sizes();

  auto intersection = teach_masks.bitwise_and(repeat_masks);
  auto unions = teach_masks.bitwise_or(repeat_masks);

  auto ious = intersection.sum({1, 2}) / unions.sum({1, 2});

  CLOG(DEBUG, "lidar.perspective") << "IoUS " << ious.cpu();

  std::vector<unsigned int> idxs_to_keep;
  auto ious_a = ious.accessor<float, 1>();


  cv::Mat teach_mask = cv::Mat::zeros(teach_masks.size(1), teach_masks.size(2), CV_8UC3);
  cv::Mat repeat_mask = cv::Mat::zeros(repeat_masks.size(1), repeat_masks.size(2), CV_8UC3);

  for (int i = 0; i < prompts.size(); ++i) {

    auto prompt = prompts[i].cpu();
    auto prompt_a = prompt.accessor<int, 1>();


    auto active_teach_tensor = teach_masks.index({i, Slice(), Slice()}).to(torch::kByte).mul(255).contiguous();
    auto active_repeat_tensor = repeat_masks.index({i, Slice(), Slice()}).to(torch::kByte).mul(255).contiguous();

    cv::Mat active_teach_mask = cv::Mat{active_teach_tensor.size(0), active_teach_tensor.size(1), CV_8UC1, active_teach_tensor.data_ptr<uint8_t>()};
    //mask_to_pointcloud(active_teach_mask, map_index_img, nn_map_point_cloud, 2+i);

    // auto map_change_point_cloud = [&]{
    //   std::vector<int> indices;
    //   indices.reserve(nn_map_point_cloud.size());
    //   for (size_t i = 0; i < nn_map_point_cloud.size(); ++i) {
    //     if (nn_map_point_cloud[i].flex24 == 2+i)
    //       indices.emplace_back(i);
    //   }
    //   return pcl::PointCloud<PointWithInfo>(nn_map_point_cloud, indices);
    // }();
    
    cv::Mat active_repeat_mask = cv::Mat{active_repeat_tensor.size(0), active_repeat_tensor.size(1), CV_8UC1, active_repeat_tensor.data_ptr<uint8_t>()};
    //mask_to_pointcloud(active_repeat_mask, live_index_img, raw_point_cloud, 2+i);

    // auto change_point_cloud = [&]{
    //   std::vector<int> indices;
    //   indices.reserve(raw_point_cloud.size());
    //   for (size_t i = 0; i < raw_point_cloud.size(); ++i) {
    //     if (raw_point_cloud[i].flex24 == 2+i)
    //       indices.emplace_back(i);
    //   }
    //   return pcl::PointCloud<PointWithInfo>(raw_point_cloud, indices);
    // }();

    cv::Scalar mapMean, mapStdDev, liveMean, liveStdDev;
    cv::meanStdDev(map_hsv_img, mapMean, mapStdDev, active_teach_mask);
    cv::meanStdDev(live_hsv_img, liveMean, liveStdDev, active_repeat_mask);

    CLOG(DEBUG, "lidar.perspective") << "Map Mean" << mapMean << "Live mean " << liveMean;
//&& abs(liveMean[0] - mapMean[0]) > 10.
    if (ious_a[i] < config_->iou_thresh && active_repeat_tensor.sum().item<float>() < config_->max_size*255*128*256 ) {
      idxs_to_keep.push_back(i);
    }
    teach_mask.setTo(color_options[2*i % color_options.size()], active_teach_mask == 255);
    repeat_mask.setTo(color_options[2*i % color_options.size()], active_repeat_mask == 255);
    

    cv::circle(teach_mask, cv::Point{prompt_a[0] / 4, prompt_a[1] / 4}, 6, color_options[(2*i + 1) % color_options.size()], 1);
    cv::circle(repeat_mask, cv::Point{prompt_a[0] / 4, prompt_a[1] / 4}, 6, color_options[(2*i + 1) % color_options.size()], 1);
    
    // PointWithInfo minPt, maxPt;
    // pcl::getMinMax3D(change_point_cloud, minPt, maxPt);
    // CLOG(DEBUG, "lidar.perspective") << "Min" << minPt.x << " max: " << maxPt.x;    
  }


  CLOG(DEBUG, "lidar.perspective") << "IDs to KEEP " << idxs_to_keep;

  // cv::Mat teach_mask = cv::Mat::zeros(teach_masks.size(1), teach_masks.size(2), CV_8UC1);
  // cv::Mat repeat_mask = cv::Mat::zeros(repeat_masks.size(1), repeat_masks.size(2), CV_8UC1);

  if (idxs_to_keep.size() > 0){
    auto changes = repeat_masks.index({torch::from_blob(idxs_to_keep.data(), {idxs_to_keep.size()}, torch::kInt), Slice(), Slice()});
    changes = changes.sum({0}).to(torch::kByte).mul(255).contiguous();

    auto repeat_mask_flat = cv::Mat{changes.size(0), changes.size(1), CV_8UC1, changes.data_ptr<uint8_t>()};
    mask_to_pointcloud(repeat_mask_flat, live_index_img, raw_point_cloud, 2);

    auto obstacle_point_cloud =
        std::make_shared<pcl::PointCloud<PointWithInfo>>(raw_point_cloud);

    /// changed cropping
    {
      std::vector<int> indices;
      indices.reserve(obstacle_point_cloud->size());
      for (size_t i = 0; i < obstacle_point_cloud->size(); ++i) {
        if ((*obstacle_point_cloud)[i].flex24 > 1)
          indices.emplace_back(i);
      }
      *obstacle_point_cloud =
          pcl::PointCloud<PointWithInfo>(*obstacle_point_cloud, indices);

      auto obs_mat = obstacle_point_cloud->getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::cartesian_offset());
      obs_mat = T_v_loc_c.matrix().cast<float>() * obs_mat;

      qdata.changed_points.emplace(*obstacle_point_cloud);
    }
  } else {
    mask_to_pointcloud(cv::Mat::zeros(repeat_masks.size(1), repeat_masks.size(2), CV_8UC1), live_index_img, raw_point_cloud, 2);

    qdata.changed_points.emplace(pcl::PointCloud<PointWithInfo>());
  }

  // /// publish the transformed pointcloud
  if (config_->visualize) {
    cv_bridge::CvImage live_mask_img_msg;
    live_mask_img_msg.header.frame_id = "lidar";
    //cv_rgb_img.header.stamp = qdata.stamp->header.stamp;
    live_mask_img_msg.encoding = "bgr8";
    live_mask_img_msg.image = repeat_mask;
    live_mask_pub_->publish(*live_mask_img_msg.toImageMsg());

    cv_bridge::CvImage map_mask_img_msg;
    map_mask_img_msg.header.frame_id = "lidar";
    //cv_rgb_img.header.stamp = qdata.stamp->header.stamp;
    map_mask_img_msg.encoding = "bgr8";
    map_mask_img_msg.image = teach_mask;
    map_mask_pub_->publish(*map_mask_img_msg.toImageMsg());

    // diff = diff.div(diff.max()).mul(255).to(torch::kByte).contiguous();

    // cv_bridge::CvImage diff_img_msg;
    // diff_img_msg.header.frame_id = "lidar";
    // //cv_rgb_img.header.stamp = qdata.stamp->header.stamp;
    // diff_img_msg.encoding = "mono8";
    // diff_img_msg.image = cv::Mat{diff.size(0), diff.size(1), CV_8UC1, diff.data_ptr<uint8_t>()};;
    // diff_pub_->publish(*diff_img_msg.toImageMsg());
  }

  if (config_->visualize) {
    PointCloudMsg pc2_msg;
    pcl::toROSMsg(raw_point_cloud, pc2_msg);
    pc2_msg.header.frame_id = "lidar"; //"lidar" for honeycomb
    pc2_msg.header.stamp = rclcpp::Time(*qdata.stamp);
    filtered_pub_->publish(pc2_msg);
  }
                            
}

}  // namespace nn
}  // namespace vtr
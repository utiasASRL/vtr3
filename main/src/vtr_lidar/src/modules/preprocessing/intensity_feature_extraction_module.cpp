/**
 * \file intensity_feature_extraction_module.cpp
 * \author Wenda Zhao, Autonomous Space Robotics Lab (ASRL)
 */

#include "vtr_lidar/modules/preprocessing/intensity_feature_extraction_module.hpp"
#include "vtr_logging/logging.hpp"

#include "cv_bridge/cv_bridge.h"
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

namespace vtr {
namespace lidar {

using namespace tactic;

auto IntensityFeatureExtractionModule::Config::fromROS(
    const rclcpp::Node::SharedPtr& node, const std::string& param_prefix)
    -> ConstPtr {
  auto config = std::make_shared<Config>();
  // clang-format off

  // ── Ouster sensor metadata (from JSON flattened by launch file) ────────
  // The launch file calls _json_to_params() on the Ouster metadata JSON,
  // which flattens it into dot-separated ROS params such as
  //   beam_intrinsics.beam_altitude_angles
  //   lidar_data_format.pixels_per_column
  // We declare them with defaults, then the launch-file overrides them.
  // Same pattern as LIVO's livo_node.cpp + projector.cpp.

  // pixels_per_column (try both keys for compat with different JSON layouts)
  auto ppc = node->declare_parameter<int64_t>("lidar_data_format.pixels_per_column", 0);
  if (ppc == 0) ppc = node->declare_parameter<int64_t>("data_format.pixels_per_column", 0);
  if (ppc > 0) config->pixels_per_column = static_cast<int>(ppc);

  // columns_per_frame
  auto cpf = node->declare_parameter<int64_t>("lidar_data_format.columns_per_frame", 0);
  if (cpf == 0) cpf = node->declare_parameter<int64_t>("data_format.columns_per_frame", 0);
  if (cpf > 0) config->columns_per_frame = static_cast<int>(cpf);

  // lidar_origin_to_beam_origin_mm
  auto beam_offset = node->declare_parameter<double>("beam_intrinsics.lidar_origin_to_beam_origin_mm", 0.0);
  if (beam_offset == 0.0) beam_offset = node->declare_parameter<double>("lidar_origin_to_beam_origin_mm", 0.0);
  if (beam_offset != 0.0) config->lidar_origin_to_beam_origin_mm = beam_offset;

  // beam_altitude_angles
  config->beam_altitude_angles = node->declare_parameter<std::vector<double>>(
      "beam_intrinsics.beam_altitude_angles", std::vector<double>{});
  if (config->beam_altitude_angles.empty())
    config->beam_altitude_angles = node->declare_parameter<std::vector<double>>(
        "beam_altitude_angles", std::vector<double>{});

  // pixel_shift_by_row
  auto ps = node->declare_parameter<std::vector<int64_t>>(
      "lidar_data_format.pixel_shift_by_row", std::vector<int64_t>{});
  if (ps.empty())
    ps = node->declare_parameter<std::vector<int64_t>>(
        "data_format.pixel_shift_by_row", std::vector<int64_t>{});
  config->pixel_shift_by_row.resize(ps.size());
  for (size_t i = 0; i < ps.size(); ++i)
    config->pixel_shift_by_row[i] = static_cast<int>(ps[i]);

  CLOG(INFO, "lidar.intensity_feature_extraction")
      << "Sensor metadata: " << config->pixels_per_column << "x"
      << config->columns_per_frame
      << "  beam_offset=" << config->lidar_origin_to_beam_origin_mm << "mm"
      << "  beam_angles=" << config->beam_altitude_angles.size()
      << "  pixel_shifts=" << config->pixel_shift_by_row.size();

  config->u_shift = node->declare_parameter<int>(param_prefix + ".u_shift", config->u_shift);
  config->destagger = node->declare_parameter<bool>(param_prefix + ".destagger", config->destagger);

  // Image processing
  config->use_reflectivity = node->declare_parameter<bool>(param_prefix + ".use_reflectivity", config->use_reflectivity);
  config->use_auto_exposure = node->declare_parameter<bool>(param_prefix + ".use_auto_exposure", config->use_auto_exposure);
  config->use_sqrt_brighten = node->declare_parameter<bool>(param_prefix + ".use_sqrt_brighten", config->use_sqrt_brighten);
  config->ae_lo_frac = node->declare_parameter<double>(param_prefix + ".ae_lo_frac", config->ae_lo_frac);
  config->ae_hi_frac = node->declare_parameter<double>(param_prefix + ".ae_hi_frac", config->ae_hi_frac);

  // ORB detection
  config->max_features = node->declare_parameter<int>(param_prefix + ".max_features", config->max_features);
  config->nlevels = node->declare_parameter<int>(param_prefix + ".nlevels", config->nlevels);
  config->scale_factor = static_cast<float>(node->declare_parameter<double>(param_prefix + ".scale_factor", config->scale_factor));
  config->fast_threshold = node->declare_parameter<int>(param_prefix + ".fast_threshold", config->fast_threshold);
  config->min_range = static_cast<float>(node->declare_parameter<double>(param_prefix + ".min_range", config->min_range));
  config->max_range = static_cast<float>(node->declare_parameter<double>(param_prefix + ".max_range", config->max_range));
  config->grid_rows = node->declare_parameter<int>(param_prefix + ".grid_rows", config->grid_rows);
  config->grid_cols = node->declare_parameter<int>(param_prefix + ".grid_cols", config->grid_cols);
  config->max_per_cell = node->declare_parameter<int>(param_prefix + ".max_per_cell", config->max_per_cell);
  config->edge_threshold = node->declare_parameter<int>(param_prefix + ".edge_threshold", config->edge_threshold);
  config->patch_size = node->declare_parameter<int>(param_prefix + ".patch_size", config->patch_size);
  config->show_features = node->declare_parameter<bool>(param_prefix + ".show_features", config->show_features);
  config->rotate_image = node->declare_parameter<bool>(param_prefix + ".rotate_image", config->rotate_image);

  // mask_rects: rectangular mask regions [x, y, w, h, ...]
  auto mask_rects_i64 = node->declare_parameter<std::vector<int64_t>>(param_prefix + ".mask_rects", std::vector<int64_t>{});
  config->mask_rects.resize(mask_rects_i64.size());
  for (size_t i = 0; i < mask_rects_i64.size(); ++i) {
    config->mask_rects[i] = static_cast<int>(mask_rects_i64[i]);
  }

  // Signal image subscription option
  config->use_signal_image = node->declare_parameter<bool>(param_prefix + ".use_signal_image", config->use_signal_image);
  config->signal_image_topic = node->declare_parameter<std::string>(param_prefix + ".signal_image_topic", config->signal_image_topic);

  // clang-format on
  return config;
}

void IntensityFeatureExtractionModule::run_(
    QueryCache& qdata0, OutputCache&, const Graph::Ptr&,
    const std::shared_ptr<TaskExecutor>&) {
  auto& qdata = dynamic_cast<LidarQueryCache&>(qdata0);

  // Lazy initialization of projector, auto-exposure, and feature manager
  if (!initialized_) {
    // Create OusterProjector from sensor metadata config
    auto proj_config = std::make_shared<OusterProjectorConfig>();
    proj_config->pixels_per_column = config_->pixels_per_column;
    proj_config->columns_per_frame = config_->columns_per_frame;
    proj_config->lidar_origin_to_beam_origin_mm =
        config_->lidar_origin_to_beam_origin_mm;
    proj_config->pixel_shift_by_row = config_->pixel_shift_by_row;
    proj_config->beam_altitude_angles = config_->beam_altitude_angles;
    proj_config->u_shift = config_->u_shift;
    proj_config->destagger = config_->destagger;
    proj_config->use_reflectivity = config_->use_reflectivity;
    projector_ = std::make_shared<OusterProjector>(proj_config);

    // Create AutoExposure
    auto_exposure_ = std::make_unique<AutoExposure>(config_->ae_lo_frac, config_->ae_hi_frac, 3);

    // Create IntensityFeatureManager for ORB detection + matching
    auto fm_config = std::make_shared<IntensityFeatureManagerConfig>();
    fm_config->image_width = config_->columns_per_frame;
    fm_config->image_height = config_->pixels_per_column;
    fm_config->max_features = config_->max_features;
    fm_config->nlevels = config_->nlevels;
    fm_config->scale_factor = config_->scale_factor;
    fm_config->fast_threshold = config_->fast_threshold;
    fm_config->min_range = config_->min_range;
    fm_config->max_range = config_->max_range;
    fm_config->grid_rows = config_->grid_rows;
    fm_config->grid_cols = config_->grid_cols;
    fm_config->max_per_cell = config_->max_per_cell;
    fm_config->edge_threshold = config_->edge_threshold;
    fm_config->patch_size = config_->patch_size;
    feature_manager_ = std::make_shared<IntensityFeatureManager>(fm_config);

    // Build mask image from rectangular mask regions
    const int img_rows = config_->pixels_per_column;
    const int img_cols = config_->columns_per_frame;
    mask_ = cv::Mat::ones(img_rows, img_cols, CV_8UC1) * 255;
    const auto& mr = config_->mask_rects;
    if (mr.size() % 4 != 0) {
      CLOG(WARNING, "lidar.intensity_feature_extraction")
          << "mask_rects must have a multiple of 4 elements [x,y,w,h,...]; got "
          << mr.size() << " — ignoring masks";
    } else {
      for (size_t i = 0; i < mr.size() / 4; ++i) {
        cv::Rect rect(mr[i * 4], mr[i * 4 + 1], mr[i * 4 + 2], mr[i * 4 + 3]);
        rect &= cv::Rect(0, 0, img_cols, img_rows);  // clamp to image bounds
        if (rect.area() > 0) {
          mask_(rect) = 0;
          mask_rects_parsed_.push_back(rect);
        }
      }
      CLOG(INFO, "lidar.intensity_feature_extraction")
          << "Loaded " << mask_rects_parsed_.size() << " mask rectangles";
    }

    // Subscribe to external signal image if enabled
    if (config_->use_signal_image) {
      signal_image_sub_ = qdata.node->create_subscription<ImageMsg>(
          config_->signal_image_topic, rclcpp::SensorDataQoS(),
          [this](const ImageMsg::SharedPtr msg) {
            auto cv_ptr = cv_bridge::toCvCopy(msg, "mono8");
            std::lock_guard<std::mutex> lock(signal_image_mutex_);
            latest_signal_image_ = cv_ptr->image;
            has_signal_image_ = true;
          });
      CLOG(INFO, "lidar.intensity_feature_extraction")
          << "Subscribed to signal image topic: " << config_->signal_image_topic;
    }

    initialized_ = true;
  }

  // Need unfiltered point cloud (full hardware grid) for image creation
  if (!qdata.unfiltered_point_cloud.valid()) {
    CLOG(WARNING, "lidar.intensity_feature_extraction")
        << "No unfiltered point cloud available, skipping feature extraction";
    return;
  }

  const auto& raw_cloud = *qdata.unfiltered_point_cloud;

  cv::Mat intensity_image_u8, range_image, pixel_to_point_index;

  // Always need range_image and pixel_to_point_index from projector
  {
    cv::Mat intensity_from_proj;
    projector_->createImages(raw_cloud, intensity_from_proj, range_image,
                             pixel_to_point_index);

    bool use_projected_fallback = false;
    if (config_->use_signal_image) {
      // Use externally subscribed signal image
      std::lock_guard<std::mutex> lock(signal_image_mutex_);
      if (!has_signal_image_) {
        // NOTE: In runtime, signal image may not have been received yet,
        //       falling back to projected intensity image
        CLOG(WARNING, "lidar.intensity_feature_extraction")
            << "No signal image received yet, falling back to projected "
               "intensity image for this frame";
        use_projected_fallback = true;
      } else {
        intensity_image_u8 = latest_signal_image_.clone();
      }
    }

    if (!config_->use_signal_image || use_projected_fallback) {
      // Use projected intensity image from point cloud
      cv::Mat& intensity_image_f32 = intensity_from_proj;

      // Step 2: Apply auto-exposure (operates on float image, normalizes to [0,1])
      if (config_->use_auto_exposure) {
        Eigen::Map<img_t<float>> img_eigen(
            reinterpret_cast<float*>(intensity_image_f32.data),
            intensity_image_f32.rows, intensity_image_f32.cols);
        (*auto_exposure_)(img_eigen, true);
      }

      // Step 2b: Sqrt gamma lift (brightens dark areas, matching LIVO pipeline)
      if (config_->use_sqrt_brighten) {
        intensity_image_f32.setTo(0.0f, intensity_image_f32 < 0.0f);
        cv::sqrt(intensity_image_f32, intensity_image_f32);
      }

      // Step 3: Convert to 8-bit for ORB detection
      if (config_->use_auto_exposure || config_->use_sqrt_brighten) {
        cv::Mat scaled;
        intensity_image_f32.convertTo(scaled, CV_32F, 255.0);
        scaled.convertTo(intensity_image_u8, CV_8U);
      } else {
        cv::normalize(intensity_image_f32, intensity_image_f32, 0, 255,
                      cv::NORM_MINMAX);
        intensity_image_f32.convertTo(intensity_image_u8, CV_8U);
      }
    }
  }

  // Step 4: Detect ORB features and back-project to 3D
  //   pixel_to_point_index maps into the unfiltered cloud, which has 
  //   the same size as the hardware grid (rows_ × cols_)
  auto features = feature_manager_->detectFeatures(
      intensity_image_u8, pixel_to_point_index, raw_cloud, mask_);

  CLOG(DEBUG, "lidar.intensity_feature_extraction")
      << "Extracted " << features.size()
      << " ORB features from intensity image ("
      << intensity_image_u8.cols << "x" << intensity_image_u8.rows << ")";

  // Step 5: Store in cache for downstream modules
  // Note: we must keep copies for publishing before moving
  cv::Mat intensity_for_pub, range_for_pub;
  std::vector<cv::KeyPoint> keypoints_for_pub;
  if (config_->show_features) {
    intensity_for_pub = intensity_image_u8.clone();
    range_for_pub = range_image.clone();
    keypoints_for_pub = features.keypoints;  // copy before move
  }
  qdata.intensity_image.emplace(std::move(intensity_image_u8));
  qdata.range_image.emplace(std::move(range_image));
  qdata.pixel_to_point_index.emplace(std::move(pixel_to_point_index));
  qdata.live_intensity_features.emplace(std::move(features));

  // Step 6: Publish images for visualization (following VTR convention)
  if (config_->show_features) {
    if (!publisher_initialized_) {
      intensity_pub_ =
          qdata.node->create_publisher<ImageMsg>("liv_intensity_image", 5);
      range_pub_ =
          qdata.node->create_publisher<ImageMsg>("liv_range_image", 5);
      publisher_initialized_ = true;
    }

    // Publish intensity image (8-bit grayscale, with mask + feature overlay)
    cv::Mat intensity_vis;
    cv::cvtColor(intensity_for_pub, intensity_vis, cv::COLOR_GRAY2BGR);
    // Draw mask rectangles as translucent teal overlays
    for (const auto& rect : mask_rects_parsed_) {
      cv::Mat roi = intensity_vis(rect);
      cv::Mat teal(roi.size(), roi.type(), cv::Scalar(128, 128, 0));
      cv::addWeighted(roi, 0.6, teal, 0.4, 0.0, roi);
      cv::rectangle(intensity_vis, rect, cv::Scalar(128, 128, 0), 1,
                     cv::LINE_AA);
    }
    // Draw ORB feature overlay (circles + gradient direction, matching LIVO)
    {
      cv::Mat dx, dy;
      cv::Sobel(intensity_for_pub, dx, CV_32F, 1, 0, 3);
      cv::Sobel(intensity_for_pub, dy, CV_32F, 0, 1, 3);
      const cv::Scalar feat_color(0, 140, 255);  // orange (BGR)
      for (const auto& kp : keypoints_for_pub) {
        const int cx = static_cast<int>(std::round(kp.pt.x));
        const int cy = static_cast<int>(std::round(kp.pt.y));
        if (cx < 0 || cx >= intensity_vis.cols ||
            cy < 0 || cy >= intensity_vis.rows)
          continue;
        const int radius = std::clamp(static_cast<int>(kp.size * 0.2f), 2, 6);
        const float gx = dx.ptr<float>(cy)[cx];
        const float gy = dy.ptr<float>(cy)[cx];
        const float grad_len = std::sqrt(gx * gx + gy * gy) + 1e-6f;
        const cv::Point tip(
            cx + static_cast<int>(radius * gx / grad_len),
            cy + static_cast<int>(radius * gy / grad_len));
        cv::circle(intensity_vis, cv::Point(cx, cy), radius, feat_color, 1,
                   cv::LINE_AA);
        cv::line(intensity_vis, cv::Point(cx, cy), tip, feat_color, 1,
                 cv::LINE_AA);
      }
    }

    cv_bridge::CvImage intensity_msg;
    intensity_msg.header.frame_id = "lidar";
    intensity_msg.encoding = "bgr8";
    if (config_->rotate_image)
      cv::rotate(intensity_vis, intensity_msg.image, cv::ROTATE_180);
    else
      intensity_msg.image = intensity_vis;

    // Draw feature count as blue text in top-left corner
    {
      const std::string text = std::to_string(keypoints_for_pub.size()) + " features";
      const cv::Scalar blue(255, 128, 0);  // blue in BGR
      cv::putText(intensity_msg.image, text, cv::Point(10, 20),
                  cv::FONT_HERSHEY_SIMPLEX, 0.5, blue, 1, cv::LINE_AA);
    }

    intensity_pub_->publish(*intensity_msg.toImageMsg());

    // Publish range image (normalize to 8-bit for visualization)
    cv_bridge::CvImage range_msg;
    range_msg.header.frame_id = "lidar";
    range_msg.encoding = "mono8";
    cv::Mat range_u8;
    cv::normalize(range_for_pub, range_u8, 0, 255, cv::NORM_MINMAX);
    range_u8.convertTo(range_u8, CV_8U);
    if (config_->rotate_image)
      cv::rotate(range_u8, range_msg.image, cv::ROTATE_180);
    else
      range_msg.image = range_u8;
    range_pub_->publish(*range_msg.toImageMsg());
  }
}

}  // namespace lidar
}  // namespace vtr
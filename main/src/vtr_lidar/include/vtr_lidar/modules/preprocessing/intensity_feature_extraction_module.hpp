/**
 * \file intensity_feature_extraction_module.hpp
 * \author Wenda Zhao, Autonomous Space Robotics Lab (ASRL)
 */

#pragma once

#include "sensor_msgs/msg/image.hpp"

#include "vtr_tactic/modules/base_module.hpp"
#include "vtr_lidar/cache.hpp"
#include "vtr_lidar/features/ouster_projector.hpp"
#include "vtr_lidar/features/auto_exposure.hpp"
#include "vtr_lidar/features/intensity_feature_manager.hpp"

namespace vtr {
namespace lidar {

class IntensityFeatureExtractionModule : public tactic::BaseModule {
 public:
  PTR_TYPEDEFS(IntensityFeatureExtractionModule);

  static constexpr auto static_name = "lidar.intensity_feature_extraction";

  struct Config : public BaseModule::Config {
    PTR_TYPEDEFS(Config);

    // Ouster sensor metadata (populated from JSON via launch file)
    int pixels_per_column = 128;
    int columns_per_frame = 1024;
    double lidar_origin_to_beam_origin_mm = 15.806;
    std::vector<int> pixel_shift_by_row;
    std::vector<double> beam_altitude_angles;
    int u_shift = 0;
    bool destagger = true;

    // Image processing
    bool use_reflectivity = false;
    bool use_auto_exposure = true;
    double ae_lo_frac = 0.1;
    double ae_hi_frac = 0.1;

    // ORB detection
    int max_features = 500;
    int nlevels = 8;
    float scale_factor = 1.2f;
    int fast_threshold = 20;
    float min_range = 0.5f;
    float max_range = 100.0f;

    bool visualize = false;
    /// Rotate published images 180° (for upside-down mounted lidars)
    bool rotate_image = false;
    /// Rectangular mask regions [x, y, w, h, ...] to block occluded areas
    std::vector<int> mask_rects;

    static ConstPtr fromROS(const rclcpp::Node::SharedPtr& node,
                            const std::string& param_prefix);
  };

  IntensityFeatureExtractionModule(
      const Config::ConstPtr& config,
      const std::shared_ptr<tactic::ModuleFactory>& module_factory = nullptr,
      const std::string& name = static_name)
      : tactic::BaseModule{module_factory, name}, config_(config) {}

 private:
  void run_(tactic::QueryCache& qdata, tactic::OutputCache& output,
            const tactic::Graph::Ptr& graph,
            const std::shared_ptr<tactic::TaskExecutor>& executor) override;

  using ImageMsg = sensor_msgs::msg::Image;

  Config::ConstPtr config_;
  OusterProjector::Ptr projector_;
  IntensityFeatureManager::Ptr feature_manager_;
  std::unique_ptr<AutoExposure> auto_exposure_;
  bool initialized_ = false;

  /// CV_8UC1 mask for ORB detection (255=valid, 0=masked)
  cv::Mat mask_;
  /// Parsed rectangular mask regions (for visualization overlay)
  std::vector<cv::Rect> mask_rects_parsed_;

  /** \brief for visualization only */
  bool publisher_initialized_ = false;
  rclcpp::Publisher<ImageMsg>::SharedPtr intensity_pub_;
  rclcpp::Publisher<ImageMsg>::SharedPtr range_pub_;

  VTR_REGISTER_MODULE_DEC_TYPE(IntensityFeatureExtractionModule);
};

}  // namespace lidar
}  // namespace vtr




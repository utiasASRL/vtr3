/**
 * \file localization_liv_module.hpp
 * \author Wenda Zhao, Autonomous Space Robotics Lab (ASRL)
 *
 * LIV localization module: combines ICP point-to-plane with lidar
 * intensity-image features (3-D point-to-point residual, or optionally the
 * 2-D reprojection error) in a single STEAM optimisation for the repeat
 * phase.
 *
 * During localization the solver uses OptimizationProblem (single pose
 * variable T_r_v) rather than SlidingWindowFilter + trajectory, since
 * we only need to estimate a single pose (robot-to-vertex).
 *
 * Visual matching is performed between the live intensity features and
 * the merged feature submap recalled from the teach graph. Its landmarks
 * are moved into the vertex frame with a fixed transform, so T_s_r · T_r_v
 * relates them to the current sensor frame. Without a feature submap the
 * module localizes on the lidar points only.
 */
#pragma once

#include <string>

#include "steam.hpp"

#include "vtr_lidar/cache.hpp"
#include "vtr_lidar/features/ouster_projector.hpp"
#include "vtr_tactic/modules/base_module.hpp"
#include "vtr_tactic/task_queue.hpp"

namespace vtr {
namespace lidar {

/** \brief LIV (Lidar + Intensity Visual) localization. */
class LocalizationLIVModule : public tactic::BaseModule {
 public:
  /** \brief Static module identifier. */
  static constexpr auto static_name = "lidar.localization_liv";

  /** \brief Config parameters. */
  struct Config : public tactic::BaseModule::Config {
    PTR_TYPEDEFS(Config);

    // ────────── Prior terms ──────────
    bool use_pose_prior = false;

    // ────────── ICP parameters ──────────
    int num_threads = 4;
    // initial alignment
    size_t first_num_steps = 3;
    size_t initial_max_iter = 100;
    float initial_max_pairing_dist = 2.0;
    float initial_max_planar_dist = 0.3;
    // refined stage
    size_t refined_max_iter = 10;
    float refined_max_pairing_dist = 2.0;
    float refined_max_planar_dist = 0.1;
    // convergence
    float averaging_num_steps = 5;
    float trans_diff_thresh = 0.01;
    float rot_diff_thresh = 0.1 * M_PI / 180.0;
    // steam optimizer
    bool verbose = false;
    unsigned int max_iterations = 1;

    // gyro bias estimation (post-optimization)
    bool calc_gy_bias = false;
    float calc_gy_bias_thresh = 1.0;

    // ────────── Visual (LIV) parameters ──────────
    /// Feature residual: "p2p" (3-D point-to-point with propagated
    /// covariances, as the LIV odometry) or "reproj" (2-D reprojection)
    std::string visual_residual = "p2p";
    /// p2p: scale of the intensity-feature weights relative to the lidar
    /// weights. The lidar term keeps the unit-less weight of the lidar-only
    /// pipeline (normal_score · n nᵀ); the feature information is
    ///   W = intensity_feature_scale · (Σ_landmark + R Σ_feature Rᵀ)⁻¹.
    /// Larger: the features count more. Tune by hand.
    double intensity_feature_scale = 9.0e-4;
    /// p2p: Cauchy scale on the Mahalanobis distance of a feature residual,
    /// before the intensity_feature_scale factor (<= 0: least squares)
    double feature_cauchy_k = 3.0;
    /// 1-σ of a feature position when it comes without a covariance [m]
    double default_feature_sigma = 0.1;
    /// reproj: reprojection error noise (pixels, isotropic σ)
    double sigma_pixel = 1.0;
    /// reproj: Cauchy loss function kernel width for reprojection error
    double reproj_loss_sigma = 1.0;
    /// Enable/disable visual cost terms
    bool use_visual = true;
    /// Enable/disable lidar ICP cost terms
    bool use_lidar = true;
    /// Match against the merged visual feature submap recalled from the
    /// teach graph (no visual costs when false or when no submap is stored)
    bool use_feature_map_matching = false;

    // ────────── OusterProjector sensor metadata ──────────
    int pixels_per_column = 128;
    int columns_per_frame = 1024;
    double lidar_origin_to_beam_origin_mm = 15.806;
    std::vector<int> pixel_shift_by_row;
    std::vector<double> beam_altitude_angles;
    int u_shift = 0;
    bool destagger = false;

    // ────────── Success criteria ──────────
    float min_matched_ratio = 0.4;

    // Time above which the localization module will be skipped (ms)
    float target_loc_time = 1000000.0;

    bool visualize = false;

    static ConstPtr fromROS(const rclcpp::Node::SharedPtr& node,
                            const std::string& param_prefix);
  };

  LocalizationLIVModule(
      const Config::ConstPtr& config,
      const std::shared_ptr<tactic::ModuleFactory>& module_factory = nullptr,
      const std::string& name = static_name)
      : tactic::BaseModule(module_factory, name), config_(config) {}

 private:
  void run_(tactic::QueryCache& qdata, tactic::OutputCache& output,
            const tactic::Graph::Ptr& graph,
            const tactic::TaskExecutor::Ptr& executor) override;

  Config::ConstPtr config_;

  /// Gyro bias estimation state (same as localization_icp_module)
  int64_t timestamp_prev_ = 0;
  Eigen::Matrix4d T_r_v_loc_prev_ = Eigen::Matrix4d::Identity();

  /// OusterProjector for reprojection error (lazy-initialized)
  std::shared_ptr<OusterProjector> projector_;
  bool projector_initialized_ = false;

  VTR_REGISTER_MODULE_DEC_TYPE(LocalizationLIVModule);
};

}  // namespace lidar
}  // namespace vtr

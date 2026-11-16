/**
 * \file odometry_liv_module.hpp
 * \author Wenda Zhao, Autonomous Space Robotics Lab (ASRL)
 *
 * LIV odometry module: combines ICP point-to-plane with lidar
 * intensity-image features in a single STEAM optimisation.
 *
 * Extends the existing OdometryICPModule with:
 *   1. ORB feature matching (mutual nearest neighbours) against the sliding
 *      landmark map, with the previous scan as fallback
 *   2. MD-RANSAC for outlier rejection (constant body velocity, 6-DoF)
 *   3. 3-D point-to-point cost terms with propagated covariances, added
 *      alongside the ICP terms (the 2-D reprojection residual is kept as
 *      an option, visual_residual: reproj)
 *
 * References:
 *   - LIVO: Lidar-Inertial-Visual Odometry 
 *   - VTR3 odometry_icp_module.cpp (ASRL)
 */
#pragma once

#include <string>

#include "sensor_msgs/msg/point_cloud2.hpp"

#include "steam.hpp"

#include "vtr_lidar/cache.hpp"
#include "vtr_lidar/features/ouster_projector.hpp"
#include "vtr_tactic/modules/base_module.hpp"
#include "vtr_tactic/task_queue.hpp"

namespace vtr {
namespace lidar {

/** \brief LIV (Lidar + Intensity Visual) odometry. */
class OdometryLIVModule : public tactic::BaseModule {
 public:
  using PointCloudMsg = sensor_msgs::msg::PointCloud2;

  /** \brief Static module identifier. */
  static constexpr auto static_name = "lidar.odometry_liv";

  /** \brief Config parameters. */
  struct Config : public tactic::BaseModule::Config {
    PTR_TYPEDEFS(Config);

    // ────────── Trajectory estimation ──────────
    bool use_trajectory_estimation = false;
    int traj_num_extra_states = 0;
    Eigen::Matrix<double, 6, 1> traj_qc_diag =
        Eigen::Matrix<double, 6, 1>::Ones();

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

    // gyro weight
    double gyro_cov = 1e-3;
    bool use_gyro = true;
    bool remove_orientation = false;
    /// Lidar point-to-plane terms as a few batched cost terms (same cost and
    /// Gauss-Newton terms as one steam term per point, much less overhead).
    /// false: one WeightedLeastSqCostTerm per point (previous code).
    bool batch_lidar_cost = true;

    // ────────── Visual (LIV) parameters ──────────
    /// Feature residual: "p2p" (3-D point-to-point with propagated
    /// covariances) or "reproj" (2-D reprojection + optional range)
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
    /// Descriptor matching: max Hamming distance of a mutual nearest neighbour
    double match_max_hamming = 50.0;
    /// MD-RANSAC: hypotheses at most (adaptive, stops at ransac_confidence)
    int ransac_max_iterations = 1000;
    double ransac_confidence = 0.999;
    /// MD-RANSAC: a match is an inlier below both thresholds
    double ransac_inlier_threshold = 3.0;  ///< [px]
    double ransac_inlier_range = 0.5;      ///< [m]
    /// MD-RANSAC: minimum number of inliers
    int ransac_min_inliers = 10;
    /// MD-RANSAC: Gauss-Newton steps per hypothesis, refinement rounds
    int ransac_gn_steps = 3;
    int ransac_refine_rounds = 2;
    /// Enable/disable visual cost terms (can run pure ICP if false)
    bool use_visual = true;
    /// Enable/disable lidar ICP cost terms (can run pure visual if false)
    bool use_lidar = true;

    /// Match live features against the sliding visual landmark map (falls
    /// back to the previous scan's features when the map yields too few
    /// MD-RANSAC inliers)
    bool use_feature_map_matching = false;
    /// Fewer map inliers than this: try the previous scan's features
    int map_match_min_matches = 10;

    /// reproj: enable/disable range cost terms on visual matches
    bool use_range_cost = false;
    /// Range measurement noise (metres, 1-σ)
    double sigma_range = 0.1;
    /// Cauchy loss function kernel width for range error
    double range_loss_scale = 1.0;

    /// Enable/disable kinematic regulation prior on each velocity knot
    /// (penalizes lateral velocity, vertical velocity, roll & pitch rates)
    bool use_kinematic_regulation = false;
    /// 1-σ for [lateral vel, vertical vel, roll rate, pitch rate]
    Eigen::Matrix<double, 4, 1> kinematic_regulation_sigma =
        (Eigen::Matrix<double, 4, 1>() << 0.05, 0.01, 0.02, 0.02).finished();

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
    float max_trans_vel_diff = 1000.0;
    float max_rot_vel_diff = 1000.0;
    float max_trans_diff = 1000.0;
    float max_rot_diff = 1000.0;

    static ConstPtr fromROS(const rclcpp::Node::SharedPtr& node,
                            const std::string& param_prefix);
  };

  OdometryLIVModule(
      const Config::ConstPtr& config,
      const std::shared_ptr<tactic::ModuleFactory>& module_factory = nullptr,
      const std::string& name = static_name)
      : tactic::BaseModule(module_factory, name), config_(config) {}

 private:
  void run_(tactic::QueryCache& qdata, tactic::OutputCache& output,
            const tactic::Graph::Ptr& graph,
            const tactic::TaskExecutor::Ptr& executor) override;

  Config::ConstPtr config_;

  /// OusterProjector for reprojection error (lazy-initialized)
  std::shared_ptr<OusterProjector> projector_;
  bool projector_initialized_ = false;

  VTR_REGISTER_MODULE_DEC_TYPE(OdometryLIVModule);
};

}  // namespace lidar
}  // namespace vtr

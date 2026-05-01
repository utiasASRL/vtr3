#pragma once

#include "steam.hpp"

#include "vtr_lidar/cache.hpp"
#include "vtr_tactic/modules/base_module.hpp"
#include "vtr_tactic/task_queue.hpp"
// #include "vtr_tactic/types.hpp"

namespace vtr {

namespace lidar {

/** \brief ICP for localization. */
class LocalizationDAICPModule : public tactic::BaseModule {
 public:
  /** \brief Static module identifier. */
  static constexpr auto static_name = "lidar.localization_daicp";

  /** \brief Config parameters. */
  struct Config : public tactic::BaseModule::Config {
    PTR_TYPEDEFS(Config);

    /// general
    int num_threads = 4;                        // number of threads for nearest neighbor search
    float target_loc_time = 1000000.0;          // time above which the localization ICP module will be skipped. (in ms)
                                                // For lidar running at 10 Hz, a good choice is < 50 ms to allow both odometry and localization to run
    // iteration control
    size_t first_num_steps = 3;
    size_t initial_max_iter = 100;
    // data association
    float lambda = 1.0;                         // weight for spatial vs curvature in correspondence search
    float curvature_similarity_thresh = 0.5;    // threshold on curvature difference for correspondence
    float initial_max_pairing_dist = 2.0;
    float initial_max_planar_dist = 0.3;
    // daicp
    int max_gn_iter = 2;                        // max gauss-newton iterations per daicp step
    // daicp - degeneracy threshold
    double degeneracy_thresh = 100.0;           // use relative condition number to set eigenvalue threshold
    // daicp - QP constraint bounds (per-iteration cap on |v_i^T x| in the
    // degenerate subspace; x is the GN perturbation about the current iterate
    // and is in *scaled* coords, i.e. rotation entries are multiplied by ell_mr
    // inside daGaussNewton). Larger values = looser constraint = closer to the
    // unconstrained step in borderline-degeneracy frames.
    double qp_eps_trans = 0.05;                 // [m]   per-iter cap on translation projection
    double qp_eps_rot   = 0.05;                 // [rad] per-iter cap on rotation projection (~2.9 deg)
    // daicp - covariance inflation factor for degenerate directions.
    // The DA-ICP covariance along each unobservable direction v_d is set to
    //   sigma^2 = degenerate_cov_alpha * v_d^T * Sigma_prior * v_d,
    // so that in STEAM's joint posterior the lidar gets weight 1/(1+alpha)
    // along that direction (i.e. the prior wins by a factor of alpha,
    // independent of the prior's absolute scale).
    //   alpha = 1e2  -> ~1% lidar weight (visible correction; default)
    //   alpha = 1e3  -> ~0.1% lidar weight (soft pull)
    //   alpha = 1e6+ -> rank-deficient limit (fully defer to prior)
    double degenerate_cov_alpha = 1e2;
    // daicp - QP solver name (CasADi conic plugin). Supported:
    //   "qrqp"  : QR-based active-set, pure C++, robust on tiny dense problems (recommended)
    //   "osqp"  : ADMM first-order; needs matching libosqp ABI, upper-tri H sparsity
    //   "qpoases", "nlpsol" (IPOPT) : also supported, see daicp_qp_lib.hpp
    std::string qp_solver_name = "qrqp";
    // daicp - range and bearing noise model
    double sigma_d = 0.02;                      // range noise std (2cm)
    double sigma_az = M_PI / 180.0 * 0.03;      // azimuth noise std (0.03 degrees)
    double sigma_el = M_PI / 180.0 * 0.03;      // elevation noise std (0.03 degrees)
    // daicp - convergence checks (numbers taken from STEAM's ICP implementation)
    double abs_cost_thresh = 1e-12;             // absolute cost threshold for convergence
    double abs_cost_change_thresh = 1e-8;       // absolute parameter change threshold for convergence
    double rel_cost_change_thresh = 1e-8;       // relative cost change threshold for convergence
    double zero_gradient_thresh = 1e-8;         // zero gradient threshold for convergence
    double inner_tolerance = 1e-6;              // tolerance for gauss-newton solver
    // prior fusion
    bool use_pfuison = true;                    // whether to use prior fusion
    int max_pfusion_iter = 1;                   // max iterations for prior fusion
    bool use_L2_loss = true;                    // whether to use L2 loss in prior fusion
    double robust_loss = 0.5;                   // parameter for robust loss function in prior fusion
    bool verbose = false;
    // refinement
    size_t refined_max_iter = 20;               // we use a fixed number of iters for now
    float refined_max_pairing_dist = 2.0;
    float refined_max_planar_dist = 0.1;
    // error calculation
    float averaging_num_steps = 10;
    float trans_diff_thresh = 0.01;             // threshold on variation of T
    float rot_diff_thresh = 0.1 * M_PI / 180.0; // threshold on variation of R
    // outlier rejection
    float trans_outlier_thresh = 0.1;           // threshold on translation outlier rejection to default to odometry
    float rot_outlier_thresh = 0.001;           // threshold on rotation outlier rejection to default to odometry
    float min_matched_ratio = 0.4;              // success criteria
    // correspondence-count floor: if fewer than this many pairs survive distance/curvature filtering,
    // skip the GN solve and fall back to the odometry prior. Prevents rank-deficient Hessians
    // (cond. number explosion) when the predicted T_r_v is so wrong that the distance filter
    // rejects nearly all correspondences. 6 DoF -> need >> 6 pairs; 100 is a safe floor.
    int min_pair_count = 100;
    // online gyroscope bias
    bool calc_gy_bias = false;
    float calc_gy_bias_thresh = 1.0;
    // curvature ratio threshold to default to odometry
    float curv_ratio_thresh = 0.95;           
    // threshold to consider a point as high curvature
    float high_curv_thresh = 0.8;               

    static ConstPtr fromROS(const rclcpp::Node::SharedPtr &node,
                            const std::string &param_prefix);
  };

  LocalizationDAICPModule(
      const Config::ConstPtr &config,
      const std::shared_ptr<tactic::ModuleFactory> &module_factory = nullptr,
      const std::string &name = static_name)
      : tactic::BaseModule(module_factory, name), config_(config) {}

 private:

  int64_t timestamp_prev_ = 0;
  Eigen::Matrix4d T_r_v_loc_prev_ = Eigen::Matrix4d::Identity();
  void run_(tactic::QueryCache &qdata, tactic::OutputCache &output,
            const tactic::Graph::Ptr &graph,
            const tactic::TaskExecutor::Ptr &executor) override;
            
  Config::ConstPtr config_;


  VTR_REGISTER_MODULE_DEC_TYPE(LocalizationDAICPModule);
};

}  // namespace lidar
}  // namespace vtr
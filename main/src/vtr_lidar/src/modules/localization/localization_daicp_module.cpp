#include "vtr_lidar/modules/localization/localization_daicp_module.hpp"
#include "vtr_lidar/modules/localization/daicp_lib.hpp"
#include "vtr_lidar/utils/nanoflann_utils.hpp"

namespace vtr {
namespace lidar {

using namespace tactic;
using namespace steam;
using namespace steam::se3;

auto LocalizationDAICPModule::Config::fromROS(const rclcpp::Node::SharedPtr &node,
                                            const std::string &param_prefix)
    -> ConstPtr {
  auto config = std::make_shared<Config>();
  // clang-format off
  // general
  config->num_threads = node->declare_parameter<int>(param_prefix + ".num_threads", config->num_threads);
  config->target_loc_time = node->declare_parameter<float>(param_prefix + ".target_loc_time", config->target_loc_time);
  // iteration control
  config->first_num_steps = node->declare_parameter<int>(param_prefix + ".first_num_steps", config->first_num_steps);
  config->initial_max_iter = node->declare_parameter<int>(param_prefix + ".initial_max_iter", config->initial_max_iter);
  // data association
  config->lambda = node->declare_parameter<float>(param_prefix + ".lambda", config->lambda);
  config->curvature_similarity_thresh = node->declare_parameter<float>(param_prefix + ".curvature_similarity_thresh", config->curvature_similarity_thresh);
  config->initial_max_pairing_dist = node->declare_parameter<float>(param_prefix + ".initial_max_pairing_dist", config->initial_max_pairing_dist);
  config->initial_max_planar_dist = node->declare_parameter<float>(param_prefix + ".initial_max_planar_dist", config->initial_max_planar_dist);
  // daicp
  config->max_gn_iter = node->declare_parameter<int>(param_prefix + ".max_gn_iter", config->max_gn_iter);
  // daicp - degeneracy threshold
  config->degeneracy_thresh = node->declare_parameter<double>(param_prefix + ".degeneracy_thresh", config->degeneracy_thresh);
  // daicp - range and bearing noise model
  config->sigma_d = node->declare_parameter<double>(param_prefix + ".sigma_d", config->sigma_d);
  config->sigma_az = node->declare_parameter<double>(param_prefix + ".sigma_az", config->sigma_az);
  config->sigma_el = node->declare_parameter<double>(param_prefix + ".sigma_el", config->sigma_el);
  // daicp - convergence checks
  config->abs_cost_thresh = node->declare_parameter<double>(param_prefix + ".abs_cost_thresh", config->abs_cost_thresh);
  config->abs_cost_change_thresh = node->declare_parameter<double>(param_prefix + ".abs_cost_change_thresh", config->abs_cost_change_thresh);
  config->rel_cost_change_thresh = node->declare_parameter<double>(param_prefix + ".rel_cost_change_thresh", config->rel_cost_change_thresh);
  config->zero_gradient_thresh = node->declare_parameter<double>(param_prefix + ".zero_gradient_thresh", config->zero_gradient_thresh);
  config->inner_tolerance = node->declare_parameter<double>(param_prefix + ".inner_tolerance", config->inner_tolerance);
  // refinement
  config->refined_max_iter = node->declare_parameter<int>(param_prefix + ".refined_max_iter", config->refined_max_iter);
  config->refined_max_pairing_dist = node->declare_parameter<float>(param_prefix + ".refined_max_pairing_dist", config->refined_max_pairing_dist);
  config->refined_max_planar_dist = node->declare_parameter<float>(param_prefix + ".refined_max_planar_dist", config->refined_max_planar_dist);
  // error calculation
  config->averaging_num_steps = node->declare_parameter<int>(param_prefix + ".averaging_num_steps", config->averaging_num_steps);
  config->rot_diff_thresh = node->declare_parameter<float>(param_prefix + ".rot_diff_thresh", config->rot_diff_thresh);
  config->trans_diff_thresh = node->declare_parameter<float>(param_prefix + ".trans_diff_thresh", config->trans_diff_thresh);
  // outlier rejection
  config->trans_outlier_thresh = node->declare_parameter<float>(param_prefix + ".trans_outlier_thresh", config->trans_outlier_thresh);
  config->rot_outlier_thresh = node->declare_parameter<float>(param_prefix + ".rot_outlier_thresh", config->rot_outlier_thresh);
  config->min_matched_ratio = node->declare_parameter<float>(param_prefix + ".min_matched_ratio", config->min_matched_ratio);
  // online gyroscope bias
  config->calc_gy_bias = node->declare_parameter<bool>(param_prefix + ".calc_gy_bias", config->calc_gy_bias);
  config->calc_gy_bias_thresh = node->declare_parameter<float>(param_prefix + ".calc_gy_bias_thresh", config->calc_gy_bias_thresh);

  // clang-format on
  return config;
}

void LocalizationDAICPModule::run_(QueryCache &qdata0, OutputCache &output,
                                 const Graph::Ptr &,
                                 const TaskExecutor::Ptr &) {
  auto &qdata = dynamic_cast<LidarQueryCache &>(qdata0);

  if (output.chain->isLocalized() && *qdata.loc_time > config_->target_loc_time && *qdata.pipeline_mode == tactic::PipelineMode::RepeatFollow) {
    CLOG(WARNING, "lidar.localization_daicp") << "Skipping localization to save on compute. EMA val=" << *qdata.loc_time;
    return;
  }

  // Inputs
  // const auto &query_stamp = *qdata.stamp;
  const auto &query_points = *qdata.undistorted_point_cloud;
  const auto &T_s_r = *qdata.T_s_r;      // T from robot to sensor, external calibration, fixed
  const auto &T_r_v = *qdata.T_r_v_loc;  // T from vertex to robot, used as prior
  const auto &T_v_m = *qdata.T_v_m_loc;  // T from submap (build in teach) to vertex
  // const auto &map_version = qdata.submap_loc->version();
  auto &point_map = qdata.submap_loc->point_cloud();

  CLOG(DEBUG, "lidar.localization_daicp") << "####### Query point cloud has " << query_points.size() << " points.";
  CLOG(DEBUG, "lidar.localization_daicp") << "######### Map point cloud has " << point_map.size() << " points.";

  /// Parameters
  int first_steps = config_->first_num_steps;
  int max_it = config_->initial_max_iter;
  float max_pair_d = config_->initial_max_pairing_dist;
  float max_planar_d = config_->initial_max_planar_dist;
  float max_pair_d2 = max_pair_d * max_pair_d;
  KDTreeSearchParams search_params;

  // clang-format off
  /// Create robot to sensor transform variable, fixed.
  const auto T_s_r_var = SE3StateVar::MakeShared(T_s_r); T_s_r_var->locked() = true;
  const auto T_v_m_var = SE3StateVar::MakeShared(T_v_m); T_v_m_var->locked() = true;

  /// Create and add the T_robot_map variable, here m = vertex frame.
  const auto T_r_v_var = SE3StateVar::MakeShared(T_r_v);

  /// compound transform for alignment (sensor to point map transform)
  const auto T_m_s_eval = inverse(compose(T_s_r_var, compose(T_r_v_var, T_v_m_var)));

  /// Create T_m_s_var for direct point cloud alignment optimization
  const auto initial_T_m_s = T_m_s_eval->evaluate();
  auto T_m_s_var = SE3StateVar::MakeShared(initial_T_m_s);

  /// Initialize aligned points for matching (Deep copy of targets)
  pcl::PointCloud<PointWithInfo> aligned_points(query_points);

  /// Eigen matrix of original data (only shallow copy of ref clouds)
  const auto map_mat = point_map.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::cartesian_offset());
  const auto map_normals_mat = point_map.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::normal_offset());
  const auto query_mat = query_points.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::cartesian_offset());
  const auto query_norms_mat = query_points.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::normal_offset());

  /// the aligned_mat and aligned_norms_mat store the aligned points and normals which
  /// will be updated during the optimization
  auto aligned_mat = aligned_points.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::cartesian_offset());
  auto aligned_norms_mat = aligned_points.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::normal_offset());

  /// create kd-tree of the map
  CLOG(DEBUG, "lidar.localization_daicp") << "Start building a kd-tree of the map.";
  NanoFLANNAdapter<PointWithInfo> adapter(point_map);
  KDTreeParams tree_params(/* max leaf */ 10);
  auto kdtree = std::make_unique<KDTree<PointWithInfo>>(3, adapter, tree_params);
  kdtree->buildIndex();

  /// perform initial alignment
  {
    const auto T_m_s = T_m_s_var->value().matrix().cast<float>();
    aligned_mat = T_m_s * query_mat;
    aligned_norms_mat = T_m_s * query_norms_mat;
  }

  using Stopwatch = common::timing::Stopwatch<>;
  std::vector<std::unique_ptr<Stopwatch>> timer;
  std::vector<std::string> clock_str;
  clock_str.push_back("Random Sample ...... ");
  timer.emplace_back(std::make_unique<Stopwatch>(false));
  clock_str.push_back("KNN Search ......... ");
  timer.emplace_back(std::make_unique<Stopwatch>(false));
  clock_str.push_back("Point Filtering .... ");
  timer.emplace_back(std::make_unique<Stopwatch>(false));
  clock_str.push_back("Optimization ....... ");
  timer.emplace_back(std::make_unique<Stopwatch>(false));
  clock_str.push_back("Alignment .......... ");
  timer.emplace_back(std::make_unique<Stopwatch>(false));
  clock_str.push_back("Check Convergence .. ");
  timer.emplace_back(std::make_unique<Stopwatch>(false));
  clock_str.push_back("Compute Covariance . ");
  timer.emplace_back(std::make_unique<Stopwatch>(false));

  // ICP results
  EdgeTransform T_r_v_icp;
  float matched_points_ratio = 0.0;

  // Convergence variables
  float mean_dT = 0;
  float mean_dR = 0;
  Eigen::MatrixXd all_tfs = Eigen::MatrixXd::Zero(4, 4);
  bool refinement_stage = false;
  int refinement_step = 0;

  // convert the covariance back to [x,y,z,roll,pitch,yaw] order
  Eigen::PermutationMatrix<6> Pm;
  Pm.indices() << 3, 4, 5, 0, 1, 2;

  CLOG(DEBUG, "lidar.localization_daicp") << "Start the Degeneracy-Aware ICP optimization loop.";
  // outer loop
  for (int step = 0;; step++) {
    // index of the corresponding points
    timer[0]->start();
    std::vector<std::pair<size_t, size_t>> sample_inds;
    sample_inds.resize(query_points.size());
    for (size_t i = 0; i < query_points.size(); i++) sample_inds[i].first = i; 
    timer[0]->stop();
    // find nearest neigbors and distances
    std::vector<float> nn_dists(sample_inds.size());
    std::vector<std::pair<size_t, size_t>> filtered_sample_inds;
    // use curvature data association if the ptcloud has curvature info, else use spatial only
    if (aligned_points[sample_inds[0].first].curvature != 0) {
      
      /// get normalization scales
      // NOTE: speed vs. accuracy tradeoff - limit to 1k samples to prevent overwighting areas with high point density.
      //       this is especially important for curvature estimation where high-density regions can dominate the sample.
      //       limiting to 1k points should not significantly affect the accuracy of the median estimate.
      std::vector<float> sample_spatial_radii;
      std::vector<float> sample_abs_curv;
      sample_spatial_radii.reserve(1000);
      sample_abs_curv.reserve(1000);

      const size_t sampleN = std::min<size_t>(1000, point_map.size());
      for (size_t ii = 0; ii < sampleN; ++ii) {
        size_t i = (ii * 37) % point_map.size();  // use pseudo-random sampling for speed

        // 1-nearest neighbor to estimate typical spacing
        size_t nn_idx;
        float nn_dist2;
        KDTreeResultSet rs(1);
        rs.init(&nn_idx, &nn_dist2);
        kdtree->findNeighbors(rs, point_map[i].data, search_params);
        sample_spatial_radii.push_back(std::sqrt(nn_dist2));

        if (std::isfinite(point_map[i].curvature))
          sample_abs_curv.push_back(std::abs(point_map[i].curvature));
      }

      // helper median function
      auto median = [](std::vector<float> &v) -> float {
        if (v.empty()) return 1.0f;
        std::nth_element(v.begin(), v.begin() + v.size() / 2, v.end());
        return v[v.size() / 2];
      };

      float spatial_scale = median(sample_spatial_radii);
      if (spatial_scale < 1e-6f) spatial_scale = 1e-3f;  // fallback
      float curv_scale = median(sample_abs_curv);
      if (curv_scale < 1e-9f) curv_scale = 1e-6f;        // fallback

      CLOG(INFO, "lidar.localization_daicp") << "Normalization scales: spatial ≈ " << spatial_scale 
                                              << " m, curvature ≈ " << curv_scale << " 1/m";

      /// find correspondences using spatial + curvature criteria
      const int kSpatialCandidates = 8; // use a small k
      // prepare per-thread buffers so nanoflann calls are thread-safe
      std::vector<std::vector<size_t>> knn_idx_buf(config_->num_threads, std::vector<size_t>(kSpatialCandidates));
      std::vector<std::vector<float>> knn_dist_buf(config_->num_threads, std::vector<float>(kSpatialCandidates));

      int replace_count = 0; // debug info

      timer[1]->start();
#pragma omp parallel for schedule(dynamic, 10) num_threads(config_->num_threads)
      for (size_t i = 0; i < sample_inds.size(); ++i) {
        int tid = omp_get_thread_num();
        auto &idxs = knn_idx_buf[tid];
        auto &dists = knn_dist_buf[tid];

        // get k nearest spatial neighbours
        size_t found = kdtree->knnSearch(aligned_points[sample_inds[i].first].data,
                                        kSpatialCandidates, idxs.data(), dists.data());

        // initialize best candidate
        size_t best_idx = idxs[0];
        float best_spatial_d2 = dists[0];
        float best_curv_diff = std::abs(aligned_points[sample_inds[i].first].curvature - point_map[best_idx].curvature);

        // compute best score by weighted sum
        float best_score = (best_curv_diff / curv_scale) + config_->lambda * (std::sqrt(best_spatial_d2) / spatial_scale);

        // check candidates from 2 to k to find better match
        for (size_t kk = 1; kk < found; ++kk) {
          size_t cand = idxs[kk];
          float spatial_d2 = dists[kk];
          float curv_diff = std::abs(aligned_points[sample_inds[i].first].curvature - point_map[cand].curvature);

          // compute candidate score by weighted sum
          float cand_score = (curv_diff / curv_scale) + config_->lambda * (std::sqrt(spatial_d2) / spatial_scale);

          if (cand_score < best_score) {
            replace_count++;
            best_idx = cand;
            best_spatial_d2 = spatial_d2;
            best_curv_diff = curv_diff;
            best_score = cand_score;
          }
        }
        sample_inds[i].second = best_idx;
        nn_dists[i] = best_spatial_d2;
      }
      timer[1]->stop();

      CLOG(DEBUG, "lidar.localization_daicp") << "Spatial+k-curv refinement done, found " << sample_inds.size() << " pairs, replaced " << replace_count << " neighbours";

      /// Filter point pairs based on distances metrics
      timer[2]->start();
      filtered_sample_inds.reserve(sample_inds.size());
      std::vector<size_t> filtered_source_indices;
      std::vector<size_t> filtered_target_indices;
      std::vector<float> filtered_distances;
      for (size_t i = 0; i < sample_inds.size(); i++) {
        if (nn_dists[i] < max_pair_d2) {
          // Check planar distance (only after a few steps for initial alignment)
          auto diff = aligned_points[sample_inds[i].first].getVector3fMap() -
                      point_map[sample_inds[i].second].getVector3fMap();
          float planar_dist = std::abs(
                      diff.dot(point_map[sample_inds[i].second].getNormalVector3fMap()));
          if (step < first_steps || planar_dist < max_planar_d) {
            filtered_sample_inds.push_back(sample_inds[i]);
          }
        }
      }

      CLOG(DEBUG, "lidar.localization_daicp") << "Distance filtering done, found " << filtered_sample_inds.size() << " pairs.";

      // Second filter by curvature if available
      if (!filtered_sample_inds.empty()) {
        std::vector<std::pair<size_t, size_t>> curvature_filtered_sample_inds;
        curvature_filtered_sample_inds.reserve(filtered_sample_inds.size());
        for (size_t i = 0; i < filtered_sample_inds.size(); i++) {
          float src_curv = aligned_points[filtered_sample_inds[i].first].curvature;
          float tgt_curv = point_map[filtered_sample_inds[i].second].curvature;
          if (std::abs(src_curv - tgt_curv) < config_->curvature_similarity_thresh) {
            curvature_filtered_sample_inds.emplace_back(filtered_sample_inds[i]);
          }
        }
        filtered_sample_inds = std::move(curvature_filtered_sample_inds);
        CLOG(DEBUG, "lidar.localization_daicp") << "Curvature filtering done, filtered to " << filtered_sample_inds.size() << " pairs.";
      } else {
        // fallback: no points passed distance filter
        filtered_sample_inds.clear();
      }
      CLOG(DEBUG, "lidar.localization_daicp") << "Filtered " << sample_inds.size() << " point pairs to "
                                          << filtered_sample_inds.size() << " pairs.";
      timer[2]->stop();

    } else {
      /// find nearest neighbours and distances
      timer[1]->start();
      std::vector<float> nn_dists(sample_inds.size());
#pragma omp parallel for schedule(dynamic, 10) num_threads(config_->num_threads)
      for (size_t i = 0; i < sample_inds.size(); i++) {
        KDTreeResultSet result_set(1);
        result_set.init(&sample_inds[i].second, &nn_dists[i]);
        kdtree->findNeighbors(result_set, aligned_points[sample_inds[i].first].data, search_params);
      }
      timer[1]->stop();

      /// filtering based on distances metrics
      timer[2]->start();
      filtered_sample_inds.reserve(sample_inds.size());
      for (size_t i = 0; i < sample_inds.size(); i++) {
        if (nn_dists[i] < max_pair_d2) {
          // Check planar distance (only after a few steps for initial alignment)
          auto diff = aligned_points[sample_inds[i].first].getVector3fMap() -
                      point_map[sample_inds[i].second].getVector3fMap();
          float planar_dist = std::abs(
              diff.dot(point_map[sample_inds[i].second].getNormalVector3fMap()));
          if (step < first_steps || planar_dist < max_planar_d) {
            filtered_sample_inds.push_back(sample_inds[i]);
          }
        }
      }
      timer[2]->stop();
      CLOG(DEBUG, "lidar.localization_daicp") << "Distance filtering done, found " << filtered_sample_inds.size() << " pairs.";
    }

    /// Degeneracy-Aware ICP point-to-plane optimization
    timer[3]->start();

    // /// ########################### Gauss-Newton solver ########################### ///
    // bool optimization_success = daicp_lib::daGaussNewtonP2Plane(filtered_sample_inds, 
    //                                                             query_mat,
    //                                                             map_mat,
    //                                                             map_normals_mat,
    //                                                             T_m_s_var,
    //                                                             max_gn_iter,
    //                                                             inner_tolerance);
    // /// ########################################################################### ///


    /// ########################### Gauss-Newton solver ########################### ///
    EdgeTransform T_s_m_edge;
    T_s_m_edge = T_s_r * T_r_v * T_v_m;
    EdgeTransform T_m_s_edge = T_s_m_edge.inverse();
    // Initialize output covariance matrix
    Eigen::MatrixXd daicp_cov = Eigen::MatrixXd::Zero(6, 6);

    Eigen::Matrix<double, 4, 4> T_m_s_prior = T_m_s_edge.matrix();
    bool optimization_success = daicp_lib::daGaussNewtonP2Plane(
        // inputs
        filtered_sample_inds, 
        query_mat,
        map_mat,
        map_normals_mat,
        // state to be optimized
        T_m_s_var,
        // prior, transformation from lidar to map, will be simplified.
        T_m_s_prior,
        // configuration
        config_,
        // output
        daicp_cov
      );
    // convert the covariance back to [x,y,z,roll,pitch,yaw] order
    daicp_cov = Pm.transpose() * daicp_cov * Pm;
    /// ########################################################################### ///

    if (!optimization_success) {
      CLOG(WARNING, "lidar.localization_daicp") << "Gauss-Newton optimization failed at step " << step;
      break;
    }

    timer[3]->stop();

    /// Alignment, update aligned_mat and aligned_norms_mat
    timer[4]->start();
    {
      const auto T_m_s = T_m_s_var->value().matrix().cast<float>();
      aligned_mat = T_m_s * query_mat;
      aligned_norms_mat = T_m_s * query_norms_mat;
    }
    /// save the transformation results
    const auto T_m_s = T_m_s_var->value().matrix();
    if (step == 0)
      all_tfs = Eigen::MatrixXd(T_m_s);
    else {
      Eigen::MatrixXd temp(all_tfs.rows() + 4, 4);
      temp.topRows(all_tfs.rows()) = all_tfs;
      temp.bottomRows(4) = Eigen::MatrixXd(T_m_s);
      all_tfs = temp;
    }
    timer[4]->stop();

    /// Check convergence
    timer[5]->start();
    // Update variations
    if (step > 0) {
      float avg_tot = step == 1 ? 1.0 : (float)config_->averaging_num_steps;

      // Get last transformation variations
      Eigen::Matrix4d T2 = all_tfs.block<4, 4>(all_tfs.rows() - 4, 0);
      Eigen::Matrix4d T1 = all_tfs.block<4, 4>(all_tfs.rows() - 8, 0);
      Eigen::Matrix4d diffT = T2 * T1.inverse();
      Eigen::Matrix<double, 6, 1> diffT_vec = lgmath::se3::tran2vec(diffT);
      float dT_b = diffT_vec.block<3, 1>(0, 0).norm();
      float dR_b = diffT_vec.block<3, 1>(3, 0).norm();

      mean_dT += (dT_b - mean_dT) / avg_tot;
      mean_dR += (dR_b - mean_dR) / avg_tot;
    }

    // Refinement incremental
    if (refinement_stage) refinement_step++;

    // Stop condition
    if (!refinement_stage && step >= first_steps) {
      if ((step >= max_it - 1) || (mean_dT < config_->trans_diff_thresh &&
                                   mean_dR < config_->rot_diff_thresh)) {
        CLOG(DEBUG, "lidar.localization_daicp") << "Initial alignment takes " << step << " steps.";

        // enter the second refine stage
        refinement_stage = true;
        max_it = step + config_->refined_max_iter;
        // reduce the max distance
        max_pair_d = config_->refined_max_pairing_dist;
        max_pair_d2 = max_pair_d * max_pair_d;
        max_planar_d = config_->refined_max_planar_dist;
      }
    }
    timer[5]->stop();

    /// Last step
    timer[6]->start();
    if ((refinement_stage && step >= max_it - 1) ||
        (refinement_step > config_->averaging_num_steps &&
         mean_dT < config_->trans_diff_thresh &&
         mean_dR < config_->rot_diff_thresh)) {

      // --- implement the filter-based fusion of the prior  
      // the edge transformation of T_m_s from DA-ICP
      EdgeTransform T_m_s_icp_edge(T_m_s_var->value(), daicp_cov);
      // the edge transformation of T_r_v computed from DA-ICP
      EdgeTransform T_r_v_icp_edge;   
      T_r_v_icp_edge =  T_s_r.inverse() * T_m_s_icp_edge.inverse() * T_v_m.inverse();

      // [DEBUGGING] print out the matrix and covariance
      CLOG(DEBUG, "lidar.localization_daicp") << "================= DA-ICP result T_r_v_icp: \n" << T_r_v_icp_edge.matrix();
      CLOG(DEBUG, "lidar.localization_daicp") << "================= DA-ICP result covariance: \n" << T_r_v_icp_edge.cov();

      // filter-based fusion [disabled, performance is not good]
      // compute the residual 
      Eigen::Matrix<double, 6, 1> residual = lgmath::se3::tran2vec(T_r_v.matrix().inverse() * T_r_v_icp_edge.matrix());
      Eigen::Matrix4d T_res = T_r_v.matrix().inverse() * T_r_v_icp_edge.matrix();
      Eigen::Matrix<double, 6, 6> T_res_cov = lgmath::se3::tranAd(T_res) * daicp_cov * lgmath::se3::tranAd(T_res).transpose();
      Eigen::Matrix<double, 6, 6> K_gain = T_r_v.cov() * (T_r_v.cov() + T_res_cov).inverse();
      Eigen::Matrix<double, 6, 1> delta_vec = K_gain * residual;
      Eigen::Matrix4d T_r_v_post = lgmath::se3::vec2tran(delta_vec) * T_r_v.matrix();
      Eigen::Matrix<double, 6, 6> T_r_v_post_cov = (Eigen::Matrix<double, 6, 6>::Identity() - K_gain) * T_r_v.cov();

      // ============================= original code block =========================== //   
      // Decode T_r_v from optimized T_m_s_var
      // T_m_s = T_m_v * T_v_r * T_r_s, so T_r_v = T_r_s * T_s_m * T_m_v
      const auto T_m_s_optimized = T_m_s_var->value();
      const auto T_s_m_optimized = T_m_s_optimized.inverse();
      const auto T_r_v_decoded = T_s_r.inverse() * T_s_m_optimized * T_v_m.inverse();
      // ============================================================================= // 

      // [Debugging]
      auto T_r_v_select = T_r_v_decoded;

      // -------- Check difference between prior and computed T_r_v
      const auto T_r_v_prior = T_r_v_var->value();
      const auto T_diff = T_r_v_select * T_r_v_prior.inverse();
      const auto T_diff_vec = lgmath::se3::tran2vec(T_diff.matrix());
      const double translation_diff = T_diff_vec.head<3>().norm();
      const double rotation_diff = T_diff_vec.tail<3>().norm();

      // OUTLIER REJECTION: larger than 0.3m or 0.2rad (11.5deg)
      if ((translation_diff) > config_->trans_outlier_thresh || (rotation_diff) > config_->rot_outlier_thresh) {
        CLOG(WARNING, "lidar.localization_daicp") << "Significant difference detected in T_r_v, fall back to odometry:";
        CLOG(DEBUG, "lidar.localization_daicp") << "Prior vs Computed T_r_v comparison:";
        CLOG(DEBUG, "lidar.localization_daicp") << "  Translation difference: " << translation_diff << " m";
        CLOG(DEBUG, "lidar.localization_daicp") << "  Rotation difference: " << rotation_diff << " rad (" 
                                                << (rotation_diff * 180.0 / M_PI) << " deg)";

        // fallback to using prior
        T_r_v_icp = EdgeTransform(T_r_v_prior, T_r_v.cov());
        matched_points_ratio = 1.0f;  // dummy value to indicate success
      } else {
        Eigen::Matrix<double, 6, 6> dummy_cov = Eigen::Matrix<double, 6, 6>::Identity();
        dummy_cov.diagonal() << 0.001, 0.001, 0.001, 1e-6, 1e-6, 1e-6;  // [x,y,z,rx,ry,rz]
        // dummy_cov.diagonal() << 1e10, 1e10, 1e10, 1e6, 1e6, 1e6;  // [x,y,z,rx,ry,rz]

        T_r_v_icp = EdgeTransform(T_r_v_select, dummy_cov);
        // T_r_v_icp = EdgeTransform(T_r_v_post, T_r_v_post_cov);
        matched_points_ratio = (float)filtered_sample_inds.size() / (float)sample_inds.size();
      }

      //
      CLOG(DEBUG, "lidar.localization_daicp") << "Total number of steps: " << step << ", with matched ratio " << matched_points_ratio;
      if (mean_dT >= config_->trans_diff_thresh ||
          mean_dR >= config_->rot_diff_thresh) {
        CLOG(WARNING, "lidar.localization_daicp") << "DA-ICP did not converge to the specified threshold.";
        if (!refinement_stage) {
          CLOG(WARNING, "lidar.localization_daicp") << "DA-ICP did not enter refinement stage at all.";
        }
      }
      break;
    }
    timer[6]->stop();
  }

  /// Dump timing info
  CLOG(DEBUG, "lidar.localization_daicp") << "Dump timing info inside loop: ";
  for (size_t i = 0; i < clock_str.size(); i++) {
    CLOG(DEBUG, "lidar.localization_daicp") << clock_str[i] << timer[i]->count();
  }

  /// Outputs
  if (matched_points_ratio > config_->min_matched_ratio) {
    // update map to robot transform
    *qdata.T_r_v_loc = T_r_v_icp;
    // set success
    *qdata.loc_success = true;

    // Gyroscope bias estimation
    if (config_->calc_gy_bias && qdata.gyro_msgs->size() > 0) {
      const auto &query_stamp = *qdata.stamp;
      Eigen::Matrix4d T_r_v_loc = T_r_v_icp.matrix();
      double dt = (query_stamp - timestamp_prev_) * 1e-9;
      
      if (timestamp_prev_ == 0) {
        timestamp_prev_ = query_stamp;
        T_r_v_loc_prev_ = T_r_v_loc;
        return;
      }

      // check if enough time has passed
      if (dt < config_->calc_gy_bias_thresh) {
        CLOG(DEBUG, "lidar.localization_daicp") << "Not enough motion since last update. Skip gyro bias estimation.";
        return;
      }

      Eigen::Matrix<double, 6, 1> phi = lgmath::se3::tran2vec(T_r_v_loc * T_r_v_loc_prev_.inverse());
      Eigen::Matrix<double, 6, 1> varpi_hat = phi / dt;

      Eigen::Vector3d w_hat = varpi_hat.tail<3>();
      
      Eigen::Vector3d gyro_avg = Eigen::Vector3d::Zero();
      for (const auto& msg : *qdata.gyro_msgs) {
        gyro_avg += Eigen::Vector3d(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z);
      }
      gyro_avg /= (double)qdata.gyro_msgs->size();

      Eigen::Vector3d gyro_bias_update = gyro_avg - w_hat;

      // check if the computed gyro_bias_update is similar to the previous gyro_bias
      bool is_similar = ((*qdata.gyro_bias) - gyro_bias_update).norm() < 1e-3;
      if (!is_similar) {
        CLOG(WARNING, "lidar.localization_daicp") << "Computed gyro_bias_update is not similar to previous estimate. Skip update.";
        return;
      }

      // update cache
      *qdata.gyro_bias = 0.8*(*qdata.gyro_bias) + 0.2*gyro_bias_update;
      CLOG(DEBUG, "lidar.localization_daicp") << "Estimated gyro bias: " << (*qdata.gyro_bias).transpose();
      timestamp_prev_ = query_stamp;
      T_r_v_loc_prev_ = T_r_v_loc;
    }
  } else {
    CLOG(WARNING, "lidar.localization_daicp")
        << "Matched points ratio " << matched_points_ratio
        << " is below the threshold. DA-ICP is considered failed.";
    // no update to map to robot transform
    // set success
    *qdata.loc_success = false;
  }

}

}  // namespace lidar
}  // namespace vtr
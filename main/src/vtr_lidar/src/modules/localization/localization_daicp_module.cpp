#include "vtr_lidar/modules/localization/localization_daicp_module.hpp"
// #include "vtr_lidar/modules/localization/daicp_lib.hpp"
#include "vtr_lidar/modules/localization/daicp_lib_new.hpp"
#include "vtr_lidar/utils/nanoflann_utils.hpp"

struct CurvatureAdapter4D {
  const pcl::PointCloud<vtr::lidar::PointWithInfo>& points;
  CurvatureAdapter4D(const pcl::PointCloud<vtr::lidar::PointWithInfo>& pts) : points(pts) {}

  inline size_t kdtree_get_point_count() const { return points.size(); }

  inline float kdtree_get_pt(size_t idx, size_t dim) const {
    const auto& p = points[idx];
    if (dim == 0) return p.x;
    if (dim == 1) return p.y;
    if (dim == 2) return p.z;
    return p.curvature;
  }

  template <class BBOX>
  bool kdtree_get_bbox(BBOX&) const { return false; }
};

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
  config->use_pose_prior = node->declare_parameter<bool>(param_prefix + ".use_pose_prior", config->use_pose_prior);

  // icp params
  config->num_threads = node->declare_parameter<int>(param_prefix + ".num_threads", config->num_threads);
  config->first_num_steps = node->declare_parameter<int>(param_prefix + ".first_num_steps", config->first_num_steps);
  config->initial_max_iter = node->declare_parameter<int>(param_prefix + ".initial_max_iter", config->initial_max_iter);
  config->initial_max_pairing_dist = node->declare_parameter<float>(param_prefix + ".initial_max_pairing_dist", config->initial_max_pairing_dist);
  config->initial_max_planar_dist = node->declare_parameter<float>(param_prefix + ".initial_max_planar_dist", config->initial_max_planar_dist);
  config->refined_max_iter = node->declare_parameter<int>(param_prefix + ".refined_max_iter", config->refined_max_iter);
  config->refined_max_pairing_dist = node->declare_parameter<float>(param_prefix + ".refined_max_pairing_dist", config->refined_max_pairing_dist);
  config->refined_max_planar_dist = node->declare_parameter<float>(param_prefix + ".refined_max_planar_dist", config->refined_max_planar_dist);
  config->averaging_num_steps = node->declare_parameter<int>(param_prefix + ".averaging_num_steps", config->averaging_num_steps);
  config->rot_diff_thresh = node->declare_parameter<float>(param_prefix + ".rot_diff_thresh", config->rot_diff_thresh);
  config->trans_diff_thresh = node->declare_parameter<float>(param_prefix + ".trans_diff_thresh", config->trans_diff_thresh);
  config->verbose = node->declare_parameter<bool>(param_prefix + ".verbose", false);
  config->max_iterations = (unsigned int)node->declare_parameter<int>(param_prefix + ".max_iterations", 1);
  config->target_loc_time = node->declare_parameter<float>(param_prefix + ".target_loc_time", config->target_loc_time);

  config->use_odo = node->declare_parameter<bool>(param_prefix + ".use_odo", config->use_odo); // temp
  config->eigenvalue_threshold = node->declare_parameter<double>(param_prefix + ".eigenvalue_threshold", config->eigenvalue_threshold);
  config->outlier_trans_thresh = node->declare_parameter<float>(param_prefix + ".outlier_trans_thresh", config->outlier_trans_thresh);
  config->outlier_rot_thresh = node->declare_parameter<float>(param_prefix + ".outlier_rot_thresh", config->outlier_rot_thresh);

  const auto cov_tuning = node->declare_parameter<std::vector<double>>(param_prefix + ".covariance", std::vector<double>());
  if (cov_tuning.size() != 6) {
    std::string err{"covariance malformed. Must be 6 elements!"};
    CLOG(ERROR, "lidar.localization_daicp") << err;
    throw std::invalid_argument{err};
  }
  config->covariance = Eigen::DiagonalMatrix<double, 6>(cov_tuning[0], cov_tuning[1], cov_tuning[2], cov_tuning[3], cov_tuning[4], cov_tuning[5]);

  config->calc_gy_bias = node->declare_parameter<bool>(param_prefix + ".calc_gy_bias", config->calc_gy_bias);
  config->calc_gy_bias_thresh = node->declare_parameter<float>(param_prefix + ".calc_gy_bias_thresh", config->calc_gy_bias_thresh);
  config->correspondence_method = node->declare_parameter<int>(param_prefix + ".correspondence_method", config->correspondence_method);
  config->curvature_similarity_thresh = node->declare_parameter<float>(param_prefix + ".curvature_similarity_thresh", config->curvature_similarity_thresh);

  config->min_matched_ratio = node->declare_parameter<float>(param_prefix + ".min_matched_ratio", config->min_matched_ratio);
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
  
  CLOG(DEBUG, "lidar.localization_daicp") << "Starting DA-ICP localization.";
  // Inputs
  // const auto &query_stamp = *qdata.stamp;
  const auto &query_points = *qdata.undistorted_point_cloud;
  const auto &T_s_r = *qdata.T_s_r;      // T from robot to sensor, external calibration, fixed
  const auto &T_r_v = *qdata.T_r_v_loc;  // T from vertex to robot, used as prior
  const auto &T_v_m = *qdata.T_v_m_loc;  // T from submap (build in teach) to vertex
  // const auto &map_version = qdata.submap_loc->version();
  auto &point_map = qdata.submap_loc->point_cloud();

  CLOG(DEBUG, "lidar.localization_daicp") << "Point map has " << point_map.size() << " points.";

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

  // temporary!!! return odometry estimate as localization result directly
  if (config_->use_odo) {
    const auto T_r_v_prior = T_r_v_var->value();
    *qdata.T_r_v_loc = EdgeTransform(T_r_v_prior, T_r_v.cov());
    *qdata.loc_success = true;
    return;
  }

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
    if (config_->correspondence_method == 0 || config_->correspondence_method == 1 || config_->correspondence_method == 2) {
      if (config_->correspondence_method == 0) {
        CurvatureAdapter4D curv_adapter(point_map);

        using CurvKDTreeType = nanoflann::KDTreeSingleIndexAdaptor<
            nanoflann::L2_Simple_Adaptor<float, CurvatureAdapter4D>,
            CurvatureAdapter4D,
            4>;

        KDTreeParams tree_params(10);
        CurvKDTreeType curv_kdtree(4, curv_adapter, tree_params);
        curv_kdtree.buildIndex();

        // compute dynamic curvature scaling
        Eigen::Vector3f mean_xyz = Eigen::Vector3f::Zero();
        float mean_curv = 0.0f;

        for (const auto &p : point_map) {
          mean_xyz += Eigen::Vector3f(p.x, p.y, p.z);
          mean_curv += p.curvature;
        }
        mean_xyz /= point_map.size();
        mean_curv /= point_map.size();

        // Variances
        float var_xyz = 0.0f;
        float var_curv = 0.0f;
        for (const auto &p : point_map) {
          Eigen::Vector3f diff = Eigen::Vector3f(p.x, p.y, p.z) - mean_xyz;
          var_xyz += diff.squaredNorm();
          float dc = p.curvature - mean_curv;
          var_curv += dc * dc;
        }
        var_xyz /= point_map.size();
        var_curv /= point_map.size();

        // if curvature has small variance, scale it up so it contributes similarly
        float curvature_scale = std::sqrt(var_xyz / (var_curv + 1e-9f));

        CLOG(INFO, "lidar.localization_daicp")
            << "Dynamic curvature scaling: " << curvature_scale;

        timer[1]->start();
#pragma omp parallel for schedule(dynamic, 10) num_threads(config_->num_threads)
        for (size_t i = 0; i < sample_inds.size(); ++i) {
          float query_pt[4] = {
            aligned_points[sample_inds[i].first].x,
            aligned_points[sample_inds[i].first].y,
            aligned_points[sample_inds[i].first].z,
            curvature_scale * aligned_points[sample_inds[i].first].curvature
          };

          size_t nearest_idx = static_cast<size_t>(-1);
          float distance_sq = 0.0f;

          nanoflann::KNNResultSet<float> result_set(1);
          result_set.init(&nearest_idx, &distance_sq);

          curv_kdtree.findNeighbors(result_set, query_pt, search_params);

          sample_inds[i].second = nearest_idx;
          nn_dists[i] = distance_sq;
        }
        CLOG(DEBUG, "lidar.localization_daicp") << "Curvature KDTree nn search done, found " << sample_inds.size() << " pairs.";
        timer[1]->stop();
      } else if (config_->correspondence_method == 1) {
        // Build a map from cluster_id to indices in point_map
        std::unordered_map<int, std::vector<size_t>> target_clusters;
        target_clusters.reserve(point_map.size());
        for (size_t i = 0; i < point_map.size(); i++) {
          target_clusters[point_map[i].cluster_id].push_back(i);
        }

        timer[1]->start();
#pragma omp parallel for schedule(dynamic,10) num_threads(config_->num_threads)
        for (size_t i = 0; i < sample_inds.size(); i++) {
          size_t src_idx = sample_inds[i].first;
          int src_cluster = aligned_points[src_idx].cluster_id;

          float best_d2 = std::numeric_limits<float>::max();
          size_t best_idx = 0;
          Eigen::Vector3f src_pt = aligned_points[src_idx].getVector3fMap();

          auto it = target_clusters.find(src_cluster);
          if (it != target_clusters.end()) {
            const auto &cluster_pts = it->second;
            for (auto tgt_idx : cluster_pts) {
              float d2 = (src_pt - point_map[tgt_idx].getVector3fMap()).squaredNorm();
              if (d2 < best_d2) {
                best_d2 = d2;
                best_idx = tgt_idx;
              }
            }
          } else {
            // fallback: brute force
            for (size_t tgt_idx = 0; tgt_idx < point_map.size(); tgt_idx++) {
              float d2 = (src_pt - point_map[tgt_idx].getVector3fMap()).squaredNorm();
              if (d2 < best_d2) {
                best_d2 = d2;
                best_idx = tgt_idx;
              }
            }
          }

          nn_dists[i] = best_d2;
          sample_inds[i].second = best_idx;
        }
        CLOG(DEBUG, "lidar.localization_daicp") << "Cluster-based nn search done, found " << sample_inds.size() << " pairs.";
        timer[1]->stop();
      } else if (config_->correspondence_method == 2) {
        timer[1]->start();
        std::vector<float> nn_dists(sample_inds.size());
#pragma omp parallel for schedule(dynamic, 10) num_threads(config_->num_threads)
        for (size_t i = 0; i < sample_inds.size(); i++) {
          KDTreeResultSet result_set(1);
          result_set.init(&sample_inds[i].second, &nn_dists[i]);
          kdtree->findNeighbors(result_set, aligned_points[sample_inds[i].first].data, search_params);
        }
        CLOG(DEBUG, "lidar.localization_daicp") << "Spatial KDTree nn search done, found " << sample_inds.size() << " pairs.";
        timer[1]->stop();
      }

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

      CLOG(DEBUG, "lidar.localization_daicp") << "Distance filtering done, found " << filtered_source_indices.size() << " pairs.";

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
    /// ########################### Gauss-Newton solver ########################### ///
    int max_gn_iter = 30;
    double inner_tolerance = 1e-6;

    // // old version in daicp_lib.hpp
    // bool optimization_success = daicp_lib::daGaussNewtonP2Plane(filtered_sample_inds, 
    //                                                             query_mat,
    //                                                             map_mat,
    //                                                             map_normals_mat,
    //                                                             T_m_s_var,
    //                                                             max_gn_iter,
    //                                                             inner_tolerance);

    /// ########################################################################### ///
    // T_m_s is the tranformation from the current submap to the lidar sensor.
    // The submap is built in the teach step and is the one we are localizing against.
    EdgeTransform T_s_m_edge;
    T_s_m_edge = T_s_r * T_r_v * T_v_m;
    EdgeTransform T_m_s_edge = T_s_m_edge.inverse();

    Eigen::Matrix<double, 4, 4> T_m_s_prior = T_m_s_edge.matrix();

    // [debugging here]
    // NOTE: T_m_s_cov is for [x, y, z, roll, pitch, yaw]
    Eigen::MatrixXd T_m_s_cov = T_m_s_edge.cov();
    
    CLOG(DEBUG, "lidar.localization_daicp") << " =========== T_s_r.cov(): \n" << T_s_r.cov().diagonal().transpose();
    CLOG(DEBUG, "lidar.localization_daicp") << " =========== T_r_v.cov(): \n" << T_r_v.cov().diagonal().transpose();
    CLOG(DEBUG, "lidar.localization_daicp") << " =========== T_v_m.cov(): \n" << T_v_m.cov().diagonal().transpose();
    CLOG(DEBUG, "lidar.localization_daicp") << " =========== T_m_s_cov diagonal: [" << T_m_s_cov.diagonal().transpose() << "]";

    // --------- [DEBUG]: set a fixed covariance
    // Eigen::MatrixXd T_m_s_cov = Eigen::MatrixXd::Identity(6, 6);  // covariance for lidar pose
    // T_m_s_cov.diagonal() << 0.05*0.05, 0.05*0.05, 0.05*0.05, 0.01*0.01, 0.01*0.01, 0.01*0.01;

    // Define permutation: new order = [3,4,5,0,1,2]
    Eigen::PermutationMatrix<6> Pm;
    Pm.indices() << 3, 4, 5, 0, 1, 2;
    // Apply permutation to have covariance in order [roll, pitch, yaw, x, y, z]
    Eigen::Matrix<double, 6, 6> T_reordered = Pm * T_m_s_cov * Pm.transpose();

    // new version in daicp_lib_new.hpp
    bool optimization_success = daicp_lib_new::daGaussNewtonScaleP2Plane(filtered_sample_inds,
                                                                         query_mat,
                                                                         map_mat,
                                                                         map_normals_mat,
                                                                         T_m_s_var,    // SE3StateVar, param to optimize
                                                                         T_m_s_prior,  // transformation from lidar to map
                                                                         T_reordered,    // covariance of the above transformation
                                                                         max_gn_iter,
                                                                         inner_tolerance);
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
      CLOG(INFO, "lidar.localization_daicp") << "Initial alignment check at step " << step 
                                          << ", mean_dT: " << mean_dT << ", mean_dR: " << mean_dR;
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
      // Decode T_r_v from optimized T_m_s_var
      // T_m_s = T_m_v * T_v_r * T_r_s, so T_r_v = T_r_s * T_s_m * T_m_v
      const auto T_m_s_optimized = T_m_s_var->value();
      const auto T_s_m_optimized = T_m_s_optimized.inverse();
      const auto T_r_v_decoded = T_s_r.inverse() * T_s_m_optimized * T_v_m.inverse();
      
      // -------- Check difference between prior and computed T_r_v
      const auto T_r_v_prior = T_r_v_var->value();
      const auto T_diff = T_r_v_decoded * T_r_v_prior.inverse();
      const auto T_diff_vec = lgmath::se3::tran2vec(T_diff.matrix());
      const double translation_diff = T_diff_vec.head<3>().norm();
      const double rotation_diff = T_diff_vec.tail<3>().norm();

      // OUTLIER REJECTION: larger than 0.2m or 0.1rad (5.7deg)
      if (translation_diff > config_->outlier_trans_thresh || rotation_diff > config_->outlier_rot_thresh) {
        CLOG(WARNING, "lidar.localization_daicp") << "Significant difference detected in T_r_v, fall back to odometry:";
        CLOG(DEBUG, "lidar.localization_daicp") << "Prior vs Computed T_r_v comparison:";
        CLOG(DEBUG, "lidar.localization_daicp") << "  Translation difference: " << translation_diff << " m";
        CLOG(DEBUG, "lidar.localization_daicp") << "  Rotation difference: " << rotation_diff << " rad (" 
                                                << (rotation_diff * 180.0 / M_PI) << " deg)";

        // fallback to using prior
        T_r_v_icp = EdgeTransform(T_r_v_prior, T_r_v.cov());
        matched_points_ratio = 1.0f;  // dummy value to indicate success
      } else {
        // Create T_r_v_icp with dummy covariance
        // Eigen::Matrix<double, 6, 6> dummy_cov = Eigen::Matrix<double, 6, 6>::Identity();
        // dummy_cov.diagonal() << 0.001, 0.001, 0.001, 1e-6, 1e-6, 1e-6;  // [x,y,z,rx,ry,rz]
        T_r_v_icp = EdgeTransform(T_r_v_decoded, config_->covariance);
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

      CLOG(DEBUG, "lidar.localization_daicp") << "Time since last update: " << dt << " s";
      
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

      CLOG(DEBUG, "lidar.localization_daicp") << "number of gyro msgs: " << qdata.gyro_msgs->size();

      Eigen::Vector3d gyro_bias_update = gyro_avg - w_hat;

      CLOG(DEBUG, "lidar.localization_daicp") << "gyro_avg: " << gyro_avg.transpose();
      CLOG(DEBUG, "lidar.localization_daicp") << "w_hat: " << w_hat.transpose();

      // check if the computed gyro_bias_update is similar to the previous gyro_bias
      bool is_similar = ((*qdata.gyro_bias) - gyro_bias_update).norm() < 1e-3;
      CLOG(DEBUG, "lidar.localization_daicp") << "gyro_bias_update similarity to previous: " << is_similar;
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
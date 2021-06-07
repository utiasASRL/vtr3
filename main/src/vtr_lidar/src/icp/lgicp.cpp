#include <vtr_lidar/icp/lgicp.hpp>

namespace vtr {
namespace lidar {

namespace {

Eigen::Matrix4d interpolatePose(float t, Eigen::Matrix4d const& H1,
                                Eigen::Matrix4d const& H2, int verbose) {
  // Assumes 0 < t < 1
  Eigen::Matrix3d R1 = H1.block(0, 0, 3, 3);
  Eigen::Matrix3d R2 = H2.block(0, 0, 3, 3);

  // Rotations to quaternions
  Eigen::Quaternion<double> rot1(R1);
  Eigen::Quaternion<double> rot2(R2);
  Eigen::Quaternion<double> rot3 = rot1.slerp(t, rot2);

  // ------------------ ICI ----------------------

  if (verbose > 0) {
    cout << R2.determinant() << endl;
    cout << R2 << endl;
    cout << "[" << rot1.x() << " " << rot1.y() << " " << rot1.z() << " "
         << rot1.w() << "] -> ";
    cout << "[" << rot2.x() << " " << rot2.y() << " " << rot2.z() << " "
         << rot2.w() << "] / " << t << endl;
    cout << "[" << rot3.x() << " " << rot3.y() << " " << rot3.z() << " "
         << rot3.w() << "]" << endl;
    cout << rot2.toRotationMatrix() << endl;
    cout << rot3.toRotationMatrix() << endl;
  }

  // Translations to vectors
  Eigen::Vector3d trans1 = H1.block(0, 3, 3, 1);
  Eigen::Vector3d trans2 = H2.block(0, 3, 3, 1);

  // Interpolation (not the real geodesic path, but good enough)
  Eigen::Affine3d result;
  result.translation() = (1.0 - t) * trans1 + t * trans2;
  result.linear() = rot1.slerp(t, rot2).normalized().toRotationMatrix();

  return result.matrix();
}

void solvePoint2PlaneLinearSystem(const Matrix6d& A, const Vector6d& b,
                                  Vector6d& x) {
  // Define a slover on matrix A
  Eigen::FullPivHouseholderQR<Matrix6d> Aqr(A);

  if (!Aqr.isInvertible()) {
    std::cout << "WARNING: Full minimization instead of cholesky" << endl;
    // Solve system (but solver is slow???)
    x = Aqr.solve(b);
  } else {
    // Cholesky decomposition
    x = A.llt().solve(b);
  }
}

void minimizePointToPlaneError(std::vector<PointXYZ>& targets,
                               std::vector<PointXYZ>& references,
                               std::vector<PointXYZ>& refNormals,
                               std::vector<float>& weights,
                               std::vector<pair<size_t, size_t>>& sample_inds,
                               Eigen::Matrix4d& mOut) {
  // See: "Linear Least-Squares Optimization for Point-to-Plane ICP Surface
  // Registration" (Kok-Lim Low) init A and b matrice
  size_t N = sample_inds.size();
  Eigen::Matrix<double, Eigen::Dynamic, 6> A(N, 6);
  Eigen::Matrix<double, Eigen::Dynamic, 6> wA(N, 6);
  Eigen::Matrix<double, Eigen::Dynamic, 1> b(N, 1);

  // Fill matrices values
  bool tgt_weights = weights.size() == targets.size();
  bool ref_weights = weights.size() == references.size();
  int i = 0;
  for (const auto& ind : sample_inds) {
    // Target point
    double sx = (double)targets[ind.first].x;
    double sy = (double)targets[ind.first].y;
    double sz = (double)targets[ind.first].z;

    // Reference point
    double dx = (double)references[ind.second].x;
    double dy = (double)references[ind.second].y;
    double dz = (double)references[ind.second].z;

    // Reference point normal
    double nx = (double)refNormals[ind.second].x;
    double ny = (double)refNormals[ind.second].y;
    double nz = (double)refNormals[ind.second].z;

    // setup least squares system
    A(i, 0) = nz * sy - ny * sz;
    A(i, 1) = nx * sz - nz * sx;
    A(i, 2) = ny * sx - nx * sy;
    A(i, 3) = nx;
    A(i, 4) = ny;
    A(i, 5) = nz;
    b(i, 0) = nx * dx + ny * dy + nz * dz - nx * sx - ny * sy - nz * sz;

    // Apply weights if needed
    if (tgt_weights)
      wA.row(i) = A.row(i) * (double)weights[ind.first];
    else if (ref_weights)
      wA.row(i) = A.row(i) * (double)weights[ind.second];
    i++;
  }

  // linear least square matrices
  Matrix6d A_ = wA.transpose() * A;
  Vector6d b_ = wA.transpose() * b;

  // Solve linear optimization
  Vector6d x;
  solvePoint2PlaneLinearSystem(A_, b_, x);

  // Get transformation rotation
  Eigen::Transform<double, 3, Eigen::Affine> transform;
  transform =
      Eigen::AngleAxis<double>(x.head(3).norm(), x.head(3).normalized());

  // Reverse roll-pitch-yaw conversion, very useful piece of knowledge, keep it
  // with you all time!
  // const float pitch = -asin(transform(2,0));
  // const float roll = atan2(transform(2,1), transform(2,2));
  // const float yaw = atan2(transform(1,0) / cos(pitch), transform(0,0) /
  // cos(pitch)); std::cerr << "d angles" << x(0) - roll << ", " << x(1) - pitch
  // << "," << x(2) - yaw << std::endl;

  // Get transformation translation
  transform.translation() = x.segment(3, 3);

  // Convert to 4x4 matrix
  mOut = transform.matrix();

  if (mOut != mOut) {
    // Degenerate situation. This can happen when the source and reading clouds
    // are identical, and then b and x above are 0, and the rotation matrix
    // cannot be determined, it comes out full of NaNs. The correct rotation is
    // the identity.
    mOut.block(0, 0, 3, 3) = Eigen::Matrix4d::Identity(3, 3);
  }
}

}  // namespace

Matrix6d computeCovariance(
    const Eigen::Matrix4d& T,
    const Eigen::Map<Eigen::Matrix<float, 3, Eigen::Dynamic>>& targets,
    const Eigen::Map<Eigen::Matrix<float, 3, Eigen::Dynamic>>& references,
    const Eigen::Map<Eigen::Matrix<float, 3, Eigen::Dynamic>>& ref_normals,
    const std::vector<float>& ref_weights,
    const std::vector<std::pair<size_t, size_t>>& sample_inds) {
  // constant
  Eigen::Matrix<double, 4, 3> D;
  D << 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0;
  const auto TD = T * D;

  /// \todo currently assumes covariance of measured points
  Eigen::Matrix3d covz = Eigen::Matrix3d::Identity() * 1e-2;

  Matrix6d d2Jdx2_inv = Matrix6d::Zero();
  Matrix6d d2Jdzdx = Matrix6d::Zero();

  for (auto ind : sample_inds) {
    Eigen::Matrix4d W = Eigen::Matrix4d::Zero();
    W.block<3, 3>(0, 0) = (ref_weights[ind.second] *
                           (ref_normals.block<3, 1>(0, ind.second) *
                            ref_normals.block<3, 1>(0, ind.second).transpose()))
                              .cast<double>();
    // LOG(INFO) << "W\n " << W;

    const auto& p = targets.block<3, 1>(0, ind.first).cast<double>();
    const auto& y = references.block<3, 1>(0, ind.second).cast<double>();

    const auto pfs = lgmath::se3::point2fs(p, 1);

    // LOG(INFO) << "pfs\n " << pfs;

    // d2Jdx2
    d2Jdx2_inv += pfs.transpose() * W * pfs;

    // LOG(INFO) << "d2Jdx2_inv\n " << d2Jdx2_inv;

    // d2Jdydx
    const auto d2Jdydx = -pfs.transpose() * W * D;
    d2Jdzdx += d2Jdydx * covz * d2Jdydx.transpose();

    // LOG(INFO) << "d2Jdzdx\n " << d2Jdzdx;

    // d2Jdpdx
    Eigen::Matrix<double, 4, 1> e = Eigen::Matrix<double, 4, 1>::Zero();
    e.block<3, 1>(0, 0) = y - (T.block<3, 3>(0, 0) * p + T.block<3, 1>(0, 3));
    // LOG(INFO) << "error is \n " << e;
    const auto d2Jdpdx1T =
        -e.transpose() * W * lgmath::se3::point2fs(TD.block<3, 1>(0, 0), 0);
    const auto d2Jdpdx2T =
        -e.transpose() * W * lgmath::se3::point2fs(TD.block<3, 1>(0, 1), 0);
    const auto d2Jdpdx3T =
        -e.transpose() * W * lgmath::se3::point2fs(TD.block<3, 1>(0, 2), 0);

    Eigen::Matrix<double, 6, 3> d2Jdpdx;
    d2Jdpdx << d2Jdpdx1T.transpose(), d2Jdpdx2T.transpose(),
        d2Jdpdx3T.transpose();
    d2Jdzdx += d2Jdpdx * covz * d2Jdpdx.transpose();

    // LOG(INFO) << "d2Jdzdx\n " << d2Jdzdx;
  }

  const auto d2Jdx2 = d2Jdx2_inv.inverse();

  // Matrix6d cov;
  Matrix6d cov = d2Jdx2 * d2Jdzdx * d2Jdx2;

  // LOG(INFO) << "cov\n " << cov;

  return cov;
}

std::ostream& operator<<(std::ostream& os, const vtr::lidar::ICPParams& s) {
  os << "ICP Parameters" << endl
     << "  n_samples:" << s.n_samples << endl
     << "  max_pairing_dist:" << s.max_pairing_dist << endl
     << "  max_planar_dist:" << s.max_planar_dist << endl
     << "  max_iter:" << s.max_iter << endl
     << "  avg_steps:" << s.avg_steps << endl
     << "  rod_diff_thresh:" << s.rot_diff_thresh << endl
     << "  trans_diff_thresh:" << s.trans_diff_thresh << endl
     << "  motion_distortion:" << s.motion_distortion << endl
     << "  init_phi:" << s.init_phi << endl;
  return os;
}

void pointToMapICP(vector<PointXYZ>& tgt_pts, vector<float>& tgt_w,
                   PointMap& map, ICPParams& params, ICPResults& results) {
  /// Parameters
  size_t N = tgt_pts.size();
  float max_pair_d2 = params.max_pairing_dist * params.max_pairing_dist;
  float max_planar_d = params.max_planar_dist;
  size_t first_steps = params.avg_steps / 2 + 1;
  nanoflann::SearchParams search_params;  // kd-tree search parameters

  /// Start ICP loop
  // Matrix of original data (only shallow copy of ref clouds)
  Eigen::Map<Eigen::Matrix<float, 3, Eigen::Dynamic>> map_mat(
      (float*)map.cloud.pts.data(), 3, map.cloud.pts.size());
  Eigen::Map<Eigen::Matrix<float, 3, Eigen::Dynamic>> map_normals_mat(
      (float*)map.normals.data(), 3, map.normals.size());

  // Aligned points (Deep copy of targets)
  vector<PointXYZ> aligned(tgt_pts);

  // Matrix for original/aligned data (Shallow copy of parts of the points
  // vector)
  Eigen::Map<Eigen::Matrix<float, 3, Eigen::Dynamic>> targets_mat(
      (float*)tgt_pts.data(), 3, N);
  Eigen::Map<Eigen::Matrix<float, 3, Eigen::Dynamic>> aligned_mat(
      (float*)aligned.data(), 3, N);

  // Apply initial transformation
  Eigen::Matrix3f R_init =
      (params.init_transform.block(0, 0, 3, 3)).cast<float>();
  Eigen::Vector3f T_init =
      (params.init_transform.block(0, 3, 3, 1)).cast<float>();
  aligned_mat = (R_init * targets_mat).colwise() + T_init;
  results.transform = params.init_transform;

  // Random generator
  default_random_engine generator;
  discrete_distribution<int> distribution(tgt_w.begin(), tgt_w.end());

  // Init result containers
  Eigen::Matrix4d H_icp;
  results.all_rms.reserve(params.max_iter);
  results.all_plane_rms.reserve(params.max_iter);

  // Convergence variables
  float mean_dT = 0;
  float mean_dR = 0;
  size_t max_it = params.max_iter;
  bool stop_cond = false;

  vector<clock_t> timer(6);
  vector<string> clock_str;
  clock_str.push_back("Random_Sample ... ");
  clock_str.push_back("KNN_search ...... ");
  clock_str.push_back("Optimization .... ");
  clock_str.push_back("Regularization .. ");
  clock_str.push_back("Result .......... ");

  for (size_t step = 0; step < max_it; step++) {
    /// Points Association
    // Pick random queries (use unordered set to ensure uniqueness)
    vector<pair<size_t, size_t>> sample_inds;
    if (params.n_samples < N) {
      unordered_set<size_t> unique_inds;
      while (unique_inds.size() < params.n_samples)
        unique_inds.insert((size_t)distribution(generator));

      sample_inds = vector<pair<size_t, size_t>>(params.n_samples);
      size_t i = 0;
      for (const auto& ind : unique_inds) {
        sample_inds[i].first = ind;
        i++;
      }
    } else {
      sample_inds = vector<pair<size_t, size_t>>(N);
      for (size_t i = 0; i < N; i++) sample_inds[i].first = i;
    }

    timer[1] = std::clock();

    // Init neighbors container
    vector<float> nn_dists(sample_inds.size());

#pragma omp parallel for schedule(dynamic, 10) num_threads(params.num_threads)
    // Find nearest neigbors
    for (size_t i = 0; i < sample_inds.size(); i++) {
      nanoflann::KNNResultSet<float> resultSet(1);
      resultSet.init(&sample_inds[i].second, &nn_dists[i]);
      map.tree.findNeighbors(resultSet, (float*)&aligned[sample_inds[i].first],
                             search_params);
    }

    timer[2] = std::clock();

    /// Filtering based on distances metrics
    // Erase sample_inds if dists is too big
    vector<pair<size_t, size_t>> filtered_sample_inds;
    filtered_sample_inds.reserve(sample_inds.size());
    float rms2 = 0;
    float prms2 = 0;
    for (size_t i = 0; i < sample_inds.size(); i++) {
      if (nn_dists[i] < max_pair_d2) {
        // Check planar distance (only after a few steps for initial alignment)
        PointXYZ diff = (map.cloud.pts[sample_inds[i].second] -
                         aligned[sample_inds[i].first]);
        float planar_dist = abs(diff.dot(map.normals[sample_inds[i].second]));
        if (step < first_steps || planar_dist < max_planar_d) {
          // Keep samples
          filtered_sample_inds.push_back(sample_inds[i]);

          // Update pt2pt rms
          rms2 += nn_dists[i];

          // update pt2pl rms
          prms2 += planar_dist;
        }
      }
    }
    // Compute RMS
    results.all_rms.push_back(sqrt(rms2 / (float)filtered_sample_inds.size()));
    results.all_plane_rms.push_back(
        sqrt(prms2 / (float)filtered_sample_inds.size()));

    timer[3] = std::clock();

    /// Point to plane optimization
    // Minimize error
    minimizePointToPlaneError(aligned, map.cloud.pts, map.normals, map.scores,
                              filtered_sample_inds, H_icp);

    timer[4] = std::clock();

    /// Alignment (\todo with motion distorsion)
    // Apply the incremental transformation found by ICP
    results.transform = H_icp * results.transform;

    // Align targets
    Eigen::Matrix3f R_tot = (results.transform.block(0, 0, 3, 3)).cast<float>();
    Eigen::Vector3f T_tot = (results.transform.block(0, 3, 3, 1)).cast<float>();
    aligned_mat = (R_tot * targets_mat).colwise() + T_tot;

    timer[5] = std::clock();

    // Update all result matrices
    if (step == 0)
      results.all_transforms = Eigen::MatrixXd(results.transform);
    else {
      Eigen::MatrixXd temp(results.all_transforms.rows() + 4, 4);
      temp.topRows(results.all_transforms.rows()) = results.all_transforms;
      temp.bottomRows(4) = Eigen::MatrixXd(results.transform);
      results.all_transforms = temp;
    }

    timer[5] = std::clock();

    /// Check convergence

    // Update variations
    if (!stop_cond && step > 0) {
      float avg_tot = (float)params.avg_steps;
      if (step == 1) avg_tot = 1.0;

      if (step > 0) {
        // Get last transformation variations
        Eigen::Matrix3d R2 = results.all_transforms.block(
            results.all_transforms.rows() - 4, 0, 3, 3);
        Eigen::Matrix3d R1 = results.all_transforms.block(
            results.all_transforms.rows() - 8, 0, 3, 3);
        Eigen::Vector3d T2 = results.all_transforms.block(
            results.all_transforms.rows() - 4, 3, 3, 1);
        Eigen::Vector3d T1 = results.all_transforms.block(
            results.all_transforms.rows() - 8, 3, 3, 1);
        R1 = R2 * R1.transpose();
        T1 = T2 - T1;
        float dT_b = T1.norm();
        float dR_b = acos((R1.trace() - 1) / 2);
        mean_dT += (dT_b - mean_dT) / avg_tot;
        mean_dR += (dR_b - mean_dR) / avg_tot;
      }
    }

    // Stop condition
    if (!stop_cond && step > params.avg_steps) {
      if (mean_dT < params.trans_diff_thresh &&
          mean_dR < params.rot_diff_thresh) {
        // Do not stop right away. Have a last few averaging steps
        stop_cond = true;
        max_it = step + params.avg_steps;

        // For these last steps, reduce the max distance (half of wall
        // thickness)
        max_planar_d = 0.08;
      }
    }

    // Last call, average the last transformations
    if (step > max_it - 2) {
      Eigen::Matrix4d mH = Eigen::Matrix4d::Identity(4, 4);
      for (size_t s = 0; s < params.avg_steps; s++) {
        Eigen::Matrix4d H = results.all_transforms.block(
            results.all_transforms.rows() - 4 * (1 + s), 0, 4, 4);
        mH = interpolatePose(1.0 / (float)(s + 1), mH, H, 0);
      }
      results.transform = mH;
      results.all_transforms.block(results.all_transforms.rows() - 4, 0, 4, 4) =
          mH;
    }
  }

  /// Compute covariance
  // Apply the final transformation
  Eigen::Matrix3f R_tot = (results.transform.block(0, 0, 3, 3)).cast<float>();
  Eigen::Vector3f T_tot = (results.transform.block(0, 3, 3, 1)).cast<float>();
  aligned_mat = (R_tot * targets_mat).colwise() + T_tot;

  // Get all the matching points
  std::vector<pair<size_t, size_t>> sample_inds(N);
  for (size_t i = 0; i < N; i++) sample_inds[i].first = i;

  // Init neighbors container
  std::vector<float> nn_dists(sample_inds.size());

#pragma omp parallel for schedule(dynamic, 10) num_threads(params.num_threads)
  // Find nearest neigbors
  for (size_t i = 0; i < sample_inds.size(); i++) {
    nanoflann::KNNResultSet<float> resultSet(1);
    resultSet.init(&sample_inds[i].second, &nn_dists[i]);
    map.tree.findNeighbors(resultSet, (float*)&aligned[sample_inds[i].first],
                           search_params);
  }

  // Filtering based on distances metrics
  std::vector<std::pair<size_t, size_t>> filtered_sample_inds;
  filtered_sample_inds.reserve(sample_inds.size());
  for (size_t i = 0; i < sample_inds.size(); i++) {
    // Check point to point distance
    if (nn_dists[i] > max_pair_d2) continue;
    // Check planar distance
    PointXYZ diff =
        (map.cloud.pts[sample_inds[i].second] - aligned[sample_inds[i].first]);
    float planar_dist = abs(diff.dot(map.normals[sample_inds[i].second]));
    if (planar_dist > max_planar_d) continue;
    // Add the point
    filtered_sample_inds.push_back(sample_inds[i]);
  }

  // Compute covariance
  results.covariance =
      computeCovariance(results.transform, targets_mat, map_mat,
                        map_normals_mat, map.scores, filtered_sample_inds);

  /// Compute other statistics
  results.matched_points_ratio =
      (float)filtered_sample_inds.size() / (float)targets_mat.cols();
}

}  // namespace lidar
}  // namespace vtr
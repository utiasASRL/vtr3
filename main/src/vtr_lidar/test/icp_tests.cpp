#include <gtest/gtest.h>

#include <vtr_lidar/icp/lgicp.hpp>
#include <vtr_logging/logging_init.hpp>

using namespace vtr::lidar;

/** Test transition from A state to B state. */
TEST(ICP, covariance) {
  using PointCloudMat = Eigen::Matrix<float, 3, Eigen::Dynamic>;

  PointCloudMat references = Eigen::Matrix<float, 3, 9>::Zero();
  // clang-format off
  references.block<3, 9>(0,0) << 0, 1, 1, 0,-1,-1,-1, 0, 1,
                                 0, 0, 1, 1, 1, 0,-1,-1,-1,
                                 0, 0, 0, 0, 0, 0, 0, 0, 0;
  // clang-format on
  PointCloudMat ref_normals = Eigen::Matrix<float, 3, 9>::Zero();
  ref_normals.block<2, 9>(0, 0) = Eigen::Matrix<float, 2, 9>::Random() * 0.1;
  ref_normals.block<1, 9>(2, 0) = Eigen::Matrix<float, 1, 9>::Constant(1);

  Eigen::Matrix4f T_tgt_ref = Eigen::Matrix4f::Identity();
  T_tgt_ref(2, 3) = 10;

  PointCloudMat targets = (T_tgt_ref.block<3, 3>(0, 0) * references).colwise() +
                          T_tgt_ref.block<3, 1>(0, 3);

  const auto T_ref_tgt = T_tgt_ref.inverse().cast<double>();

  LOG(INFO) << "Reference points:\n" << references;
  LOG(INFO) << "Reference normals:\n" << ref_normals;
  LOG(INFO) << "Query points:\n" << targets;

  // sampled_inds and weights
  std::vector<float> weights(9);
  std::vector<std::pair<size_t, size_t>> sampled_inds(9);
  for (int i = 0; i < 9; i++) {
    sampled_inds[i].first = i;
    sampled_inds[i].second = i;
    weights[i] = 1;
  }

  // input from maps
  Eigen::Map<PointCloudMat> targets_map(targets.data(), 3, 9);
  Eigen::Map<PointCloudMat> references_map(references.data(), 3, 9);
  Eigen::Map<PointCloudMat> ref_normals_map(ref_normals.data(), 3, 9);

  const auto cov = computeCovariance(T_ref_tgt, targets_map, references_map,
                                     ref_normals_map, weights, sampled_inds);

  LOG(INFO) << "Resulting covariance is:\n" << cov;
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
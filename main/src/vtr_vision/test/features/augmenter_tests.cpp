#include <gtest/gtest.h>

#include <vtr_vision/features/augment/descriptor_augment.hpp>
#include <vtr_vision/types.hpp>

#include <vtr_logging/logging_init.hpp>

#if 0
using namespace vtr::vision;

TEST(Vision, augmenter) {

  // CALIBRATION //
  RigCalibration cal;
  cal.rectified = true;

  // params
  const double width = 512;
  const double height = 380;
  const double cx = width / 2.0;
  const double cy = height / 2.0;
  const double baseline = 100; // mm
  const double focal = 400; // pixels

  const float sigma_desc = 1.f;
  const float sigma_disp = 2.f;
  const float sigma_forward = 100.f; // mm
  const float sigma_size = 2.f; // log base for ratio

  // Extrinsic calibration
  cal.extrinsics.emplace_back(lgmath::se3::Transformation());
  cal.extrinsics.emplace_back(lgmath::se3::Transformation());
  cal.extrinsics.back().matrix()(0, 3) = -baseline; // 100 mm baseline

  // Intrinsic calibration
  CameraIntrinsic intrinsic;
  intrinsic << focal, 0, cx,
      0, focal, cy,
      0, 0, 1;
  cal.intrinsics.emplace_back(intrinsic);
  cal.intrinsics.emplace_back(intrinsic);

  // FEATURE/DESCRIPTOR //
  RigFeatures rig;
  rig.name = "front";
  rig.channels.resize(1);
  ChannelFeatures &grey = rig.channels.front();
  grey.name = "grey";
  grey.fully_matched = true;
  grey.cameras.resize(2);
  Features &left = grey.cameras[0];
  Features &right = grey.cameras[1];
  left.name = "left";
  right.name = "right";

  double fb = focal * baseline;
  std::vector<std::pair<cv::KeyPoint, float>> pts_disp = {
      {cv::KeyPoint(cx, cy, 8), 1},
      {cv::KeyPoint(cx, cy, 8), 1 + sigma_disp},
      {cv::KeyPoint(width, cy + 2, 16), width},
      {cv::KeyPoint(width, cy + 3, 32), fb / (fb / width + sigma_forward)}
  };
  const unsigned n_pts = pts_disp.size();
  const unsigned n_dims = 1;

  for (unsigned i = 0; i < grey.cameras.size(); ++i) {
    auto &cam = grey.cameras[i];

    cam.feat_type.impl = FeatureImpl::ASRL_GPU_SURF;
    cam.feat_type.dims = 1;
    cam.feat_type.bytes_per_desc = cam.feat_type.dims * sizeof(float);
    cam.feat_type.upright = true;

    for (auto &pt_d : pts_disp) {
      if (i) pt_d.first.pt.x -= pt_d.second;
      cam.keypoints.emplace_back(pt_d.first);
    }
  }

  left.descriptors = cv::Mat(n_pts, n_dims, CV_32F);
  for (unsigned j = 0; j < pts_disp.size(); ++j) {
    left.descriptors.row(j) = j + 1;
  }
  Eigen::Map<Eigen::Matrix<float,
                           Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> left_desc_eig
      (reinterpret_cast<float *>(left.descriptors.data), n_pts, n_dims);

  AugmentorEngine augmentor;
  augmentor.add(std::make_shared<AugmentorDescriptor>(sigma_desc));
  augmentor.add(std::make_shared<AugmentorDisparity>(cal, sigma_disp, sigma_forward));
  augmentor.add(std::make_shared<AugmentorSize>(sigma_size));
  cv::Mat aug_cv = augmentor.augment(grey);

  EXPECT_EQ(aug_cv.type(), CV_32F);
  EXPECT_EQ(aug_cv.cols, n_dims + 2);

  Eigen::Map<Eigen::Matrix<float,
                           Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> aug_eig
      (reinterpret_cast<float *>(aug_cv.data), aug_cv.rows, aug_cv.cols);
  LOG(INFO) << "The augmented descriptor:\n" << aug_eig;

  // Check the descriptor augment
  LOG(INFO) << left_desc_eig << "\n~=\n" << aug_eig.leftCols(n_dims);
  EXPECT_TRUE(aug_eig.leftCols(n_dims).isApprox(left_desc_eig / sigma_desc));

  // The disparity augment sigma test (should be ~ 1 sigma)
  EXPECT_EQ(n_pts, 4);
  auto disp_col = aug_eig.col(n_dims);
  EXPECT_NEAR(std::abs(disp_col(1) - disp_col(0)), 1, 0.2 * std::abs(disp_col(1) - disp_col(0)));
  EXPECT_NEAR(std::abs(disp_col(3) - disp_col(2)), 1, 0.2 * std::abs(disp_col(3) - disp_col(2)));

  // The size test
  EXPECT_EQ(aug_eig.col(n_dims + 1), (Eigen::VectorXf(n_pts) << 3, 3, 4, 5).finished());

}
#endif
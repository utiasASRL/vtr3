// Internal
#include <asrl/vision/features/augment/DescriptorAugment.hpp>
#include <asrl/vision/Types.hpp>

#include <asrl/common/logging.hpp>
#include <catch.hpp>

namespace av = asrl::vision;

SCENARIO("Test descriptor augmenter", "[features]") {

  // CALIBRATION //
  av::RigCalibration cal;
  cal.rectified = true;

  // params
  const double width = 512;
  const double height = 380;
  const double cx = width/2.0;
  const double cy = height/2.0;
  const double baseline = 100; // mm
  const double focal = 400; // pixels

  const float sigma_desc = 1.f;
  const float sigma_disp = 2.f;
  const float sigma_forward = 100.f; // mm
  const float sigma_size = 2.f; // log base for ratio

  // Extrinsic calibration
  cal.extrinsics.emplace_back(lgmath::se3::Transformation());
  cal.extrinsics.emplace_back(lgmath::se3::Transformation());
  cal.extrinsics.back().matrix()(0,3) = -baseline; // 100 mm baseline

  // Intrinsic calibration
  av::CameraIntrinsic intrinsic;
  intrinsic << focal, 0, cx,
      0, focal, cy,
      0, 0, 1;
  cal.intrinsics.emplace_back(intrinsic);
  cal.intrinsics.emplace_back(intrinsic);

  GIVEN("Some features") {

    // FEATURE/DESCRIPTOR //
    av::RigFeatures rig;
    rig.name = "front";
    rig.channels.resize(1);
    av::ChannelFeatures & grey = rig.channels.front();
    grey.name = "grey";
    grey.fully_matched = true;
    grey.cameras.resize(2);
    av::Features & left = grey.cameras[0];
    av::Features & right = grey.cameras[1];
    left.name = "left";
    right.name = "right";

    double fb = focal*baseline;
    std::vector<std::pair<cv::KeyPoint, float>> pts_disp = {
      {cv::KeyPoint(cx,cy,8), 1},
      {cv::KeyPoint(cx,cy,8), 1+sigma_disp},
      {cv::KeyPoint(width,cy+2,16), width},
      {cv::KeyPoint(width,cy+3,32), fb/(fb/width + sigma_forward)}
    };
    const unsigned n_pts = pts_disp.size();
    const unsigned n_dims = 1;

    for (unsigned i = 0; i < grey.cameras.size(); ++i) {
      auto & cam = grey.cameras[i];

      cam.feat_type.impl = av::FeatureImpl::ASRL_GPU_SURF;
      cam.feat_type.dims = 1;
      cam.feat_type.bytes_per_desc = cam.feat_type.dims * sizeof(float);
      cam.feat_type.upright = true;

      for (unsigned j = 0; j < pts_disp.size(); ++j) {
        auto & pt_d = pts_disp[j];
        if (i) pt_d.first.pt.x -= pt_d.second;
        cam.keypoints.emplace_back(pt_d.first);
      }
    }

    left.descriptors = cv::Mat(n_pts,n_dims,CV_32F);
    for (unsigned j = 0; j < pts_disp.size(); ++j) {
      left.descriptors.row(j) = j+1;
    }
    Eigen::Map<Eigen::Matrix<float,
        Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor>> left_desc_eig
        (reinterpret_cast<float*>(left.descriptors.data), n_pts, n_dims);


    WHEN("We augment the features") {
      av::AugmentorEngine augmentor;
      augmentor.add(std::make_shared<av::AugmentorDescriptor>(sigma_desc));
      augmentor.add(std::make_shared<av::AugmentorDisparity>(cal,sigma_disp,sigma_forward));
      augmentor.add(std::make_shared<av::AugmentorSize>(sigma_size));
      cv::Mat aug_cv = augmentor.augment(grey);

      THEN("They are augmented correctly") {
        REQUIRE(aug_cv.type() == CV_32F);
        REQUIRE(aug_cv.cols == n_dims+2);

        Eigen::Map<Eigen::Matrix<float,
            Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor>> aug_eig
            (reinterpret_cast<float*>(aug_cv.data), aug_cv.rows, aug_cv.cols);
        INFO( "The augmented descriptor:\n" << aug_eig);

        { // Check the descriptor augment
          INFO(left_desc_eig << "\n~=\n" << aug_eig.leftCols(n_dims));
          CHECK(aug_eig.leftCols(n_dims).isApprox(left_desc_eig/sigma_desc));
        }

        // The disparity augment sigma test (should be ~ 1 sigma
        REQUIRE(n_pts == 4);
        auto disp_col = aug_eig.col(n_dims);
        CHECK(std::abs(disp_col(1)-disp_col(0)) == Approx(1).epsilon(0.2).scale(0));
        CHECK(std::abs(disp_col(3)-disp_col(2)) == Approx(1).epsilon(0.2).scale(0));

        // The size test
        CHECK(aug_eig.col(n_dims+1) ==
              (Eigen::VectorXf(n_pts) << 3, 3, 4, 5).finished() );


      } // THEN
    } // WHEN
  } // GIVEN
} // SCENARIO

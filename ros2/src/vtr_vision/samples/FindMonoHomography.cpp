// ASRL
#include "asrl/vision/outliers.hpp"
#include "asrl/vision/sensors.hpp"

// Logging
#include "easylogging++.h"

// OpenCV
#include "opencv2/core/version.hpp"
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/opencv.hpp>

#if CV_MAJOR_VERSION == 2
#include <opencv2/nonfree/nonfree.hpp>
#elif CV_MAJOR_VERSION >= 3 // vtr3 change: opencv 4+
#include <opencv2/xfeatures2d/nonfree.hpp>
#endif

// STL
#include <vector>
#include <iostream>

INITIALIZE_EASYLOGGINGPP

int main()
{
  using namespace std;
  using namespace cv;
  namespace av = asrl::vision;

  // Parameters
  const float inlier_threshold = 2.5f; // Distance threshold to identify inliers
  const float nn_match_ratio = 0.8f;   // Nearest neighbor matching ratio

  // Read the images
  Mat img1 = imread("data/graf1.png", IMREAD_GRAYSCALE);
  Mat img2 = imread("data/graf3.png", IMREAD_GRAYSCALE);

  // Read the true homography
  Mat true_homography;
  FileStorage fs("data/H1to3p.xml", FileStorage::READ);
  fs.getFirstTopLevelNode() >> true_homography;

  // Detect keypoints
  av::Keypoints kpts1, kpts2;
#if CV_MAJOR_VERSION == 2
  SurfFeatureDetector detector(1000.);
  detector.detect(img1, kpts1);
  detector.detect(img2, kpts2);
#elif CV_MAJOR_VERSION >= 3 // vtr3 change: opencv 4+
  cv::xfeatures2d::SURF *surf = cv::xfeatures2d::SURF::create();
  surf->detect(img1, kpts1);
  surf->detect(img2, kpts2);
#endif

  // Compute descriptors
  Mat desc1, desc2;
#if CV_MAJOR_VERSION == 2
  SurfDescriptorExtractor extractor;
  extractor.compute( img1, kpts1, desc1 );
  extractor.compute( img2, kpts2, desc2 );
#elif CV_MAJOR_VERSION >= 3 // vtr3 change: opencv 4+
  surf->compute( img1, kpts1, desc1 );
  surf->compute( img2, kpts2, desc2 );
#endif

  // Matching
  FlannBasedMatcher matcher;
  vector<vector<DMatch> > nn_matches;
  matcher.knnMatch(desc1, desc2, nn_matches, 2);
  vector<KeyPoint> matched1, matched2, inliers1, inliers2;

  // Filter Matches for OpenCV
  vector<DMatch> good_matches;
  std::vector<Point2f> pts1, pts2;
  for(size_t i = 0; i < nn_matches.size(); i++) {
    DMatch first = nn_matches[i][0];
    float dist1 = nn_matches[i][0].distance;
    float dist2 = nn_matches[i][1].distance;
    if(dist1 < nn_match_ratio * dist2) {
      matched1.push_back(kpts1[first.queryIdx]);
      matched2.push_back(kpts2[first.trainIdx]);
      pts1.push_back(kpts1[first.queryIdx].pt);
      pts2.push_back(kpts2[first.trainIdx].pt);
    }
  }

  // Filter matches for us
  av::SimpleMatches close_matches;
  for(size_t i = 0; i < nn_matches.size(); i++) {
    DMatch first = nn_matches[i][0];
    float dist1 = nn_matches[i][0].distance;
    float dist2 = nn_matches[i][1].distance;
    if(dist1 < nn_match_ratio * dist2) {
      close_matches.push_back(av::SimpleMatch(first.queryIdx,first.trainIdx));
    }
  }

  // Find the homography with OpenCV's method
  Mat found_homography = findHomography(pts1, pts2, RANSAC); // vtr3 change: opencv 4+

  // Find the homography and inliers with our method
  av::BasicSampler::Ptr sampler = std::make_shared<av::BasicSampler>();
  // Set up the model
  av::MonoRotationModel::Ptr model = std::make_shared<av::MonoRotationModel>();
  model->setPoints(&kpts1, &kpts2);
  // Set up ransac
  av::VanillaRansac<av::MonoRotationModel::SolutionType> ransac;
  ransac.setCallback(model);
  // Run the model
  Eigen::Matrix3d solution;
  av::SimpleMatches inliers;
  ransac.run(close_matches, &solution, &inliers);

  for(unsigned i = 0; i < matched1.size(); i++) {
    Mat col = Mat::ones(3, 1, CV_64F);
    col.at<double>(0) = matched1[i].pt.x;
    col.at<double>(1) = matched1[i].pt.y;
    col = true_homography * col;
    col /= col.at<double>(2);
    double dist = sqrt( pow(col.at<double>(0) - matched2[i].pt.x, 2) +
                        pow(col.at<double>(1) - matched2[i].pt.y, 2));
    if(dist < inlier_threshold) {
      int new_i = static_cast<int>(inliers1.size());
      inliers1.push_back(matched1[i]);
      inliers2.push_back(matched2[i]);
      good_matches.push_back(DMatch(new_i, new_i, 0));
    }
  }

  Mat res;
  drawMatches(img1, inliers1, img2, inliers2, good_matches, res);
  imwrite("data/res.png", res);
  double inlier_ratio = inliers1.size() * 1.0 / matched1.size();
  cout << "Matching Results" << endl;
  cout << "*******************************" << endl;
  cout << "# Keypoints 1:                        \t" << kpts1.size() << endl;
  cout << "# Keypoints 2:                        \t" << kpts2.size() << endl;
  cout << "# Matches:                            \t" << matched1.size() << endl;
  cout << "# Inliers:                            \t" << inliers1.size() << endl;
  cout << "# Inliers Ratio:                      \t" << inlier_ratio << endl;
  cout << "# Homography:\n" << true_homography << "\nFound:\n" << found_homography;
  cout << endl;

  return 0;
}

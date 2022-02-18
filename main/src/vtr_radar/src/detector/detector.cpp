#include "vtr_radar/detector/detector.hpp"

void BASD::compute(const cv::Mat& cartesian,
                   const std::vector<cv::KeyPoint> &keypoints,
                   cv::Mat &descriptors) {
  const int max_range = nbins_;
  const float max_range_sq = max_range * max_range;
  const int dim = std::ceil(nbins_ * (nbins_ + 1) / 8.0);  // dimension of descriptor in bytes
  const int cols = cartesian.cols;
  const int rows = cartesian.rows;
  descriptors = cv::Mat::zeros(keypoints.size(), dim, CV_8UC1);
  for (size_t kp_idx = 0; kp_idx < keypoints.size(); ++kp_idx) {
    const auto &kp = keypoints[kp_idx];
    std::vector<std::vector<float>> bins(nbins_);
    const int minrow = std::max(int(std::ceil(kp.pt.y - max_range)), 0);
    const int maxrow = std::min(int(std::floor(kp.pt.y + max_range)), rows);
    const int mincol = std::max(int(std::ceil(kp.pt.x - max_range)), 0);
    const int maxcol = std::min(int(std::floor(kp.pt.x + max_range)), cols);
    for (int i = minrow; i < maxrow; ++i) {
      for (int j = mincol; j < maxcol; ++j) {
        float r = pow(i - kp.pt.y, 2) + pow(j - kp.pt.x, 2);
        if (r > max_range_sq)
          continue;
        r = sqrt(r);
        int bin = std::floor(r / bin_size_);
        bins[bin].push_back(cartesian.at<float>(i, j));
      }
    }
    // compute statistics for each bin
    std::vector<float> means(nbins_, 0);
    std::vector<float> stds(nbins_, 0);
    for (int i = 0; i < nbins_; ++i) {
      float mean = 0;
      if (bins[i].size() == 0)
        continue;
      for (size_t j = 0; j < bins[i].size(); ++j) {
        mean += bins[i][j];
      }
      mean /= bins[i].size();
      means[i] = mean;
      float std = 0;
      for (size_t j = 0; j < bins[i].size(); ++j) {
        std += pow(bins[i][j] - mean, 2);
      }
      std /= bins[i].size();
      std = sqrt(std);
      stds[i] = std;
    }
    // compare statistics between rings to create binary descriptor
    int k = 0;
    for (int i = 0; i < nbins_; ++i) {
      for (int j = 0; j < nbins_; ++j) {
        if (i == j)
          continue;
        if (means[i] > means[j]) {
          int byte = std::floor(k / 8.0);
          int bit = k % 8;
          descriptors.at<uchar>(kp_idx, byte) |= (1 << bit);
        }
        k++;
        if (stds[i] > stds[j]) {
          int byte = std::floor(k / 8.0);
          int bit = k % 8;
          descriptors.at<uchar>(kp_idx, byte) |= (1 << bit);
        }
        k++;
      }
    }
  }
}
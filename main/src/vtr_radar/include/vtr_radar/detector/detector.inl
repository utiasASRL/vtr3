// Copyright 2021, Autonomous Space Robotics Lab (ASRL)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * \file detector.inl
 * \author Keenan Burnett, Autonomous Space Robotics Lab (ASRL)
 * \brief Keypoint extraction methods for Navtech radar
 */
#pragma once

#include "vtr_radar/detector/detector.hpp"

#include <opencv2/plot.hpp>
#include <opencv2/core/cvstd_wrapper.hpp>

// #include "opencv2/xfeatures2d.hpp"
#include "opencv2/features2d.hpp"
// #include "opencv2/nonfree/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
// #include "opencv2/nonfree/nonfree.hpp"
// #include "vtr_radar/modules/preprocessing/conversions/navtech_extraction_module.hpp"

// #include <utility>
#include <functional>
// #include <cstddef>
#include <set>


namespace vtr {
namespace radar {

namespace {
bool sort_desc_by_first(const std::pair<float, int> &a,
                        const std::pair<float, int> &b) {
  return (a.first > b.first);
}

bool sort_asc_by_second(const std::pair<int, float> &a,
                        const std::pair<int, float> &b) {
  return (a.second < b.second);
}

}  // namespace

template <class PointT>
void KStrongest<PointT>::run(const cv::Mat &raw_scan, const float &res,
                             const std::vector<int64_t> &azimuth_times,
                             const std::vector<double> &azimuth_angles,
                             pcl::PointCloud<PointT> &pointcloud) {
  pointcloud.clear();
  const int rows = raw_scan.rows;
  const int cols = raw_scan.cols;
  auto mincol = minr_ / res;
  // auto mincol = minr_ / res + w2 + guard_ + 1;
  if (mincol > cols || mincol < 0) mincol = 0;
  auto maxcol = maxr_ / res;
  // auto maxcol = maxr_ / res - w2 - guard_;
  if (maxcol > cols || maxcol < 0) maxcol = cols;
  // const auto N = maxcol - mincol;

  // float conversion_factor = 255.0/20.0*std::log(10.0);
  // cv::Mat raw_scan_watts_sqrd = raw_scan * conversion_factor;
  // cv::exp(raw_scan_watts_sqrd, raw_scan_watts_sqrd);
  // // apply square law detector to test cell
  // cv::pow(raw_scan_watts_sqrd, 2, raw_scan_watts_sqrd);

  // double threshold3_watts = std::pow(10,255.0*threshold3_/20);
  // double static_threshold_squared = threshold3_watts*threshold3_watts; 

  // cv::Mat L = cv::Mat::zeros(rows,cols,CV_32F);


  // #pragma omp parallel for
  for (int i = 0; i < rows; ++i) {
    std::vector<std::pair<float, int>> intens;
    // intens.reserve(static_cast<std::size_t>(std::ceil(N / 2)));
    
    // double mean = 0;
    // for (int j = mincol; j < maxcol; ++j) {
    //   mean += raw_scan.at<float>(i, j);
    // }
    // mean /= N;
    // const double thres = mean * threshold2_ + static_threshold_squared;
    const float thres = threshold3_;
    for (int j = mincol; j < maxcol; ++j) {
      if (raw_scan.at<float>(i, j) >= thres){
        intens.push_back(std::make_pair(raw_scan.at<float>(i, j), j));
      }
    }

    int thresholded_point_count = intens.size();
    if(thresholded_point_count > 0){
      // sort intensities in descending order
      // if(thresholded_point_count<30){
      //   std::cout<<"BEFORE"<<std::endl;
      //   for(int k=0;k<thresholded_point_count;k++){
      //     std::cout<<intens[k].first;
      //   }
      // }
      std::sort(intens.begin(), intens.end(), sort_desc_by_first);
      // if(thresholded_point_count<30){
      //   std::cout<<"AFTER"<<std::endl;
        
      //   for(int k=0;k<thresholded_point_count;k++){
      //     std::cout<<intens[k].first;
      //   }
      // }
      const double azimuth = azimuth_angles[i];
      const int64_t time = azimuth_times[i];
      pcl::PointCloud<PointT> polar_time;
      for (int j = 0; j < kstrong_; ++j) {
        if (j >= thresholded_point_count){break;}
        // if (intens[j].first < thres){break;}
        // L.at<float>(i, intens[j].second) = 245.0;

        PointT p;
        p.rho = static_cast<float>(intens[j].second) * res + static_cast<float>(range_offset_);
        p.phi = azimuth;
        p.theta = 0;
        p.timestamp = time;
        polar_time.push_back(p);
      }
      // #pragma omp critical
      {
        pointcloud.insert(pointcloud.end(), polar_time.begin(), polar_time.end());
      }
    }
  }

  // double cart_resolution_= 0.2384;
  // double radar_resolution_ = 0.0596;
  // int cart_pixel_width = (2 * maxr_) / cart_resolution_;
  // cv::Mat L_cart;
  // cv::Mat L_cart_colour;
  // radar_polar_to_cartesian(L, azimuth_angles, L_cart, radar_resolution_, cart_resolution_, cart_pixel_width, true, CV_32F);
  // cv::cvtColor(L_cart, L_cart_colour, cv::COLOR_GRAY2BGR);
  // // cv::imwrite("/home/epk/radar_topometric_localization/data/epk_test_plots/TEST_L.jpg", L_cart_colour);
  // cv::Mat raw_scan_convert, raw_colour_convert;
  // radar_polar_to_cartesian((raw_scan*255.0/2.0), azimuth_angles, raw_scan_convert, radar_resolution_, cart_resolution_, cart_pixel_width, true, CV_32F);
  // cv::cvtColor(raw_scan_convert, raw_colour_convert, cv::COLOR_GRAY2BGR);
  // raw_colour_convert = raw_colour_convert + L_cart_colour;
  // cv::imwrite("/home/epk/radar_topometric_localization/data/epk_test_plots/kstrong/kstrong.jpg", raw_colour_convert);

}

template <class PointT>
void Cen2018<PointT>::run(const cv::Mat &raw_scan, const float &res,
                          const std::vector<int64_t> &azimuth_times,
                          const std::vector<double> &azimuth_angles,
                          pcl::PointCloud<PointT> &pointcloud) {
  pointcloud.clear();
  const int rows = raw_scan.rows;
  const int cols = raw_scan.cols;
  auto mincol = minr_ / res;
  if (mincol > cols || mincol < 0) mincol = 0;
  auto maxcol = maxr_ / res;
  if (maxcol > cols || maxcol < 0) maxcol = cols;
  const auto N = maxcol - mincol;

  bool graphing = false;
  // float conversion_factor = 255.0/20.0*std::log(10.0);
  // cv::Mat raw_scan_watts_sqrd = raw_scan * conversion_factor;
  // cv::exp(raw_scan_watts_sqrd, raw_scan_watts_sqrd);
  // // apply square law detector to test cell
  // cv::pow(raw_scan_watts_sqrd, 2, raw_scan_watts_sqrd);

  std::vector<float> sigma_q(rows, 0);
  // TODO: try implementing an efficient median filter
  // Estimate the bias and subtract it from the signal

  // cv::Mat raw_scan_alt = raw_scan_watts_sqrd.clone();
  // raw_scan_alt.convertTo(raw_scan_alt, CV_32F);
  // Convert raw signal to dB

  cv::Mat raw_scan_alt = raw_scan.clone() *255/2;
  // cv::Mat raw_scan_alt;
  // raw_scan.convertTo(raw_scan_alt, CV_64F);  // Convert raw_scan (float) to double
  // raw_scan_alt = raw_scan_alt * 255.0 / 2.0;

  cv::Mat q = raw_scan_alt.clone();
  for (int i = 0; i < rows; ++i) {
    double mean = 0;
    for (int j = mincol; j < maxcol; ++j) {
      mean += raw_scan_alt.at<float>(i, j);
    }
    mean /= N;
    for (int j = mincol; j < maxcol; ++j) {
      q.at<float>(i, j) = raw_scan_alt.at<float>(i, j) - mean;
    }
  }

  // // MEDIAN FILTER
  // int filterSize = 50;
  // int halfSize = filterSize / 2;
  // // cv::Mat median_filter = raw_scan_alt.clone();
  // for (int i = 0; i < rows; i++) {  
  //   for (int j = halfSize; j < cols - halfSize; j++) {
  //     std::vector<float> values;
  //     for (int k = -halfSize; k <= halfSize; k++) {
  //         values.push_back(raw_scan_alt.at<float>(i, j+k));
  //     }
  //     std::sort(values.begin(), values.end());
  //     // median_filter.at<float>(i, j) = values[halfSize];
  //     q.at<float>(i, j) = values[halfSize];
  //   }
  // }

  // // Create 1D Gaussian Filter
  // // TODO: binomial filter may be more efficient
  int fsize = sigma_ * 2 * 3;
  if (fsize % 2 == 0) fsize += 1;
  // const int mu = fsize / 2;
  // const float sig_sqr = sigma_ * sigma_;
  // cv::Mat filter = cv::Mat::zeros(1, fsize, CV_32F);
  // float s = 0;
  // for (int i = 0; i < fsize; ++i) {
  //   filter.at<float>(0, i) = exp(-0.5 * (i - mu) * (i - mu) / sig_sqr);
  //   s += filter.at<float>(0, i);
  // }
  // filter /= s;
  cv::Mat p;
  // cv::filter2D(q, p, -1, filter, cv::Point(-1, -1), 0, cv::BORDER_REFLECT101);

  cv::GaussianBlur(q,p,cv::Size(fsize,1),sigma_,sigma_);
  
  // Estimate variance of noise at each azimuth
  for (int i = 0; i < rows; ++i) {
    int nonzero = 0;
    for (int j = mincol; j < maxcol; ++j) {
      double n = q.at<float>(i, j);
      if (n < 0) {
        sigma_q[i] += 2 * (n * n);
        nonzero++;
      }
    }
    if (nonzero)
      sigma_q[i] = sqrt(sigma_q[i] / nonzero);
    else
      sigma_q[i] = 0.034;
  }

  // cv::Mat L = cv::Mat::zeros(rows,cols,CV_32F);


  // Extract peak centers from each azimuth
  // #pragma omp parallel for
  for (int i = 0; i < rows; ++i) {
    pcl::PointCloud<PointT> polar_time;
    float peak_points = 0;
    int num_peak_points = 0;
    const float thres = zq_ * sigma_q[i];
    const double azimuth = azimuth_angles[i];
    const int64_t time = azimuth_times[i];
    
    // if(graphing){
    //   cv::Mat display;
    //   cv::Mat s = raw_scan_alt.row(i).clone();
    //   // for (int j = 0; j < cols; ++j) {s.at<float>(j) = 10.0*std::log10(s.at<float>(j));}
    //   s.convertTo(s, CV_64F);


    //   cv::Ptr<cv::plot::Plot2d> test_plot = cv::plot::Plot2d::create(s);


    //   cv::Mat q_graph = q.row(i).clone();
    //   // for (int j = 0; j < cols; ++j) {
    //   //   if(q_graph.at<float>(j) > 0.0){
    //   //     q_graph.at<float>(j) = 10.0*std::log10(q_graph.at<float>(j));
    //   //   }else{
    //   //     // q_graph.at<float>(j) = 0.0;
    //   //     q_graph.at<float>(j) = -10.0*std::log10(-q_graph.at<float>(j));
    //   //   }
    //   // }
    //   q_graph.convertTo(q_graph, CV_64F);

    //   test_plot = cv::plot::Plot2d::create(q_graph);
    //   test_plot->setPlotSize(800, 600);
    //   test_plot->setMaxX(cols);
    //   test_plot->setMinX(0);
    //   test_plot->setMaxY(130);
    //   test_plot->setMinY(-50);
    //   test_plot->setInvertOrientation(true);
    //   test_plot->setGridLinesNumber(8);
    //   test_plot->render(display);
    //   cv::imwrite("/home/epk/radar_topometric_localization/data/epk_test_plots/cen2018/q_dB_mean.jpg", display);

    //   cv::Mat original_graph = raw_scan_alt.row(i).clone();
    //   // for (int j = 0; j < cols; ++j) {
    //   //   original_graph.at<float>(j) = 10.0*std::log10(original_graph.at<float>(j));
    //   // }
    //   original_graph.convertTo(original_graph, CV_64F);

    //   test_plot = cv::plot::Plot2d::create(original_graph);
    //   test_plot->setPlotSize(800, 600);
    //   test_plot->setMaxX(cols);
    //   test_plot->setMinX(0);
    //   test_plot->setMaxY(130);
    //   test_plot->setMinY(-50);
    //   test_plot->setInvertOrientation(true);
    //   test_plot->setGridLinesNumber(8);
    //   test_plot->render(display);
    //   cv::imwrite("/home/epk/radar_topometric_localization/data/epk_test_plots/cen2018/original_graph.jpg", display);


    //   cv::Mat p_graph = p.row(i).clone();
    //   // for (int j = 0; j < cols; ++j) {p_graph.at<float>(j) = 10.0*std::log10(p_graph.at<float>(j));}
    //   p_graph.convertTo(p_graph, CV_64F);

    //   test_plot = cv::plot::Plot2d::create(p_graph);
    //   test_plot->setPlotSize(800, 600);
    //   test_plot->setMaxX(cols);
    //   test_plot->setMinX(0);
    //   test_plot->setMaxY(130);
    //   test_plot->setMinY(-20);
    //   test_plot->setInvertOrientation(true);
    //   test_plot->setGridLinesNumber(8);
    //   test_plot->render(display);
    //   cv::imwrite("/home/epk/radar_topometric_localization/data/epk_test_plots/cen2018/p_dB_gauss.jpg", display);

    //   cv::Mat overlay = raw_scan.row(i).clone()*0.0;
    //   cv::Mat final_points = raw_scan.row(i).clone()*0.0;
    // }


    for (int j = mincol; j < maxcol; ++j) {
      const float nqp = exp(
          -0.5 * pow((q.at<float>(i, j) - p.at<float>(i, j)) / sigma_q[i], 2));
      const float npp = exp(-0.5 * pow(p.at<float>(i, j) / sigma_q[i], 2));
      const float b = nqp - npp;
      const float y = q.at<float>(i, j) * (1 - nqp) + p.at<float>(i, j) * b;
      // if(graphing){
      //   overlay.at<float>(j) = y;
      //   // overlay.at<float>(j) = 10.0*std::log10(y);
      // }
      
      if (y > thres) {
        peak_points += j;
        num_peak_points += 1;
      } else if (num_peak_points > 0) {
        // if(graphing){final_points.at<float>(j) = y;}
        // L.at<float>(i, int(res*peak_points / num_peak_points +range_offset_)) = 245.0;
        // L.at<float>(i, int(peak_points / num_peak_points)) = 245.0;

        PointT p;
        p.rho = res * peak_points / num_peak_points + static_cast<float>(range_offset_);
        p.phi = azimuth;
        p.theta = 0;
        p.timestamp = time;
        polar_time.push_back(p);
        peak_points = 0;
        num_peak_points = 0;
      }
    }

    
    // if(graphing){
    //   // for (int j = 0; j < cols; ++j) {
    //   //   if(overlay.at<float>(j)>0){ overlay.at<float>(j) = 10.0*std::log10(overlay.at<float>(j));}
    //   // }
    //   overlay.convertTo(overlay, CV_64F);
    //   test_plot = cv::plot::Plot2d::create(overlay);
    //   test_plot->setPlotSize(800, 600);
    //   test_plot->setMaxX(cols);
    //   test_plot->setMinX(0);
    //   test_plot->setMaxY(130);
    //   test_plot->setMinY(-1);
    //   test_plot->setInvertOrientation(true);
    //   test_plot->setGridLinesNumber(8);
    //   test_plot->render(display);
    //   cv::imwrite("/home/epk/radar_topometric_localization/data/epk_test_plots/cen2018/cen2018_y_watts_db.jpg", display); 
    // }
    

    // if (num_peak_points > 0) {

    //   // if(graphing){final_points.at<float>(res * peak_points / num_peak_points) = overlay.at<float>(res * peak_points / num_peak_points);}

    //   const double azimuth = azimuth_angles[rows - 1];
    //   const int64_t time = azimuth_times[rows - 1];
    //   PointT p;
    //   p.rho = res * peak_points / num_peak_points + static_cast<float>(range_offset_);
    //   p.phi = azimuth;
    //   p.theta = 0;
    //   p.timestamp = time;
    //   polar_time.push_back(p);
    // }

    // if(graphing){
    // // for (int j = 0; j < cols; ++j) {
    // //   if(final_points.at<float>(j)>0){ final_points.at<float>(j) = 10.0*std::log10(final_points.at<float>(j));}
    // // }
    // final_points.convertTo(final_points, CV_64F);
    // test_plot = cv::plot::Plot2d::create(final_points);
    // test_plot->setPlotSize(800, 600);
    // test_plot->setMaxX(cols);
    // test_plot->setMinX(0);
    // test_plot->setMaxY(130);
    // test_plot->setMinY(-1);
    // test_plot->setInvertOrientation(true);
    // test_plot->setGridLinesNumber(8);
    // test_plot->render(display);
    // cv::imwrite("/home/epk/radar_topometric_localization/data/epk_test_plots/cen2018/cen208_final_db.jpg", display); 
    // }

    // #pragma omp critical
    {
      pointcloud.insert(pointcloud.end(), polar_time.begin(), polar_time.end());
    }
  }

  // double cart_resolution_= 0.2384;
  // double radar_resolution_ = 0.0596;
  // int cart_pixel_width = (2 * maxr_) / cart_resolution_;
  // cv::Mat L_cart;
  // cv::Mat L_cart_colour;
  // radar_polar_to_cartesian(L, azimuth_angles, L_cart, radar_resolution_, cart_resolution_, cart_pixel_width, true, CV_32F);
  // cv::cvtColor(L_cart, L_cart_colour, cv::COLOR_GRAY2BGR);
  // // cv::imwrite("/home/epk/radar_topometric_localization/data/epk_test_plots/TEST_L.jpg", L_cart_colour);
  // cv::Mat raw_scan_convert, raw_colour_convert;
  // radar_polar_to_cartesian((raw_scan*255.0/2.0), azimuth_angles, raw_scan_convert, radar_resolution_, cart_resolution_, cart_pixel_width, true, CV_32F);
  // cv::cvtColor(raw_scan_convert, raw_colour_convert, cv::COLOR_GRAY2BGR);
  // raw_colour_convert = raw_colour_convert + L_cart_colour;
  // cv::imwrite("/home/epk/radar_topometric_localization/data/epk_test_plots/cen2018/cen2018_overlay.jpg", raw_colour_convert);
  // sleep(10);
}

template <class PointT>
void CACFAR<PointT>::run(const cv::Mat &raw_scan, const float &res,
                         const std::vector<int64_t> &azimuth_times,
                         const std::vector<double> &azimuth_angles,
                         pcl::PointCloud<PointT> &pointcloud) {
  pointcloud.clear();
  const int rows = raw_scan.rows;
  const int cols = raw_scan.cols;
  if (width_ % 2 == 0) width_ += 1;
  const int w2 = std::floor(width_ / 2);
  const int window = width_ + guard_ * 2;
  auto mincol = minr_ / res + w2 + guard_ + 1;
  if (mincol > cols || mincol < 0) mincol = 0;
  auto maxcol = maxr_ / res - w2 - guard_;
  if (maxcol > cols || maxcol < 0) maxcol = cols;
  const int N = maxcol - mincol;

  // #pragma omp parallel for
  for (int i = 0; i < rows; ++i) {
    const double azimuth = azimuth_angles[i];
    const int64_t time = azimuth_times[i];
    pcl::PointCloud<PointT> polar_time;
    double mean = 0;
    for (int j = mincol; j < maxcol; ++j) {
      // mean += raw_scan.at<float>(i, j);
      // Fix convert 8-bit dB half steps to power intensity
      mean += static_cast<double>(pow(10,raw_scan.at<float>(i, j)/20));
    }
    mean /= N;

    for (int j = mincol; j < maxcol; ++j) {
      double stat = 0;  // (statistic) estimate of clutter power
      double left = 0;
      double right = 0;
      // for (int k = -w2 - guard_; k <= w2 + guard_; ++k) {
      //   if (k < -guard_ || k > guard_)
      //     stat += raw_scan.at<float>(i, j + k);
      // }
      for (int k = -w2 - guard_; k < -guard_; ++k) {
        // left += raw_scan.at<float>(i, j + k);
        // Fix convert 8-bit dB half steps to power intensity
        left += static_cast<double>(pow(10,raw_scan.at<float>(i, j + k)/20));
      }
      for (int k = guard_ + 1; k <= w2 + guard_; ++k) {
        // right += raw_scan.at<float>(i, j + k);
        // Fix convert 8-bit dB half steps to power intensity
        right += static_cast<double>(pow(10,raw_scan.at<float>(i, j + k)/20));
      }
      stat = std::max(left, right);
      const float thres =
          threshold_ * stat / (window / 2) + threshold2_ * mean + threshold3_;
      if (raw_scan.at<float>(i, j) > thres) {
        PointT p;
        p.rho = j * res + static_cast<float>(range_offset_);
        p.phi = azimuth;
        p.theta = 0;
        p.timestamp = time;
        polar_time.push_back(p);
      }
    }
    // #pragma omp critical
    {
      pointcloud.insert(pointcloud.end(), polar_time.begin(), polar_time.end());
    }
  }
}

template <class PointT>
void OSCFAR<PointT>::run(const cv::Mat &raw_scan, const float &res,
                         const std::vector<int64_t> &azimuth_times,
                         const std::vector<double> &azimuth_angles,
                         pcl::PointCloud<PointT> &pointcloud) {
  pointcloud.clear();
  const int rows = raw_scan.rows;
  const int cols = raw_scan.cols;
  if (width_ % 2 == 0) width_ += 1;
  const int w2 = std::floor(width_ / 2);
  auto mincol = minr_ / res + w2 + 1;
  if (mincol > cols || mincol < 0) mincol = 0;
  auto maxcol = maxr_ / res - w2;
  if (maxcol > cols || maxcol < 0) maxcol = cols;
  const int N = maxcol - mincol;

  float conversion_factor = 255.0/20.0*std::log(10.0);
  cv::Mat raw_scan_watts_sqrd = raw_scan * conversion_factor;
  cv::exp(raw_scan_watts_sqrd, raw_scan_watts_sqrd);
  // apply square law detector to test cell
  cv::pow(raw_scan_watts_sqrd, 2, raw_scan_watts_sqrd);

  // cv::Mat L = cv::Mat::zeros(rows,cols,CV_32F);

  // #pragma omp parallel for
  for (int i = 0; i < rows; ++i) {
    const double azimuth = azimuth_angles[i];
    const int64_t time = azimuth_times[i];
    pcl::PointCloud<PointT> polar_time;
    // double mean = 0;
    // for (int j = mincol; j < maxcol; ++j) {
    //   mean += raw_scan_watts_sqrd.at<float>(i, j);
    // }
    // mean /= N;

    std::vector<std::pair<int, float>> window;
    window.reserve(width_ - 1);
    int j = mincol - 1;
    for (int k = -w2; k < 0; ++k) {
      window.emplace_back(j + k, raw_scan_watts_sqrd.at<float>(i, j + k));
    }
    for (int k = 1; k <= w2; ++k) {
      window.emplace_back(j + k, raw_scan_watts_sqrd.at<float>(i, j + k));
    }
    std::sort(window.begin(), window.end(), sort_asc_by_second);

    float peak_points = 0;
    int num_peak_points = 0;

    for (j = mincol; j < maxcol; ++j) {
      // remove cell under test and left-most cell
      window.erase(std::remove_if(window.begin(), window.end(),
                                  [&](const std::pair<float, int> &p) -> bool {
                                    return p.first == j ||
                                           p.first == j - w2 - 1;
                                  }),
                   window.end());
      // insert prev CUT and right-most cell
      auto prevcut = std::make_pair(j - 1, raw_scan_watts_sqrd.at<float>(i, j - 1));
      auto it = std::lower_bound(window.begin(), window.end(), prevcut,
                                 sort_asc_by_second);
      window.insert(it, prevcut);
      auto newentry = std::make_pair(j + w2, raw_scan_watts_sqrd.at<float>(i, j + w2));
      it = std::lower_bound(window.begin(), window.end(), newentry,
                            sort_asc_by_second);
      window.insert(it, newentry);

      // DEBUG: check that the window is indeed sorted:
      // assert(window.size() == width_ - 1);
      // for (size_t k = 1; k < window.size(); ++k) {
      //   if (window[k - 1].second > window[k].second) {
      //     throw std::runtime_error("window not sorted.");
      //   }
      // }
      // (statistic) estimate of clutter power
      double stat = window[kstat_].second;
      // const float thres = threshold_ * stat + threshold2_ * mean + threshold3_;
      const float thres = threshold_ * stat;

      // std::cout<<"thresh: "<<threshold_<< " stat: "<<stat<<std::endl;

      if (raw_scan_watts_sqrd.at<float>(i, j) > thres) {
        peak_points += j;
        num_peak_points += 1;
      } else if (num_peak_points > 0) {
        // L.at<float>(i, int(res*peak_points / num_peak_points +range_offset_)) = 245.0;
        
        PointT p;
        p.rho = res * peak_points / num_peak_points + static_cast<float>(range_offset_);
        p.phi = azimuth;
        p.theta = 0;
        p.timestamp = time;
        polar_time.push_back(p);
        peak_points = 0;
        num_peak_points = 0;
      }
    }
    // cv::Mat s = raw_scan.row(i).clone();
    // s.convertTo(s, CV_64F);
    // // Convert raw signal to dB
    // s = s *255;
    // cv::Mat display;
    // cv::Ptr<cv::plot::Plot2d> test_plot = cv::plot::Plot2d::create(s);
    // test_plot->setPlotSize(800, 600);
    // test_plot->setMaxX(cols);
    // test_plot->setMinX(0);
    // test_plot->setMaxY(255);
    // test_plot->setMinY(-1);
    // test_plot->setInvertOrientation(true);
    // test_plot->setGridLinesNumber(8);
    // test_plot->render(display);
    // cv::imwrite("/home/epk/radar_topometric_localization/data/epk_test_plots/os_cfar/oscfar_squared_db.jpg", display);

    // overlay.convertTo(overlay, CV_64F);
    // test_plot = cv::plot::Plot2d::create(overlay);
    // test_plot->setPlotSize(800, 600);
    // test_plot->setMaxX(cols);
    // test_plot->setMinX(0);
    // test_plot->setMaxY(255);
    // test_plot->setMinY(-1);
    // test_plot->setInvertOrientation(true);
    // test_plot->setGridLinesNumber(8);
    // test_plot->render(display);
    // cv::imwrite("/home/epk/radar_topometric_localization/data/epk_test_plots/os_cfar/oscfar_thresh_db.jpg", display); 


    // #pragma omp critical
    {
      pointcloud.insert(pointcloud.end(), polar_time.begin(), polar_time.end());
    }
  }
  
  // double cart_resolution_= 0.2384;
  // double radar_resolution_ = 0.0596;
  // int cart_pixel_width = (2 * maxr_) / cart_resolution_;
  // cv::Mat L_cart;
  // cv::Mat L_cart_colour;
  // radar_polar_to_cartesian(L, azimuth_angles, L_cart, radar_resolution_, cart_resolution_, cart_pixel_width, true, CV_32F);
  // cv::cvtColor(L_cart, L_cart_colour, cv::COLOR_GRAY2BGR);
  // // cv::imwrite("/home/epk/radar_topometric_localization/data/epk_test_plots/TEST_L.jpg", L_cart_colour);
  // cv::Mat raw_scan_convert, raw_colour_convert;
  // radar_polar_to_cartesian((raw_scan*255.0/2.0), azimuth_angles, raw_scan_convert, radar_resolution_, cart_resolution_, cart_pixel_width, true, CV_32F);
  // cv::cvtColor(raw_scan_convert, raw_colour_convert, cv::COLOR_GRAY2BGR);
  // raw_colour_convert = raw_colour_convert + L_cart_colour;
  // cv::imwrite("/home/epk/radar_topometric_localization/data/epk_test_plots/os_cfar/os_cfar_overlay.jpg", raw_colour_convert);

}

template <class PointT>
void TM_CFAR<PointT>::run(const cv::Mat &raw_scan, const float &res,
                         const std::vector<int64_t> &azimuth_times,
                         const std::vector<double> &azimuth_angles,
                         pcl::PointCloud<PointT> &pointcloud) {
  pointcloud.clear();
  const int rows = raw_scan.rows;
  const int cols = raw_scan.cols;
  if (width_ % 2 == 0) width_ += 1;
  const int w2 = std::floor(width_ / 2);
  auto mincol = minr_ / res + w2 + 1;
  if (mincol > cols || mincol < 0) mincol = 0;
  auto maxcol = maxr_ / res - w2;
  if (maxcol > cols || maxcol < 0) maxcol = cols;
  // const int N = maxcol - mincol;

  float conversion_factor = 255.0/20.0*std::log(10.0);
  cv::Mat raw_scan_watts_sqrd = raw_scan * conversion_factor;
  cv::exp(raw_scan_watts_sqrd, raw_scan_watts_sqrd);
  // apply square law detector to test cell
  cv::pow(raw_scan_watts_sqrd, 2, raw_scan_watts_sqrd);

  // cv::Mat L = cv::Mat::zeros(rows,cols,CV_32F);

  // #pragma omp parallel for
  for (int i = 0; i < rows; ++i) {
    const double azimuth = azimuth_angles[i];
    const int64_t time = azimuth_times[i];
    pcl::PointCloud<PointT> polar_time;
    // double mean = 0;

    std::vector<std::pair<int, float>> window;
    window.reserve(width_ - 1);
    int j = mincol - 1;
    for (int k = -w2; k < 0; ++k) {
      window.emplace_back(j + k, raw_scan_watts_sqrd.at<float>(i, j + k));
    }
    for (int k = 1; k <= w2; ++k) {
      window.emplace_back(j + k, raw_scan_watts_sqrd.at<float>(i, j + k));
    }
    std::sort(window.begin(), window.end(), sort_asc_by_second);

    float peak_points = 0;
    int num_peak_points = 0;

    // Temp change for simplified tuning
    int N1 = N1_;
    int N2 = N1_;

    // cv::Mat overlay = raw_scan.row(i).clone()*0.0;

    for (j = mincol; j < maxcol; ++j) {
      // remove cell under test and left-most cell
      window.erase(std::remove_if(window.begin(), window.end(),
                                  [&](const std::pair<float, int> &p) -> bool {
                                    return p.first == j ||
                                           p.first == j - w2 - 1;
                                  }),
                   window.end());
      // insert prev CUT and right-most cell
      auto prevcut = std::make_pair(j - 1, raw_scan_watts_sqrd.at<float>(i, j - 1));
      auto it = std::lower_bound(window.begin(), window.end(), prevcut,
                                 sort_asc_by_second);
      window.insert(it, prevcut);
      auto newentry = std::make_pair(j + w2, raw_scan_watts_sqrd.at<float>(i, j + w2));
      it = std::lower_bound(window.begin(), window.end(), newentry,
                            sort_asc_by_second);
      window.insert(it, newentry);

      // Copy window
      std::vector<std::pair<int, float>> trimmed_window = window;

      // Remove first N1 elements and last N2 elements
      if (N1 > 0 && N1 <= trimmed_window.size()) {
        trimmed_window.erase(trimmed_window.begin(), trimmed_window.begin() + N1);
      }
      if (N2 > 0 && N2 <= trimmed_window.size()) {
        trimmed_window.erase(trimmed_window.end() - N2, trimmed_window.end());
      }

      double sum = 0;
      for (const auto& cell : trimmed_window) {sum += cell.second;}
      double stat = sum / trimmed_window.size();
      
      const float thres = threshold_ * stat;


      if (raw_scan_watts_sqrd.at<float>(i, j) > thres) {
        peak_points += j;
        num_peak_points += 1;
      } else if (num_peak_points > 0) {
        // L.at<float>(i, int(peak_points / num_peak_points)) = 245.0;
        PointT p;
        p.rho = res * peak_points / num_peak_points + static_cast<float>(range_offset_);
        p.phi = azimuth;
        p.theta = 0;
        p.timestamp = time;
        polar_time.push_back(p);
        peak_points = 0;
        num_peak_points = 0;
      }
    }
    // cv::Mat s = raw_scan.row(i).clone();
    // s.convertTo(s, CV_64F);
    // // Convert raw signal to dB
    // s = s *255;
    // cv::Mat display;
    // cv::Ptr<cv::plot::Plot2d> test_plot = cv::plot::Plot2d::create(s);
    // test_plot->setPlotSize(800, 600);
    // test_plot->setMaxX(cols);
    // test_plot->setMinX(0);
    // test_plot->setMaxY(255);
    // test_plot->setMinY(-1);
    // test_plot->setInvertOrientation(true);
    // test_plot->setGridLinesNumber(8);
    // test_plot->render(display);
    // cv::imwrite("/home/epk/radar_topometric_localization/data/epk_test_plots/tm_cfar/tmcfar_squared_db.jpg", display);

    // overlay.convertTo(overlay, CV_64F);
    // test_plot = cv::plot::Plot2d::create(overlay);
    // test_plot->setPlotSize(800, 600);
    // test_plot->setMaxX(cols);
    // test_plot->setMinX(0);
    // test_plot->setMaxY(255);
    // test_plot->setMinY(-1);
    // test_plot->setInvertOrientation(true);
    // test_plot->setGridLinesNumber(8);
    // test_plot->render(display);
    // cv::imwrite("/home/epk/radar_topometric_localization/data/epk_test_plots/tm_cfar/tmcfar_thresh_db.jpg", display); 

    // #pragma omp critical
    {
      pointcloud.insert(pointcloud.end(), polar_time.begin(), polar_time.end());
    }
  }
  // double cart_resolution_= 0.2384;
  // double radar_resolution_ = 0.0596;
  // int cart_pixel_width = (2 * maxr_) / cart_resolution_;
  // cv::Mat L_cart;
  // cv::Mat L_cart_colour;
  // radar_polar_to_cartesian(L, azimuth_angles, L_cart, radar_resolution_, cart_resolution_, cart_pixel_width, true, CV_32F);
  // cv::cvtColor(L_cart, L_cart_colour, cv::COLOR_GRAY2BGR);
  // // cv::imwrite("/home/epk/radar_topometric_localization/data/epk_test_plots/TEST_L.jpg", L_cart_colour);
  // cv::Mat raw_scan_convert, raw_colour_convert;
  // radar_polar_to_cartesian((raw_scan*255.0/2.0), azimuth_angles, raw_scan_convert, radar_resolution_, cart_resolution_, cart_pixel_width, true, CV_32F);
  // cv::cvtColor(raw_scan_convert, raw_colour_convert, cv::COLOR_GRAY2BGR);
  // raw_colour_convert = raw_colour_convert + L_cart_colour;
  // cv::imwrite("/home/epk/radar_topometric_localization/data/epk_test_plots/tm_cfar/tm_cfar_overlay.jpg", raw_colour_convert);
}


template <class PointT>
void ModifiedCACFAR<PointT>::run(const cv::Mat &raw_scan, const float &res,
                                 const std::vector<int64_t> &azimuth_times,
                                 const std::vector<double> &azimuth_angles,
                                 pcl::PointCloud<PointT> &pointcloud) {
  pointcloud.clear();
  const int rows = raw_scan.rows;
  const int cols = raw_scan.cols;
  if (width_ % 2 == 0) width_ += 1;
  const int w2 = std::floor(width_ / 2);
  auto mincol = minr_ / res + w2 + guard_ + 1;
  if (mincol > cols || mincol < 0) mincol = 0;
  auto maxcol = maxr_ / res - w2 - guard_;
  if (maxcol > cols || maxcol < 0) maxcol = cols;
  const int N = maxcol - mincol;

  for (int i = 0; i < rows; ++i) {
    const double azimuth = azimuth_angles[i];
    const int64_t time = azimuth_times[i];
    pcl::PointCloud<PointT> polar_time;
    double mean = 0;
    for (int j = mincol; j < maxcol; ++j) {
      mean += static_cast<double>(raw_scan.at<float>(i, j));
    }
    mean /= N;

    float peak_points = 0;
    int num_peak_points = 0;

    for (int j = mincol; j < maxcol; ++j) {
      double left = 0;
      double right = 0;
      for (int k = -w2 - guard_; k < -guard_; ++k)
        left += static_cast<double>(raw_scan.at<float>(i, j + k));
      for (int k = guard_ + 1; k <= w2 + guard_; ++k)
        right += static_cast<double>(raw_scan.at<float>(i, j + k));
      // (statistic) estimate of clutter power
      // const double stat = (left + right) / (2 * w2);
      const double stat = std::max(left, right) / w2;  // GO-CFAR
      const double thres = threshold_ * stat + threshold2_ * mean + threshold3_;
      if (static_cast<double>(raw_scan.at<float>(i, j)) > thres) {
        peak_points += j;
        num_peak_points += 1;
      } else if (num_peak_points > 0) {
        PointT p;
        p.rho = res * peak_points / num_peak_points + static_cast<float>(range_offset_);
        p.phi = azimuth;
        p.theta = 0;
        p.timestamp = time;
        polar_time.push_back(p);
        peak_points = 0;
        num_peak_points = 0;
      }
    }
    pointcloud.insert(pointcloud.end(), polar_time.begin(), polar_time.end());
  }
}


template <class PointT>
void ModifiedCACFARPower<PointT>::run(const cv::Mat &raw_scan, const float &res,
                                 const std::vector<int64_t> &azimuth_times,
                                 const std::vector<double> &azimuth_angles,
                                 pcl::PointCloud<PointT> &pointcloud) {
  pointcloud.clear();
  const int rows = raw_scan.rows;
  const int cols = raw_scan.cols;
  if (width_ % 2 == 0) width_ += 1;
  const int w2 = std::floor(width_ / 2);
  auto mincol = minr_ / res + w2 + guard_ + 1;
  if (mincol > cols || mincol < 0) mincol = 0;
  auto maxcol = maxr_ / res - w2 - guard_;
  if (maxcol > cols || maxcol < 0) maxcol = cols;

  // cv::Mat raw_scan_watts_sqrd = raw_scan.clone();
  float conversion_factor = 255.0/20.0*std::log(10.0);
  cv::Mat raw_scan_watts_sqrd = raw_scan * conversion_factor;
  cv::exp(raw_scan_watts_sqrd, raw_scan_watts_sqrd);
  // apply square law detector to test cell
  cv::pow(raw_scan_watts_sqrd, 2, raw_scan_watts_sqrd);

  for (int i = 0; i < rows; ++i) {
    const double azimuth = azimuth_angles[i];
    const int64_t time = azimuth_times[i];
    pcl::PointCloud<PointT> polar_time;
    float peak_points = 0;
    int num_peak_points = 0;

    double left_test = 0;
    double right_test = 0;
    double mean = 0;

    for (int k = -w2 - guard_; k < -guard_; ++k){
        left_test += static_cast<double>(raw_scan_watts_sqrd.at<float>(i, mincol-1 + k));
      }

    for (int k = guard_ + 1; k <= w2 + guard_; ++k){
      right_test += static_cast<double>(raw_scan_watts_sqrd.at<float>(i, mincol-1 + k));
    }


    for (int j = mincol; j < maxcol; ++j) {
      // Intensity of test cell Y with sqaure law
      float intensity_Y=raw_scan_watts_sqrd.at<float>(i, j);

      left_test = left_test - static_cast<double>(raw_scan_watts_sqrd.at<float>(i, j + -w2 - guard_ - 1)) + static_cast<double>(raw_scan_watts_sqrd.at<float>(i, j -guard_ - 1));
      right_test = right_test - static_cast<double>(raw_scan_watts_sqrd.at<float>(i, j + guard_)) + static_cast<double>(raw_scan_watts_sqrd.at<float>(i, j + w2 + guard_));
      mean = (left_test+right_test) / (2 * w2);


      // // suggestion from other code base in CFEAR paper
      // if(raw_scan.at<float>(i, j)*127.5 < static_threshold_squared){continue;}

      // float left = 0;
      // float right = 0;
      // for (int k = -w2 - guard_; k < -guard_; ++k){
      //   left +=  raw_scan_watts_sqrd.at<float>(i, j + k);
      // }
      // for (int k = guard_ + 1; k <= w2 + guard_; ++k){
      //   right += raw_scan_watts_sqrd.at<float>(i, j + k);
      // }

      // // Just do mean over the entire window for now
      // mean = (left+right) / (2 * w2);

  
      const float thres = threshold_ * mean;

      if (intensity_Y > thres) {
        peak_points += j;
        num_peak_points += 1;
      } else if (num_peak_points > 0) {
        PointT p;
        p.rho = res * peak_points / num_peak_points + static_cast<float>(range_offset_);
        p.phi = azimuth;
        p.theta = 0;
        p.timestamp = time;
        polar_time.push_back(p);
        peak_points = 0;
        num_peak_points = 0;
      }
    } 

    // cv::Mat scaled_signal = s.clone();
    // cv::Mat overlay2 =overlay.clone()*0.0;
    // for(int j = mincol; j < maxcol; ++j){overlay2.at<double>(j) = 10.0*std::log10(overlay.at<double>(j));} 
    // for(int j = mincol; j < maxcol; ++j){overlay_db_2.at<double>(j) = overlay_db.at<double>(j);} 

    // cv::Mat scaled = scaled_signal.clone();

    // test_plot = cv::plot::Plot2d::create(scaled);
    // test_plot->setPlotSize(800, 600);
    // test_plot->setMaxX(cols);
    // test_plot->setMinX(0);
    // test_plot->setMaxY(128);
    // test_plot->setMinY(-1);
    // test_plot->setInvertOrientation(true);
    // test_plot->setGridLinesNumber(8);
    // test_plot->render(display);
    // cv::imwrite("/home/epk/radar_topometric_localization/data/epk_test_plots/power_watts_thresh.jpg", display);

    // cv::Mat overlay3;
    // overlay2.convertTo(overlay3, CV_64F);
    // test_plot = cv::plot::Plot2d::create(overlay3);
    // test_plot->setPlotSize(800, 600);
    // test_plot->setMaxX(cols);
    // test_plot->setMinX(0);
    // test_plot->setMaxY(128);
    // test_plot->setMinY(-1);
    // test_plot->setInvertOrientation(true);
    // test_plot->setGridLinesNumber(8);
    // test_plot->render(display);
    // cv::imwrite("/home/epk/radar_topometric_localization/data/epk_test_plots/thresh.jpg", display);


    pointcloud.insert(pointcloud.end(), polar_time.begin(), polar_time.end());
  }
}

template <class PointT>
void CAGO_CFAR<PointT>::run(const cv::Mat &raw_scan, const float &res,
                                 const std::vector<int64_t> &azimuth_times,
                                 const std::vector<double> &azimuth_angles,
                                 pcl::PointCloud<PointT> &pointcloud) {
  pointcloud.clear();
  const int rows = raw_scan.rows;
  const int cols = raw_scan.cols;
  if (width_ % 2 == 0) width_ += 1;
  const int w2 = std::floor(width_ / 2);
  auto mincol = minr_ / res + w2 + guard_ + 1;
  if (mincol > cols || mincol < 0) mincol = 0;
  auto maxcol = maxr_ / res - w2 - guard_;
  if (maxcol > cols || maxcol < 0) maxcol = cols;
  const int N = maxcol - mincol;  

  // Convert Navtechs 8-bit dB half steps to watts
  cv::Mat raw_scan_watts_sqrd = raw_scan.clone();
  float conversion_factor = 255.0/20.0*std::log(10.0);
  raw_scan_watts_sqrd = raw_scan_watts_sqrd * conversion_factor;
  cv::exp(raw_scan_watts_sqrd, raw_scan_watts_sqrd);
  // apply square law detector to test cell
  cv::pow(raw_scan_watts_sqrd, 2, raw_scan_watts_sqrd);


  // cv::Mat L = cv::Mat::zeros(rows,cols,CV_32F);


  for (int i = 0; i < rows; ++i) {
    const double azimuth = azimuth_angles[i];
    const int64_t time = azimuth_times[i];
    pcl::PointCloud<PointT> polar_time;
    float peak_points = 0;
    int num_peak_points = 0;

    // cv::Mat overlay = raw_scan.row(i).clone()*0.0;

    double left = 0;
    double right = 0;
    double mean = 0;

    for (int k = -w2 - guard_; k < -guard_; ++k){left += static_cast<double>(raw_scan_watts_sqrd.at<float>(i, mincol-1 + k));}
    for (int k = guard_ + 1; k <= w2 + guard_; ++k){right += static_cast<double>(raw_scan_watts_sqrd.at<float>(i, mincol-1 + k));}

    for (int j = mincol; j < maxcol; ++j) {
      // Intensity of test cell Y with sqaure law
      float intensity_Y= raw_scan_watts_sqrd.at<float>(i, j);

      left = left - static_cast<double>(raw_scan_watts_sqrd.at<float>(i, j + -w2 - guard_ - 1)) + static_cast<double>(raw_scan_watts_sqrd.at<float>(i, j -guard_ - 1));
      right = right - static_cast<double>(raw_scan_watts_sqrd.at<float>(i, j + guard_)) + static_cast<double>(raw_scan_watts_sqrd.at<float>(i, j + w2 + guard_));

      mean = std::max(left, right) / w2;

      const float thres = threshold_ * mean;
      // overlay.at<float>(j) = 10.0*std::log10(thres);

      if (intensity_Y > thres) {
        peak_points += j;
        num_peak_points += 1;
      } else if (num_peak_points > 0) {
        // L.at<float>(i, j) = 245.0;
        // L.at<float>(i, int(peak_points / num_peak_points)) = 245.0;


        PointT p;
        p.rho = res * peak_points / num_peak_points + static_cast<float>(range_offset_);
        p.phi = azimuth;
        p.theta = 0;
        p.timestamp = time;
        polar_time.push_back(p);
        peak_points = 0;
        num_peak_points = 0;
      }
    } 

    // cv::Mat s = raw_scan.row(i).clone();
    // s.convertTo(s, CV_64F);
    // // Convert raw signal to dB
    // s = s *255;
    // cv::Mat display;
    // cv::Ptr<cv::plot::Plot2d> test_plot = cv::plot::Plot2d::create(s);
    // test_plot->setPlotSize(800, 600);
    // test_plot->setMaxX(cols);
    // test_plot->setMinX(0);
    // test_plot->setMaxY(255);
    // test_plot->setMinY(-1);
    // test_plot->setInvertOrientation(true);
    // test_plot->setGridLinesNumber(8);
    // test_plot->render(display);
    // cv::imwrite("/home/epk/radar_topometric_localization/data/epk_test_plots/cago_cfar/cago_squared_db.jpg", display);

    // overlay.convertTo(overlay, CV_64F);
    // test_plot = cv::plot::Plot2d::create(overlay);
    // test_plot->setPlotSize(800, 600);
    // test_plot->setMaxX(cols);
    // test_plot->setMinX(0);
    // test_plot->setMaxY(255);
    // test_plot->setMinY(-1);
    // test_plot->setInvertOrientation(true);
    // test_plot->setGridLinesNumber(8);
    // test_plot->render(display);
    // cv::imwrite("/home/epk/radar_topometric_localization/data/epk_test_plots/cago_cfar/cago_thresh_db.jpg", display); 

    pointcloud.insert(pointcloud.end(), polar_time.begin(), polar_time.end());
  }

  // double cart_resolution_= 0.2384;
  // double radar_resolution_ = 0.0596;
  // int cart_pixel_width = (2 * maxr_) / cart_resolution_;
  // cv::Mat L_cart;
  // cv::Mat L_cart_colour;
  // radar_polar_to_cartesian(L, azimuth_angles, L_cart, radar_resolution_, cart_resolution_, cart_pixel_width, true, CV_32F);
  // cv::cvtColor(L_cart, L_cart_colour, cv::COLOR_GRAY2BGR);
  // // cv::imwrite("/home/epk/radar_topometric_localization/data/epk_test_plots/TEST_L.jpg", L_cart_colour);
  // cv::Mat raw_scan_convert, raw_colour_convert;
  // radar_polar_to_cartesian((raw_scan*255.0/2.0), azimuth_angles, raw_scan_convert, radar_resolution_, cart_resolution_, cart_pixel_width, true, CV_32F);
  // cv::cvtColor(raw_scan_convert, raw_colour_convert, cv::COLOR_GRAY2BGR);
  // raw_colour_convert = raw_colour_convert + L_cart_colour;
  // cv::imwrite("/home/epk/radar_topometric_localization/data/epk_test_plots/cago_cfar/cago_overlay.jpg", raw_colour_convert);

}

template <class PointT>
void CASO_CFAR<PointT>::run(const cv::Mat &raw_scan, const float &res,
                                 const std::vector<int64_t> &azimuth_times,
                                 const std::vector<double> &azimuth_angles,
                                 pcl::PointCloud<PointT> &pointcloud) {
  pointcloud.clear();
  const int rows = raw_scan.rows;
  const int cols = raw_scan.cols;
  if (width_ % 2 == 0) width_ += 1;
  const int w2 = std::floor(width_ / 2);
  auto mincol = minr_ / res + w2 + guard_ + 1;
  if (mincol > cols || mincol < 0) mincol = 0;
  auto maxcol = maxr_ / res - w2 - guard_;
  if (maxcol > cols || maxcol < 0) maxcol = cols;
  const int N = maxcol - mincol;  

  // Convert Navtechs 8-bit dB half steps to watts
  cv::Mat raw_scan_watts_sqrd = raw_scan.clone();
  float conversion_factor = 255.0/20.0*std::log(10.0);
  raw_scan_watts_sqrd = raw_scan_watts_sqrd * conversion_factor;
  cv::exp(raw_scan_watts_sqrd, raw_scan_watts_sqrd);
  // apply square law detector to test cell
  cv::pow(raw_scan_watts_sqrd, 2, raw_scan_watts_sqrd);

  // cv::Mat L = cv::Mat::zeros(rows,cols,CV_32F);

  for (int i = 0; i < rows; ++i) {
    const double azimuth = azimuth_angles[i];
    const int64_t time = azimuth_times[i];
    pcl::PointCloud<PointT> polar_time;
    double mean = 0;
    float peak_points = 0;
    int num_peak_points = 0;

    // cv::Mat overlay = raw_scan.row(i).clone()*0.0;

    double left = 0;
    double right = 0;
    for (int k = -w2 - guard_; k < -guard_; ++k){left += static_cast<double>(raw_scan_watts_sqrd.at<float>(i, mincol-1 + k));}
    for (int k = guard_ + 1; k <= w2 + guard_; ++k){right += static_cast<double>(raw_scan_watts_sqrd.at<float>(i, mincol-1 + k));}

    for (int j = mincol; j < maxcol; ++j) {
      // Intensity of test cell Y with sqaure law
      float intensity_Y= raw_scan_watts_sqrd.at<float>(i, j);

      // left = left - static_cast<double>(raw_scan_watts_sqrd.at<float>(i, j + -w2 - guard_ - 1) + raw_scan_watts_sqrd.at<float>(i, j -guard_ - 1));
      // right = right - static_cast<double>(raw_scan_watts_sqrd.at<float>(i, j + guard_) + raw_scan_watts_sqrd.at<float>(i, j + w2 + guard_));

      left = left - static_cast<double>(raw_scan_watts_sqrd.at<float>(i, j + -w2 - guard_ - 1)) + static_cast<double>(raw_scan_watts_sqrd.at<float>(i, j -guard_ - 1));
      right = right - static_cast<double>(raw_scan_watts_sqrd.at<float>(i, j + guard_)) + static_cast<double>(raw_scan_watts_sqrd.at<float>(i, j + w2 + guard_));

      mean = std::min(left, right) / w2;

      const float thres = threshold_ * mean;

      if (intensity_Y > thres) {
        peak_points += j;
        num_peak_points += 1;
      } else if (num_peak_points > 0) {
        // L.at<float>(i, j) = 245.0;
        // L.at<float>(i, int(peak_points / num_peak_points)) = 245.0;

        PointT p;
        p.rho = res * peak_points / num_peak_points + static_cast<float>(range_offset_);
        p.phi = azimuth;
        p.theta = 0;
        p.timestamp = time;
        polar_time.push_back(p);
        peak_points = 0;
        num_peak_points = 0;
      }
    } 

    // cv::Mat s = raw_scan.row(i).clone();
    // s.convertTo(s, CV_64F);
    // // Convert raw signal to dB
    // s = s *255;
    // cv::Mat display;
    // cv::Ptr<cv::plot::Plot2d> test_plot = cv::plot::Plot2d::create(s);
    // test_plot->setPlotSize(800, 600);
    // test_plot->setMaxX(cols);
    // test_plot->setMinX(0);
    // test_plot->setMaxY(255);
    // test_plot->setMinY(-1);
    // test_plot->setInvertOrientation(true);
    // test_plot->setGridLinesNumber(8);
    // test_plot->render(display);
    // cv::imwrite("/home/epk/radar_topometric_localization/data/epk_test_plots/caso_cfar/caso_squared_db.jpg", display);

    // overlay.convertTo(overlay, CV_64F);
    // test_plot = cv::plot::Plot2d::create(overlay);
    // test_plot->setPlotSize(800, 600);
    // test_plot->setMaxX(cols);
    // test_plot->setMinX(0);
    // test_plot->setMaxY(255);
    // test_plot->setMinY(-1);
    // test_plot->setInvertOrientation(true);
    // test_plot->setGridLinesNumber(8);
    // test_plot->render(display);
    // cv::imwrite("/home/epk/radar_topometric_localization/data/epk_test_plots/caso_cfar/caso_thresh_db.jpg", display); 

    pointcloud.insert(pointcloud.end(), polar_time.begin(), polar_time.end());
  }

  // double cart_resolution_= 0.2384;
  // double radar_resolution_ = 0.0596;
  // int cart_pixel_width = (2 * maxr_) / cart_resolution_;
  // cv::Mat L_cart;
  // cv::Mat L_cart_colour;
  // radar_polar_to_cartesian(L, azimuth_angles, L_cart, radar_resolution_, cart_resolution_, cart_pixel_width, true, CV_32F);
  // cv::cvtColor(L_cart, L_cart_colour, cv::COLOR_GRAY2BGR);
  // // cv::imwrite("/home/epk/radar_topometric_localization/data/epk_test_plots/TEST_L.jpg", L_cart_colour);
  // cv::Mat raw_scan_convert, raw_colour_convert;
  // radar_polar_to_cartesian((raw_scan*255.0/2.0), azimuth_angles, raw_scan_convert, radar_resolution_, cart_resolution_, cart_pixel_width, true, CV_32F);
  // cv::cvtColor(raw_scan_convert, raw_colour_convert, cv::COLOR_GRAY2BGR);
  // raw_colour_convert = raw_colour_convert + L_cart_colour;
  // cv::imwrite("/home/epk/radar_topometric_localization/data/epk_test_plots/caso_cfar/caso_overlay.jpg", raw_colour_convert);
}


struct hash_pair {
    template <class T1, class T2>
    size_t operator()(const std::pair<T1, T2>& p) const
    {
        // Hash the first element
        size_t hash1 = std::hash<T1>{}(p.first);
        // Hash the second element
        size_t hash2 = std::hash<T2>{}(p.second);
        // Combine the two hash values
        return hash1 ^ (hash2 + 0x9e3779b9 + (hash1 << 6) + (hash1 >> 2));
    }
};

// int frame_num = 2626;

template <class PointT>
void CFEAR_KStrong<PointT>::run(const cv::Mat &raw_scan, const float &res,
                             const std::vector<int64_t> &azimuth_times,
                             const std::vector<double> &azimuth_angles,
                             pcl::PointCloud<PointT> &pointcloud) {

  pointcloud.clear();
  const int rows = raw_scan.rows;
  const int cols = raw_scan.cols;
  auto mincol = minr_ / res;
  if (mincol > cols || mincol < 0) mincol = 0;
  auto maxcol = maxr_ / res;
  if (maxcol > cols || maxcol < 0) maxcol = cols;
  const auto N = maxcol - mincol;

  // // Convert Navtechs 8-bit dB half steps to watts
  // // cv::Mat raw_scan_watts_sqrd = raw_scan.clone();
  // float conversion_factor = 255.0/20.0*std::log(10.0);
  // cv::Mat  raw_scan_watts_sqrd = raw_scan * conversion_factor;
  // cv::exp(raw_scan_watts_sqrd, raw_scan_watts_sqrd);
  // // apply square law detector to test cell
  // cv::pow(raw_scan_watts_sqrd, 2, raw_scan_watts_sqrd);

  // float threshold3_watts = std::pow(10,255.0*z_min_/20);
  // float static_threshold_squared = threshold3_watts*threshold3_watts; 

  // // // int frame_num= 1729;
  // // int frame_num = 2626;

  cv::Mat L_f = cv::Mat::zeros(rows,cols,CV_32F);
  cv::Mat L = cv::Mat::zeros(rows,cols,CV_32F);
  cv::Mat L_grid = cv::Mat::zeros(rows,cols,CV_32F);
  

  int first_cloud = 0;
  int second_cloud = 0;
  bool mean_mode = true;

  // float grid_size = r_/f_;
  // float grid_size = 1.0/3.0;
  float grid_size = 0.5;


  int N_a = rows;
  float theta_factor = 2 * M_PI / N_a;
  std::vector<std::pair<float,float>> P_f;
  std::vector<std::pair<float,float>> P_d;
  std::unordered_map<std::pair<float, float>, std::vector<std::pair<float,float>>, hash_pair> P_d_points;
  std::unordered_map<std::pair<float, float>, std::pair<int,float>, hash_pair> cart_to_polar_points;
  std::set<std::pair<double, double>> added_points;


  for (int i = 0; i < rows; ++i) {
    std::vector<std::pair<float, int>> intens;
  
    const float thres = z_min_;

    for (int j = mincol; j < maxcol; ++j) {
      if (raw_scan.at<float>(i, j) >= thres){
        intens.push_back(std::make_pair(raw_scan.at<float>(i, j), j));
      }
    }

    int thresholded_point_count = intens.size();
    if(thresholded_point_count > 0){
      // sort intensities in descending order
      std::sort(intens.begin(), intens.end(), sort_desc_by_first);

      // const double azimuth = azimuth_angles[i];
      // const int64_t time = azimuth_times[i];
      pcl::PointCloud<PointT> polar_time;
      for (int j = 0; j < kstrong_; ++j) {
        if (j >= thresholded_point_count){break;}
        const double az = azimuth_angles[i];
        float cart_x = (intens[j].second * res + range_offset_) * cos(az);
        float cart_y = (intens[j].second * res + range_offset_) * sin(az);

        if(mean_mode==false){
          float dist = res * intens[j].second + range_offset_;
          cart_to_polar_points[std::make_pair(cart_x,cart_y)] = std::make_pair(i,dist);
        }

        P_f.push_back(std::make_pair(cart_x,cart_y));

        L_f.at<float>(i,intens[j].second) = 245.0;

        first_cloud++;
      }
    }
  }

  // // // // std::cout<<"PF count"<<P_f.size()<<std::endl;
  // double cart_resolution_= 0.2384;
  // double radar_resolution_ = 0.0596;
  // int cart_pixel_width = (2 * maxr_) / cart_resolution_;
  // cv::Mat L_cart;
  // cv::Mat L_cart_colour;
  // radar_polar_to_cartesian(L_f, azimuth_angles, L_cart, radar_resolution_, cart_resolution_, cart_pixel_width, true, CV_32F);
  // cv::cvtColor(L_cart, L_cart_colour, cv::COLOR_GRAY2BGR);
  // // cv::imwrite("/home/epk/radar_topometric_localization/data/epk_test_plots/TEST_L.jpg", L_cart_colour);
  // cv::Mat raw_scan_convert, raw_colour_convert;
  // radar_polar_to_cartesian((raw_scan*255.0/2.0), azimuth_angles, raw_scan_convert, radar_resolution_, cart_resolution_, cart_pixel_width, true, CV_32F);
  // cv::cvtColor(raw_scan_convert, raw_colour_convert, cv::COLOR_GRAY2BGR);
  // raw_colour_convert = raw_colour_convert + L_cart_colour;
  // cv::imwrite("/home/epk/radar_topometric_localization/data/epk_test_plots/CFEAR/testing/CFEAR_P_f"+ std::to_string(1) +".jpg", raw_colour_convert);
  
  for(auto&[p_x,p_y] : P_f){

    int grid_x = p_x / grid_size;
    int grid_y = p_y / grid_size;

    float center_x = grid_x * grid_size + grid_size / 2;
    float center_y = grid_y * grid_size + grid_size / 2;

    P_d.push_back(std::make_pair(center_x,center_y));

  }

  for(auto& [pd_x,pd_y] : P_d){
    std::vector<std::pair<float,float>> point_set;
    std::vector<std::pair<float,float>> polar_point_set;

    for(auto& [pf_x,pf_y]: P_f){
      float dx = pd_x-pf_x;
      float dy = pd_y-pf_y;
      float diff = std::sqrt(dx*dx + dy*dy);
  
      if(diff < r_){
        point_set.push_back(std::make_pair(pf_x,pf_y));
      }

    }
    P_d_points[std::make_pair(pd_x,pd_y)] = point_set;
  }

  for(auto& [grid_point,filtered_points] : P_d_points){
    pcl::PointCloud<PointT> polar_time;
  
    float sample_mean_x = 0.0, sample_mean_y = 0.0;

    // Compute the sample mean
    int N_f = filtered_points.size();
    if(N_f < 6){continue;}

    for(auto& [p_x,p_y] : filtered_points){
      sample_mean_x += p_x;
      sample_mean_y += p_y;
    }
    sample_mean_x /= N_f;
    sample_mean_y /= N_f;

    // Compute the sample covariance
    float sum_xx = 0.0, sum_yy = 0.0, sum_xy = 0.0;
    for (auto& [p_x,p_y]  : filtered_points) {
        float dx = p_x - sample_mean_x;
        float dy = p_y - sample_mean_y;
        sum_xx += dx * dx;
        sum_yy += dy * dy;
        sum_xy += dx * dy;
    }
    float cov_xx = sum_xx / (N_f - 1);
    float cov_yy = sum_yy / (N_f - 1);
    float cov_xy = sum_xy / (N_f - 1);

    Eigen::Matrix2d covarianceMatrix;
    covarianceMatrix << cov_xx, cov_xy,
                        cov_xy, cov_yy;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(covarianceMatrix);
    Eigen::Vector2d eigen_values = solver.eigenvalues().real();
    float max_eigen = eigen_values.maxCoeff();
    float min_eigen = eigen_values.minCoeff();

    // if(max_eigen/min_eigen > 10e5){
    //   std::cout<<"HERE: " << " "<<max_eigen<< "/"<<min_eigen<< " mean coord: "<<sample_mean_x<< " "<<sample_mean_y<< " count "<< N_f<<std::endl;
    //   std::cout<<covarianceMatrix<<std::endl;

    //   }

    // if(max_eigen/min_eigen > 10e5 || filtered_points.size() < 6){
    if(max_eigen/min_eigen > 10e5){// || filtered_points.size() < 6){
      // std::cout<<"HERE: " << " "<<max_eigen<< "/"<<min_eigen<< " mean coord: "<<sample_mean_x<< " "<<sample_mean_y<< " count "<< N_f<<std::endl;
      // std::cout<<covarianceMatrix<<std::endl;
      continue;
    } else{
      if(mean_mode){
        float dist = std::sqrt(sample_mean_x * sample_mean_x + sample_mean_y * sample_mean_y);
        double theta = std::atan2(sample_mean_y, sample_mean_x);

        double normalizedAngle = std::fmod(theta, 2 * M_PI);
        if (normalizedAngle < 0) {
            normalizedAngle += 2 * M_PI;
        }

        int index = static_cast<int>(std::floor(normalizedAngle * rows / (2 * M_PI))) % rows;

        double a1 = azimuth_angles[index];
        double a2 = azimuth_angles[index+1];

        if(a1 > normalizedAngle){
          if(index >=1){
            index--;
            a1 = azimuth_angles[index];
            a2 = azimuth_angles[index+1];
          }
        }
        if(a2 < normalizedAngle){
          if(index < rows-1){
            index++;
            a1 = azimuth_angles[index];
            a2 = azimuth_angles[index+1];
          }
        }
        int64_t t1=0;
        int64_t t2=0;
        int64_t time=0;
        if(index<=0){
          time = azimuth_times[0];
        } else if(index >= rows-1){
          time = azimuth_times[rows-1];
        } else{
          t1 = azimuth_times[index];
          t2 = azimuth_times[index+1];
          time = t1 + (t2-t1)*(normalizedAngle-a1)/(a2-a1);
        }

        PointT p;
        p.rho = dist;// * res;// + range_offset_;
        p.phi = normalizedAngle;
        p.theta = 0;
        p.timestamp = time;

        polar_time.push_back(p);

        // L.at<float>(index, int((dist- range_offset_)/res)) = 245.0;
        
        second_cloud++;
      } 
      else{
        for(auto& [p_x,p_y]  : filtered_points) {

          if (added_points.find(std::make_pair(p_x,p_y)) != added_points.end()){continue;}
          else{added_points.insert(std::make_pair(p_x,p_y));}

          auto az_pair = cart_to_polar_points[std::make_pair(p_x,p_y)];
          int az_idx = az_pair.first;
          float dist = az_pair.second;

          // int range_bin = int((dist-range_offset_)/res);
          // if(range_bin<mincol||range_bin>maxcol){
          //   std::cout<<"range issue: "<<range_bin<<std::endl;
          // }


          PointT p;
          // p.rho = dist;
          // p.phi = normalizedAngle;
          p.rho =dist;
          p.phi = azimuth_angles[az_idx];
          p.timestamp = azimuth_times[az_idx];
          p.theta = 0;
          // p.timestamp = time;

          polar_time.push_back(p);

          // L.at<float>(az_idx, int((dist-range_offset_)/res)) = 245.0;
          second_cloud++;

        }
      }


      // float dist = std::sqrt(grid_point.first * grid_point.first + grid_point.second * grid_point.second);
      // double theta = std::atan2(grid_point.second, grid_point.first);

      // double normalizedAngle = std::fmod(theta, 2 * M_PI);
      // if (normalizedAngle < 0) {
      //     normalizedAngle += 2 * M_PI;
      // }

      // int index = static_cast<int>(std::floor(normalizedAngle * rows / (2 * M_PI))) % rows;

      // // double a1 = azimuth_angles[index];
      // int dist_ind = int((dist-range_offset_)/res);

      // L_grid.at<float>(index, dist_ind) = 245.0;

      // L_grid.at<float>(index, dist_ind+1) = 245.0;
      // L_grid.at<float>(index, dist_ind-1) = 245.0;
      // L_grid.at<float>(index+1, dist_ind) = 245.0;
      // L_grid.at<float>(index-1, dist_ind) = 245.0;
      // L_grid.at<float>(index+1, dist_ind+1) = 245.0;
      // L_grid.at<float>(index-1, dist_ind+1) = 245.0;
      // L_grid.at<float>(index+1, dist_ind-1) = 245.0;
      // L_grid.at<float>(index-1, dist_ind-1) = 245.0;

    }
    // #pragma omp critical
    {
      pointcloud.insert(pointcloud.end(), polar_time.begin(), polar_time.end());
    }
  }
  
  // std::cout<<first_cloud<< " , "<<second_cloud<<std::endl;


  // cv::Mat L_cart2;
  // cv::Mat L_cart_colour2;
  // radar_polar_to_cartesian(L, azimuth_angles, L_cart2, radar_resolution_, cart_resolution_, cart_pixel_width, true, CV_32F);
  // cv::cvtColor(L_cart2, L_cart_colour2, cv::COLOR_GRAY2BGR);
  // cv::Mat raw_scan_convert2, raw_colour_convert2;
  // radar_polar_to_cartesian((raw_scan*255.0/2.0), azimuth_angles, raw_scan_convert2, radar_resolution_, cart_resolution_, cart_pixel_width, true, CV_32F);
  // cv::cvtColor(raw_scan_convert2, raw_colour_convert2, cv::COLOR_GRAY2BGR);
  // raw_colour_convert2 = raw_colour_convert2 + L_cart_colour2;
  // cv::imwrite("/home/epk/radar_topometric_localization/data/epk_test_plots/CFEAR/testing/CFEAR_overlay_full_"+ std::to_string(r_) +"_"+std::to_string(f_) +".jpg", raw_colour_convert2);


  // cv::Mat L_cart3;
  // cv::Mat L_cart_colour3;
  // radar_polar_to_cartesian(L_grid, azimuth_angles, L_cart3, radar_resolution_, cart_resolution_, cart_pixel_width, true, CV_32F);
  // cv::cvtColor(L_cart3, L_cart_colour3, cv::COLOR_GRAY2BGR);
  // cv::Mat raw_scan_convert3, raw_colour_convert3;
  // radar_polar_to_cartesian((raw_scan*255.0/2.0), azimuth_angles, raw_scan_convert3, radar_resolution_, cart_resolution_, cart_pixel_width, true, CV_32F);
  // cv::cvtColor(raw_scan_convert3, raw_colour_convert3, cv::COLOR_GRAY2BGR);
  // raw_colour_convert3 = raw_colour_convert3 + L_cart_colour3;
  // cv::imwrite("/home/epk/radar_topometric_localization/data/epk_test_plots/CFEAR/testing/CFEAR_grid_"+ std::to_string(r_)+ "_"+std::to_string(f_) +".jpg", raw_colour_convert3);


  // // frame_num++;
  // sleep(6);

}

// int frame_num = 1167;

template <class PointT>
void BFAR_PURE<PointT>::run(const cv::Mat &raw_scan, const float &res,
                                 const std::vector<int64_t> &azimuth_times,
                                 const std::vector<double> &azimuth_angles,
                                 pcl::PointCloud<PointT> &pointcloud) {
  pointcloud.clear();
  const int rows = raw_scan.rows;
  const int cols = raw_scan.cols;
  if (width_ % 2 == 0) width_ += 1;
  const int w2 = std::floor(width_ / 2);
  auto mincol = minr_ / res + w2 + guard_ + 1;
  if (mincol > cols || mincol < 0) mincol = 0;
  auto maxcol = maxr_ / res - w2 - guard_;
  if (maxcol > cols || maxcol < 0) maxcol = cols;

  // int frame_num = 0;

  double threshold3_watts = std::pow(10,(255.0*threshold3_/20.0));
  double static_threshold_squared = threshold3_watts*threshold3_watts;
  if(threshold3_ == 0.0){static_threshold_squared=0.0;}


  // cv::Mat raw_scan_watts_sqrd = raw_scan.clone();
  float conversion_factor = 255.0/20.0*std::log(10.0);
  cv::Mat raw_scan_watts_sqrd = raw_scan * conversion_factor;
  cv::exp(raw_scan_watts_sqrd, raw_scan_watts_sqrd);
  // apply square law detector to test cell
  cv::pow(raw_scan_watts_sqrd, 2, raw_scan_watts_sqrd);
  // cv::Mat L = cv::Mat::zeros(rows,cols,CV_32F);


  for (int i = 0; i < rows; ++i) {
    const double azimuth = azimuth_angles[i];
    const int64_t time = azimuth_times[i];
    pcl::PointCloud<PointT> polar_time;
    float peak_points = 0;
    int num_peak_points = 0;

    // cv::Mat overlay = raw_scan.row(i).clone()*0.0;

    double left_test = 0.0;
    double right_test = 0.0;
    // double mean = 0.0;

    for (int k = -w2 - guard_; k < -guard_; ++k){
        left_test += static_cast<double>(raw_scan_watts_sqrd.at<float>(i, mincol-1 + k));
      }

    for (int k = guard_ + 1; k <= w2 + guard_; ++k){
      right_test += static_cast<double>(raw_scan_watts_sqrd.at<float>(i, mincol-1 + k));
    }

    for (int j = mincol; j < maxcol; ++j) {
      // Intensity of test cell Y
      // double intensity_Y = static_cast<double>(raw_scan_watts_sqrd.at<float>(i, j));
      float intensity_Y = raw_scan_watts_sqrd.at<float>(i, j);

      left_test = left_test - static_cast<double>(raw_scan_watts_sqrd.at<float>(i, j + -w2 - guard_ - 1)) + static_cast<double>(raw_scan_watts_sqrd.at<float>(i, j -guard_ - 1));
      right_test = right_test - static_cast<double>(raw_scan_watts_sqrd.at<float>(i, j + guard_)) + static_cast<double>(raw_scan_watts_sqrd.at<float>(i, j + w2 + guard_));

      // suggestion from other code base in CFEAR paper
      if(intensity_Y < static_threshold_squared){continue;}

      double mean = (left_test+right_test) / (2 * w2);

      // double thres = threshold_ * mean + static_threshold_squared;
      float thres = threshold_ * mean + static_threshold_squared;

      if (intensity_Y > thres) {
        peak_points += j;
        num_peak_points += 1;
      } else if (num_peak_points > 0) {
        // L.at<float>(i, int(peak_points / num_peak_points)) = 245.0;

        PointT p;
        p.rho =res * peak_points /static_cast<float>(num_peak_points) + static_cast<float>(range_offset_);
        p.phi = azimuth;
        p.theta = 0;
        p.timestamp = time;
        polar_time.push_back(p);
        peak_points = 0.0;
        num_peak_points = 0;
      }
    }

    // cv::Mat s = raw_scan.row(i).clone();
    // s.convertTo(s, CV_64F);
    // // Convert raw signal to dB
    // s = s *255;
    // cv::Mat display;
    // cv::Ptr<cv::plot::Plot2d> test_plot = cv::plot::Plot2d::create(s);
    // test_plot->setPlotSize(800, 600);
    // test_plot->setMaxX(cols);
    // test_plot->setMinX(0);
    // test_plot->setMaxY(255);
    // test_plot->setMinY(-1);
    // test_plot->setInvertOrientation(true);
    // test_plot->setGridLinesNumber(8);
    // test_plot->render(display);
    // cv::imwrite("/home/epk/radar_topometric_localization/data/epk_test_plots/bfar/bfar_squared_db.jpg", display);

    // overlay.convertTo(overlay, CV_64F);
    // test_plot = cv::plot::Plot2d::create(overlay);
    // test_plot->setPlotSize(800, 600);
    // test_plot->setMaxX(cols);
    // test_plot->setMinX(0);
    // test_plot->setMaxY(255);
    // test_plot->setMinY(-1);
    // test_plot->setInvertOrientation(true);
    // test_plot->setGridLinesNumber(8);
    // test_plot->render(display);
    // cv::imwrite("/home/epk/radar_topometric_localization/data/epk_test_plots/bfar/bfar_thresh_db.jpg", display); 

    pointcloud.insert(pointcloud.end(), polar_time.begin(), polar_time.end());
  }
  // if(frame_num > 1260 && frame_num<1278){
  //   double cart_resolution_= 0.2384;
  //   double radar_resolution_ = 0.0596;
  //   int cart_pixel_width = (2 * maxr_) / cart_resolution_;
  //   cv::Mat L_cart;
  //   cv::Mat L_cart_colour;
  //   radar_polar_to_cartesian(L, azimuth_angles, L_cart, radar_resolution_, cart_resolution_, cart_pixel_width, true, CV_32F);
  //   cv::cvtColor(L_cart, L_cart_colour, cv::COLOR_GRAY2BGR);
  //   // cv::imwrite("/home/epk/radar_topometric_localization/data/epk_test_plots/TEST_L.jpg", L_cart_colour);
  //   cv::Mat raw_scan_convert, raw_colour_convert;
  //   radar_polar_to_cartesian((raw_scan*255.0/2.0), azimuth_angles, raw_scan_convert, radar_resolution_, cart_resolution_, cart_pixel_width, true, CV_32F);
  //   cv::cvtColor(raw_scan_convert, raw_colour_convert, cv::COLOR_GRAY2BGR);
  //   raw_colour_convert = raw_colour_convert + L_cart_colour;
  //   cv::imwrite("/home/epk/radar_topometric_localization/data/epk_test_plots/bfar/testing/bfar_overlay_full_"+ std::to_string(frame_num) +".jpg", raw_colour_convert);
  // }
  // std::cout<<frame_num<<std::endl;
  // frame_num++;
}

template <class PointT>
void MSCA_CFAR<PointT>::run(const cv::Mat &raw_scan, const float &res,
                                 const std::vector<int64_t> &azimuth_times,
                                 const std::vector<double> &azimuth_angles,
                                 pcl::PointCloud<PointT> &pointcloud) {
  pointcloud.clear();
  const int rows = raw_scan.rows;
  const int cols = raw_scan.cols;
  if (width_ % 2 == 0) width_ += 1;
  const int w2 = std::floor(width_ / 2);
  auto mincol = minr_ / res + w2 + guard_ + 1;
  if (mincol > cols || mincol < 0) mincol = 0;
  auto maxcol = maxr_ / res - w2 - guard_;
  if (maxcol > cols || maxcol < 0) maxcol = cols;
  const int N = maxcol - mincol;

  cv::Mat raw_scan_watts_sqrd = raw_scan.clone();
  float conversion_factor = 255.0/20.0*std::log(10.0);
  raw_scan_watts_sqrd = raw_scan_watts_sqrd * conversion_factor;
  cv::exp(raw_scan_watts_sqrd, raw_scan_watts_sqrd);
  // apply square law detector to test cell
  cv::pow(raw_scan_watts_sqrd, 2, raw_scan_watts_sqrd);

  // cv::Mat L = cv::Mat::zeros(rows,cols,CV_32F);

  for (int i = 0; i < rows; ++i) {
    const double azimuth = azimuth_angles[i];
    const int64_t time = azimuth_times[i];
    pcl::PointCloud<PointT> polar_time;
    float peak_points = 0;
    int num_peak_points = 0;
    

    // cv::Mat overlay = raw_scan.row(i).clone()*0.0;

    for (int j = mincol; j < maxcol; ++j) {
      // Intensity of test cell Y
      float intensity_Y =raw_scan_watts_sqrd.at<float>(i, j);

      // There is no mention of guard cells in the MSCA paper, however I will assume we use them
      // It is also ambiguous how the sub-reference window behaves at the edges of the left and right window
      double left = 0;
      double right = 0;
      double mean = 0;

      for (int k = -w2 - guard_; k < -guard_ - M_; ++k){
        left += static_cast<double>(std::min(raw_scan_watts_sqrd.at<float>(i, j + k), raw_scan_watts_sqrd.at<float>(i, j + k + M_)));
      }

      for (int k = guard_ + 1; k <= w2 + guard_ - M_; ++k){
        right += static_cast<double>(std::min(raw_scan_watts_sqrd.at<float>(i, j + k), raw_scan_watts_sqrd.at<float>(i, j + k + M_)));
      }

      mean = (left+right) / (2 * (w2 - M_));

      const float thres = threshold_ * mean;
      // overlay.at<float>(j) = 10.0*std::log10(thres);

      if (intensity_Y > thres) {
        peak_points += j;
        num_peak_points += 1;
      } else if (num_peak_points > 0) {
        // L.at<float>(i, j) = 245.0;
        // L.at<float>(i, int(peak_points / num_peak_points)) = 245.0;


        PointT p;
        p.rho = res * peak_points / num_peak_points + static_cast<float>(range_offset_);
        p.phi = azimuth;
        p.theta = 0;
        p.timestamp = time;
        polar_time.push_back(p);
        peak_points = 0;
        num_peak_points = 0;
      }
    }

    // cv::Mat s = raw_scan.row(i).clone();
    // s.convertTo(s, CV_64F);
    // // Convert raw signal to dB
    // s = s *255;
    // cv::Mat display;
    // cv::Ptr<cv::plot::Plot2d> test_plot = cv::plot::Plot2d::create(s);
    // test_plot->setPlotSize(800, 600);
    // test_plot->setMaxX(cols);
    // test_plot->setMinX(0);
    // test_plot->setMaxY(255);
    // test_plot->setMinY(-1);
    // test_plot->setInvertOrientation(true);
    // test_plot->setGridLinesNumber(8);
    // test_plot->render(display);
    // cv::imwrite("/home/epk/radar_topometric_localization/data/epk_test_plots/msca_cfar/msca_cfar_squared_db.jpg", display);

    // overlay.convertTo(overlay, CV_64F);
    // test_plot = cv::plot::Plot2d::create(overlay);
    // test_plot->setPlotSize(800, 600);
    // test_plot->setMaxX(cols);
    // test_plot->setMinX(0);
    // test_plot->setMaxY(255);
    // test_plot->setMinY(-1);
    // test_plot->setInvertOrientation(true);
    // test_plot->setGridLinesNumber(8);
    // test_plot->render(display);
    // cv::imwrite("/home/epk/radar_topometric_localization/data/epk_test_plots/msca_cfar/msca_cfar_thresh_db.jpg", display); 

    pointcloud.insert(pointcloud.end(), polar_time.begin(), polar_time.end());
  }
  // double cart_resolution_= 0.2384;
  // double radar_resolution_ = 0.0596;
  // int cart_pixel_width = (2 * maxr_) / cart_resolution_;
  // cv::Mat L_cart;
  // cv::Mat L_cart_colour;
  // radar_polar_to_cartesian(L, azimuth_angles, L_cart, radar_resolution_, cart_resolution_, cart_pixel_width, true, CV_32F);
  // cv::cvtColor(L_cart, L_cart_colour, cv::COLOR_GRAY2BGR);
  // // cv::imwrite("/home/epk/radar_topometric_localization/data/epk_test_plots/TEST_L.jpg", L_cart_colour);
  // cv::Mat raw_scan_convert, raw_colour_convert;
  // radar_polar_to_cartesian((raw_scan*255.0/2.0), azimuth_angles, raw_scan_convert, radar_resolution_, cart_resolution_, cart_pixel_width, true, CV_32F);
  // cv::cvtColor(raw_scan_convert, raw_colour_convert, cv::COLOR_GRAY2BGR);
  // raw_colour_convert = raw_colour_convert + L_cart_colour;
  // cv::imwrite("/home/epk/radar_topometric_localization/data/epk_test_plots/msca_cfar/msca_cfar_overlay.jpg", raw_colour_convert);
}

template <class PointT>
void IS_CFAR<PointT>::run(const cv::Mat &raw_scan, const float &res,
                                 const std::vector<int64_t> &azimuth_times,
                                 const std::vector<double> &azimuth_angles,
                                 pcl::PointCloud<PointT> &pointcloud) {
  pointcloud.clear();
  const int rows = raw_scan.rows;
  const int cols = raw_scan.cols;
  if (width_ % 2 == 0) width_ += 1;
  const int w2 = std::floor(width_ / 2);
  auto mincol = minr_ / res + w2 + guard_ + 1;
  if (mincol > cols || mincol < 0) mincol = 0;
  auto maxcol = maxr_ / res - w2 - guard_;
  if (maxcol > cols || maxcol < 0) maxcol = cols;
  const int N = maxcol - mincol;

  cv::Mat raw_scan_watts_sqrd = raw_scan.clone();
  float conversion_factor = 255.0/20.0*std::log(10.0);
  raw_scan_watts_sqrd = raw_scan_watts_sqrd * conversion_factor;
  cv::exp(raw_scan_watts_sqrd, raw_scan_watts_sqrd);
  // apply square law detector to test cell
  cv::pow(raw_scan_watts_sqrd, 2, raw_scan_watts_sqrd);

  // cv::Mat L = cv::Mat::zeros(rows,cols,CV_32F);


  for (int i = 0; i < rows; ++i) {
    const double azimuth = azimuth_angles[i];
    const int64_t time = azimuth_times[i];
    pcl::PointCloud<PointT> polar_time;
    double mean = 0;
    float peak_points = 0;
    int num_peak_points = 0;

    // cv::Mat overlay = raw_scan.row(i).clone()*0.0;
    // cv::Mat detection_tracker = raw_scan.row(i).clone()*0.0;

    for (int j = mincol; j < maxcol; ++j) {
      // Intensity of test cell Y
      float intensity_Y = raw_scan_watts_sqrd.at<float>(i, j);

      double left = 0;
      double right = 0;
      double left_0 = 0;
      double right_0 = 0;
      int n_a = 0;
      int n_b = 0;
      
      for (int k = -w2 - guard_; k < -guard_; ++k){
        float intensity = raw_scan_watts_sqrd.at<float>(i, j + k);
        left += intensity;
        if(intensity < intensity_Y*alpha_I_){
          left_0 += intensity;
          n_a++;
        }
      }

      for (int k = guard_ + 1; k <= w2 + guard_; ++k){
        float intensity = raw_scan_watts_sqrd.at<float>(i, j + k);
        right += intensity;
        if(intensity < intensity_Y*alpha_I_){
          right_0 += intensity;
          n_b++;
        }
      }

      float Th = 0;
      if(n_a <= N_TI_ && n_b <= N_TI_){
        Th = beta_I_ * (left+right / (2 * w2));
      } else if(n_a > N_TI_ && n_b > N_TI_){
        Th = beta_I_ * (left_0+right_0 / (n_a+n_b));
      } else if(n_a <= N_TI_ && n_b > N_TI_){
        Th = beta_I_ * (left / w2);
      } else{
        Th = beta_I_ * (right / w2);
      }

      // overlay.at<float>(j) = 10.0*std::log10(Th);

      if (intensity_Y > Th) {
        peak_points += j;
        num_peak_points += 1;
        // detection_tracker.at<float>(j) = 10.0*std::log10(intensity_Y);

      } else if (num_peak_points > 0) {
        // L.at<float>(i, int(peak_points / num_peak_points)) = 245.0;
        PointT p;
        p.rho = res * peak_points / num_peak_points + static_cast<float>(range_offset_);
        p.phi = azimuth;
        p.theta = 0;
        p.timestamp = time;
        polar_time.push_back(p);
        peak_points = 0;
        num_peak_points = 0;
      }
    } 

    // cv::Mat s = raw_scan.row(i).clone();
    // s.convertTo(s, CV_64F);
    // // Convert raw signal to dB
    // s = s *255;
    // cv::Mat display;
    // cv::Ptr<cv::plot::Plot2d> test_plot = cv::plot::Plot2d::create(s);
    // test_plot->setPlotSize(800, 600);
    // test_plot->setMaxX(cols);
    // test_plot->setMinX(0);
    // test_plot->setMaxY(255);
    // test_plot->setMinY(-1);
    // test_plot->setInvertOrientation(true);
    // test_plot->setGridLinesNumber(8);
    // test_plot->render(display);
    // cv::imwrite("/home/epk/radar_topometric_localization/data/epk_test_plots/is_cfar/iscfar_squared_db.jpg", display);

    // overlay.convertTo(overlay, CV_64F);
    // test_plot = cv::plot::Plot2d::create(overlay);
    // test_plot->setPlotSize(800, 600);
    // test_plot->setMaxX(cols);
    // test_plot->setMinX(0);
    // test_plot->setMaxY(255);
    // test_plot->setMinY(-1);
    // test_plot->setInvertOrientation(true);
    // test_plot->setGridLinesNumber(8);
    // test_plot->render(display);
    // cv::imwrite("/home/epk/radar_topometric_localization/data/epk_test_plots/is_cfar/iscfar_thresh_db.jpg", display); 

    // detection_tracker.convertTo(detection_tracker, CV_64F);
    // test_plot = cv::plot::Plot2d::create(detection_tracker);
    // test_plot->setPlotSize(800, 600);
    // test_plot->setMaxX(cols);
    // test_plot->setMinX(0);
    // test_plot->setMaxY(255);
    // test_plot->setMinY(-1);
    // test_plot->setInvertOrientation(true);
    // test_plot->setGridLinesNumber(8);
    // test_plot->render(display);
    // cv::imwrite("/home/epk/radar_topometric_localization/data/epk_test_plots/is_cfar/iscfar_detections_db.jpg", display); 

    pointcloud.insert(pointcloud.end(), polar_time.begin(), polar_time.end());
  }
  // double cart_resolution_= 0.2384;
  // double radar_resolution_ = 0.0596;
  // int cart_pixel_width = (2 * maxr_) / cart_resolution_;
  // cv::Mat L_cart;
  // cv::Mat L_cart_colour;
  // radar_polar_to_cartesian(L, azimuth_angles, L_cart, radar_resolution_, cart_resolution_, cart_pixel_width, true, CV_32F);
  // cv::cvtColor(L_cart, L_cart_colour, cv::COLOR_GRAY2BGR);
  // cv::Mat raw_scan_convert, raw_colour_convert;
  // radar_polar_to_cartesian((raw_scan*255.0/2.0), azimuth_angles, raw_scan_convert, radar_resolution_, cart_resolution_, cart_pixel_width, true, CV_32F);
  // cv::cvtColor(raw_scan_convert, raw_colour_convert, cv::COLOR_GRAY2BGR);
  // raw_colour_convert = raw_colour_convert + L_cart_colour;
  // cv::imwrite("/home/epk/radar_topometric_localization/data/epk_test_plots/is_cfar/is_cfar_overlay.jpg", raw_colour_convert);
}

template <class PointT>
void VI_CFAR<PointT>::run(const cv::Mat &raw_scan, const float &res,
                                 const std::vector<int64_t> &azimuth_times,
                                 const std::vector<double> &azimuth_angles,
                                 pcl::PointCloud<PointT> &pointcloud) {
  pointcloud.clear();
  const int rows = raw_scan.rows;
  const int cols = raw_scan.cols;
  if (width_ % 2 == 0) width_ += 1;
  const int w2 = std::floor(width_ / 2);
  auto mincol = minr_ / res + w2 + guard_ + 1;
  if (mincol > cols || mincol < 0) mincol = 0;
  auto maxcol = maxr_ / res - w2 - guard_;
  if (maxcol > cols || maxcol < 0) maxcol = cols;
  const int N = maxcol - mincol; 

  float conversion_factor = 255.0/20.0*std::log(10.0);
  cv::Mat raw_scan_watts_sqrd = raw_scan * conversion_factor;
  cv::exp(raw_scan_watts_sqrd, raw_scan_watts_sqrd);
  // apply square law detector to test cell
  cv::pow(raw_scan_watts_sqrd, 2, raw_scan_watts_sqrd);

  // cv::Mat L = cv::Mat::zeros(rows,cols,CV_32F);

  for (int i = 0; i < rows; ++i) {
    const double azimuth = azimuth_angles[i];
    const int64_t time = azimuth_times[i];
    pcl::PointCloud<PointT> polar_time;
    double mean = 0;
    float peak_points = 0;
    int num_peak_points = 0;

    // cv::Mat overlay = raw_scan.row(i).clone()*0.0;
    

    // double left_test = 0;
    // double right_test = 0;
    // double mean_test = 0;

    // double VI_simplified_leading = 0.0;
    // double VI_simplified_lagging = 0.0;
    // double MR = 0.0;

    // double left_vi_sqrd = 0.0;
    // double right_vi_sqrd = 0.0;

    // for (int k = -w2 - guard_; k < -guard_; ++k){
    //   double intensity = raw_scan_watts_sqrd.at<double>(i, mincol-1 + k);
    //   left_test += intensity;
    //   left_vi_sqrd = left_vi_sqrd + std::pow(intensity,2);;
    // }

    // for (int k = guard_ + 1; k <= w2 + guard_; ++k){
    //   double intensity = raw_scan_watts_sqrd.at<double>(i, mincol-1 + k);
    //   right_test += intensity;
    //   right_vi_sqrd = right_vi_sqrd + std::pow(intensity,2);
    // }


    // std::cout<<"new azimuth, initial right sum: " <<right_vi_sqrd<<std::endl;

    double thresh_1=0;
    double thresh_2=0;
    double thresh_3=0;
    double thresh_4=0;
    double thresh_5=0;
    for (int j = mincol; j < maxcol; ++j) {
      bool variable_leading = false;
      bool variable_lagging = false;
      bool different_means = false;

      // Intensity of test cell Y with sqaure law
      float intensity_Y= raw_scan_watts_sqrd.at<float>(i, j);

      // double left_old = raw_scan_watts_sqrd.at<double>(i, j - w2 - guard_ - 1);
      // double left_new = raw_scan_watts_sqrd.at<double>(i, j - guard_ - 1);
      // double right_old = raw_scan_watts_sqrd.at<double>(i, j + guard_);
      // double right_new = raw_scan_watts_sqrd.at<double>(i, j + w2 + guard_);

      // left_test = left_test - left_old + left_new;
      // right_test = right_test - right_old + right_new;

      // left_vi_sqrd = left_vi_sqrd - std::pow(left_old,2) + std::pow(left_new,2);
      // right_vi_sqrd = right_vi_sqrd - std::pow(right_old,2) + std::pow(right_new,2);

      // double sum_comp = 0.0;
      // for (int k = guard_ + 1; k <= w2 + guard_; ++k){
      //   double intensity = raw_scan_watts_sqrd.at<double>(i, j + k);
      //   sum_comp = sum_comp + std::pow(intensity,2);
      // }

      // std::cout<<"sum comp: "<<sum_comp<< " old sum: "<<right_vi_sqrd<< std::endl;

      double left_test = 0.0;
      double right_test = 0.0;
      double left_vi_sqrd = 0.0;
      double right_vi_sqrd = 0.0;

      double VI_simplified_leading = 0.0;
      double VI_simplified_lagging = 0.0;
      double MR = 0.0;

      for (int k = -w2 - guard_; k < -guard_; ++k){
        double intensity = static_cast<double>(raw_scan_watts_sqrd.at<float>(i, j + k));
        left_test += intensity;
        left_vi_sqrd += std::pow(intensity,2);;
      }

      for (int k = guard_ + 1; k <= w2 + guard_; ++k){
        double intensity = static_cast<double>(raw_scan_watts_sqrd.at<float>(i, j + k));
        right_test += intensity;
        right_vi_sqrd += std::pow(intensity,2);
      }

      VI_simplified_leading = 2*w2*(right_vi_sqrd)/(right_test*right_test);
      VI_simplified_lagging = 2*w2*(left_vi_sqrd)/(left_test*left_test);
      
      MR = left_test/right_test;

      if(VI_simplified_leading > K_VI_){variable_leading = true;}
      if(VI_simplified_lagging > K_VI_){variable_lagging = true;}

      if(1.0/K_MR_ > MR || MR > K_MR_){different_means=true;}

      float threshold = 0.0;

      // mean_test = (left_test+right_test) / (2 * w2);

      // std::cout<<"constants K_VI: "<<K_VI_<< " Test val of right_vi_sqrd: "<< right_vi_sqrd<< " right old srd: "<<right_old*right_old<<" right new sqrd: "<< right_new*right_new <<" VI lead: "<<VI_simplified_leading<<std::endl;

      // std::cout<<"constants K_MR: " << K_MR_<< " K_VI:  "<<K_VI_<< " MR: "<<MR<<" VI lead: "<<VI_simplified_leading<< " VI_lag: "<<VI_simplified_lagging<<std::endl;

      if(!variable_leading && !variable_lagging && !different_means){
        threshold = C_N_ * (left_test+right_test) / (2*w2);
        thresh_1++;
      }
      else if(!variable_leading && !variable_lagging && different_means){
        threshold = C_N_ * std::max(left_test,right_test) / w2;
        thresh_2++;
      }
      else if(variable_leading && !variable_lagging){
        threshold = C_N_ * right_test / w2;
        thresh_3++;
      }
      else if(!variable_leading && variable_lagging){
        threshold = C_N_ * left_test / w2;
        thresh_4++;
      }
      else if(variable_leading && variable_lagging){
        threshold = C_N_ * std::min(left_test,right_test) / w2;
        thresh_5++;
      }

      // overlay.at<float>(j) = 10.0*std::log10(threshold);

      if (intensity_Y > threshold) {
        peak_points += j;
        num_peak_points += 1;
      } else if (num_peak_points > 0) {
        // L.at<float>(i, int(peak_points / num_peak_points)) = 245.0;

        PointT p;
        p.rho = res * peak_points / num_peak_points + static_cast<float>(range_offset_);
        p.phi = azimuth;
        p.theta = 0;
        p.timestamp = time;
        polar_time.push_back(p);
        peak_points = 0;
        num_peak_points = 0;
      }
    } 
    // thresh_1=(thresh_1/N);
    // thresh_2=(thresh_2/N);
    // thresh_3=(thresh_3/N);
    // thresh_4=(thresh_4/N);
    // thresh_5=(thresh_5/N);
    // std::cout<<"% CA: "<<thresh_1<<" % CAGO: "<< (thresh_2)<< " % VI Lead: "<<(thresh_3)<<" % VI Lagg: "<<(thresh_4)<<" % CASO: "<<(thresh_5)<<std::endl;

    // cv::Mat s = raw_scan.row(i).clone();
    // s.convertTo(s, CV_64F);
    // // Convert raw signal to dB
    // s = s *255;
    // cv::Mat display;
    // cv::Ptr<cv::plot::Plot2d> test_plot = cv::plot::Plot2d::create(s);
    // test_plot->setPlotSize(800, 600);
    // test_plot->setMaxX(cols);
    // test_plot->setMinX(0);
    // test_plot->setMaxY(255);
    // test_plot->setMinY(-1);
    // test_plot->setInvertOrientation(true);
    // test_plot->setGridLinesNumber(8);
    // test_plot->render(display);
    // cv::imwrite("/home/epk/radar_topometric_localization/data/epk_test_plots/vi_cfar/vi_cfar_squared_db.jpg", display);

    // overlay.convertTo(overlay, CV_64F);
    // test_plot = cv::plot::Plot2d::create(overlay);
    // test_plot->setPlotSize(800, 600);
    // test_plot->setMaxX(cols);
    // test_plot->setMinX(0);
    // test_plot->setMaxY(255);
    // test_plot->setMinY(-1);
    // test_plot->setInvertOrientation(true);
    // test_plot->setGridLinesNumber(8);
    // test_plot->render(display);
    // cv::imwrite("/home/epk/radar_topometric_localization/data/epk_test_plots/vi_cfar/vi_cfar_thresh_db.jpg", display); 


    pointcloud.insert(pointcloud.end(), polar_time.begin(), polar_time.end());
  }
  // double cart_resolution_= 0.2384;
  // double radar_resolution_ = 0.0596;
  // int cart_pixel_width = (2 * maxr_) / cart_resolution_;
  // cv::Mat L_cart;
  // cv::Mat L_cart_colour;
  // radar_polar_to_cartesian(L, azimuth_angles, L_cart, radar_resolution_, cart_resolution_, cart_pixel_width, true, CV_32F);
  // cv::cvtColor(L_cart, L_cart_colour, cv::COLOR_GRAY2BGR);
  // // cv::imwrite("/home/epk/radar_topometric_localization/data/epk_test_plots/TEST_L.jpg", L_cart_colour);
  // cv::Mat raw_scan_convert, raw_colour_convert;
  // radar_polar_to_cartesian((raw_scan*255.0/2.0), azimuth_angles, raw_scan_convert, radar_resolution_, cart_resolution_, cart_pixel_width, true, CV_32F);
  // cv::cvtColor(raw_scan_convert, raw_colour_convert, cv::COLOR_GRAY2BGR);
  // raw_colour_convert = raw_colour_convert + L_cart_colour;
  // cv::imwrite("/home/epk/radar_topometric_localization/data/epk_test_plots/vi_cfar/vi_cfar_overlay.jpg", raw_colour_convert);
}


template <class PointT>
void LandmarkExtraction<PointT>::run(const cv::Mat &raw_scan, const float &res,
                                 const std::vector<int64_t> &azimuth_times,
                                 const std::vector<double> &azimuth_angles,
                                 pcl::PointCloud<PointT> &pointcloud) {
  pointcloud.clear();
  const int rows = raw_scan.rows;
  const int cols = raw_scan.cols;
  if (width_ % 2 == 0) width_ += 1;
  const int w2 = std::floor(width_ / 2);
  auto mincol = minr_ / res + w2 + guard_ + 1;
  if (mincol > cols || mincol < 0) mincol = 0;
  auto maxcol = maxr_ / res - w2 - guard_;
  if (maxcol > cols || maxcol < 0) maxcol = cols;
  const int N = maxcol - mincol;

  for (int i = 0; i < rows; ++i) {
    const double azimuth = azimuth_angles[i];
    const int64_t time = azimuth_times[i];
    pcl::PointCloud<PointT> polar_time;
    float peak_points = 0;
    int num_peak_points = 0;


    cv::Mat s = raw_scan.row(i).clone();
    s.convertTo(s, CV_64F);
    // Convert raw signal to dB
    s = s *255/2;

    cv::Mat display;
    cv::Ptr<cv::plot::Plot2d> test_plot = cv::plot::Plot2d::create(s);
    test_plot->setPlotSize(800, 600);
    test_plot->setMaxX(cols);
    test_plot->setMinX(0);
    test_plot->setMaxY(128);
    test_plot->setMinY(-1);
    test_plot->setInvertOrientation(true);
    test_plot->setGridLinesNumber(8);
    test_plot->render(display);
    cv::imwrite("/home/epk/radar_topometric_localization/data/epk_test_plots/test1.jpg", display);

    cv::Mat scaled_signal = s.clone();

    for(int j = 0; j < rows; ++j){scaled_signal.at<double>(j) = std::pow(10, scaled_signal.at<double>(j)/10.0);} 

    cv::Mat scaled;
    cv::normalize(scaled_signal,scaled,0,255);

    test_plot = cv::plot::Plot2d::create(scaled);
    test_plot->setPlotSize(800, 600);
    test_plot->setMaxX(cols);
    test_plot->setMinX(0);
    test_plot->setMaxY(255);
    test_plot->setMinY(-1);
    test_plot->setInvertOrientation(true);
    test_plot->setGridLinesNumber(8);
    test_plot->render(display);
    cv::imwrite("/home/epk/radar_topometric_localization/data/epk_test_plots/scaled_signal.jpg", display);


    // cv::waitKey();

    // q is the unbiased signal
    // std::vector<double> q(cols);
    cv::Mat q;

    // p is the smoothed signal
    // std::vector<double> p(cols);
    cv::Mat p;

    // Q is the set of points in q below 0
    std::vector<double> Q;
    // cv::Mat Q;

    // y_hat is the estimated signal without noise
    std::vector<double> y_hat(cols);

    // sig_q is the standard deviation of guassian noise of Q
    double sig_q = 0;
    double q_count = 0;

    // q = s - medianFilter(s, wmedian);
    cv::Mat temp; // = s.clone();

    // for(int j = 0; j < cols; ++j) {
    for(int j = mincol; j < maxcol; ++j) {
        // Calculate the starting and ending indices for the current kernel
        int startIdx = std::max(j - w_median_ / 2, 0);
        int endIdx = std::min(j + w_median_ / 2, s.cols - 1);

        // Copy the values within the kernel to a temporary vector
        std::vector<double> values;
        for (int k = startIdx; k <= endIdx; ++k) {values.push_back(s.at<double>(k));}
        // Sort the values to find the median
        std::sort(values.begin(), values.end());

        // Assign the median value to the output matrix
        if(values.size() % 2 == 1){temp.at<double>(j) = values[values.size() / 2];}
        else{temp.at<double>(j) = (values[values.size() / 2 - 1] + values[values.size() / 2]) / 2 ;}
    }

    q = s - temp;

    test_plot = cv::plot::Plot2d::create(q);
    test_plot->setPlotSize(800, 600);
    test_plot->setMaxX(cols);
    test_plot->setMinX(0);
    test_plot->setMaxY(128);
    test_plot->setMinY(-1);
    test_plot->setInvertOrientation(true);
    test_plot->setGridLinesNumber(8);
    test_plot->render(display);
    cv::imwrite("/home/epk/radar_topometric_localization/data/epk_test_plots/test2.jpg", display);

    // for(int j = mincol; j < maxcol; ++j){q[j] -= w_median_;} 
    
    // p = binomialFilter(s, wbinom);
    // cv::binomialFilter(q, p, w_binom_);
    // cv::GaussianBlur(q, p, cv::Size(w_binom_,1),0);
    // for(int j = mincol; j < maxcol; ++j){q[j] -= w_binom_;} 
    // Output matrix
    // p = q.clone();

    // cv::Mat kernel = cv::getGaussianKernel(w_binom_, -1, CV_64F);

    // // Perform 1D convolution
    // cv::filter2D(q, p, -1, kernel, cv::Point(-1, -1), 0, cv::BORDER_DEFAULT);
    const double sigma_test = 8.0;
    for(int j = mincol; j < maxcol; ++j){
      double val = q.at<double>(j);
      if(val < 0){
        //  Q.push_back(val);
        //  Q.push_back(-val);
        q_count += 2;
        sig_q += 2*val*val;
      }
    }
    // GaussianBlur(q,p,cv::Size(1,w_binom_),0);
    GaussianBlur(q,p,cv::Size(1,w_binom_),sigma_test);

    

    test_plot = cv::plot::Plot2d::create(p);
    test_plot->setPlotSize(800, 600);
    test_plot->setMaxX(cols);
    test_plot->setMinX(0);
    test_plot->setMaxY(128);
    test_plot->setMinY(-1);
    test_plot->setInvertOrientation(true);
    test_plot->setGridLinesNumber(8);
    test_plot->render(display);
    cv::imwrite("/home/epk/radar_topometric_localization/data/epk_test_plots/test3.jpg", display);


    // std::vector<int> binomialCoeffs;
    // for (int k = 0; k <= w_binom_; ++k) {
    //     int coeff = 1;
    //     for (int l = 1; l <= k; ++l) {
    //         coeff *= (w_binom_ - l + 1);
    //         coeff = coeff / l;
    //     }
    //     binomialCoeffs.push_back(coeff);
    // }

    

    // // Iterate through each element of the input matrix
    // // for (int j = 0; j < cols; ++j) {
    // for (int j = mincol; j < maxcol; ++j) {
    //     // Calculate the starting and ending indices for the current kernel
    //     int startIdx = std::max(j - w_binom_ / 2, 0);
    //     int endIdx = std::min(j + w_binom_ / 2, q.cols - 1);

    //     // Calculate weighted average using binomial coefficients
    //     double sum = 0;
    //     for (int k = startIdx, l = 0; k <= endIdx; ++k, ++l) {sum += q.at<double>(k) * binomialCoeffs[l];}

    //     // Assign the averaged value to the output matrix
    //     p.at<double>(j) = sum / (w_binom_ + 1);

    //     // // Testing efficency
    //     // double val = q.at<double>(j);
    //     // if(val < 0){
    //     //   // Q.push_back(val);
    //     //   // Q.push_back(-val);
    //     //   q_count += 2;
    //     //   sig_q += 2*val*val;
    //     // }
    // }



    test_plot = cv::plot::Plot2d::create(p);
    test_plot->setPlotSize(800, 600);
    test_plot->setMaxX(cols);
    test_plot->setMinX(0);
    test_plot->setMaxY(128);
    test_plot->setMinY(-1);
    test_plot->setInvertOrientation(true);
    test_plot->setGridLinesNumber(8);
    test_plot->render(display);
    cv::imwrite("/home/epk/radar_topometric_localization/data/epk_test_plots/test3_5.jpg", display);

    // Q = {qi : qi ≤ 0}
    // for(int j = mincol; j < maxcol; ++j){
    //   double val = q.at<double>(j);
    //   if(val < 0){
    //     //  Q.push_back(val);
    //     //  Q.push_back(-val);
    //     q_count += 2;
    //     sig_q += 2*val*val;
    //   }
    // }

    // N (mu_q, sig_q^2) = normalDistribution(Q union -Q)
    // cv::Scalar mu_q, sig_q;
    // cv::meanStdDev(Q, mu_q, sig_q);
    // cv::Scalar tempVal = cv::mean( myMat );
    // float myMAtMean = tempVal.val[0];

    // for(auto &val : Q){sig_q += val*val;}
    // sig_q = std::sqrt(sig_q / Q.size());
    sig_q = std::sqrt(sig_q / q_count);



    // Initialize N * 1 vector y_hat to zeros.
    // for i = 1 to N do
    for(int j = mincol; j < maxcol; ++j){
      // if qi > 0 then
      if(q.at<double>(j) > 0){

        // y_i = pi *(1 - f(pi|0,sig_q^2)/f(0|0,sig_q^2))
        y_hat[j] = p.at<double>(j) * (1 - std::exp(-pow(p.at<double>(j),2) / (2 * sig_q * sig_q)));
        // std::cout<<y_hat[j]<<std::endl;
        // y_i = y_i + (qi - pi) *1 - f(qi-pi|0,sig_q^2)/f(0|0,sig_q^2)
        y_hat[j] += (q.at<double>(j) - p.at<double>(j)) * (1 - std::exp(pow((p.at<double>(j) - q.at<double>(j)),2) / (2 * sig_q * sig_q)));
        // std::cout<<y_hat[j]<<std::endl;

        // Threshold y_hat_i values below z_qsig_q.
        if(y_hat[j] < z_q_*sig_q){y_hat[j] = 0;}
      }
    }


    cv::Mat y(y_hat.size(), 1, CV_64F);
    
    // Copy the data from the vector to the cv::Mat
    for (int i = 0; i < cols; ++i) {
        y.at<double>(i, 0) = y_hat[i];
    }

    test_plot = cv::plot::Plot2d::create(y);
    test_plot->setPlotSize(800, 600);
    test_plot->setMaxX(cols);
    test_plot->setMinX(0);
    test_plot->setMaxY(128);
    test_plot->setMinY(-1);
    test_plot->setInvertOrientation(true);
    test_plot->setGridLinesNumber(8);
    test_plot->render(display);
    cv::imwrite("/home/epk/radar_topometric_localization/data/epk_test_plots/test4.jpg", display);

    for(int j = mincol; j < maxcol; ++j) {
      // L(s) ← {(a, r(i)) : ˆyi > 0 ∩ yˆi+1 > 0 ∩ yˆi−1 = 0}
      if(y_hat[j] > 0 && y_hat[j+1] > 0 && y_hat[j-1] == 0) {
        peak_points += j;
        num_peak_points += 1;
      } else if (num_peak_points > 0) {
        PointT p;
        p.rho = res * peak_points / num_peak_points + range_offset_;
        p.phi = azimuth;
        p.theta = 0;
        p.timestamp = time;
        polar_time.push_back(p);
        peak_points = 0;
        num_peak_points = 0;
      }

      // If F, remove multipath reflections in L(s) using dthresh
    } 
    pointcloud.insert(pointcloud.end(), polar_time.begin(), polar_time.end());
  }
}


template <class PointT>
void SURFExtraction<PointT>::run(const cv::Mat &raw_scan, const float &res,
                                 const std::vector<int64_t> &azimuth_times,
                                 const std::vector<double> &azimuth_angles,
                                 pcl::PointCloud<PointT> &pointcloud) {
  pointcloud.clear();
  const int rows = raw_scan.rows;
  const int cols = raw_scan.cols;
  if (width_ % 2 == 0) width_ += 1;
  const int w2 = std::floor(width_ / 2);
  auto mincol = minr_ / res + w2 + guard_ + 1;
  if (mincol > cols || mincol < 0) mincol = 0;
  auto maxcol = maxr_ / res - w2 - guard_;
  if (maxcol > cols || maxcol < 0) maxcol = cols;
  const int N = maxcol - mincol;



  cv::Mat s = raw_scan.clone();
  // Convert raw signal to dB
  s = s *255;
  s.convertTo(s, CV_8U);


  // SIFT OR SURF
  // cv::Mat raw_scan_dB_cart;
  // // dont know int cart_pixel_width = 640, bool fix_wobble = true)
  // double cart_resolution_= 0.2384;
  // double radar_resolution_ = 0.0596;
  // int cart_pixel_width = (2 * maxr_) / cart_resolution_;
  // // radar_polar_to_cartesian(s, azimuth_angles, raw_scan_dB_cart, radar_resolution_, cart_resolution_, cart_pixel_width, true, CV_8U);
  // // cv::imwrite("/home/epk/radar_topometric_localization/data/epk_test_plots/sift_raw.jpg", s);

  // cv::Ptr<cv::SIFT> siftPtr = cv::SIFT::create();
  // std::vector<cv::KeyPoint> keypoints;
  // siftPtr->detect(s, keypoints);

  // // Add results to image and save.
  // cv::Mat output;
  // cv::drawKeypoints(s, keypoints, output);

  // // radar_polar_to_cartesian(output, azimuth_angles, output, radar_resolution_, cart_resolution_, cart_pixel_width, true, CV_8U);


  // // cv::imwrite("/home/epk/radar_topometric_localization/data/epk_test_plots/surf_test.jpg", output);



  // Harris corner detector
  int thresh = 200;
  
  int blockSize = 2;
  int apertureSize = 3;
  double k = 0.04;
  cv::Mat dst = cv::Mat::zeros( s.size(), CV_32FC1 );
  cv::cornerHarris( s, dst, blockSize, apertureSize, k );
  cv::Mat dst_norm, dst_norm_scaled;
  cv::normalize( dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat() );
  cv::convertScaleAbs( dst_norm, dst_norm_scaled );
  for( int i = 0; i < dst_norm.rows ; i++ )
  {
  for( int j = 0; j < dst_norm.cols; j++ )
  {
  if( (int) dst_norm.at<float>(i,j) > thresh )
  {
  cv::circle( dst_norm_scaled, cv::Point(j,i), 5, cv::Scalar(0), 2, 8, 0 );
  }
  }
  }

  cv::Mat output;

  // namedWindow( corners_window );
  // imshow( corners_window, dst_norm_scaled );
  // cv::drawKeypoints(corners_window, dst_norm_scaled, output);

  cv::imwrite("/home/epk/radar_topometric_localization/data/epk_test_plots/harris_test.jpg", dst_norm_scaled);




  // for(auto& point : keypoints){
  //   int i = point.pt.x;
  //   int j = point.pt.y;
  //   const double azimuth = azimuth_angles[i];
  //   const int64_t time = azimuth_times[i];
  //   pcl::PointCloud<PointT> polar_time;
  //   float peak_points = 0;
  //   int num_peak_points = 0;

  //   PointT p;
  //   p.rho = res * peak_points / num_peak_points + range_offset_;
  //   p.phi = azimuth;
  //   p.theta = 0;
  //   p.timestamp = time;
  //   polar_time.push_back(p);
  //   peak_points = 0;
  //   num_peak_points = 0;

  //   pointcloud.insert(pointcloud.end(), polar_time.begin(), polar_time.end());
  // }

}


template <class PointT>
void Cen2019<PointT>::run(const cv::Mat &raw_scan, const float &res,
                                 const std::vector<int64_t> &azimuth_times,
                                 const std::vector<double> &azimuth_angles,
                                 pcl::PointCloud<PointT> &pointcloud) {

                                  
  pointcloud.clear();
  const int rows = raw_scan.rows;
  const int cols = raw_scan.cols;
  if (width_ % 2 == 0) width_ += 1;
  const int w2 = std::floor(width_ / 2);
  auto mincol = minr_ / res + w2 + guard_ + 1;
  if (mincol > cols || mincol < 0) mincol = 0;
  auto maxcol = maxr_ / res - w2 - guard_;
  if (maxcol > cols || maxcol < 0) maxcol = cols;
  const int N = maxcol - mincol;

  // const cv::Mat raw_scan_dB = raw_scan.clone()*255.0/2.0;

  // This acctually shouldnt be in dB but instead pixel values from 0-255
  // const cv::Mat raw_scan_dB = raw_scan.clone()*255.0;

  const cv::Mat raw_scan_dB = raw_scan.clone()*255.0/2.0;

  cv::Mat raw_scan_dB_cart;

  // dont know int cart_pixel_width = 640, bool fix_wobble = true)
  double cart_resolution_= 0.2384;
  double radar_resolution_ = 0.0596;
  int cart_pixel_width = (2 * maxr_) / cart_resolution_;
  // radar_polar_to_cartesian(raw_scan_dB, azimuth_angles, raw_scan_dB_cart, radar_resolution_, cart_resolution_, cart_pixel_width, true, CV_32F);
  // cv::imwrite("/home/epk/radar_topometric_localization/data/epk_test_plots/cen2019/2019_raw.jpg", raw_scan_dB_cart);

  cv::Mat grad_x;
  cv::Mat filt_x = (cv::Mat_<float>(3,3) << 1, 1, 1, 0, 0, 0, -1, -1, -1);
  cv::filter2D(raw_scan_dB, grad_x, -1, filt_x, cv::Point(-1, -1), 0, cv::BORDER_REFLECT101);

  // cv::imwrite("/home/epk/radar_topometric_localization/data/epk_test_plots/grad_x.jpg", grad_x);

  cv::Mat grad_y;
  cv::Mat filt_y = (cv::Mat_<float>(3,3) << 1, 0, -1, 1, 0, -1, 1, 0, -1);
  cv::filter2D(raw_scan_dB, grad_y, -1, filt_y, cv::Point(-1, -1), 0, cv::BORDER_REFLECT101);

  // cv::imwrite("/home/epk/radar_topometric_localization/data/epk_test_plots/grad_y.jpg", grad_y);

  cv::Mat grad_mag;
  cv::pow(grad_x,2,grad_x);
  cv::pow(grad_y,2,grad_y);
  cv::sqrt(grad_x + grad_y, grad_mag);
  cv::normalize(grad_mag, grad_mag, 0, 1, cv::NORM_MINMAX);

  cv::Mat s_prime = raw_scan_dB - cv::mean(raw_scan_dB)[0];

  cv::Mat H = (1-grad_mag).mul(s_prime);

  // cv::Mat H_cart;
  // radar_polar_to_cartesian(H, azimuth_angles, H_cart, radar_resolution_, cart_resolution_, cart_pixel_width, true, CV_32F);
  // cv::imwrite("/home/epk/radar_topometric_localization/data/epk_test_plots/cen2019/H_post_cart.jpg", H_cart);

  // cv::Mat L_points = cv::Mat::zeros(rows,cols,CV_32F);
  

  // sortIdx can only sort by col or row so we must convert H to a single column matrix
  cv::Mat I = H.clone();
  I = I.reshape(1, rows * cols);
  cv::sortIdx(I, I, cv::SORT_EVERY_COLUMN  + cv::SORT_DESCENDING);

  int l = 0;
  int curr_idx = 0;
  int check_points_count = 0;
  int max_bounded_points = rows * (maxcol-mincol+1);

  // Treat 1 as False and 0 as True
  // Counter intuitive but allows the use of cv::hasNonZero which will be used to check if there are and "False" left in R
  cv::Mat R = cv::Mat::ones(rows,cols,CV_32F);
  std::vector<std::tuple<int, int, int>> Q_regions;

  std::unordered_map<int, std::vector<std::pair<int,int>>> Q_Map;

  while(l < l_max_ && check_points_count <  max_bounded_points){
    // Convert linear index to (a,r)
    int raw_idx = I.at<int>(curr_idx);
    int a = raw_idx / cols;
    int r = raw_idx % cols;
    curr_idx++;

    if(r>maxcol || r<mincol){continue;}

    // where R(a, r) == false is the same as R(a, r) == 1
    if(R.at<float>(a,r) == 1.0){
      int r_low = r;
      int r_high = r;

      // bool new_region = false;
      bool r_low_found = false;
      bool r_high_found = false;

      // Search s_prime along a for the closest range indices (rlow, rhigh) below and above r with values less than 0
      for(int k = r-1; k >= mincol; --k){
        if(s_prime.at<float>(a,k) < 0.0){
          r_low = k+1;
          r_low_found = true;
          break;
        }
      }
      if(!r_low_found){r_low = mincol;}

      for(int k = r+1; k < maxcol; ++k){
        if(s_prime.at<float>(a,k) < 0.0){
          r_high = k;
          r_high_found = true;
          break;
        }
      }
      if(!r_high_found){r_high = maxcol-1;}

      // cv::Mat R_slice = cv::Mat(R, cv::Range(a,a+1), cv::Range(r_low,r_high));
      cv::Mat R_slice = cv::Mat(R, cv::Range(a,a+1), cv::Range(r_low,r_high+1));

      // if none in R[a, rlow : rhigh] then
      if(cv::countNonZero(R_slice) == r_high-r_low+1){
        Q_Map[a].push_back(std::make_pair(r_low, r_high));
        // L_points.at<float>(a,r) =255;
        l++;
      }
      for(int k = r_low-1; k <= r_high; ++k){
        R.at<float>(a,k) = 0.0;
        check_points_count++;  
      }
    }
  }

  // cv::Mat L_points_cart, L_points_cart_colour;
  // radar_polar_to_cartesian(L_points, azimuth_angles, L_points_cart, radar_resolution_, cart_resolution_, cart_pixel_width, true, CV_32F);
  // cv::cvtColor(L_points_cart, L_points_cart_colour, cv::COLOR_GRAY2BGR);
  // cv::Mat raw_scan_convert1, raw_colour_convert1;
  // radar_polar_to_cartesian(raw_scan_dB, azimuth_angles, raw_scan_convert1, radar_resolution_, cart_resolution_, cart_pixel_width, true, CV_32F);
  // cv::cvtColor(raw_scan_convert1, raw_colour_convert1, cv::COLOR_GRAY2BGR);
  // raw_colour_convert1 = raw_colour_convert1 + L_points_cart_colour;
  // cv::imwrite("/home/epk/radar_topometric_localization/data/epk_test_plots/cen2019/cen2019_l_points_overlay.jpg", raw_colour_convert1);

  // std::cout<<"l reached: "<<l<<std::endl;
  // cv::Mat print_R = R.clone();
  // cv::Mat R_cart;
  // radar_polar_to_cartesian(print_R*255, azimuth_angles, R_cart, radar_resolution_, cart_resolution_, cart_pixel_width, true, CV_32F);
  // cv::imwrite("/home/epk/radar_topometric_localization/data/epk_test_plots/cen2019/mask.jpg", R_cart);
  

  // cv::Mat L = cv::Mat::zeros(rows,cols,CV_32F);

  // MAP alternative
  // For every azimuth
  for(auto& [a,ranges] : Q_Map){
    const double azimuth = azimuth_angles[a];
    const int64_t time = azimuth_times[a];
    pcl::PointCloud<PointT> polar_time;

    // For every region in this azimuth
    for(auto& pair : ranges){
      int r_low = pair.first;
      int r_high = pair.second;

      cv::Mat Q_minus, Q_plus;
      if(a == 0){
        Q_minus = cv::Mat(R, cv::Range(rows-1,rows), cv::Range(r_low,r_high));
        Q_plus = cv::Mat(R, cv::Range(a+1,a+2), cv::Range(r_low,r_high));
      } else if(a == rows-1){
        Q_minus = cv::Mat(R, cv::Range(a-1,a), cv::Range(r_low,r_high));
        Q_plus = cv::Mat(R, cv::Range(0,1), cv::Range(r_low,r_high));
      } else {
        Q_minus = cv::Mat(R, cv::Range(a-1,a), cv::Range(r_low,r_high));
        Q_plus = cv::Mat(R, cv::Range(a+1,a+2), cv::Range(r_low,r_high));
      }

      int range = r_high - r_low;
      if(cv::countNonZero(Q_minus) < range || cv::countNonZero(Q_plus) < range){
        cv::Mat H_vals = cv::Mat(H, cv::Range(a,a+1), cv::Range(r_low,r_high));
        double maxVal;
        int maxIdx[2];
        cv::minMaxIdx(H_vals, 0, &maxVal, 0, maxIdx);
        int max_r_idx = maxIdx[1] + r_low;

        // L.at<float>(a,max_r_idx) =255;

        PointT p;
        p.rho = res * max_r_idx + static_cast<float>(range_offset_);
        p.phi = azimuth;
        p.theta = 0;
        p.timestamp = time;
        polar_time.push_back(p);
      }
    }
    pointcloud.insert(pointcloud.end(), polar_time.begin(), polar_time.end());
  }

  // cv::Mat L_cart, L_cart_colour;
  // radar_polar_to_cartesian(L, azimuth_angles, L_cart, radar_resolution_, cart_resolution_, cart_pixel_width, true, CV_32F);
  // cv::cvtColor(L_cart, L_cart_colour, cv::COLOR_GRAY2BGR);
  // // cv::imwrite("/home/epk/radar_topometric_localization/data/epk_test_plots/TEST_L.jpg", L_cart_colour);
  // cv::Mat raw_scan_convert, raw_colour_convert;
  // radar_polar_to_cartesian(raw_scan_dB, azimuth_angles, raw_scan_convert, radar_resolution_, cart_resolution_, cart_pixel_width, true, CV_32F);
  // cv::cvtColor(raw_scan_convert, raw_colour_convert, cv::COLOR_GRAY2BGR);
  // raw_colour_convert = raw_colour_convert + L_cart_colour;
  // cv::imwrite("/home/epk/radar_topometric_localization/data/epk_test_plots/cen2019/cen2019_overlay.jpg", raw_colour_convert);
  
  
  // sleep(10);

}

}  // namespace radar
}  // namespace vtr
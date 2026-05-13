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
 * \file detector.hpp
 * \author Keenan Burnett, Autonomous Space Robotics Lab (ASRL)
 * \brief Keypoint extraction methods for Navtech radar
 */

 #pragma once

 #include "opencv2/opencv.hpp"
 
 #include "vtr_radar/data_types/point.hpp"
 #include "vtr_radar/utils/utils.hpp"
 
 namespace vtr {
 namespace radar {
 
 template <class PointT>
 class Detector {
  public:
   virtual ~Detector() = default;
 
   virtual void run(const cv::Mat &raw_scan, const float &res,
                    const std::vector<int64_t> &azimuth_times,
                    const std::vector<double> &azimuth_angles,
                    const std::vector<bool> &up_chirps,
                    pcl::PointCloud<PointT> &pointcloud) = 0;
 };

 // added k-peaks detector here 
template <class PointT>
class KPeaks : public Detector<PointT> {
 public:
  KPeaks() = default;
  KPeaks(int kstrong, double threshold2, double threshold3, double minr,
             double maxr, double range_offset)
      : kstrong_(kstrong),
        threshold2_(threshold2),
        threshold3_(threshold3),
        minr_(minr),
        maxr_(maxr),
        range_offset_(range_offset) {}
  void run(const cv::Mat &raw_scan, const float &res,
           const std::vector<int64_t> &azimuth_times,
           const std::vector<double> &azimuth_angles,
           const std::vector<bool> &up_chirps,
           pcl::PointCloud<PointT> &pointcloud) override;
 private:
  int kstrong_ = 10;
  double threshold2_ = 0;
  double threshold3_ = 0.22;
  double minr_ = 1.0;
  double maxr_ = 69.0;
  double res_ = 0.040308 ;
  double range_offset_ = -0.319910 ;
};
 
 template <class PointT>
 class KStrongest : public Detector<PointT> {
  public:
   KStrongest() = default;
   KStrongest(int kstrong, double static_threshold, double minr,
              double maxr, double range_offset)
       : kstrong_(kstrong),
         static_threshold_(static_threshold),
         minr_(minr),
         maxr_(maxr),
         range_offset_(range_offset) {}
 
   void run(const cv::Mat &raw_scan, const float &res,
            const std::vector<int64_t> &azimuth_times,
            const std::vector<double> &azimuth_angles,
            const std::vector<bool> &up_chirps,
            pcl::PointCloud<PointT> &pointcloud) override;
 
  private:
   int kstrong_ = 10;
   double static_threshold_ = 0.22;
   double minr_ = 2.0;
   double maxr_ = 100.0;
   double range_offset_ = -0.31;
 };
 
 template <class PointT>
 class Cen2018 : public Detector<PointT> {
  public:
   Cen2018() = default;
   Cen2018(double zq, int sigma, double minr, double maxr, double range_offset)
       : zq_(zq),
         sigma_(sigma),
         minr_(minr),
         maxr_(maxr),
         range_offset_(range_offset) {}
 
   void run(const cv::Mat &raw_scan, const float &res,
            const std::vector<int64_t> &azimuth_times,
            const std::vector<double> &azimuth_angles,
            const std::vector<bool> &up_chirps,
            pcl::PointCloud<PointT> &pointcloud) override;
 
  private:
   double zq_ = 3.0;
   int sigma_ = 17;  // kernel size = sigma_ * 2 * 3 (+1 to make it odd)
   double minr_ = 2.0;
   double maxr_ = 100.0;
   double range_offset_ = -0.31;
 };
 
 template <class PointT>
 class OSCFAR : public Detector<PointT> {
  public:
   OSCFAR() = default;
   OSCFAR(int width, int guard, int kstat, double threshold, double minr, double maxr, double range_offset)
       : width_(width),
         guard_(guard),
         kstat_(kstat),
         threshold_(threshold),
         minr_(minr),
         maxr_(maxr),
         range_offset_(range_offset) {}
 
   void run(const cv::Mat &raw_scan, const float &res,
            const std::vector<int64_t> &azimuth_times,
            const std::vector<double> &azimuth_angles,
            const std::vector<bool> &up_chirps,
            pcl::PointCloud<PointT> &pointcloud) override;
 
  private:
   int width_ = 40;
   int guard_ = 2;
   int kstat_ = 20;
   double threshold_ = 1.25;
   double minr_ = 2.0;
   double maxr_ = 100.0;
   double range_offset_ = -0.31;
 };
 
 template <class PointT>
 class TM_CFAR : public Detector<PointT> {
  public:
   TM_CFAR() = default;
   TM_CFAR(int width, int guard, double threshold, int N1,
          int N2, double minr, double maxr, double range_offset)
       : width_(width),
         guard_(guard),
         threshold_(threshold),
         N1_(N1),
         N2_(N2),
         minr_(minr),
         maxr_(maxr),
         range_offset_(range_offset) {}
 
   void run(const cv::Mat &raw_scan, const float &res,
            const std::vector<int64_t> &azimuth_times,
            const std::vector<double> &azimuth_angles,
            const std::vector<bool> &up_chirps,
            pcl::PointCloud<PointT> &pointcloud) override;
 
  private:
   int width_ = 40;
   int guard_ = 2;
   double threshold_ = 1.25;
   int N1_ = 5;
   int N2_ = 5;
   double minr_ = 2.0;
   double maxr_ = 100.0;
   double range_offset_ = -0.31;
 };
 
 template <class PointT>
 class CACFAR : public Detector<PointT> {
  public:
   CACFAR() = default;
   CACFAR(int width, int guard, double threshold, double minr, double maxr, double range_offset)
       : width_(width),
         guard_(guard),
         threshold_(threshold),
         minr_(minr),
         maxr_(maxr),
         range_offset_(range_offset) {}
 
   void run(const cv::Mat &raw_scan, const float &res,
            const std::vector<int64_t> &azimuth_times,
            const std::vector<double> &azimuth_angles,
            const std::vector<bool> &up_chirps,
            pcl::PointCloud<PointT> &pointcloud) override;
 
  private:
   int width_ = 41;  // window = width + 2 * guard
   int guard_ = 2;
   double threshold_ = 3.0;
   double minr_ = 2.0;
   double maxr_ = 100.0;
   double range_offset_ = -0.31;
 };
 
 template <class PointT>
 class ModifiedCACFAR : public Detector<PointT> {
  public:
   ModifiedCACFAR() = default;
   ModifiedCACFAR(int width, int guard, double threshold, double threshold2,
                  double threshold3, double minr, double maxr,
                  double range_offset)
       : width_(width),
         guard_(guard),
         threshold_(threshold),
         threshold2_(threshold2),
         threshold3_(threshold3),
         minr_(minr),
         maxr_(maxr),
         range_offset_(range_offset) {}
 
   void run(const cv::Mat &raw_scan, const float &res,
            const std::vector<int64_t> &azimuth_times,
            const std::vector<double> &azimuth_angles,
            const std::vector<bool> &up_chirps,
            pcl::PointCloud<PointT> &pointcloud) override;
 
  private:
   int width_ = 41;  // window = width + 2 * guard
   int guard_ = 2;
   double threshold_ = 3.0;
   double threshold2_ = 1.1;
   double threshold3_ = 0.22;
   double minr_ = 2.0;
   double maxr_ = 100.0;
   double range_offset_ = -0.31;
 };
 
 template <class PointT>
 class CAGO_CFAR : public Detector<PointT> {
  public:
   CAGO_CFAR() = default;
   CAGO_CFAR(int width, int guard, double threshold, double minr, double maxr, double range_offset)
       : width_(width),
         guard_(guard),
         threshold_(threshold),
         minr_(minr),
         maxr_(maxr),
         range_offset_(range_offset) {}
 
   void run(const cv::Mat &raw_scan, const float &res,
            const std::vector<int64_t> &azimuth_times,
            const std::vector<double> &azimuth_angles,
            const std::vector<bool> &up_chirps,
            pcl::PointCloud<PointT> &pointcloud) override;
  private:
   int width_ = 41;  // window = width + 2 * guard
   int guard_ = 2;
   double threshold_ = 3.0;
   double minr_ = 2.0;
   double maxr_ = 100.0;
   double range_offset_ = -0.31;
 };
 
 template <class PointT>
 class CASO_CFAR : public Detector<PointT> {
  public:
   CASO_CFAR() = default;
   CASO_CFAR(int width, int guard, double threshold, double minr, double maxr, double range_offset)
       : width_(width),
         guard_(guard),
         threshold_(threshold),
         minr_(minr),
         maxr_(maxr),
         range_offset_(range_offset) {}
 
   void run(const cv::Mat &raw_scan, const float &res,
            const std::vector<int64_t> &azimuth_times,
            const std::vector<double> &azimuth_angles,
            const std::vector<bool> &up_chirps,
            pcl::PointCloud<PointT> &pointcloud) override;
  private:
   int width_ = 41;  // window = width + 2 * guard
   int guard_ = 2;
   double threshold_ = 3.0;
   double minr_ = 2.0;
   double maxr_ = 100.0;
   double range_offset_ = -0.31;
 };
 
 template <class PointT>
 class CFEAR_KStrong : public Detector<PointT> {
  public:
   CFEAR_KStrong() = default;
   CFEAR_KStrong(int width, int guard, int kstrong, double z_min, double r, double f, 
             double minr, double maxr, double range_offset)
       : width_(width),
         guard_(guard),
         kstrong_(kstrong),
         z_min_(z_min),
         r_(r),
         f_(f),
         minr_(minr),
         maxr_(maxr),
         range_offset_(range_offset) {}
 
   void run(const cv::Mat &raw_scan, const float &res,
            const std::vector<int64_t> &azimuth_times,
            const std::vector<double> &azimuth_angles,
            const std::vector<bool> &up_chirps,
            pcl::PointCloud<PointT> &pointcloud) override;
 
  private:
   int width_ = 41;  // window = width + 2 * guard
   int guard_ = 2;
   int kstrong_ = 12;
   double z_min_ = 0.22;
   double r_ = 3.5;
   double f_ = 1.0;
   double minr_ = 2.0;
   double maxr_ = 100.0;
   double range_offset_ = -0.31;
 };
 
 template <class PointT>
 class BFAR : public Detector<PointT> {
  public:
   BFAR() = default;
   BFAR(int width, int guard, double threshold,
                  double static_threshold, double minr, double maxr, double range_offset)
       : width_(width),
         guard_(guard),
         threshold_(threshold),
         static_threshold_(static_threshold),
         minr_(minr),
         maxr_(maxr),
         range_offset_(range_offset) {}
 
   void run(const cv::Mat &raw_scan, const float &res,
            const std::vector<int64_t> &azimuth_times,
            const std::vector<double> &azimuth_angles,
            const std::vector<bool> &up_chirps,
            pcl::PointCloud<PointT> &pointcloud) override;
 
  private:
   int width_ = 41;  // window = width + 2 * guard
   int guard_ = 2;
   double threshold_ = 3.0;
   double static_threshold_ = 0.22;
   double minr_ = 2.0;
   double maxr_ = 100.0;
   double range_offset_ = -0.31;
 };
 
 template <class PointT>
 class MSCA_CFAR : public Detector<PointT> {
  public:
   MSCA_CFAR() = default;
   MSCA_CFAR(int width, int guard, double threshold,
                  int M, double minr, double maxr, double range_offset)
       : width_(width),
         guard_(guard),
         threshold_(threshold),
         M_(M),
         minr_(minr),
         maxr_(maxr),
         range_offset_(range_offset) {}
 
   void run(const cv::Mat &raw_scan, const float &res,
            const std::vector<int64_t> &azimuth_times,
            const std::vector<double> &azimuth_angles,
            const std::vector<bool> &up_chirps,
            pcl::PointCloud<PointT> &pointcloud) override;
 
  private:
   int width_ = 41;  // window = width + 2 * guard
   int guard_ = 2;
   double threshold_ = 3.0;
   int M_ = 5;
   double minr_ = 2.0;
   double maxr_ = 100.0;
   double range_offset_ = -0.31;
 };
 
 template <class PointT>
 class IS_CFAR : public Detector<PointT> {
  public:
   IS_CFAR() = default;
   IS_CFAR(int width, int guard, double alpha_I, int N_TI,
                  double beta_I, double minr, double maxr, double range_offset)
       : width_(width),
         guard_(guard),
         alpha_I_(alpha_I),
         N_TI_(N_TI),
         beta_I_(beta_I),
         minr_(minr),
         maxr_(maxr),
         range_offset_(range_offset) {}
 
   void run(const cv::Mat &raw_scan, const float &res,
            const std::vector<int64_t> &azimuth_times,
            const std::vector<double> &azimuth_angles,
            const std::vector<bool> &up_chirps,
            pcl::PointCloud<PointT> &pointcloud) override;
 
  private:
   int width_ = 41;  // window = width + 2 * guard
   int guard_ = 2;
   double alpha_I_ = 0.05;
   int N_TI_ = 7;
   double beta_I_ = 20.02;
   double minr_ = 2.0;
   double maxr_ = 100.0;
   double range_offset_ = -0.31;
 };
 
 template <class PointT>
 class VI_CFAR : public Detector<PointT> {
  public:
   VI_CFAR() = default;
   VI_CFAR(int width, int guard, double K_VI, double K_MR,
                  double C_N, double minr, double maxr, double range_offset)
       : width_(width),
         guard_(guard),
         K_VI_(K_VI),
         K_MR_(K_MR),
         C_N_(C_N),
         minr_(minr),
         maxr_(maxr),
         range_offset_(range_offset) {}
 
   void run(const cv::Mat &raw_scan, const float &res,
            const std::vector<int64_t> &azimuth_times,
            const std::vector<double> &azimuth_angles,
            const std::vector<bool> &up_chirps,
            pcl::PointCloud<PointT> &pointcloud) override;
 
  private:
   int width_ = 41;  // window = width + 2 * guard
   int guard_ = 2;
   double alpha_I_ = 0.05;
   int K_VI_ = 7;
   double K_MR_ = 1.5;
   double C_N_ = 20.02;
   double minr_ = 2.0;
   double maxr_ = 100.0;
   double range_offset_ = -0.31;
 };
 
 template <class PointT>
 class Cen2019 : public Detector<PointT> {
  public:
   Cen2019() = default;
   Cen2019(int width, int guard,
                 int l_max, double minr, double maxr, double range_offset)
       : width_(width),
         guard_(guard),
         l_max_(l_max),
         minr_(minr),
         maxr_(maxr),
         range_offset_(range_offset) {}
 
   void run(const cv::Mat &raw_scan, const float &res,
            const std::vector<int64_t> &azimuth_times,
            const std::vector<double> &azimuth_angles,
            const std::vector<bool> &up_chirps,
            pcl::PointCloud<PointT> &pointcloud) override;
 
  private:
   int width_ = 41;  // window = width + 2 * guard
   int guard_ = 2;
   int l_max_ = 200;
   double minr_ = 2.0;
   double maxr_ = 100.0;
   double range_offset_ = -0.31;
 };
 
 }  // namespace radar
 }  // namespace vtr
 
 #include "vtr_radar/detector/detector.inl"
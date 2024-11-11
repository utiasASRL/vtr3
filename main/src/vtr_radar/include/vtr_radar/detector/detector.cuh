#pragma once
#include "vtr_radar/utils/cuda_utils.cuh"
#include <vector>
#include "vtr_radar/data_types/point.hpp"
#include "vtr_radar/utils/utils.hpp"
#include "opencv2/opencv.hpp"

namespace vtr {
namespace radar {

  struct CudaMem {
   public:
    __host__ CudaMem() = default;
    __host__ ~CudaMem(){
      if(raw_scan_device) {
        CUDA_CHECK(cudaFree(raw_scan_device));
        CUDA_CHECK(cudaFree(th_matrix_device));
        CUDA_CHECK(cudaFree(azimuth_times_device));
        CUDA_CHECK(cudaFree(azimuth_angles_device));
        CUDA_CHECK(cudaFree(means_device));
      }
    };
    __host__ void toGpu(const cv::Mat& raw_scan, const std::vector<int64_t> &azimuth_times, const std::vector<double> &azimuth_angles) {
      if (!raw_scan_device) {
        rows = raw_scan.rows;
        cols = raw_scan.cols;
        CUDA_CHECK(cudaMalloc((void**) &raw_scan_device, sizeof(float) * rows * cols));
        CUDA_CHECK(cudaMalloc((void**) &th_matrix_device, sizeof(char) * rows * cols));
        CUDA_CHECK(cudaMalloc((void**) &azimuth_times_device, sizeof(int64_t) * rows));
        CUDA_CHECK(cudaMalloc((void**) &azimuth_angles_device, sizeof(int64_t) * rows));
        CUDA_CHECK(cudaMalloc((void**) &means_device, sizeof(float) * rows));
        th_mat = cv::Mat(rows, cols, CV_8UC1);
      }
      CUDA_CHECK(cudaMemcpy(raw_scan_device, raw_scan.ptr<float>(), sizeof(float) * rows * cols, cudaMemcpyHostToDevice));
      CUDA_CHECK(cudaMemset(th_matrix_device, 0, sizeof(char) * rows * cols));
      CUDA_CHECK(cudaMemcpy(azimuth_times_device, azimuth_times.data(), sizeof(float) * rows, cudaMemcpyHostToDevice));
      CUDA_CHECK(cudaMemcpy(azimuth_angles_device, azimuth_angles.data(), sizeof(float) * rows, cudaMemcpyHostToDevice));
      return;
    };
    __host__ void fromGpu() {
      CUDA_CHECK(cudaMemcpy(th_mat.ptr<char>(), th_matrix_device, sizeof(char) * rows * cols, cudaMemcpyDeviceToHost));
    };
    float* raw_scan_device = nullptr;
    char* th_matrix_device = nullptr;
    int64_t* azimuth_times_device = nullptr;
    double* azimuth_angles_device = nullptr;
    double* means_device = nullptr;
    int rows;
    int cols;
    cv::Mat th_mat;
  };


  __global__ void computeMeans_kernel(float* raw_scan, int rows, int cols, int min_col, int max_col, float* means);
  __global__ void modifiedCACFAR_kernel(float* raw_scan,
                                        char* th_matrix,
                                        float* means,
                                        int min_col,
                                        int max_col,
                                        int rows,
                                        int cols,
                                        int w2,
                                        int guard,
                                        double threshold,
                                        double threshold2,
                                        double threshold3);
  
  template <typename PointT>                                 
  void cudaModifiedCACFAR(CudaMem& gpu_mem,
                            double minr,
                            double maxr,
                            int w2,
                            int guard,
                            double range_offset,
                            double th,
                            double th2,
                            double th3,
                            const cv::Mat &raw_scan, const float &res, 
                            const std::vector<int64_t> &azimuth_times,
                            const std::vector<double> &azimuth_angles,
                            pcl::PointCloud<PointT> &pointcloud);
} // namespace radar
} // namespace vtr
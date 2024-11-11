#include "vtr_radar/detector/detector.cuh"
#include "vtr_radar/data_types/point.hpp"
// #include <opencv2/imgcodecs.hpp>
// #include <opencv2/highgui.hpp>
namespace vtr {
namespace radar {
  __global__ void computeMeans_kernel(float* raw_scan, int rows, int cols, int min_col, int max_col, double* means) {
    int row_index = blockIdx.x * blockDim.x + threadIdx.x;
    
    if (row_index >= rows)
      return;
    means[row_index] = 0;
    for (int j = min_col; j < max_col; ++j) {
      means[row_index] = means[row_index] + raw_scan[row_index * cols + j];
    }
    means[row_index] = means[row_index] / (max_col - min_col);
    return;
  }
  __global__ void modifiedCACFAR_kernel(float* raw_scan,
                                     char* th_matrix,
                                     double* means,
                                     int min_col,
                                     int max_col,
                                     int rows,
                                     int cols,
                                     int w2,
                                     int guard,
                                     double threshold,
                                     double threshold2,
                                     double threshold3) {
    int row_index = blockIdx.x * blockDim.x + threadIdx.x;
    int col_index = blockIdx.y * blockDim.y + threadIdx.y;
    if (row_index >= rows || col_index < min_col || col_index >= max_col)
      return;

    double left = 0;
    double right = 0;
    for (int k = -w2 - guard; k < -guard; ++k) left = left + raw_scan[row_index * cols + col_index + k];
    for (int k = guard + 1; k <= w2 + guard; ++k) right = right + raw_scan[row_index * cols + col_index + k];
    // (statistic) estimate of clutter power
    // const double stat = (left + right) / (2 * w2);
    const double stat = fmax(left, right) / w2;  // GO-CFAR
    const float thres = threshold * stat + threshold2 * means[row_index] + threshold3;
    if (raw_scan[row_index * cols + col_index] > thres) 
      th_matrix[row_index * cols + col_index] = 255;
    return;
  }

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
                            pcl::PointCloud<PointT> &pointcloud) {

    pointcloud.clear();
    int rows = raw_scan.rows;
    const int cols = raw_scan.cols;
    
    int mincol = minr / res + w2 + guard + 1;
    if (mincol > cols || mincol < 0) mincol = 0;
    
    int maxcol = maxr / res - w2 - guard;
    if (maxcol > cols || maxcol < 0) maxcol = cols;
    

    dim3 dim_block(16, 16);
    dim3 dim_grid;
    dim_grid.x = (rows + dim_block.x - 1) / dim_block.x;
    dim_grid.y = (cols + dim_block.y - 1) / dim_block.y;

    const unsigned int block_size = 256;
    const unsigned int num_blocks = (rows + block_size - 1) / block_size;

    computeMeans_kernel<<<num_blocks, block_size>>>(gpu_mem.raw_scan_device, rows, cols, mincol, maxcol, gpu_mem.means_device);
    cudaDeviceSynchronize();
    // call the kernel

    modifiedCACFAR_kernel<<<dim_grid, dim_block>>>(gpu_mem.raw_scan_device, 
                          gpu_mem.th_matrix_device, gpu_mem.means_device,
                          mincol, maxcol, rows, cols, w2, guard, th, th2, th3);
    cudaDeviceSynchronize();
    gpu_mem.fromGpu();
    // std::cerr << rows << "   " << cols << std::endl;
    // cv::imshow("Display window", gpu_mem.th_mat);
    // int k = cv::waitKey();
    // #pragma omp declare reduction(merge_points : std::vector<Point3D> : omp_out.insert( \
    //       omp_out.end(), omp_in.begin(), omp_in.end())) initializer(omp_priv = decltype(omp_orig)(omp_orig.size()))
    // #pragma omp parallel for num_threads(num_threads_) reduction(merge_points : raw_points)
    for (int row = 0; row < rows; ++row) {
      float peak_points = 0;
      int num_peak_points = 0;
      const double azimuth = azimuth_angles[row];
      const double time = azimuth_times[row];
      pcl::PointCloud<PointT> polar_time;

      for (int col = mincol; col < maxcol; ++col) {
        if (gpu_mem.th_mat.at<uchar>(row, col) == 255) {
          peak_points += col;
          num_peak_points += 1;
        } else if (num_peak_points > 0) {
          PointT p;
          p.rho = res * peak_points / num_peak_points + range_offset;
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
    return;
  }


template 
void cudaModifiedCACFAR<PointWithInfo>(CudaMem& gpu_mem,
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
                            pcl::PointCloud<PointWithInfo> &pointcloud);
}  // namespace radar
}  // namespace vtr
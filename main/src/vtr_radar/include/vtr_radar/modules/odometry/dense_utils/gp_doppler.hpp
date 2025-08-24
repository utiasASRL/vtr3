#pragma once

#include <torch/torch.h>
#include <vector>
#include <string>
#include <memory>
#include <map>
#include <utility>
#include <opencv2/core.hpp>
#include <yaml-cpp/yaml.h>
#include "vtr_radar/modules/odometry/odometry_dense_module.hpp"
#include "vtr_radar/modules/odometry/dense_utils/motion_models.hpp" 
// #include "vtr_radar/modules/utils.hpp"

namespace vtr {
namespace radar {

using OptionalTensor = std::optional<torch::Tensor>;

// using vtr::radar::utils::applyGaussianBlur2D;

double inline maxAngVel(const torch::Tensor& vel) {
    const double min_ang_vel = 0.15;
    const double max_ang_vel = 1.0;
    const double max_vel = 20.0;
    const double min_vel = 10.0;

    double vel_norm = vel.norm().item<double>();

    if (vel_norm < min_vel) {
        return max_ang_vel;
    } else if (vel_norm > max_vel) {
        return min_ang_vel;
    } else {
        double a = (min_ang_vel - max_ang_vel) / (max_vel - min_vel);
        double b = max_ang_vel - a * min_vel;
        return a * vel_norm + b;
    }
}

class GPStateEstimator {
public:
      /** \brief Static module identifier. */
    static constexpr auto static_name = "radar.gp_doppler";

    GPStateEstimator(vtr::radar::OdometryDenseModule::Config::ConstPtr config_);

    torch::Tensor odometryStep(const torch::Tensor& polar_image, const torch::Tensor& azimuths, const torch::Tensor& timestamps, bool chirp_up = true);

    // cv::Mat generateVisualisation(const RadarFrame& radar_frame, int img_size, double img_res, bool inverted = false, bool text = true);

    void setGyroData(const std::vector<double>& imu_time, const std::vector<double>& imu_yaw);

    void setGyroBias(const double& gyro_bias);

    torch::Tensor getDopplerVelocity();

    void setVyBias(const double& vy_bias) {
        vy_bias_ = torch::tensor(vy_bias, torch::TensorOptions().dtype(torch::kFloat64).device(device_));
    }

    torch::Device getDevice() const {
        return device_;
    }

    std::pair<OptionalTensor, OptionalTensor> getAzPosRot();

private:
    torch::Tensor solve(const torch::Tensor& state_init, int nb_iter = 20, double cost_tol = 1e-6, double step_tol = 1e-6, bool verbose = false, bool degraded = false);
    
    std::pair<torch::Tensor, torch::Tensor> getUpDownPolarImages(const torch::Tensor& img);

    std::pair<torch::Tensor, OptionalTensor> bilinearInterpolation(const torch::Tensor& im, const torch::Tensor& az_r, bool with_jac = false);

    std::pair<torch::Tensor, torch::Tensor> bilinearInterpolationSparse(const torch::Tensor& im, const torch::Tensor& az_r);

    std::pair<torch::Tensor, torch::Tensor> costFunctionAndJacobian(const torch::Tensor& state, bool doppler, bool direct, bool degraded = false);
  
    torch::Tensor perLineInterpolation(const torch::Tensor& img, const torch::Tensor& shift);
    
    torch::Tensor localMapToPolarCoord();

    std::tuple<torch::Tensor, torch::Tensor, torch::Tensor> polarToCartCoordCorrectionSparse(const torch::Tensor& pos, const torch::Tensor& rot, const torch::Tensor& doppler_shift);
    
    torch::Tensor polarCoordCorrection(const torch::Tensor& pos, const torch::Tensor& rot);

    std::pair<torch::Tensor, torch::Tensor> imgDopplerInterpAndJacobian(const torch::Tensor& shift);

    torch::Tensor cartToLocalMapID(const torch::Tensor& xy);
    torch::Tensor cartToLocalMapIDSparse(const torch::Tensor& xy);

    void moveLocalMap(const torch::Tensor& pos, const torch::Tensor& rot);
    
    torch::Tensor seKernel(const torch::Tensor& X1, const torch::Tensor& X2, double l_az, double l_range);

//     // Device
    torch::Device device_;

    // Configuration parameters
    double radar_res_;
    bool use_doppler_, use_direct_, use_gyro_, estimate_ang_vel_, doppler_radar_;
    double optimisation_first_step_;
    double kImgPadding_;
    double max_acc_;
    torch::Tensor vy_bias_;
    double max_diff_vel_;
    torch::Tensor previous_vel_;
    double local_map_res_;
    int local_map_zero_idx_;
    float one_minus_alpha_, alpha_;
    double radar_beta_, vel_to_bin_;
    int max_range_idx_, min_range_idx_;
    int max_range_idx_direct_, min_range_idx_direct_;
    torch::Tensor local_map_, local_map_blurred_, local_map_mask_;
    torch::Tensor local_map_xy_, local_map_polar_;
    int nb_bins_;
    torch::Tensor shift_to_range_, range_vec_;
    torch::Tensor polar_intensity_;
    torch::Tensor polar_coord_raw_gp_infered_;
    torch::Tensor temp_even_img_, temp_even_img_sparse_, odd_coeff_, odd_bias_;
    torch::Tensor polar_image_;
    torch::Tensor azimuths_;
    torch::Tensor vel_to_bin_vec_;
    torch::Tensor state_init_, current_pos_, current_rot_;
    int nb_azimuths_;
    torch::Tensor beta_smooth_torch_conv_weight_, beta_interp_torch_conv_weight_;
    torch::nn::Conv2d beta_smooth_torch_conv_ = nullptr;
    torch::nn::Conv2d beta_interp_torch_conv_ = nullptr;

//     // Sparse data for direct cost
//     torch::Tensor polar_intensity_sparse_;
//     torch::Tensor direct_az_ids_sparse_, direct_r_ids_sparse_, direct_r_sparse_;
//     torch::Tensor mask_direct_even_, mask_direct_odd_;
//     int direct_nb_non_zero_;
//     torch::Tensor direct_r_ids_even_, direct_r_ids_odd_;
//     torch::Tensor direct_r_even_, direct_r_odd_;
//     torch::Tensor direct_az_ids_even_, direct_az_ids_odd_;

//     // Sparse data for doppler cost
//     torch::Tensor doppler_az_ids_sparse_, doppler_bin_vec_sparse_;

//     // GP parameters
//     int size_az_, size_range_;
//     torch::Tensor beta_smooth_, beta_interp_;
//     torch::Tensor X_smooth_, X_interp_;

//     // Doppler or direct cost flags
    bool pose_estimation_;
    bool chirp_up_, prev_chirp_up_;

    // Motion model
    std::shared_ptr<MotionModel> motion_model_;

//     // Other
    torch::Tensor timestamps_;
    torch::Tensor display_intensity_normalisation_;
    int step_counter_;
    torch::Tensor prev_state_;
    double ang_vel_bias_;
    torch::Tensor odd_img_, even_img_;
};

}  // namespace radar
}  // namespace vtr
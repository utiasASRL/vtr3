#include "vtr_radar/modules/odometry/dense_utils/gp_doppler.hpp"
#include "vtr_radar/utils/utils.hpp"
#include <limits>
namespace vtr {
namespace radar {
using torch::indexing::Slice;
using torch::indexing::None;

/// Helper: convert a CPU float32 tensor [H×W] into a CV_32FC1 Mat
static cv::Mat tensorToMat(const torch::Tensor& t) {
    auto tt = t.contiguous();  // ensure contiguous
    int H = tt.size(0), W = tt.size(1);
    // wrap the data pointer (no copy) then clone to own the memory
    cv::Mat m(H, W, CV_32FC1, const_cast<float*>(tt.data_ptr<float>()));
    return m.clone();
}

GPStateEstimator::GPStateEstimator(vtr::radar::OdometryDenseModule::Config::ConstPtr config_) :
    device_(torch::cuda::is_available() ? torch::kCUDA : torch::kCPU)
{
    torch::NoGradGuard no_grad;

    CLOG(DEBUG, "radar.gp_doppler") << "I am in the constructor of GPStateEstimator";
    CLOG(DEBUG, "radar.gp_doppler") << "Hi, I AM SAMMMMMMMMMMM ";

    kImgPadding_ = 200.0;
    display_intensity_normalisation_ = torch::Tensor();
    timestamps_ = torch::Tensor();

    // set config parameters within the gp_doppler class
    radar_res_ = static_cast<double>(config_->radar_resolution);
    CLOG(DEBUG, "radar.gp_doppler") << "The radar resolution is: " << radar_res_;

    optimisation_first_step_ = static_cast<double>(config_->optimization_first_step);
    CLOG(DEBUG, "radar.gp_doppler") << "The optimisation first step is: " << optimisation_first_step_;

    use_doppler_ = config_->doppler_cost;
    CLOG(DEBUG, "radar.gp_doppler") << "The config param: doppler_cost is: " << use_doppler_;

    use_direct_ = config_->direct_cost;
    CLOG(DEBUG, "radar.gp_doppler") << "The config param: direct_cost is: " << use_direct_;

    use_gyro_ = config_->use_gyro;
    CLOG(DEBUG, "radar.gp_doppler") << "The config param: use_gyro is: " << use_gyro_;

    estimate_ang_vel_ = config_->motion_model.find("const_w") != std::string::npos;
    CLOG(DEBUG, "radar.gp_doppler") << "The config param: estimate_angular_velocity is: " << estimate_ang_vel_;

    doppler_radar_ = config_->doppler_enabled;
    CLOG(DEBUG, "radar.gp_doppler") << "The config param: doppler_enabled is: " << doppler_radar_;


    if (!use_doppler_ && !use_direct_)
        throw std::runtime_error("Invalid estimation parameters: need at least one cost function to be enabled");

    if (use_doppler_ && !doppler_radar_)
        throw std::runtime_error("Doppler cost function enabled but no doppler radar");

    if (estimate_ang_vel_ && !use_direct_)
        throw std::runtime_error("Angular velocity estimation enabled but no direct cost function");

    std::string model_key = config_->motion_model; // this is already a string 
    CLOG(DEBUG, "radar.gp_doppler") << "The config param: motion_model is: " << model_key;

    if (use_doppler_ && !use_direct_)
    {
        motion_model_ = use_gyro_
            ? std::static_pointer_cast<MotionModel>(std::make_shared<ConstBodyVelGyro>(device_))
            : std::static_pointer_cast<MotionModel>(std::make_shared<ConstVel>(device_));
    } 
    else 
    {
        if (model_key == "const_vel_const_w") 
            motion_model_ = std::make_shared<ConstVelConstW>(device_);
        else if (model_key == "const_body_vel_gyro") 
            motion_model_ = std::make_shared<ConstBodyVelGyro>(device_);
        else if (model_key == "const_vel") 
            motion_model_ = std::make_shared<ConstVel>(device_);
        else 
            throw std::runtime_error("Unknown motion model: " + model_key);
    }

    state_init_ = motion_model_->getInitialState();

    if (use_direct_ && !estimate_ang_vel_ && !use_gyro_)
        throw std::runtime_error("Direct cost enabled but no angular velocity or gyro.");

    pose_estimation_ = use_gyro_ || estimate_ang_vel_;

    vy_bias_ = torch::tensor(config_->vy_bias_prior, torch::TensorOptions().device(device_));

    double kNeighbourhoodFactor = 1.0;
    double l_az = static_cast<double>(config_->lengthscale_az);
    double l_range = static_cast<double>(config_->lengthscale_range);
    size_az_ = static_cast<int>(kNeighbourhoodFactor * l_az);
    size_range_ = static_cast<int>(kNeighbourhoodFactor * l_range);

    double df_dt = config_->del_f * config_->meas_freq;
    CLOG(DEBUG, "radar.gp_doppler") << "line 105: df_dt is: " << df_dt;

    radar_beta_ = config_->beta_corr_fact *
                                (config_->ft + config_->del_f / 2.0) / df_dt;

    vel_to_bin_ = 2.0 * radar_beta_ / radar_res_;
    CLOG(DEBUG, "radar.gp_doppler") << "line 106: vel_to_bin_ is: " << vel_to_bin_;

    int range_start = static_cast<int>(std::ceil(config_->min_range / radar_res_));
    int range_end = static_cast<int>(std::floor(config_->max_range / radar_res_));
    nb_bins_ = range_end - range_start + 1 + 2 * static_cast<int>(kImgPadding_);
    CLOG(DEBUG, "radar.gp_doppler") << "line 111: nb_bins_ is: " << nb_bins_;
    CLOG(DEBUG, "radar.gp_doppler") << "line 113: range start is: " << range_start;
    CLOG(DEBUG, "radar.gp_doppler") << "line 114: range end is: " << range_end;

    // GP convolution preparation
    auto x = torch::arange(-size_az_, size_az_ + 1, torch::TensorOptions().dtype(torch::kFloat32));
    auto mask_smooth = (x.remainder(2) == 0);
    auto mask_interp = (x.remainder(2) != 0);
    auto x_smooth = x.masked_select(mask_smooth);
    auto x_interp = x.masked_select(mask_interp);
    auto y = torch::arange(-size_range_, size_range_ + 1, torch::TensorOptions().dtype(torch::kFloat32));

    auto meshgrid = torch::meshgrid({x_smooth, y}, "ij");
    auto XX_smooth = meshgrid[0];
    auto YY_smooth = meshgrid[1];
    meshgrid = torch::meshgrid({x_interp, y}, "ij");
    auto XX_interp = meshgrid[0];
    auto YY_interp = meshgrid[1];

    X_smooth_ = torch::stack({XX_smooth.flatten(), YY_smooth.flatten()}, 1);
    X_interp_ = torch::stack({XX_interp.flatten(), YY_interp.flatten()}, 1);

    double sz = config_->sz;
    CLOG(DEBUG, "radar.gp_doppler") << "The config param: sz is: " << sz;
    auto n_smooth = X_smooth_.size(0);
    auto K_smooth = seKernel(X_smooth_, X_smooth_, l_az, l_range) + torch::eye(n_smooth) * (sz * sz);
    auto Kinv_smooth = torch::linalg_inv(K_smooth);
    auto ks_smooth = seKernel(torch::tensor({{0.0, 0.0}}, torch::TensorOptions().dtype(torch::kFloat32)), X_smooth_, l_az, l_range);
    beta_smooth_ = (ks_smooth.matmul(Kinv_smooth)).squeeze();

    auto n_interp = X_interp_.size(0);
    auto K_interp = seKernel(X_interp_, X_interp_, l_az, l_range) + torch::eye(n_interp) * (sz * sz);
    auto Kinv_interp = torch::linalg_inv(K_interp);
    auto ks_interp = seKernel(torch::tensor({{0.0, 0.0}}, torch::TensorOptions().dtype(torch::kFloat32)), X_interp_, l_az, l_range);
    beta_interp_ = (ks_interp.matmul(Kinv_interp)).squeeze();

    beta_smooth_torch_conv_ = torch::nn::Conv2d(torch::nn::Conv2dOptions(1, 1, {x_smooth.size(0), y.size(0)}).bias(false).padding({x_smooth.size(0)/2, y.size(0)/2}));
    beta_interp_torch_conv_ = torch::nn::Conv2d(torch::nn::Conv2dOptions(1, 1, {x_interp.size(0), y.size(0)}).bias(false).padding({x_interp.size(0)/2, y.size(0)/2}));

    beta_smooth_torch_conv_->weight.set_data(beta_smooth_.reshape({1, 1, x_smooth.size(0), y.size(0)}).to(device_).to(torch::kFloat32));
    beta_interp_torch_conv_->weight.set_data(beta_interp_.reshape({1, 1, x_interp.size(0), y.size(0)}).to(device_).to(torch::kFloat32));

    max_range_idx_ = static_cast<int>(std::floor(config_->max_range / radar_res_)); // doppler range is the same on the RAS3 since it does not have doppler
    min_range_idx_ = static_cast<int>(std::ceil(config_->min_range / radar_res_));

    if (use_direct_) 
    {
        max_range_idx_direct_ = static_cast<int>(std::floor(config_->max_range / radar_res_)); // max range should be 69 for RAS3
        min_range_idx_direct_ = static_cast<int>(std::ceil(config_->min_range / radar_res_));

        // Prepare the local map
        local_map_res_ = config_->local_map_res;
        double max_local_map_range = config_->max_local_map_range;
        int local_map_size = static_cast<int>(max_local_map_range / local_map_res_) * 2 + 1;
        CLOG(DEBUG, "radar.gp_doppler") << "The config param: local_map_size is: " << local_map_size;
        local_map_ = torch::zeros({local_map_size, local_map_size}, torch::TensorOptions().device(device_).dtype(torch::kFloat32));

    //     // auto temp_x = (torch::arange(-local_map_size / 2, local_map_size / 2, torch::TensorOptions().device(device_)) + 1) * local_map_res_;

        auto temp_x = ((torch::arange(-int(local_map_size/2)-1, int(local_map_size/2), 1).to(device_).to(torch::kFloat64)) + 1) * local_map_res_;
        auto X = -temp_x.unsqueeze(0).transpose(0, 1).repeat({1, local_map_size});
        auto Y = temp_x.unsqueeze(0).repeat({local_map_size, 1});

        local_map_xy_ = torch::stack({X, Y}, 2).unsqueeze(-1);
        local_map_zero_idx_ = static_cast<int>(max_local_map_range / local_map_res_);
        local_map_polar_ = localMapToPolarCoord().to(torch::kFloat32);  

        double min_r = static_cast<double>(config_->min_range);
        double max_r = static_cast<double>(config_->max_local_map_range);

        local_map_mask_ = (local_map_polar_.index({Slice(), Slice(), 1}) < max_local_map_range) &
                          (local_map_polar_.index({Slice(), Slice(), 1}) > min_r);

        alpha_ = config_->local_map_update_alpha;
        one_minus_alpha_ = 1.0 - alpha_;

        // Doppler shift to range
        shift_to_range_ = torch::tensor(radar_res_ / 2.0, torch::TensorOptions().device(device_));
        range_vec_ = torch::arange(max_range_idx_direct_, torch::TensorOptions().device(device_)).to(torch::kFloat32) * radar_res_ + (radar_res_ / 2.0);
    }

    current_rot_ = torch::tensor(0.0, torch::TensorOptions().device(device_).dtype(torch::kFloat64));
    current_pos_ = torch::zeros({2}, torch::TensorOptions().device(device_).dtype(torch::kFloat64));

    max_acc_ = static_cast<double>(config_->max_acceleration); 
    CLOG(DEBUG, "radar.gp_doppler") << "The config param: max_acc_ is: " << max_acc_;
    previous_vel_ = torch::tensor(0.0, torch::TensorOptions().device(device_));

    step_counter_ = 0;

    if (model_key.find("const_w") != std::string::npos)
        ang_vel_bias_ = static_cast<double>(config_->ang_vel_bias);
        CLOG(DEBUG, "radar.gp_doppler") << "The config param: ang_vel_bias_ is: " << ang_vel_bias_;

    cv::namedWindow("local_map_cv", cv::WINDOW_AUTOSIZE);
}


torch::Tensor GPStateEstimator::seKernel(const torch::Tensor& X1, const torch::Tensor& X2, double l_az, double l_range) 
{
    // Normalize inputs
    auto temp_X1 = X1.clone();
    auto temp_X2 = X2.clone();

    temp_X1.index_put_({Slice(), 0}, temp_X1.index({Slice(), 0}) / l_az);
    temp_X2.index_put_({Slice(), 0}, temp_X2.index({Slice(), 0}) / l_az);
    temp_X1.index_put_({Slice(), 1}, temp_X1.index({Slice(), 1}) / l_range);
    temp_X2.index_put_({Slice(), 1}, temp_X2.index({Slice(), 1}) / l_range);

    // Compute pairwise squared Euclidean distances
    auto X1_norm = temp_X1.pow(2).sum(1).unsqueeze(1);  // [N, 1]
    auto X2_norm = temp_X2.pow(2).sum(1).unsqueeze(0);  // [1, M]
    auto dist = X1_norm + X2_norm - 2.0 * torch::mm(temp_X1, temp_X2.transpose(0, 1));

    // Apply Gaussian kernel
    return torch::exp(-dist / 2.0);
}

std::pair<torch::Tensor, torch::Tensor> GPStateEstimator::getUpDownPolarImages(const torch::Tensor& img) 
{
    torch::NoGradGuard no_grad;

    auto in_even = img.index({Slice(0, None, 2)});
    auto in_odd = img.index({Slice(1, None, 2)});
    auto mean_even = in_even.mean();
    auto mean_odd = in_odd.mean();
    in_even = in_even - mean_even;
    in_odd = in_odd - mean_odd;

    auto in_even_device = in_even.unsqueeze(0).unsqueeze(0).to(device_);
    auto in_odd_device = in_odd.unsqueeze(0).unsqueeze(0).to(device_);

    in_even_device.set_requires_grad(false);
    in_odd_device.set_requires_grad(false);

    auto even_smooth_torch = beta_smooth_torch_conv_->forward(in_even_device);
    auto even_interp_torch = beta_interp_torch_conv_->forward(in_even_device);
    auto odd_smooth_torch = beta_smooth_torch_conv_->forward(in_odd_device);
    auto odd_interp_torch = beta_interp_torch_conv_->forward(in_odd_device);

    if (even_smooth_torch.size(2) > in_even.size(0)) 
        even_smooth_torch = even_smooth_torch.index({Slice(), Slice(), Slice(0, -1), Slice()});
    if (even_interp_torch.size(2) > in_even.size(0)) 
        even_interp_torch = even_interp_torch.index({Slice(), Slice(), Slice(0, -1), Slice()});
    if (odd_smooth_torch.size(2) > in_odd.size(0)) 
        odd_smooth_torch = odd_smooth_torch.index({Slice(), Slice(), Slice(0, -1), Slice()});
    if (odd_interp_torch.size(2) > in_odd.size(0)) 
        odd_interp_torch = odd_interp_torch.index({Slice(), Slice(), Slice(0, -1), Slice()});

    auto out_even = torch::zeros({1, 1, img.size(0), img.size(1)}, torch::TensorOptions().dtype(torch::kFloat32).device(device_));
    auto out_odd = torch::zeros({1, 1, img.size(0), img.size(1)}, torch::TensorOptions().dtype(torch::kFloat32).device(device_));

    out_even.index_put_({Slice(), Slice(), Slice(0, None, 2), Slice()}, even_smooth_torch);
    out_even.index_put_({Slice(), Slice(), Slice(1, -1, 2), Slice()}, even_interp_torch.index({Slice(), Slice(), Slice(1, None), Slice()}));
    out_odd.index_put_({Slice(), Slice(), Slice(0, None, 2), Slice()}, odd_interp_torch);
    out_odd.index_put_({Slice(), Slice(), Slice(1, None, 2), Slice()}, odd_smooth_torch);
    out_odd.index_put_({Slice(), Slice(), -1, Slice()}, torch::zeros_like(out_odd.index({Slice(), Slice(), -1, Slice()})));

    auto even_std = torch::std(out_even, 3, false, true);
    auto odd_std = torch::std(out_odd, 3, false, true);

    out_even = out_even - 2.0 * even_std;
    out_odd = out_odd - 2.0 * odd_std;

    out_even = torch::where(out_even < 0, torch::zeros_like(out_even), out_even);
    out_odd = torch::where(out_odd < 0, torch::zeros_like(out_odd), out_odd);

    out_even = applyGaussianBlur2D(out_even, 9, 1, 3.0, 3.0);
    out_odd = applyGaussianBlur2D(out_odd, 9, 1, 3.0, 3.0);

    out_even = out_even / std::get<0>(torch::max(out_even, 3, true));
    out_odd = out_odd / std::get<0>(torch::max(out_odd, 3, true));

    out_even = torch::nan_to_num(out_even, 0);
    out_odd = torch::nan_to_num(out_odd, 0);

    out_even = out_even.squeeze();
    out_odd = out_odd.squeeze();

    out_even.index_put_({Slice(0, size_az_)}, 0);
    out_even.index_put_({Slice(out_even.size(0) - size_az_, None)}, 0);
    out_odd.index_put_({Slice(0, size_az_)}, 0);
    out_odd.index_put_({Slice(out_odd.size(0) - size_az_, None)}, 0);
    out_even.index_put_({Slice(), Slice(0, size_range_)}, 0);
    out_even.index_put_({Slice(), Slice(out_even.size(1) - size_range_, None)}, 0);
    out_odd.index_put_({Slice(), Slice(0, size_range_)}, 0);
    out_odd.index_put_({Slice(), Slice(out_odd.size(1) - size_range_, None)}, 0);

    return {out_odd, out_even};
}

std::pair<torch::Tensor, OptionalTensor> GPStateEstimator::bilinearInterpolation(const torch::Tensor& im, const torch::Tensor& az_r, bool with_jac) 
{
    torch::NoGradGuard no_grad;

    auto az0 = torch::floor(az_r.index({Slice(), Slice(), 0})).to(torch::kInt32);
    auto az1 = az0 + 1;
    auto r0 = torch::floor(az_r.index({Slice(), Slice(), 1})).to(torch::kInt32);
    auto r1 = r0 + 1;

    az0 = torch::clamp(az0, 0, im.size(0) - 1);
    az1 = torch::clamp(az1, 0, im.size(0) - 1);
    r0 = torch::clamp(r0, 0, im.size(1) - 1);
    r1 = torch::clamp(r1, 0, im.size(1) - 1);

    auto az_r_clamped = az_r.clone().to(torch::kFloat32);
    az_r_clamped.index_put_({Slice(), Slice(), 0}, torch::clamp(az_r_clamped.index({Slice(), Slice(), 0}), 0, im.size(0) - 1));
    az_r_clamped.index_put_({Slice(), Slice(), 1}, torch::clamp(az_r_clamped.index({Slice(), Slice(), 1}), 0, im.size(1) - 1));

    auto Ia = im.index({az0, r0});
    auto Ib = im.index({az1, r0});
    auto Ic = im.index({az0, r1});
    auto Id = im.index({az1, r1});

    auto local_1_minus_r = r1.to(torch::kFloat32) - az_r_clamped.index({Slice(), Slice(), 1});
    auto local_r = az_r_clamped.index({Slice(), Slice(), 1}) - r0.to(torch::kFloat32);
    auto local_1_minus_az = az1.to(torch::kFloat32) - az_r_clamped.index({Slice(), Slice(), 0});
    auto local_az = az_r_clamped.index({Slice(), Slice(), 0}) - az0.to(torch::kFloat32);

    auto wa = local_1_minus_az * local_1_minus_r;
    auto wb = local_az * local_1_minus_r;
    auto wc = local_1_minus_az * local_r;
    auto wd = local_az * local_r;

    auto img_interp = wa * Ia + wb * Ib + wc * Ic + wd * Id;

    if (!with_jac) 
        return {img_interp, std::nullopt};
    else 
    {
        auto d_I_d_az_r = torch::empty({az_r.size(0), az_r.size(1), 1, 2}, torch::TensorOptions().device(device_));
        d_I_d_az_r.index_put_({Slice(), Slice(), 0, 0}, (Ib - Ia) * local_1_minus_r + (Id - Ic) * local_r);
        d_I_d_az_r.index_put_({Slice(), Slice(), 0, 1}, (Ic - Ia) * local_1_minus_az + (Id - Ib) * local_az);
        return {img_interp, d_I_d_az_r};
    }
}

std::pair<torch::Tensor, torch::Tensor> GPStateEstimator::bilinearInterpolationSparse(const torch::Tensor& im, const torch::Tensor& az_r) 
{
    torch::NoGradGuard no_grad;

    auto az0 = torch::floor(az_r.index({Slice(), 0})).to(torch::kInt32);
    auto az1 = az0 + 1;
    auto r0 = torch::floor(az_r.index({Slice(), 1})).to(torch::kInt32);
    auto r1 = r0 + 1;

    az0 = torch::clamp(az0, 0, im.size(0) - 1);
    az1 = torch::clamp(az1, 0, im.size(0) - 1);
    r0 = torch::clamp(r0, 0, im.size(1) - 1);
    r1 = torch::clamp(r1, 0, im.size(1) - 1);

    auto az_r_clamped = az_r.clone();
    az_r_clamped.index_put_({Slice(), 0}, torch::clamp(az_r_clamped.index({Slice(), 0}), 0, im.size(0) - 1));
    az_r_clamped.index_put_({Slice(), 1}, torch::clamp(az_r_clamped.index({Slice(), 1}), 0, im.size(1) - 1));

    auto Ia = im.index({az0, r0});
    auto Ib = im.index({az1, r0});
    auto Ic = im.index({az0, r1});
    auto Id = im.index({az1, r1});

    auto local_1_minus_r = r1.to(torch::kFloat32) - az_r_clamped.index({Slice(), 1});
    auto local_r = az_r_clamped.index({Slice(), 1}) - r0.to(torch::kFloat32);
    auto local_1_minus_az = az1.to(torch::kFloat32) - az_r_clamped.index({Slice(), 0});
    auto local_az = az_r_clamped.index({Slice(), 0}) - az0.to(torch::kFloat32);

    auto wa = local_1_minus_az * local_1_minus_r;
    auto wb = local_az * local_1_minus_r;
    auto wc = local_1_minus_az * local_r;
    auto wd = local_az * local_r;

    auto img_interp = wa * Ia + wb * Ib + wc * Ic + wd * Id;

    auto d_I_d_az_r = torch::empty({az_r.size(0), 1, 2}, torch::TensorOptions().dtype(torch::kFloat64).device(device_));
    d_I_d_az_r.index_put_({Slice(), 0, 0}, (Ib - Ia) * local_1_minus_r + (Id - Ic) * local_r);
    d_I_d_az_r.index_put_({Slice(), 0, 1}, (Ic - Ia) * local_1_minus_az + (Id - Ib) * local_az);

    return {img_interp, d_I_d_az_r};
}

std::pair<torch::Tensor, torch::Tensor> GPStateEstimator::costFunctionAndJacobian(const torch::Tensor& state, bool doppler, bool direct, bool degraded) 
{
    torch::NoGradGuard no_grad;

    int state_size = state.size(0);
    auto [velocities, d_vel_d_state_opt, pos, d_pos_d_state_opt, rot, d_rot_d_state_opt] = motion_model_->getVelPosRot(state, true);
    
    auto d_vel_d_state = d_vel_d_state_opt.value();
    auto d_pos_d_state = d_pos_d_state_opt.value();

    velocities = velocities.view({-1, 1, 2});
    auto mask = velocities.index({Slice(), 0, 0}) > 3.0;

    velocities.index_put_({mask, 0, 1}, velocities.index({mask, 0, 1}) + vy_bias_);
    velocities.index_put_({~mask, 0, 1}, velocities.index({~mask, 0, 1}) + velocities.index({~mask, 0, 0}) * vy_bias_ / 3.0);

    d_vel_d_state.index_put_({~mask, 1, Slice()}, d_vel_d_state.index({~mask, 1, Slice()}) + vy_bias_ / 3.0 * d_vel_d_state.index({~mask, 0, Slice()}));

    auto shifts = torch::matmul(velocities, vel_to_bin_vec_.view({-1, 2, 1})).squeeze();
    auto d_shift_d_state = torch::matmul(vel_to_bin_vec_.view({-1, 1, 2}), d_vel_d_state);

    if (!chirp_up_) {
        shifts = -shifts;
        d_shift_d_state = -d_shift_d_state;
    }

    torch::Tensor residual, jacobian, residual_direct, jacobian_direct;

    if (doppler) 
    {
        auto [interp_sparse, aligned_odd_coeff_sparse] = imgDopplerInterpAndJacobian(shifts);
        residual = interp_sparse * temp_even_img_sparse_;

        jacobian = torch::matmul(
            aligned_odd_coeff_sparse.view({-1, 1, 1}).to(torch::kFloat64),
            d_shift_d_state.index({doppler_az_ids_sparse_, Slice(), Slice()})
        ) * temp_even_img_sparse_.unsqueeze(-1).unsqueeze(-1);
        
        residual = residual.flatten();

        jacobian = jacobian.view({-1, state_size});

        if (degraded) 
        {
            auto weights = (torch::clamp(torch::abs(interp_sparse - temp_even_img_sparse_), 0, 1) - 1).pow(6).flatten().unsqueeze(-1);
            jacobian = jacobian * weights;
        }
    }

    if (direct) 
    {
        auto [cart_corrected_sparse, d_cart_d_rot_sparse, d_cart_d_shift_sparse] = polarToCartCoordCorrectionSparse(pos, rot, shifts);

        auto cart_idx_sparse = cartToLocalMapIDSparse(cart_corrected_sparse).squeeze();

        auto [interp_direct_sparse, d_interp_direct_d_xy_sparse] = bilinearInterpolationSparse(local_map_blurred_, cart_idx_sparse);

        auto residual_direct_sparse = interp_direct_sparse * polar_intensity_sparse_;

        auto d_cart_sparse_d_state = torch::matmul(d_cart_d_shift_sparse, d_shift_d_state.view({-1, 1, state_size})).index({direct_az_ids_sparse_, Slice(), Slice()});

        if (d_rot_d_state_opt.has_value()) 
        {
            auto d_rot_d_state = d_rot_d_state_opt.value();
            d_cart_sparse_d_state.index_put_({Slice(), Slice(), -1},
                d_cart_sparse_d_state.index({Slice(), Slice(), -1}) +
                torch::matmul(d_cart_d_rot_sparse, d_rot_d_state.index({direct_az_ids_sparse_}).view({-1, 1, 1})).squeeze());
        }

        d_cart_sparse_d_state += d_pos_d_state.index({direct_az_ids_sparse_}).view({-1, 2, state_size});

        d_cart_sparse_d_state.index_put_({Slice(), 0, Slice()},
            d_cart_sparse_d_state.index({Slice(), 0, Slice()}) / (-local_map_res_));
        d_cart_sparse_d_state.index_put_({Slice(), 1, Slice()},
            d_cart_sparse_d_state.index({Slice(), 1, Slice()}) / local_map_res_);

        auto jacobian_direct_sparse = (torch::matmul(d_interp_direct_d_xy_sparse, d_cart_sparse_d_state) *
                                       polar_intensity_sparse_.unsqueeze(-1).unsqueeze(-1)).squeeze();

        residual_direct = residual_direct_sparse.flatten();
        jacobian_direct = jacobian_direct_sparse.view({-1, state_size});

        if (degraded) 
        {
            auto weights_direct = (torch::clamp(torch::abs(interp_direct_sparse - polar_intensity_sparse_), 0, 1) - 1).pow(6).flatten().unsqueeze(-1);
            jacobian_direct = jacobian_direct * weights_direct;
        }
    }


    if (doppler && direct) 
    {
        return {torch::cat({residual, residual_direct}, 0), torch::cat({jacobian, jacobian_direct}, 0)};
    } else if (doppler) 
    {
        return {residual, jacobian};
    } else 
    {
        return {residual_direct, jacobian_direct};
    }
}

torch::Tensor GPStateEstimator::perLineInterpolation(const torch::Tensor& img, const torch::Tensor& shift) 
{
    torch::NoGradGuard no_grad;

    auto shift_int = torch::floor(shift).to(torch::kInt32);
    auto shift_frac = shift - shift_int.to(torch::kFloat32);

    auto az = torch::arange(img.size(0), torch::TensorOptions().device(device_)).unsqueeze(1).repeat({1, img.size(1)});
    auto r_0 = torch::arange(img.size(1), torch::TensorOptions().device(device_)).unsqueeze(0).repeat({img.size(0), 1});

    r_0 = r_0 + shift_int.view({-1, 1});
    auto r_1 = torch::clamp(r_0 + 1, 0, img.size(1) - 1);
    r_0 = torch::clamp(r_0, 0, img.size(1) - 1);

    auto Ia = img.index({az, r_0});
    auto Ib = img.index({az, r_1});

    auto interp = (1.0 - shift_frac.view({-1, 1})) * Ia + shift_frac.view({-1, 1}) * Ib;
    return interp;
}

torch::Tensor GPStateEstimator::localMapToPolarCoord() 
{
    torch::NoGradGuard no_grad;

    auto cart = local_map_xy_;
    auto polar = torch::zeros({cart.size(0), cart.size(1), 2}, torch::TensorOptions().device(device_));

    polar.index_put_({Slice(), Slice(), 0},
        torch::atan2(cart.index({Slice(), Slice(), 1, 0}),
                     cart.index({Slice(), Slice(), 0, 0})));

    polar.index_put_({Slice(), Slice(), 1},
        torch::sqrt(torch::pow(cart.index({Slice(), Slice(), 0, 0}), 2) +
                    torch::pow(cart.index({Slice(), Slice(), 1, 0}), 2)));

    return polar;
}

std::tuple<torch::Tensor, torch::Tensor, torch::Tensor> GPStateEstimator::polarToCartCoordCorrectionSparse(const torch::Tensor& pos, const torch::Tensor& rot, const torch::Tensor& doppler_shift) 
{
    torch::NoGradGuard no_grad;
    
    auto c_az_min = torch::cos(azimuths_);
    auto s_az_min = torch::sin(azimuths_);
    
    auto c_az = c_az_min.index({direct_az_ids_sparse_});
    auto s_az = s_az_min.index({direct_az_ids_sparse_});
    
    auto even_range = range_vec_.index({direct_r_ids_even_}) - doppler_shift.index({direct_az_ids_even_}) * shift_to_range_;
    auto odd_range = range_vec_.index({direct_r_ids_odd_}) + doppler_shift.index({direct_az_ids_odd_}) * shift_to_range_;
    
    auto x = torch::empty({direct_nb_non_zero_}, torch::TensorOptions().device(device_).dtype(torch::kFloat64));
    x.index_put_({mask_direct_even_}, c_az.index({mask_direct_even_}) * even_range);
    x.index_put_({mask_direct_odd_}, c_az.index({mask_direct_odd_}) * odd_range);
    
    auto y = torch::empty({direct_nb_non_zero_}, torch::TensorOptions().device(device_).dtype(torch::kFloat64));
    y.index_put_({mask_direct_even_}, s_az.index({mask_direct_even_}) * even_range);
    y.index_put_({mask_direct_odd_}, s_az.index({mask_direct_odd_}) * odd_range);
    
    auto c_rot_min = torch::cos(rot.squeeze());
    auto s_rot_min = torch::sin(rot.squeeze());
    
    auto c_rot = c_rot_min.index({direct_az_ids_sparse_});
    auto s_rot = s_rot_min.index({direct_az_ids_sparse_});
    auto x_c_rot = x * c_rot;
    auto y_s_rot = y * s_rot;
    auto x_s_rot = x * s_rot;
    auto y_c_rot = y * c_rot;
    
    auto x_rot = x_c_rot - y_s_rot;
    auto y_rot = x_s_rot + y_c_rot;
    
    auto x_trans = x_rot + pos.squeeze().index({direct_az_ids_sparse_, 0});
    auto y_trans = y_rot + pos.squeeze().index({direct_az_ids_sparse_, 1});
    
    auto cart = torch::stack({x_trans.unsqueeze(-1), y_trans.unsqueeze(-1)}, 1);
    
    auto d_cart_d_rot = torch::zeros({direct_nb_non_zero_, 2, 1}, torch::TensorOptions().device(device_).dtype(torch::kFloat64));
    d_cart_d_rot.index_put_({Slice(), 0, 0}, -y_rot);
    d_cart_d_rot.index_put_({Slice(), 1, 0}, x_rot);
    
    auto d_cart_d_shift = torch::empty({nb_azimuths_, 2, 1}, torch::TensorOptions().device(device_).dtype(torch::kFloat64));
    
    d_cart_d_shift.index_put_({Slice(0, None, 2), 0, 0}, c_az_min.index({Slice(0, None, 2)}) * -shift_to_range_);
    d_cart_d_shift.index_put_({Slice(1, None, 2), 0, 0}, c_az_min.index({Slice(1, None, 2)}) * shift_to_range_);
    d_cart_d_shift.index_put_({Slice(0, None, 2), 1, 0}, s_az_min.index({Slice(0, None, 2)}) * -shift_to_range_);
    d_cart_d_shift.index_put_({Slice(1, None, 2), 1, 0}, s_az_min.index({Slice(1, None, 2)}) * shift_to_range_);
    
    auto d_trans_d_cart = torch::empty({nb_azimuths_, 2, 2}, torch::TensorOptions().device(device_).dtype(torch::kFloat64));
    
    d_trans_d_cart.index_put_({Slice(), 0, 0}, c_rot_min);
    d_trans_d_cart.index_put_({Slice(), 0, 1}, -s_rot_min);
    d_trans_d_cart.index_put_({Slice(), 1, 0}, s_rot_min);
    d_trans_d_cart.index_put_({Slice(), 1, 1}, c_rot_min);
    
    d_cart_d_shift = torch::matmul(d_trans_d_cart, d_cart_d_shift);
    
    return {cart, d_cart_d_rot, d_cart_d_shift};
}

torch::Tensor GPStateEstimator::polarCoordCorrection(const torch::Tensor& pos, const torch::Tensor& rot) 
{
    torch::NoGradGuard no_grad;

    auto polar_coord = polar_coord_raw_gp_infered_;
    auto c_az = torch::cos(polar_coord.index({Slice(), Slice(), 0}));
    auto s_az = torch::sin(polar_coord.index({Slice(), Slice(), 0}));

    auto x = c_az * polar_coord.index({Slice(), Slice(), 1});
    auto y = s_az * polar_coord.index({Slice(), Slice(), 1});

    auto c_rot = torch::cos(rot);
    auto s_rot = torch::sin(rot);

    auto x_rot = x * c_rot - y * s_rot;
    auto y_rot = x * s_rot + y * c_rot;

    auto x_trans = x_rot + pos.index({Slice(), 0});
    auto y_trans = y_rot + pos.index({Slice(), 1});

    auto polar = torch::zeros({nb_azimuths_, polar_coord.size(1), 2}, torch::TensorOptions().dtype(torch::kFloat32).device(device_));
    polar.index_put_({Slice(), Slice(), 0}, torch::atan2(y_trans, x_trans));
    polar.index_put_({Slice(), Slice(), 1}, torch::sqrt(x_trans * x_trans + y_trans * y_trans));

    return polar;
}

std::pair<torch::Tensor, torch::Tensor> GPStateEstimator::imgDopplerInterpAndJacobian(const torch::Tensor& shift) 
{
    torch::NoGradGuard no_grad;

    auto shift_int = torch::floor(-shift).to(torch::kInt32);
    auto bin_mat_shifted_int = doppler_bin_vec_sparse_ + shift_int.index({doppler_az_ids_sparse_});
    auto shift_frac = (-shift - shift_int.to(torch::kFloat32)).index({doppler_az_ids_sparse_});

    auto aligned_odd_coeff = odd_coeff_.index({doppler_az_ids_sparse_, bin_mat_shifted_int});
    auto odd_interp = shift_frac * aligned_odd_coeff + odd_bias_.index({doppler_az_ids_sparse_, bin_mat_shifted_int});

    return {odd_interp, -aligned_odd_coeff};
}

torch::Tensor GPStateEstimator::solve(const torch::Tensor& state_init, int nb_iter, double cost_tol, double step_tol, bool verbose, bool degraded) 
{
    torch::NoGradGuard no_grad;

    bool remove_angular = estimate_ang_vel_ && step_counter_ == 0;

    if (step_counter_ == 0 && !use_doppler_) {
        return state_init;
    }

    auto state = state_init.clone();
    auto first_cost = torch::tensor(std::numeric_limits<double>::infinity(), torch::TensorOptions().device(device_));
    auto prev_cost = first_cost.clone();
    double step_quantum = optimisation_first_step_;
    auto last_increasing_state = state.clone();
    auto last_increasing_grad = torch::zeros_like(state);

    for (int i = 0; i < nb_iter; ++i) {
        auto [res, jac] = costFunctionAndJacobian(state, use_doppler_, use_direct_ && (step_counter_ > 0), degraded);

        if (remove_angular && !use_gyro_) {
            jac = jac.index({Slice(), Slice(0, -1)});
        }
        auto jac_view = jac.reshape({-1, jac.size(-1)});
        auto grad = 3 * torch::sum(res.flatten().unsqueeze(-1).pow(2) * jac_view, 0);
        auto cost = torch::sum(res.pow(3).flatten());

        if (i == 0) {
            last_increasing_grad = grad.clone();
        } else {
            if (cost.item<double>() < prev_cost.item<double>()) {
                state = last_increasing_state.clone();
                grad = last_increasing_grad.clone();
                step_quantum /= 2.0;
            } else {
                last_increasing_state = state.clone();
                last_increasing_grad = grad.clone();
            }
        }

        auto grad_norm = torch::linalg_norm(grad);
        if (step_quantum < 1e-5 || grad_norm.item<double>() < 1e-9) break;

        auto step = grad / grad_norm * step_quantum;

        if (remove_angular && !use_gyro_) {
            step = torch::cat({step, torch::zeros({1}, torch::TensorOptions().device(device_))}, 0);
        }

        state += step;

        auto step_norm = torch::linalg_norm(step);
        auto cost_change = cost - prev_cost;
        if (i == 0) first_cost = cost.clone();

        if (verbose) {
            std::cout << "Iter: " << i << " - Cost: " << cost.item<double>()
                      << " - Step norm: " << step_norm.item<double>()
                      << " - Cost change: " << cost_change.item<double>() 
                      << " - Grad norm: " << grad_norm.item<double>() << std::endl;
        }

        if (step_norm.item<double>() < step_tol ||
            torch::abs(cost_change / cost).item<double>() < cost_tol) {
            break;
        }

        prev_cost = cost.clone();
    }

    auto result = motion_model_->getVelPosRot(state, false);
    auto vel = std::get<0>(result);
    bool try_degraded = std::dynamic_pointer_cast<ConstVelConstW>(motion_model_) &&
                        (torch::abs(state[2]).item<double>() > maxAngVel(state.index({Slice(0, 2)})));

    try_degraded = try_degraded ||
                   (torch::abs(torch::norm(vel[-1]) - previous_vel_).item<double>() > max_diff_vel_);

    if (try_degraded && !degraded) {
        state = solve(state_init, nb_iter, cost_tol, step_tol, verbose, true);
    }

    if (!degraded) {
        auto result = motion_model_->getVelPosRot(state, false);
        auto vel = std::get<0>(result);
        previous_vel_ = torch::norm(vel[-1]);
        double last_time = motion_model_->getTime().index({-1}).item<double>();
        max_diff_vel_ = last_time * max_acc_;
    }

    return state;
}

torch::Tensor GPStateEstimator::cartToLocalMapID(const torch::Tensor& xy) 
{
    torch::Tensor out = torch::empty_like(xy, torch::TensorOptions().device(device_));
    out.index_put_({Slice(), Slice(), 0, 0}, xy.index({Slice(), Slice(), 0, 0}) / (-local_map_res_) + local_map_zero_idx_);
    out.index_put_({Slice(), Slice(), 1, 0}, xy.index({Slice(), Slice(), 1, 0}) / local_map_res_ + local_map_zero_idx_);
    return out;
}

torch::Tensor GPStateEstimator::cartToLocalMapIDSparse(const torch::Tensor& xy) 
{
    torch::Tensor out = torch::empty_like(xy, torch::TensorOptions().device(device_));
    out.index_put_({Slice(), 0, 0}, xy.index({Slice(), 0, 0}) / (-local_map_res_) + local_map_zero_idx_);
    out.index_put_({Slice(), 1, 0}, xy.index({Slice(), 1, 0}) / local_map_res_ + local_map_zero_idx_);
    return out;
}

void GPStateEstimator::moveLocalMap(const torch::Tensor& pos, const torch::Tensor& rot) 
{
    torch::NoGradGuard no_grad;
    // Set to zero the first and last row and column of the localMap
    local_map_.index_put_({0, Slice()}, 0);
    local_map_.index_put_({-1, Slice()}, 0);
    local_map_.index_put_({Slice(), 0}, 0);
    local_map_.index_put_({Slice(), -1}, 0);

    // Get the coordinate of the new localMap in the former localMap
    auto cos_rot = torch::cos(rot);
    auto sin_rot = torch::sin(rot);
    torch::Tensor temp_rot_mat = torch::stack({torch::stack({cos_rot, -sin_rot}),
                                               torch::stack({sin_rot, cos_rot})}).to(device_);
    torch::Tensor temp_pos = pos.reshape({-1,1});

    // Get the new coordinates
    torch::Tensor new_xy = torch::matmul(temp_rot_mat, local_map_xy_.to(torch::kFloat64)) + temp_pos;
    torch::Tensor new_idx = cartToLocalMapID(new_xy);

    // Get the new localMap via bilinear interpolation
    
    auto [local_map, _] = bilinearInterpolation(local_map_, new_idx);
    local_map_ = local_map.squeeze().to(torch::kFloat32);
    return;
}

torch::Tensor GPStateEstimator::odometryStep(const torch::Tensor& polar_image, const torch::Tensor& azimuths, const torch::Tensor& timestamps, bool chirp_up) 
{
    torch::NoGradGuard no_grad;

    CLOG(DEBUG, "radar.gp_doppler") << "****** I am in the odometry step and the step counter is: **************" << step_counter_;

    chirp_up_ = chirp_up;

    double last_scan_time;
    if (!timestamps_.defined()) 
    {
        
        double t_start = timestamps[0].item<double>();
        double t_end = timestamps[timestamps.size(0) - 1].item<double>();
        last_scan_time = t_start - (t_end - t_start);
        max_diff_vel_ = max_acc_ * (t_end - t_start) * 1e-5; // Sam suspicious 
    } 
    else 
    {
        last_scan_time = timestamps_[0].item<double>();
    }

    timestamps_ = timestamps.to(device_).squeeze();
    double delta_time = 0.25;

    if (pose_estimation_) 
    {
        if (step_counter_ > 0) 
        {   

            auto result = motion_model_->getVelPosRot(state_init_, false);
            auto vel_body = std::get<0>(result);
            auto prev_scan_pos = std::get<2>(result);
            auto prev_scan_rot = std::get<4>(result);
            auto [frame_pos, frame_rot] = motion_model_->getPosRotSingle(state_init_, timestamps_[0].item<double>());
            
            auto cos_rot = torch::cos(current_rot_);
            auto sin_rot = torch::sin(current_rot_);
            auto rot_mat = torch::stack({torch::stack({cos_rot, -sin_rot}), torch::stack({sin_rot, cos_rot})}).to(device_);
            current_pos_ = current_pos_ + torch::matmul(rot_mat, frame_pos.to(torch::kFloat64));
            current_rot_ = current_rot_ + frame_rot.to(torch::kFloat64);
            
            if (std::dynamic_pointer_cast<ConstVelConstW>(motion_model_)) 
            {
                current_rot_ -= ang_vel_bias_ * delta_time;
            }
            
            if (use_direct_) 
            {
                auto shift = torch::matmul(vel_body.view({-1, 1, 2}), vel_to_bin_vec_.view({-1, 2, 1})).squeeze();
                auto per_line_shift = shift / 2.0;
                
                if (!prev_chirp_up_) 
                {
                    per_line_shift = -per_line_shift;
                }
                if (doppler_radar_) 
                {
                    per_line_shift.index_put_({Slice(1, None, 2)}, per_line_shift.index({Slice(1, None, 2)}) * -1);
                }
                auto cos_prev = torch::cos(prev_scan_rot);
                auto sin_prev = torch::sin(prev_scan_rot);
                auto rot_mats_transposed = torch::stack({torch::stack({cos_prev, sin_prev}, 1), torch::stack({-sin_prev, cos_prev}, 1)}, 2).squeeze(-1);
                
                prev_scan_pos = prev_scan_pos.view({-1, 2, 1});
                
                auto pos = torch::matmul(rot_mats_transposed, (-prev_scan_pos + frame_pos.view({-1, 2, 1})));
                auto rot = -prev_scan_rot + frame_rot;
                
                auto polar_coord_corrected = polarCoordCorrection(pos, rot);
                
                auto prev_shifted = perLineInterpolation(polar_intensity_, per_line_shift);

                polar_coord_corrected.index_put_({Slice(), Slice(), 0}, polar_coord_corrected.index({Slice(), Slice(), 0}) - azimuths_[0].to(torch::kFloat32));
                auto mask_neg = polar_coord_corrected.index({Slice(), Slice(), 0}) < 0;
                polar_coord_corrected.index_put_({mask_neg}, polar_coord_corrected.index({mask_neg}) + torch::tensor({2.0 * M_PI, 0.0}, device_));
                polar_coord_corrected.index_put_({Slice(), Slice(), 0}, polar_coord_corrected.index({Slice(), Slice(), 0}) * (static_cast<float>(nb_azimuths_) / (2.0 * M_PI)));
                polar_coord_corrected.index_put_({Slice(), Slice(), 1}, (polar_coord_corrected.index({Slice(), Slice(), 1}) - radar_res_ / 2.0) / radar_res_);
                
                prev_shifted = torch::cat({prev_shifted, prev_shifted.index({0}).unsqueeze(0)}, 0);
                auto polar_target = bilinearInterpolation(prev_shifted, polar_coord_corrected).first;

                auto temp_polar = local_map_polar_.clone();

                temp_polar.index_put_({Slice(), Slice(), 0}, temp_polar.index({Slice(), Slice(), 0}) - azimuths_[0].to(torch::kFloat32));
                auto mask_temp_neg = temp_polar.index({Slice(), Slice(), 0}) < 0;
                temp_polar.index_put_({mask_temp_neg}, temp_polar.index({mask_temp_neg}) + torch::tensor({2.0 * M_PI, 0.0}, device_).to(torch::kFloat32));
                temp_polar.index_put_({Slice(), Slice(), 0}, temp_polar.index({Slice(), Slice(), 0}) * static_cast<float>(nb_azimuths_ / (2.0 * M_PI)));
                temp_polar.index_put_({Slice(), Slice(), 1}, ((temp_polar.index({Slice(), Slice(), 1}) - static_cast<float>(radar_res_ / 2.0)) / static_cast<float>(radar_res_)));
                
                polar_target = torch::cat({polar_target, polar_target.index({0}).unsqueeze(0)}, 0);
                auto local_map_update = bilinearInterpolation(polar_target, temp_polar).first.to(torch::kFloat32);
                
                if (step_counter_ == 1) 
                {   
                    local_map_.index_put_({local_map_mask_}, local_map_update.index({local_map_mask_}));
                } 
                else 
                {
                    moveLocalMap(frame_pos, frame_rot);
                    local_map_.index_put_({local_map_mask_}, one_minus_alpha_ * local_map_.index({local_map_mask_}) + alpha_ * local_map_update.index({local_map_mask_}).to(torch::kFloat32));
                }
                
                local_map_blurred_ = applyGaussianBlur2D(local_map_.unsqueeze(0).unsqueeze(0), 3, 3, 1, 1).squeeze(0).squeeze(0);
                auto normalizer = torch::max(local_map_) / torch::max(local_map_blurred_).item<double>();
                local_map_blurred_ *= normalizer;
            }
        }
    }
    
    azimuths_ = azimuths.to(device_).to(torch::kFloat64);
    nb_azimuths_ = azimuths.size(0);
    motion_model_->setTime(timestamps_, timestamps_[0].item<double>());
    
    auto dirs = torch::empty({nb_azimuths_, 2}, torch::TensorOptions().device(device_).dtype(torch::kFloat64));
    dirs.index_put_({Slice(), 0}, torch::cos(azimuths_));
    dirs.index_put_({Slice(), 1}, torch::sin(azimuths_));
    vel_to_bin_vec_ = vel_to_bin_ * dirs;

    if (use_doppler_) {
        
        auto range_slice = polar_image.index({Slice(), Slice(min_range_idx_, max_range_idx_ + 1)});
        auto up_down_imgs = getUpDownPolarImages(range_slice);
        odd_img_ = up_down_imgs.first;
        even_img_ = up_down_imgs.second;
        
        auto pad = torch::zeros({nb_azimuths_, static_cast<int>(kImgPadding_)}, torch::TensorOptions().dtype(torch::kFloat32).device(device_));
        temp_even_img_ = torch::cat({pad, even_img_, pad}, 1);
        auto temp_odd_img = torch::cat({pad, odd_img_, pad}, 1);
        
        odd_coeff_ = torch::empty_like(temp_even_img_, torch::TensorOptions().device(device_).dtype(torch::kFloat32));
        odd_coeff_.index_put_({Slice(), Slice(None, -1)}, temp_odd_img.index({Slice(), Slice(1, None)}) - temp_odd_img.index({Slice(), Slice(None, -1)}));
        odd_coeff_.index_put_({Slice(), -1}, 0);
        odd_bias_ = temp_odd_img.clone();
        auto mask_doppler = temp_even_img_ != 0;
        temp_even_img_sparse_ = temp_even_img_.index({mask_doppler});
        
        doppler_az_ids_sparse_ = torch::arange(nb_azimuths_, torch::TensorOptions().device(device_).dtype(torch::kInt32)).unsqueeze(1).repeat({1, temp_even_img_.size(1)}).index({mask_doppler});
        doppler_bin_vec_sparse_ = torch::arange(nb_bins_, torch::TensorOptions().device(device_).dtype(torch::kInt32)).unsqueeze(0).repeat({nb_azimuths_, 1}).index({mask_doppler});
    }
    
    if (use_direct_) 
    {
        if (use_doppler_) 
        {
            polar_intensity_ = torch::zeros({azimuths.size(0), min_range_idx_ + odd_img_.size(1)}, torch::TensorOptions().device(device_).dtype(torch::kFloat32));
            polar_intensity_.index_put_({Slice(None, None, 2), Slice(min_range_idx_, None)}, even_img_.index({Slice(None, None, 2), Slice()}));
            polar_intensity_.index_put_({Slice(1, None, 2), Slice(min_range_idx_, None)}, odd_img_.index({Slice(1, None, 2), Slice()}));
        } 
        else 
        {
            polar_intensity_ = polar_image.to(device_);
            auto polar_std = torch::std(polar_intensity_, 1);
            auto polar_mean = torch::mean(polar_intensity_, 1);
            polar_intensity_ -= (polar_mean.unsqueeze(1) + 2 * polar_std.unsqueeze(1));
            polar_intensity_ = torch::where(polar_intensity_ < 0, torch::zeros_like(polar_intensity_), polar_intensity_);
            polar_intensity_ = applyGaussianBlur2D(polar_intensity_.unsqueeze(0), 9, 1, 3.0, 3.0).squeeze(0);
            polar_intensity_ /= std::get<0>(torch::max(polar_intensity_, 1, true));
            polar_intensity_ = torch::where(torch::isnan(polar_intensity_), torch::zeros_like(polar_intensity_), polar_intensity_);
        }

        auto range_vec = torch::arange(max_range_idx_, torch::TensorOptions().device(device_).dtype(torch::kFloat32)) * radar_res_ + (radar_res_ * 0.5);

        polar_coord_raw_gp_infered_ = torch::zeros({nb_azimuths_, max_range_idx_, 2}, torch::TensorOptions().device(device_).dtype(torch::kFloat32));
        polar_coord_raw_gp_infered_.index_put_({Slice(), Slice(), 0}, (azimuths_.unsqueeze(1).repeat({1, max_range_idx_})).to(torch::kFloat32));
        polar_coord_raw_gp_infered_.index_put_({Slice(), Slice(), 1}, range_vec.unsqueeze(0).repeat({nb_azimuths_, 1}));

        auto temp_intensity = polar_intensity_.index({Slice(), Slice(None, max_range_idx_direct_)}).to(torch::kFloat32);
        auto mask_direct = temp_intensity != 0;
        mask_direct.index_put_({Slice(), Slice(None, min_range_idx_direct_)}, false);
        polar_intensity_sparse_ = temp_intensity.index({mask_direct});
        direct_r_sparse_ = range_vec_.unsqueeze(0).repeat({nb_azimuths_, 1}).index({mask_direct});
        direct_az_ids_sparse_ = torch::arange(nb_azimuths_, torch::TensorOptions().device(device_).dtype(torch::kInt32)).unsqueeze(1).repeat({1, max_range_idx_direct_}).index({mask_direct});

        direct_r_ids_sparse_ = torch::arange(max_range_idx_direct_, torch::TensorOptions().device(device_).dtype(torch::kInt32)).unsqueeze(0).repeat({nb_azimuths_, 1}).index({mask_direct});
        if (doppler_radar_) 
        {
            auto mde = torch::empty_like(mask_direct, torch::TensorOptions().dtype(torch::kBool).device(device_));
            mde.index_put_({Slice(1, None, 2)}, false);
            mde.index_put_({Slice(None, None, 2)}, true);
            mask_direct_even_ = mde.index({mask_direct});

            auto mdo = torch::empty_like(mask_direct, torch::TensorOptions().dtype(torch::kBool).device(device_));
            mdo.index_put_({Slice(None, None, 2)}, false);
            mdo.index_put_({Slice(1, None, 2)}, true);
            mask_direct_odd_ = mdo.index({mask_direct});
        } 
        else 
        {
            mask_direct_even_ = torch::ones_like(polar_intensity_sparse_, torch::TensorOptions().dtype(torch::kBool).device(device_));
            mask_direct_odd_ = torch::zeros_like(polar_intensity_sparse_, torch::TensorOptions().dtype(torch::kBool).device(device_));
        }

        direct_nb_non_zero_ = polar_intensity_sparse_.size(0);
        direct_r_ids_even_ = direct_r_ids_sparse_.index({mask_direct_even_});
        direct_r_ids_odd_ = direct_r_ids_sparse_.index({mask_direct_odd_});
        direct_r_even_ = direct_r_sparse_.index({mask_direct_even_});
        direct_r_odd_ = direct_r_sparse_.index({mask_direct_odd_});
        direct_az_ids_even_ = direct_az_ids_sparse_.index({mask_direct_even_});
        direct_az_ids_odd_ = direct_az_ids_sparse_.index({mask_direct_odd_});
    }

    // Final result from gradient solver
    if (motion_model_->getStateSize() == 3 && use_gyro_) {
        state_init_.index({Slice(None, 2)}) *= (1 + state_init_[2].item<double>() * delta_time);
    }
    if (torch::norm(state_init_.index({Slice(None, 2)})).item<double>() < 0.75) {
        state_init_.zero_();
    }

    // visualization
    auto polar_intensity_cpu = polar_intensity_.detach().cpu().to(torch::kFloat32);
    auto polar_intensity_cv = tensorToMat(polar_intensity_cpu);
    // auto local_map_blurred_cv = tensorToMat(local_map_blurred_cpu);
    cv::resize(polar_intensity_cv, polar_intensity_cv, cv::Size(), 0.3, 0.3, cv::INTER_LINEAR);
    
    cv::namedWindow("polar_intensity_cv", cv::WINDOW_AUTOSIZE);
    cv::imshow("polar_intensity_cv", polar_intensity_cv);
    
    
    // cv::waitKey(0);
    // cv::destroyAllWindows();
    // if (step_counter_ > 0) {
    //         auto local_map_blurred_cpu = local_map_blurred_.detach().cpu().to(torch::kFloat32);
    //         auto local_map_blurred_cv = tensorToMat(local_map_blurred_cpu);
    //         cv::resize(local_map_blurred_cv, local_map_blurred_cv, cv::Size(), 0.3, 0.3, cv::INTER_LINEAR);
    //         cv::namedWindow("local_map_blurred_cv", cv::WINDOW_AUTOSIZE);
    //         cv::imshow("local_map_blurred_cv", local_map_blurred_cv);
    //         cv::waitKey(0);
    //     }
        //     cv::destroyWindow("local_map_blurred_cv");
        // }
        // cv::namedWindow("local_map_blurred_cv", cv::WINDOW_AUTOSIZE);
        // cv::imshow("local_map_blurred_cv", local_map_blurred_cv);
        // cv::waitKey(0);
        // cv::destroyWindow("local_map_blurred_cv");
        
    auto result = solve(state_init_, 250, 1e-6, 1e-5);

    // auto local_map_cpu = local_map_.detach().cpu().to(torch::kFloat32);
    // auto local_map_cv = tensorToMat(local_map_cpu);
    // cv::resize(local_map_cv, local_map_cv, cv::Size(), 0.3, 0.3, cv::INTER_LINEAR);
    // cv::imshow("local_map_cv", local_map_cv);
    // cv::waitKey(1);
        
    if (std::dynamic_pointer_cast<ConstVelConstW>(motion_model_)) {
        if (step_counter_ > 0) {
            if (std::abs(result[2].item<double>()) > maxAngVel(result.index({Slice(None, 2)}))) {
                result[2] = prev_state_[2];
            }
        }
        prev_state_ = result.clone();
    }

    state_init_ = result.clone();
    prev_chirp_up_ = chirp_up;
    step_counter_ += 1;

    return result.detach().cpu();
}

torch::Tensor GPStateEstimator::getDopplerVelocity() {
    if (!use_doppler_) {
        throw std::runtime_error("Doppler not used");
    }

    bool save_use_direct = use_direct_;
    use_direct_ = false;
    auto result = solve(state_init_, 250, 1e-6, 1e-5);
    use_direct_ = save_use_direct;

    return result.index({Slice(None, 2)}).detach().cpu();
}

std::pair<OptionalTensor, OptionalTensor> GPStateEstimator::getAzPosRot() {
    if (!pose_estimation_) {
        return {std::nullopt, std::nullopt};
    }

    auto c = torch::cos(current_rot_);
    auto s = torch::sin(current_rot_);
    auto rot_mat = torch::stack({torch::stack({c, -s}), torch::stack({s, c})}).to(device_);

    auto result = motion_model_->getVelPosRot(state_init_, false);
    torch::Tensor scan_pos = std::get<2>(result);
    torch::Tensor scan_rot = std::get<4>(result);

    auto pos = rot_mat.matmul(scan_pos.to(torch::kFloat64)) + current_pos_.unsqueeze(1);
    auto rot = scan_rot.to(torch::kFloat64) + current_rot_;

    return {pos.detach().cpu(), rot.detach().cpu()};
}

void GPStateEstimator::setGyroData(const std::vector<double>& imu_time,
                 const std::vector<double>& imu_yaw) {
    auto options = torch::TensorOptions()
                     .dtype(torch::kFloat64)
                     .device(device_);
    auto t_time = torch::tensor(imu_time, options);
    auto t_yaw  = torch::tensor(imu_yaw,  options);
    std::dynamic_pointer_cast<ConstBodyVelGyro>(motion_model_)->setGyroData(t_time, t_yaw);
    return;
}

void GPStateEstimator::setGyroBias(const double& gyro_bias) {
    auto options = torch::TensorOptions()
                   .dtype(torch::kFloat64)
                   .device(device_);
    auto t2 = torch::tensor(gyro_bias, options);
    std::dynamic_pointer_cast<ConstBodyVelGyro>(motion_model_)->setGyroBias(t2);
    return;
}


}  // namespace radar
}  // namespace vtr
// usefull resource: https://docs.pytorch.org/cppdocs/notes/tensor_indexing.html
#include "vtr_radar/modules/odometry/dense_utils/motion_models.hpp"

namespace vtr {
namespace radar {

using torch::indexing::Slice;
using torch::indexing::None;

void MotionModel::setTime(const torch::Tensor& time, const double& t0) 
{
    time_ = (time - t0).to(torch::kFloat32) * 1.0e-6;
    t0_ = t0;
    num_steps_ = torch::tensor(time.size(0), device_);
    return;
}

double MotionModel::getLocalTime(const double& time) const 
{
    return (time - t0_) * 1.0e-6;
}

torch::Tensor MotionModel::getInitialState() const 
{
    return torch::zeros(state_size_, torch::TensorOptions().device(device_).dtype(torch::kFloat64));
}


std::tuple<torch::Tensor, OptionalTensor, torch::Tensor, OptionalTensor, torch::Tensor, OptionalTensor> 
ConstVelConstW::getVelPosRot(const torch::Tensor& state, bool with_jac) 
{
    // rotation = omega * dt
    auto rot = state[2] * time_.unsqueeze(1);
    // position = [v_x * t, v_y * t]
    auto pos = torch::stack({state[0] * time_, state[1] * time_}, 1).unsqueeze(2);
    // body velocity tensor
    auto vel = state.slice(0, 0, 2).unsqueeze(0).unsqueeze(2).clone(); //check maybe use .index({Slice()})

    auto c = torch::cos(rot);
    auto s = torch::sin(rot);
    
    auto vx = vel.squeeze(-1).index({Slice(), 0});
    auto vy = vel.squeeze(-1).index({Slice(), 1});
    

    auto vx_c = vx * c;
    auto vy_s = vy * s;
    auto vx_s = vx * s;
    auto vy_c = vy * c;

    auto vel_body = torch::cat({vx_c + vy_s, vy_c - vx_s}, 1);
    
    if (!with_jac)
        return {vel_body, std::nullopt, pos, std::nullopt, rot, std::nullopt};

    auto d_rot_d_state = torch::zeros({num_steps_.item<int64_t>(), 1, 1}, torch::TensorOptions().device(device_).dtype(torch::kFloat64));
    d_rot_d_state.index_put_({torch::indexing::Ellipsis, 0, 0}, time_);             // torch::indexing::Ellipsis should be equal to Slice() check

    auto d_pos_d_state = torch::zeros({num_steps_.item<int64_t>(), 2, 3}, torch::TensorOptions().device(device_).dtype(torch::kFloat64));
    d_pos_d_state.index_put_({Slice(), 0, 0}, time_);
    d_pos_d_state.index_put_({Slice(), 1, 1}, time_);

    auto d_vel_body_d_state = torch::zeros({num_steps_.item<int64_t>(), 2, 3}, torch::TensorOptions().device(device_).dtype(torch::kFloat64));
    d_vel_body_d_state.index_put_({Slice(), 0, 0}, c.squeeze());
    d_vel_body_d_state.index_put_({Slice(), 0, 1}, s.squeeze());
    d_vel_body_d_state.index_put_({Slice(), 1, 0}, -s.squeeze());
    d_vel_body_d_state.index_put_({Slice(), 1, 1}, c.squeeze());
    d_vel_body_d_state.index_put_({Slice(), 0, 2}, (vy_c - vx_s).squeeze() * time_);
    d_vel_body_d_state.index_put_({Slice(), 1, 2}, (-vx_c - vy_s).squeeze() * time_);

    return {vel_body, d_vel_body_d_state, pos, d_pos_d_state, rot, d_rot_d_state};
}

std::tuple<torch::Tensor, torch::Tensor> 
ConstVelConstW::getPosRotSingle(const torch::Tensor& state, const double& time) 
{
    torch::NoGradGuard no_grad;
    
    auto local_time = getLocalTime(time);
    auto rot = state[2] * local_time;
    auto pos = torch::stack({state[0] * local_time, state[1] * local_time});
    return {pos, rot};
}


void ConstBodyVelGyro::setGyroData(const torch::Tensor& gyro_time, const torch::Tensor& gyro_yaw) 
{
    first_gyro_time_ = gyro_time[0];
    gyro_time_ = (gyro_time - first_gyro_time_).to(torch::kFloat64).to(device_);
    gyro_yaw_ = gyro_yaw.to(torch::kFloat64).to(device_);
    gyro_yaw_original_ = gyro_yaw_.clone();

    auto dt = gyro_time_.slice(0, 1) - gyro_time_.slice(0, 0, -1);
    bin_integral_ = 0.5 * (gyro_yaw_.slice(0, 1) + gyro_yaw_.slice(0, 0, -1)) * dt;
    coeff_ = (gyro_yaw_.slice(0, 1) - gyro_yaw_.slice(0, 0, -1)) / dt;
    // I wanted to see if indexing using -1 is the same as using the last element
    auto coeff_last = coeff_.index({coeff_.size(0) - 1}).unsqueeze(0);
    // std::cout << "Sam: Comparing coeff_last with coeff_[-1]: " << coeff_last << " " << coeff_[-1].unsqueeze(0) << std::endl;
    coeff_ = torch::cat({coeff_, coeff_[-1].unsqueeze(0)}, 0);
    // I want to print the same for offset_
    offset_ = gyro_yaw_.slice(0, 0, -1) - coeff_.slice(0, 0, -1) * gyro_time_.slice(0, 0, -1);
    auto offset_last = offset_.index({offset_.size(0) - 1}).unsqueeze(0);
    // std::cout << "Sam: Comparing offset_last with offset_[-1]: " << offset_last << " " << offset_[-1].unsqueeze(0) << std::endl;
    offset_ = torch::cat({offset_, offset_[-1].unsqueeze(0)}, 0);

    initialised_ = true;
    return;
}

void ConstBodyVelGyro::setGyroBias(const torch::Tensor& gyro_bias) 
{
    gyro_yaw_ = gyro_yaw_original_ - gyro_bias;

    auto dt = gyro_time_.slice(0, 1) - gyro_time_.slice(0, 0, -1);
    bin_integral_ = 0.5 * (gyro_yaw_.slice(0, 1) + gyro_yaw_.slice(0, 0, -1)) * dt;
    coeff_ = (gyro_yaw_.slice(0, 1) - gyro_yaw_.slice(0, 0, -1)) / dt;
    coeff_ = torch::cat({coeff_, coeff_[-1].unsqueeze(0)}, 0);
    offset_ = gyro_yaw_.slice(0, 0, -1) - coeff_.slice(0, 0, -1) * gyro_time_.slice(0, 0, -1);
    offset_ = torch::cat({offset_, offset_[-1].unsqueeze(0)}, 0);
    return;
}

void ConstBodyVelGyro::setTime(const torch::Tensor& time, const double& t0) // you select the imu relative to radar scan 
{
    if (!initialised_) throw std::runtime_error("Gyro data not set");

    MotionModel::setTime(time, t0);

    auto time_local = time.to(torch::kFloat64) * 1e-6 - first_gyro_time_;
    auto time_start = t0 * 1e-6 - first_gyro_time_;
    auto start_idx = torch::searchsorted(gyro_time_, time_start);
    auto end_idx = torch::searchsorted(gyro_time_, time_local);

    if (end_idx[-1].item<int>() == 0)
        throw std::runtime_error("Time is before the first gyro data: Currently not supported");

    
    auto mask = start_idx == end_idx;
    r_ = torch::zeros_like(time, torch::TensorOptions().device(device_).dtype(torch::kFloat64));
    r_.index_put_({mask}, ((time_local.index({mask}) * coeff_.index({start_idx - 1}) + offset_.index({start_idx - 1}) +
                    time_start * coeff_.index({start_idx - 1}) + offset_.index({start_idx - 1})) *
                    0.5 * (time_local.index({mask}) - time_start)).to(torch::kFloat64));
                    
    auto first_bucket = (coeff_.index({start_idx - 1}) * time_start + offset_.index({start_idx - 1}) + gyro_yaw_.index({start_idx})) *
                        0.5 * (gyro_time_.index({start_idx}) - time_start);

    auto last_bucket = (coeff_.index({end_idx - 1}) * time_local + offset_.index({end_idx - 1}) + gyro_yaw_.index({end_idx - 1})) *
                       0.5 * (time_local - gyro_time_.index({end_idx - 1}));

    auto mask_one = start_idx + 1 == end_idx;
    r_.index_put_({mask_one}, (first_bucket + last_bucket.index({mask_one})).to(torch::kFloat64));

    auto mask_rest = start_idx + 1 < end_idx;
    if (mask_rest.any().item<bool>()) {
        auto cumulative_integral = torch::cumsum(bin_integral_.slice(0, start_idx.item<int>(), end_idx.max().item<int>()), 0);
        r_.index_put_({mask_rest}, (first_bucket + cumulative_integral.index({end_idx.index({mask_rest}) - 2 - start_idx}) + last_bucket.index({mask_rest})).to(torch::kFloat64));
    }

    cos_r_ = torch::cos(r_);
    sin_r_ = torch::sin(r_);

    auto delta_time = time_local.slice(0, 1) - time_local.slice(0, 0, -1);
    auto cumulative_cos_r = torch::cumsum((cos_r_.slice(0, 0, -1) + cos_r_.slice(0, 1)) * 0.5 * delta_time, 0);
    auto cumulative_sin_r = torch::cumsum((sin_r_.slice(0, 0, -1) + sin_r_.slice(0, 1)) * 0.5 * delta_time, 0);

    R_integral_ = torch::zeros({time.size(0), 2, 2}, torch::TensorOptions().device(device_).dtype(torch::kFloat64));
    R_integral_[0] = 0;
    R_integral_.slice(0, 1).select(1, 0).select(1, 0) = cumulative_cos_r;
    R_integral_.slice(0, 1).select(1, 0).select(1, 1) = -cumulative_sin_r;
    R_integral_.slice(0, 1).select(1, 1).select(1, 0) = cumulative_sin_r;
    R_integral_.slice(0, 1).select(1, 1).select(1, 1) = cumulative_cos_r;

    // i want to print what R_integral is
    // what is the size of R_integral_
    // std::cout << "Sam: R_integral_: " << R_integral_ << std::endl;
    // std::cout << "Sam: R_integral_ size: " << R_integral_.sizes() << std::endl;

    return;
}

std::tuple<torch::Tensor, OptionalTensor, torch::Tensor, OptionalTensor, torch::Tensor, OptionalTensor>
ConstBodyVelGyro::getVelPosRot(const torch::Tensor& state, bool with_jac) 
{
    auto rot = r_.unsqueeze(1).clone();
    auto body_vel = state.unsqueeze(0).clone();
    auto pos = torch::matmul(R_integral_, body_vel.unsqueeze(2));

    if (!with_jac) {
        return {body_vel, std::nullopt, pos, std::nullopt, rot, std::nullopt};
    }

    OptionalTensor d_rot_d_state = std::nullopt;

    torch::Tensor d_vel_body_d_state = torch::zeros({1, 2, 2}, torch::TensorOptions().device(device_).dtype(torch::kFloat64));
    d_vel_body_d_state.index_put_({Slice(), 0, 0}, 1.0);
    d_vel_body_d_state.index_put_({Slice(), 1, 1}, 1.0);

    OptionalTensor d_pos_d_state = R_integral_.clone();
    return {body_vel, d_vel_body_d_state, pos, d_pos_d_state, rot, d_rot_d_state};
}

std::tuple<torch::Tensor, torch::Tensor> ConstBodyVelGyro::getPosRotSingle(const torch::Tensor& state, const double& time) {
    torch::NoGradGuard no_grad;
    
    auto times = torch::arange(t0_, time, 625, torch::kInt64).to(device_);
    if (times[-1].item<int64_t>() != time) {
        auto new_time_tensor = torch::tensor(time, torch::TensorOptions().dtype(torch::kInt64).device(times.device())).unsqueeze(0);
        times = torch::cat({times, new_time_tensor}, 0);
    }
    setTime(times, t0_);

    auto pos = torch::matmul(R_integral_[-1], state.unsqueeze(1)).squeeze();
    auto rot = r_[-1];
    return {pos, rot};
}


std::tuple<torch::Tensor, OptionalTensor, torch::Tensor, OptionalTensor, torch::Tensor, OptionalTensor>
ConstVel::getVelPosRot(const torch::Tensor& state, bool with_jac) {
    auto vel = state.unsqueeze(0).clone();

    if (!with_jac) {
        return {vel, std::nullopt, torch::Tensor(), std::nullopt, torch::Tensor(), std::nullopt};
    }

    torch::Tensor d_vel_d_state = torch::zeros({1, 2, 2}, device_);
    d_vel_d_state.index_put_({Slice(), 0, 0}, 1);
    d_vel_d_state.index_put_({Slice(), 1, 1}, 1);
    return {vel, d_vel_d_state, torch::Tensor(), std::nullopt, torch::Tensor(), std::nullopt};
}

std::tuple<torch::Tensor, torch::Tensor> ConstVel::getPosRotSingle(const torch::Tensor& state, const double& time) {
    throw std::runtime_error("Querying single position and rotation is not sensible for constant velocity model");
    return {torch::Tensor(), torch::Tensor()};
}

}  // namespace radar
}  // namespace vtr
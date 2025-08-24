#pragma once

#include <torch/torch.h>
#include <memory>
#include <optional>
#include <stdexcept>

namespace vtr {
namespace radar {

using OptionalTensor = std::optional<torch::Tensor>;

class MotionModel {
public:
    MotionModel(int state_size, torch::Device device = torch::kCPU)
        : state_size_(state_size), device_(device) {}

    virtual ~MotionModel() = default;

        /** \brief Static module identifier. */
    static constexpr auto static_name = "radar.motion_model";

    virtual void setTime(const torch::Tensor& time, const double& t0);
    virtual double getLocalTime(const double& time) const;
    virtual torch::Tensor getInitialState() const;

    virtual std::tuple<torch::Tensor, OptionalTensor, torch::Tensor, OptionalTensor, torch::Tensor, OptionalTensor> 
        getVelPosRot(const torch::Tensor& state, bool with_jac = false) = 0;

    virtual std::tuple<torch::Tensor, torch::Tensor> getPosRotSingle(
        const torch::Tensor& state, const double& time) = 0;

    torch::Tensor getTime() const {
        return time_;
    }
    int getStateSize() const {
        return state_size_;
    }

protected:
    int state_size_;
    torch::Device device_;
    torch::Tensor time_;
    double t0_;
    torch::Tensor num_steps_;
};


// ConstVelConstW model
class ConstVelConstW : public MotionModel {
public:
    ConstVelConstW(torch::Device device = torch::kCPU)
        : MotionModel(3, device) {}

    std::tuple<torch::Tensor, OptionalTensor, torch::Tensor, OptionalTensor, torch::Tensor, OptionalTensor> 
        getVelPosRot(const torch::Tensor& state, bool with_jac = false) override;

    virtual std::tuple<torch::Tensor, torch::Tensor> getPosRotSingle(
        const torch::Tensor& state, const double& time) override;
};

// ConstBodyVelGyro model
class ConstBodyVelGyro : public MotionModel {
public:
    ConstBodyVelGyro(torch::Device device = torch::kCPU)
        : MotionModel(2, device), initialised_(false) {}

    void setGyroData(const torch::Tensor& gyro_time, const torch::Tensor& gyro_yaw);
    void setGyroBias(const torch::Tensor& gyro_bias);
    void setTime(const torch::Tensor& time, const double& t0) override;

    std::tuple<torch::Tensor, OptionalTensor, torch::Tensor, OptionalTensor, torch::Tensor, OptionalTensor>
        getVelPosRot(const torch::Tensor& state, bool with_jac = false) override;

    virtual std::tuple<torch::Tensor, torch::Tensor> getPosRotSingle(
        const torch::Tensor& state, const double& time) override;

private:
    bool initialised_;
    torch::Tensor first_gyro_time_;
    torch::Tensor gyro_time_, gyro_yaw_, gyro_yaw_original_;
    torch::Tensor bin_integral_, coeff_, offset_;
    torch::Tensor r_, cos_r_, sin_r_, R_integral_;
};

// ConstVel model
class ConstVel : public MotionModel {
public:
    ConstVel(torch::Device device = torch::kCPU)
        : MotionModel(2, device) {}

    std::tuple<torch::Tensor, OptionalTensor, torch::Tensor, OptionalTensor, torch::Tensor, OptionalTensor> 
        getVelPosRot(const torch::Tensor& state, bool with_jac = false) override;

    virtual std::tuple<torch::Tensor, torch::Tensor> getPosRotSingle(
        const torch::Tensor& state, const double& time) override;
};

}  // namespace radar
}  // namespace vtr

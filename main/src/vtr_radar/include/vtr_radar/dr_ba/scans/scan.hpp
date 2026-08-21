// scan.hpp
#pragma once

#include <Eigen/Dense>
#include <lgmath/se3/Transformation.hpp>
#include <lgmath/se2/Transformation.hpp>
#include <lgmath/se3/Operations.hpp>
#include <vtr_radar/dr_ba/utils/ba_config.hpp>
#include <random>
#include <optional>

namespace ba {

class Scan {
public:
	struct Measurement {
		double x;
		double y;
		double intensity;
		double covariance;
		Eigen::Matrix<double, 1, 3> jacobian; // Jacobian w.r.t. SE(2) pose
	};

	virtual ~Scan() = default;

	// Identification
	int id() const { return id_; }

	int64_t timestamp() const { return timestamp_; }

	// Pose accessors
	const lgmath::se3::Transformation &pose() const { return pose_; }
	const lgmath::se2::Transformation pose2d() const { return pose_.toSE2(); }
	const lgmath::se3::Transformation &gt_pose() const { return gt_pose_; }
	const lgmath::se2::Transformation gt_pose2d() const { return gt_pose_.toSE2(); }

	// Update pose
	void update_pose(const Eigen::Matrix<double, 6, 1> &delta_xi) {
		lgmath::se3::Transformation T_update((-delta_xi).eval());
		pose_ = pose_ * T_update;
	}

	void update_pose(const Eigen::Matrix<double, 3, 1> &delta_xi) {
		Eigen::Matrix<double, 6, 1> delta_xi_se3;
		delta_xi_se3 << delta_xi(0), delta_xi(1), 0.0, 0.0, 0.0, delta_xi(2);
		update_pose(delta_xi_se3);
	}

	void set_pose(const lgmath::se3::Transformation &new_pose) {
		pose_ = new_pose;
	}
	void set_gt_pose(const lgmath::se3::Transformation &new_gt_pose) {
		gt_pose_ = new_gt_pose;
	}

	void apply_noise_to_pose(double pos_stddev, double yaw_stddev) {
		// Initialize uniform distribution for noise
		std::uniform_real_distribution<double> translation_dist(-pos_stddev, pos_stddev);
		double rotation_std_rad = yaw_stddev* M_PI / 180.0;
		std::uniform_real_distribution<double> rotation_dist(-rotation_std_rad, rotation_std_rad);
		std::mt19937 rng(99); // Fixed seed for reproducibility

		// Create random noise
		Eigen::Vector3d noise;
		noise << translation_dist(rng), translation_dist(rng), rotation_dist(rng);
		lgmath::se3::Transformation T_noise = lgmath::se2::Transformation(noise).toSE3();

		// Apply noise
		pose_ = pose_ * T_noise;
	}

	// Compute pose error (SE3)
	Eigen::Matrix<double, 6, 1> pose_error() const {
		lgmath::se3::Transformation T_err = gt_pose_.inverse() * pose_;
		// Pick off x, y, z, roll, pitch, yaw
		Eigen::Matrix<double, 3, 1> trans_err = T_err.r_ab_inb();
		Eigen::Matrix<double, 3, 1> rot_err = T_err.vec().tail<3>();
		Eigen::Matrix<double, 6, 1> pose_err;
		pose_err << trans_err, rot_err;
		return pose_err;
	}

	void set_ate_error(double ate) { ate_error_ = ate; }
	double get_ate_error() const { return ate_error_; }

	// Set fixed flag
	void set_fixed(bool fixed) { fixed_ = fixed; }
	bool is_fixed() const { return fixed_; }

	// Clone method for deep copying
	virtual std::shared_ptr<Scan> clone() const = 0;

	// Interpolate intensity value at a query point in world frame
    // No value will be provided if the requested point is out of bounds
    // Additionally provides optional Jacobian of intensity w.r.t. SE(2) pose (1x3)
	virtual std::optional<Measurement> interpolate(double x, double y) const = 0;

	// Coverage check at a query point in world frame
	virtual bool check_coverage_at_point(double x, double y) const = 0;


protected:
	Scan(int64_t timestamp, int scan_id, const double meas_std, const lgmath::se3::Transformation &pose,
		 const lgmath::se3::Transformation &gt_pose)
		: timestamp_(timestamp), id_(scan_id), meas_std_(meas_std), pose_(pose), gt_pose_(gt_pose) {}

	int64_t timestamp_;
	int id_;
	double meas_std_;
	lgmath::se3::Transformation pose_;
	lgmath::se3::Transformation gt_pose_;
	double ate_error_ = 0.0;
	bool fixed_ = false;
};

} // namespace ba


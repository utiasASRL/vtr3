#ifndef DRO_WRAPPER_H
#define DRO_WRAPPER_H

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h> // Enables automatic conversion between C++ and Python containers
#include <pybind11/eigen.h> // Required for automatic conversion
#include <cv_bridge/cv_bridge.h>
#include <Eigen/Dense>
#include "cvnp/cvnp.h"


namespace vtr::radar {
namespace py = pybind11;

class DroWrapper {
public:

    struct RadarData {
        cv::Mat polar;
        std::vector<double> azimuths;
        std::vector<int64_t> timestamps;
        std::vector<bool> chirps;
    };

    struct ImuData {
        int64_t timestamp;
        Eigen::Vector3d angular_velocity;
        Eigen::Vector3d linear_acceleration;
    };

    struct OdometryResult {
        Eigen::Matrix4d T;
        Eigen::VectorXf v;
    };

    /**
     * @brief Initializes the Dro C++ wrapper.
     * @param opts A pybind11 dictionary containing the DRO options.
     */
    DroWrapper(py::dict opts);

    /**
     * @brief Executes a single odometry step.
     * @param radar_data A pybind11 dictionary containing radar data (timestamps, azimuths, polar, resolution, chirps).
     * @param imu_data A pybind11 list containing IMU dictionaries.
     * @return A NumPy array representing the result state.
     */
    OdometryResult odometryStep(const RadarData& radar_data, const std::vector<ImuData>& imu_data, cv::Mat& local_map);

    /**
     * @brief Retrieves the pose at a specific time.
     * @param time The timestamp to get the pose for.
     * @return A 4x4 NumPy array of doubles representing the pose matrix.
     */
    Eigen::Matrix4d getPose(uint64_t time);

    /**
     * @brief reset DRO to identity and begin tracking again
     */
    void reset();

    py::object dro_instance_; // Holds the instantiated Python Dro object
};

}//vtr::radar

#endif // DRO_WRAPPER_H
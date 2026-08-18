#include "vtr_radar/dro/dro_wrapper.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace vtr::radar
{

DroWrapper::DroWrapper(py::dict opts) {
    // 1. Get the ROS 2 share directory for this package
    std::string pkg_share = ament_index_cpp::get_package_share_directory("vtr_radar");

    // 2. Append the installed python directory to Python's sys.path
    py::module_ sys = py::module_::import("sys");
    sys.attr("path").attr("append")(pkg_share);

    // 3. Now import the module safely
    py::module_ dro_module = py::module_::import("dro");
    dro_instance_ = dro_module.attr("Dro")(opts);
}

Eigen::Vector2f DroWrapper::odometryStep(const RadarData& radar_data, const std::vector<ImuData>& imu_data, cv::Mat& local_map) {

    py::dict radar_py;
    radar_py["polar"] = radar_data.polar;
    radar_py["azimuths"] = radar_data.azimuths;
    radar_py["timestamps"] = radar_data.timestamps;
    radar_py["chirps"] = radar_data.chirps;
    
    std::vector<py::dict> imu_py_list;

    imu_py_list.reserve(imu_data.size()); 

    // Convert each int to string and push it to the destination vector
    std::transform(imu_data.begin(), imu_data.end(), 
                   std::back_inserter(imu_py_list), 
                   [](const ImuData& imu) { 
                    py::dict imu_py;
                    imu_py["timestamp"] = imu.timestamp;
                    imu_py["angular_velocity"] = imu.angular_velocity;
                    imu_py["linear_acceleration"] = imu.linear_acceleration;
                    return imu_py; 
                
                });

    py::tuple result = dro_instance_.attr("odometryStep")(radar_py, imu_py_list);
    local_map = result[1].cast<cv::Mat>();
    // The Python function returns a numpy array via detach().cpu().numpy()[cite: 1].
    return result[0].cast<Eigen::Vector2f>();
}

Eigen::Matrix4d DroWrapper::getPose(uint64_t time) {
    py::object result = dro_instance_.attr("getPose")(time);
    
    // The Python function returns a 4x4 pose numpy array of np.float64[cite: 1].
    // We cast it to a pybind11::array_t<double> (which directly maps to float64).
    return result.cast<Eigen::Matrix4d>();
}
    
} // namespace vtr::radar

//////////////////////////////////////////////////////////////////////////////////////////////
/// \file ParseBA.cpp
/// \brief Parses a simple bundle adjustment problem (poses, landmarks and measurements)
///
/// \author Sean Anderson, ASRL
//////////////////////////////////////////////////////////////////////////////////////////////

#include <steam/data/ParseBA.hpp>

#include <iostream>

#include <steam/common/ParseUtils.hpp>

namespace steam {
namespace data {

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Function that parses a simple BA dataset
//////////////////////////////////////////////////////////////////////////////////////////////
SimpleBaDataset parseSimpleBaDataset(const std::string& file) {

  // Parse comma delimited file into a matrix of strings
  std::vector<std::vector<std::string> > temp = steam::parse::loadData(file, ',');
  if (temp.size() < 1u) {
    std::stringstream ss; ss << "The file: " << file << ", was empty.";
    throw std::invalid_argument(ss.str());
  }

  // Loop over each line and parse accordingly
  SimpleBaDataset result;
  for (std::vector<std::vector<std::string> >::iterator it = temp.begin(); it != temp.end(); ++it) {

    if (it->at(0) == "FRAME_GT") { // check if line is a ground-truth pose
      SimpleBaDataset::Frame temp;
      unsigned int i = 0;
      temp.frameID = atoi(it->at(++i).c_str());
      temp.time = atof(it->at(++i).c_str());
      for (unsigned int j = 0; j < 6; j++) {
        temp.pose_vec_k0[j] = atof(it->at(++i).c_str());
      }
      temp.T_k0 = lgmath::se3::Transformation(temp.pose_vec_k0);
      result.frames_gt.push_back(temp);
    } else if (it->at(0) == "LAND_GT") { // check if line is a ground-truth landmark
      SimpleBaDataset::Landmark temp;
      unsigned int i = 0;
      temp.landID = atoi(it->at(++i).c_str());
      for (unsigned int j = 0; j < 3; j++) {
        temp.point[j] = atof(it->at(++i).c_str());
      }
      result.land_gt.push_back(temp);
    } else if (it->at(0) == "FRAME_IC") { // check if line is a pose initial condition
      SimpleBaDataset::Frame temp;
      unsigned int i = 0;
      temp.frameID = atoi(it->at(++i).c_str());
      temp.time = atof(it->at(++i).c_str());
      for (unsigned int j = 0; j < 6; j++) {
        temp.pose_vec_k0[j] = atof(it->at(++i).c_str());
      }
      temp.T_k0 = lgmath::se3::Transformation(temp.pose_vec_k0);
      result.frames_ic.push_back(temp);
    } else if (it->at(0) == "LAND_IC") { // check if line is a landmark initial condition
      SimpleBaDataset::Landmark temp;
      unsigned int i = 0;
      temp.landID = atoi(it->at(++i).c_str());
      for (unsigned int j = 0; j < 3; j++) {
        temp.point[j] = atof(it->at(++i).c_str());
      }
      result.land_ic.push_back(temp);
    } else if (it->at(0) == "EXTRINSIC") { // check if line is the extrinsic calibration
      Eigen::Matrix<double,6,1> vector_cam_veh;
      unsigned int i = 0;
      for (unsigned int j = 0; j < 6; j++) {
        vector_cam_veh[j] = atof(it->at(++i).c_str());
      }
      result.T_cv = lgmath::se3::Transformation(vector_cam_veh);
    } else if (it->at(0) == "CAMPARAMS") { // check if line is the intrinsic calibration
      unsigned int i = 0;
      result.camParams.b  = atof(it->at(++i).c_str());
      result.camParams.fu = atof(it->at(++i).c_str());
      result.camParams.fv = atof(it->at(++i).c_str());
      result.camParams.cu = atof(it->at(++i).c_str());
      result.camParams.cv = atof(it->at(++i).c_str());
    } else if (it->at(0) == "DIAGNOISE") { // check if line is the diagonal sensor noise
      result.noise = Eigen::Matrix4d::Zero();
      unsigned int i = 1;
      result.noise(i-1,i-1) = atof(it->at(i).c_str()); i++;
      result.noise(i-1,i-1) = atof(it->at(i).c_str()); i++;
      result.noise(i-1,i-1) = atof(it->at(i).c_str()); i++;
      result.noise(i-1,i-1) = atof(it->at(i).c_str()); i++;
    } else if (it->at(0) == "STEREO") { // check if line is a stereo camera measurement
      SimpleBaDataset::StereoMeas temp;
      unsigned int i = 0;
      temp.frameID = atoi(it->at(++i).c_str());
      temp.landID = atoi(it->at(++i).c_str());
      temp.time = atof(it->at(++i).c_str());
      for (unsigned int j = 0; j < 4; j++) {
        temp.data[j] = atof(it->at(++i).c_str());
      }
      result.meas.push_back(temp);
    } else { // unrecognized line
      throw std::logic_error("file contained unrecognized field");
    }
  }

  return result;
}

} // data
} // steam


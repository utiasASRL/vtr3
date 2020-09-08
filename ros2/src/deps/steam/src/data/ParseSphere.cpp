//////////////////////////////////////////////////////////////////////////////////////////////
/// \file ParseSphere.cpp
/// \brief Parses a sphere dataset, as released with iSAM1
///
/// \author Sean Anderson, modified from iSAM1 parsing tools
//////////////////////////////////////////////////////////////////////////////////////////////

#include <steam/data/ParseSphere.hpp>
#include <steam/common/ParseUtils.hpp>

namespace steam {
namespace data {

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Construct a rotation matrix from roll, pitch and yaw angles
//////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix3d CfromRPY(double roll, double pitch, double yaw)
{
  Eigen::Matrix3d C;
  double cx = cos(roll);
  double sx = sin(roll);
  double cy = cos(pitch);
  double sy = sin(pitch);
  double cz = cos(yaw);
  double sz = sin(yaw);
  C(0,0) =  cz*cy;  C(0,1) = sz*cx + cz*sy*sx;  C(0,2) = sz*sx - cz*sy*cx;
  C(1,0) = -sz*cy;  C(1,1) = cz*cx - sz*sy*sx;  C(1,2) = cz*sx + sz*sy*cx;
  C(2,0) =     sy;  C(2,1) =           -cy*sx;  C(2,2) =            cy*cx;
  return C;
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Function that parses an iSAM1 sphere dataset
//////////////////////////////////////////////////////////////////////////////////////////////
std::vector<SphereEdge> parseSphereDataset(const std::string& file) {

  // Parse space delimited file into a matrix of strings
  std::vector<std::vector<std::string> > temp = steam::parse::loadData(file, ' ');
  if (temp.size() < 1u) {
    std::stringstream ss; ss << "The file: " << file << ", was empty.";
    throw std::invalid_argument(ss.str());
  }

  // Loop over each line and parse edge
  std::vector<SphereEdge> result;
  for (std::vector<std::vector<std::string> >::iterator it = temp.begin(); it != temp.end(); ++it) {

    // Check it is an edge
    if (it->at(0) != "EDGE3") {
      throw std::logic_error("File format is incorrect, expected prefix EDGE3.");
    }

    // Check for one of the two valid sizes
    if (it->size() != 30 && it->size() != 9) {
      throw std::logic_error("File format (number of entries per line) is incorrect.");
    }

    // Parse IDs
    SphereEdge edge;
    unsigned int i = 0;
    edge.idA = atoi(it->at(++i).c_str());
    edge.idB = atoi(it->at(++i).c_str());

    // Parse angles
    double x, y, z, yaw, pitch, roll;
    x = atof(it->at(++i).c_str());
    y = atof(it->at(++i).c_str());
    z = atof(it->at(++i).c_str());
    roll = atof(it->at(++i).c_str());
    pitch = atof(it->at(++i).c_str());
    yaw = atof(it->at(++i).c_str());

    // Construct transformation
    Eigen::Matrix3d C_ba = CfromRPY(-roll, -pitch, -yaw);
    Eigen::Vector3d r_ba_ina = Eigen::Vector3d(x, y, z);
    edge.T_BA = lgmath::se3::Transformation(C_ba.transpose(), r_ba_ina);
    if (edge.idB < edge.idA) { // reverse constraint if needed?
      edge.T_BA = edge.T_BA.inverse();
    }

    // Get covariance data (Information is inverse of covariance)
    edge.sqrtInformation = Eigen::MatrixXd::Identity(6,6);
    if (it->size() == 30) {
      double i11, i12, i13, i14, i15, i16, i22, i23, i24, i25, i26,
             i33, i34, i35, i36, i44, i45, i46, i55, i56, i66;
      i11 = atof(it->at(++i).c_str());
      i12 = atof(it->at(++i).c_str());
      i13 = atof(it->at(++i).c_str());
      i14 = atof(it->at(++i).c_str());
      i15 = atof(it->at(++i).c_str());
      i16 = atof(it->at(++i).c_str());
      i22 = atof(it->at(++i).c_str());
      i23 = atof(it->at(++i).c_str());
      i24 = atof(it->at(++i).c_str());
      i25 = atof(it->at(++i).c_str());
      i26 = atof(it->at(++i).c_str());
      i33 = atof(it->at(++i).c_str());
      i34 = atof(it->at(++i).c_str());
      i35 = atof(it->at(++i).c_str());
      i36 = atof(it->at(++i).c_str());
      i44 = atof(it->at(++i).c_str());
      i45 = atof(it->at(++i).c_str());
      i46 = atof(it->at(++i).c_str());
      i55 = atof(it->at(++i).c_str());
      i56 = atof(it->at(++i).c_str());
      i66 = atof(it->at(++i).c_str());
      edge.sqrtInformation <<
        i11, i12, i13, i14, i15, i16,
         0., i22, i23, i24, i25, i26,
         0.,  0., i33, i34, i35, i36,
         0.,  0.,  0., i44, i45, i46,
         0.,  0.,  0.,  0., i55, i56,
         0.,  0.,  0.,  0.,  0., i66;
    }

    // Store edge
    result.push_back(edge);
  }

  return result;
}

} // data
} // steam


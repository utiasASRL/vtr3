//////////////////////////////////////////////////////////////////////////////////////////////
/// \file TransformationWithCovariance.cpp
/// \details Light weight transformation class with added covariance propagation, intended to
///          be fast, and not to provide unnecessary functionality, but still much slower than
///          the base Transformation class due to extra matrix multiplications associated with
///          covariance propagation.  Only use this class if you need covariance.
///
/// \author Kai van Es
//////////////////////////////////////////////////////////////////////////////////////////////


#include <lgmath/se3/TransformationWithCovariance.hpp>

#include <stdexcept>

#include <lgmath/so3/Operations.hpp>
#include <lgmath/se3/Operations.hpp>

namespace lgmath {
namespace se3 {

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Default constructor
//////////////////////////////////////////////////////////////////////////////////////////////
TransformationWithCovariance::TransformationWithCovariance(bool initCovarianceToZero) :
  Transformation(), covariance_(Eigen::Matrix<double,6,6>::Zero()), covarianceSet_(initCovarianceToZero) {
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Copy constructor. Default implementation causes functional failure.
/// \todo (yuchen) Figure out why default does not work.
//////////////////////////////////////////////////////////////////////////////////////////////
TransformationWithCovariance::TransformationWithCovariance(const TransformationWithCovariance& T) :
  Transformation(T), covariance_(T.covariance_), covarianceSet_(T.covarianceSet_) {
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Move constructor. Manually implemented as Eigen doesn't support moving.
//////////////////////////////////////////////////////////////////////////////////////////////
TransformationWithCovariance::TransformationWithCovariance(TransformationWithCovariance&& T) :
    Transformation(T), covariance_(std::move(T.covariance_)), covarianceSet_(std::move(T.covarianceSet_)) {
  // TODO: Eigen doesn't support move construction, so right now this is mostly the same as a copy...
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Copy constructor from deterministic Transformation
//////////////////////////////////////////////////////////////////////////////////////////////
TransformationWithCovariance::TransformationWithCovariance(const Transformation& T, bool initCovarianceToZero) :
  Transformation(T), covariance_(Eigen::Matrix<double,6,6>::Zero()), covarianceSet_(initCovarianceToZero) {
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Move constructor from deterministic Transformation
//////////////////////////////////////////////////////////////////////////////////////////////
TransformationWithCovariance::TransformationWithCovariance(Transformation&& T, bool initCovarianceToZero) :
    Transformation(T), covariance_(Eigen::Matrix<double,6,6>::Zero()), covarianceSet_(initCovarianceToZero) {
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Copy constructor from deterministic Transformation, with covariance
//////////////////////////////////////////////////////////////////////////////////////////////
TransformationWithCovariance::TransformationWithCovariance(const Transformation& T,
                                                           const Eigen::Matrix<double,6,6>& covariance) :
  Transformation(T), covariance_(covariance), covarianceSet_(true) {
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Constructor
//////////////////////////////////////////////////////////////////////////////////////////////
TransformationWithCovariance::TransformationWithCovariance(const Eigen::Matrix4d& T) :
  Transformation(T), covariance_(Eigen::Matrix<double,6,6>::Zero()), covarianceSet_(false) {
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Constructor with covariance
//////////////////////////////////////////////////////////////////////////////////////////////
TransformationWithCovariance::TransformationWithCovariance(const Eigen::Matrix4d& T,
                                                           const Eigen::Matrix<double,6,6>& covariance) :
  Transformation(T), covariance_(covariance), covarianceSet_(true) {
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Constructor. The transformation will be T_ba = [C_ba, -C_ba*r_ba_ina; 0 0 0 1]
//////////////////////////////////////////////////////////////////////////////////////////////
TransformationWithCovariance::TransformationWithCovariance(const Eigen::Matrix3d& C_ba, const Eigen::Vector3d& r_ba_ina) :
  Transformation(C_ba, r_ba_ina), covariance_(Eigen::Matrix<double,6,6>::Zero()), covarianceSet_(false) {
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Constructor with covariance. The transformation will be
/// T_ba = [C_ba, -C_ba*r_ba_ina; 0 0 0 1]
//////////////////////////////////////////////////////////////////////////////////////////////
TransformationWithCovariance::TransformationWithCovariance(const Eigen::Matrix3d& C_ba, const Eigen::Vector3d& r_ba_ina,
                             const Eigen::Matrix<double,6,6>& covariance) :
  Transformation(C_ba, r_ba_ina), covariance_(covariance), covarianceSet_(true) {
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Constructor. The transformation will be T_ba = vec2tran(xi_ab)
//////////////////////////////////////////////////////////////////////////////////////////////
TransformationWithCovariance::TransformationWithCovariance(const Eigen::Matrix<double,6,1>& xi_ab, unsigned int numTerms) :
  Transformation(xi_ab, numTerms), covariance_(Eigen::Matrix<double,6,6>::Zero()), covarianceSet_(false) {
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Constructor with covariance. The transformation will be T_ba = vec2tran(xi_ab)
//////////////////////////////////////////////////////////////////////////////////////////////
TransformationWithCovariance::TransformationWithCovariance(const Eigen::Matrix<double,6,1>& xi_ab,
                                                           const Eigen::Matrix<double,6,6>& covariance,
                                                           unsigned int numTerms) :
  Transformation(xi_ab, numTerms), covariance_(covariance), covarianceSet_(true) {
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Constructor. The transformation will be T_ba = vec2tran(xi_ab), xi_ab must be 6x1
//////////////////////////////////////////////////////////////////////////////////////////////
TransformationWithCovariance::TransformationWithCovariance(const Eigen::VectorXd& xi_ab) :
  Transformation(xi_ab), covariance_(Eigen::Matrix<double,6,6>::Zero()), covarianceSet_(false) {
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Constructor. The transformation will be T_ba = vec2tran(xi_ab), xi_ab must be 6x1
//////////////////////////////////////////////////////////////////////////////////////////////
TransformationWithCovariance::TransformationWithCovariance(const Eigen::VectorXd& xi_ab,
                                                           const Eigen::Matrix<double,6,6>& covariance) :
  Transformation(xi_ab), covariance_(covariance), covarianceSet_(true) {
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Copy assignment operator. Default implementation causes functional failure
/// \todo (yuchen) Figure out why default does not work.
//////////////////////////////////////////////////////////////////////////////////////////////
TransformationWithCovariance& TransformationWithCovariance::operator=(const TransformationWithCovariance& T) {

  Transformation::operator=(T);

  covariance_ = T.covariance_;
  covarianceSet_ = T.covarianceSet_;

  return *this;
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Move assignment operator. Manually implemented as Eigen doesn't support moving.
//////////////////////////////////////////////////////////////////////////////////////////////
TransformationWithCovariance& TransformationWithCovariance::operator=(TransformationWithCovariance&& T) {
  // Call the assignment operator on the super class, as the internal members are not accessible here
  Transformation::operator=(T);

  // TODO: Eigen doesn't support move construction, so right now this is mostly the same as a copy...
  this->covariance_ = std::move(T.covariance_);
  this->covarianceSet_ = std::move(T.covarianceSet_);

  return (*this);
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Copy assignment operator from basic Transform.
/// \description This assignment resets the covariance to the uninitialized state.  You must
///              manually call setZeroCovariance() or setCovariance(const Eigen::Matrix6d&)
///              before querying it with the public method cov(), or an exception will be
///              thrown.
//////////////////////////////////////////////////////////////////////////////////////////////
TransformationWithCovariance& TransformationWithCovariance::operator=(const Transformation& T) {

  // Call the assignment operator on the super class, as the internal members are not accessible here
  Transformation::operator=(T);

  // The covarianceSet_ flag is set to false to prevent unintentional bad covariance propagation
  this->covariance_.setZero();
  this->covarianceSet_ = false;

  return (*this);
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Move assignment operator from basic Transform.
/// \description This assignment resets the covariance to the uninitialized state.  You must
///              manually call setZeroCovariance() or setCovariance(const Eigen::Matrix6d&)
///              before querying it with the public method cov(), or an exception will be
///              thrown.
//////////////////////////////////////////////////////////////////////////////////////////////
TransformationWithCovariance& TransformationWithCovariance::operator=(Transformation&& T) {

  // Call the assignment operator on the super class, as the internal members are not accessible here
  Transformation::operator=(T);

  // The covarianceSet_ flag is set to false to prevent unintentional bad covariance propagation
  this->covariance_.setZero();
  this->covarianceSet_ = false;

  return (*this);
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Gets the underlying covariance matrix
//////////////////////////////////////////////////////////////////////////////////////////////
const Eigen::Matrix<double,6,6>& TransformationWithCovariance::cov() const {

  if (!covarianceSet_) {
    throw std::logic_error("Covariance accessed before being set.  "
                           "Use setCovariance or initialize with a covariance.");
  }
  return covariance_;
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Returns whether or not a covariance has been set. If it is unset, then querying it
///        with the public method cov() will throw an exception.
//////////////////////////////////////////////////////////////////////////////////////////////
bool TransformationWithCovariance::covarianceSet() const {
  return covarianceSet_;
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Sets the underlying covariance matrix
//////////////////////////////////////////////////////////////////////////////////////////////
void TransformationWithCovariance::setCovariance(const Eigen::Matrix<double,6,6>& covariance) {
  covariance_ = covariance;
  covarianceSet_ = true;
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Sets the underlying covariance matrix to the 6x6 zero matrix (perfect certainty)
//////////////////////////////////////////////////////////////////////////////////////////////
void TransformationWithCovariance::setZeroCovariance() {
  covariance_.setZero();
  covarianceSet_ = true;
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Get the inverse matrix
//////////////////////////////////////////////////////////////////////////////////////////////
TransformationWithCovariance TransformationWithCovariance::inverse() const {
  TransformationWithCovariance temp(Transformation::inverse(), false);
  Eigen::Matrix<double,6,6> adjointOfInverse = temp.adjoint();
  temp.setCovariance(adjointOfInverse * covariance_ * adjointOfInverse.transpose());
  return temp;
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief In-place right-hand side multiply T_rhs
//////////////////////////////////////////////////////////////////////////////////////////////
TransformationWithCovariance& TransformationWithCovariance::operator*=(const TransformationWithCovariance& T_rhs) {

  // The covarianceSet_ flag is only set to true if BOTH transforms have a properly set covariance
  Eigen::Matrix<double,6,6> Ad_lhs = Transformation::adjoint();
  this->covariance_ = this->covariance_ + Ad_lhs * T_rhs.covariance_ * Ad_lhs.transpose();
  this->covarianceSet_ = (this->covarianceSet_ && T_rhs.covarianceSet_);

  // Compound mean transform
  Transformation::operator*=(T_rhs);
  return *this;
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief In-place right-hand side multiply basic (certain) T_rhs
///
/// Note: Assumes that the Transformation matrix has perfect certainty
//////////////////////////////////////////////////////////////////////////////////////////////
TransformationWithCovariance& TransformationWithCovariance::operator*=(const Transformation& T_rhs) {
  Transformation::operator*=(T_rhs);
  return *this;
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief In-place right-hand side multiply this matrix by the inverse of T_rhs
//////////////////////////////////////////////////////////////////////////////////////////////
TransformationWithCovariance& TransformationWithCovariance::operator/=(const TransformationWithCovariance& T_rhs) {

  // Note very carefully that we modify the internal transform before taking the adjoint
  // in order to avoid having to convert the rhs covariance explicitly
  Transformation::operator/=(T_rhs);
  Eigen::Matrix<double,6,6> Ad_lhs_rhs = Transformation::adjoint();
  this->covariance_ = this->covariance_ + Ad_lhs_rhs * T_rhs.covariance_ * Ad_lhs_rhs.transpose();
  this->covarianceSet_ = (this->covarianceSet_ && T_rhs.covarianceSet_);
  return *this;
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief In-place right-hand side multiply this matrix by the inverse of basic (certain) T_rhs
///
/// Note: Assumes that the Transformation matrix has perfect certainty
//////////////////////////////////////////////////////////////////////////////////////////////
TransformationWithCovariance& TransformationWithCovariance::operator/=(const Transformation& T_rhs) {
  Transformation::operator/=(T_rhs);
  return *this;
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Multiplication of TransformWithCovariance by TransformWithCovariance
//////////////////////////////////////////////////////////////////////////////////////////////
TransformationWithCovariance operator*(TransformationWithCovariance T_lhs, const TransformationWithCovariance& T_rhs) {
  T_lhs *= T_rhs;
  return T_lhs;
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Multiplication of TransformWithCovariance by Transform
///
/// Note: Assumes that the Transformation matrix has perfect certainty
//////////////////////////////////////////////////////////////////////////////////////////////
TransformationWithCovariance operator*(TransformationWithCovariance T_lhs, const Transformation& T_rhs) {
  T_lhs *= T_rhs;
  return T_lhs;
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Multiplication of Transform by TransformWithCovariance
///
/// Note: Assumes that the Transformation matrix has perfect certainty
//////////////////////////////////////////////////////////////////////////////////////////////
TransformationWithCovariance operator*(const Transformation& T_lhs, const TransformationWithCovariance& T_rhs) {

  // Convert the Transform to a TransformWithCovariance with perfect certainty
  TransformationWithCovariance temp(T_lhs, true);
  temp *= T_rhs;
  return temp;
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Multiplication of TransformWithCovariance by inverse TransformWithCovariance
//////////////////////////////////////////////////////////////////////////////////////////////
TransformationWithCovariance operator/(TransformationWithCovariance T_lhs, const TransformationWithCovariance& T_rhs) {
  T_lhs /= T_rhs;
  return T_lhs;
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Multiplication of TransformWithCovariance by inverse Transform
///
/// Note: Assumes that the Transformation matrix has perfect certainty
//////////////////////////////////////////////////////////////////////////////////////////////
TransformationWithCovariance operator/(TransformationWithCovariance T_lhs, const Transformation& T_rhs) {
  T_lhs /= T_rhs;
  return T_lhs;
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Multiplication of Transform by inverse TransformWithCovariance
//////////////////////////////////////////////////////////////////////////////////////////////
TransformationWithCovariance operator/(const Transformation& T_lhs, const TransformationWithCovariance& T_rhs) {

  // Convert the Transform to a TransformWithCovariance with perfect certainty
  TransformationWithCovariance temp(T_lhs, true);
  temp /= T_rhs;
  return temp;
}

} // se3
} // lgmath

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief print transformation
//////////////////////////////////////////////////////////////////////////////////////////////
std::ostream& operator<<(std::ostream& out, const lgmath::se3::TransformationWithCovariance& T) {
  out << std::endl << T.matrix() << std::endl;
  if (T.covarianceSet()) {
    out << std::endl << T.cov() << std::endl;
  } else {
    out << std::endl << "unset covariance" << std::endl;
  }
  return out;
}

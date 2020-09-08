//////////////////////////////////////////////////////////////////////////////////////////////
/// \file NoiseModel.hpp
///
/// \author Sean Anderson, ASRL
//////////////////////////////////////////////////////////////////////////////////////////////

#ifndef STEAM_NOISE_MODEL_HPP
#define STEAM_NOISE_MODEL_HPP

#include <Eigen/Dense>
#include <boost/shared_ptr.hpp>
#include <mutex>

namespace steam {

/// @class NoiseEvaluator evaluates uncertainty based on a derived model.
template <int MEAS_DIM>
class NoiseEvaluator {
public:
  /// Convenience typedefs
  typedef boost::shared_ptr<NoiseEvaluator<MEAS_DIM> > Ptr;
  typedef boost::shared_ptr<const NoiseEvaluator<MEAS_DIM> > ConstPtr;
  
  /// \brief Default constructor.
  NoiseEvaluator()=default;

  /// \brief Default destructor.
  virtual ~NoiseEvaluator()=default;

  /// \brief mutex locked public exposure of the virtual evaluate() function.
  virtual Eigen::Matrix<double,MEAS_DIM,MEAS_DIM> evaluateCovariance() {
    std::lock_guard<std::mutex> lock(eval_mutex_);
    return evaluate();
  }

protected:
  /// \brief Evaluates the uncertainty based on a derived model.
  /// \return the uncertainty, in the form of a covariance matrix.
  virtual Eigen::Matrix<double,MEAS_DIM,MEAS_DIM> evaluate()=0;

private:
  std::mutex eval_mutex_;
};

/// Enumeration of ways to set the noise
enum MatrixType { COVARIANCE, INFORMATION, SQRT_INFORMATION };

/// @class BaseNoiseModel Base class for the steam noise models
template <int MEAS_DIM>
class BaseNoiseModel 
{
 public:

  /// \brief Default constructor.
  BaseNoiseModel()=default;

  /// \brief Constructor
  /// \brief A noise matrix, determined by the type parameter.
  /// \brief The type of noise matrix set in the previous paramter.
  BaseNoiseModel(const Eigen::Matrix<double,MEAS_DIM,MEAS_DIM>& matrix,
             MatrixType type = COVARIANCE);

  /// \brief Deault destructor
  virtual ~BaseNoiseModel() = default;

  /// Convenience typedefs
  typedef boost::shared_ptr<BaseNoiseModel<MEAS_DIM> > Ptr;
  typedef boost::shared_ptr<const BaseNoiseModel<MEAS_DIM> > ConstPtr;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Set by covariance matrix
  //////////////////////////////////////////////////////////////////////////////////////////////
  void setByCovariance(const Eigen::Matrix<double,MEAS_DIM,MEAS_DIM>& matrix) const;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Set by information matrix
  //////////////////////////////////////////////////////////////////////////////////////////////
  void setByInformation(const Eigen::Matrix<double,MEAS_DIM,MEAS_DIM>& matrix) const ;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Set by square root of information matrix
  //////////////////////////////////////////////////////////////////////////////////////////////
  void setBySqrtInformation(const Eigen::Matrix<double,MEAS_DIM,MEAS_DIM>& matrix) const ;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Get a reference to the square root information matrix
  //////////////////////////////////////////////////////////////////////////////////////////////
  virtual const Eigen::Matrix<double,MEAS_DIM,MEAS_DIM>& getSqrtInformation() const = 0;


  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Get the norm of the whitened error vector, sqrt(rawError^T * info * rawError)
  //////////////////////////////////////////////////////////////////////////////////////////////
  virtual double getWhitenedErrorNorm(const Eigen::Matrix<double,MEAS_DIM,1>& rawError) const = 0;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Get the whitened error vector, sqrtInformation*rawError
  //////////////////////////////////////////////////////////////////////////////////////////////
  virtual Eigen::Matrix<double,MEAS_DIM,1> whitenError(
      const Eigen::Matrix<double,MEAS_DIM,1>& rawError) const = 0;

 protected:

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Assert that the matrix is positive definite
  //////////////////////////////////////////////////////////////////////////////////////////////
  void assertPositiveDefiniteMatrix(const Eigen::Matrix<double,MEAS_DIM,MEAS_DIM>& matrix) const;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief The square root information (found by performing an LLT decomposition on the
  ///        information matrix (inverse covariance matrix). This triangular matrix is
  ///        stored directly for faster error whitening.
  //////////////////////////////////////////////////////////////////////////////////////////////
  mutable Eigen::Matrix<double,MEAS_DIM,MEAS_DIM> sqrtInformation_;

 private:

};

/// @class StaticNoiseModel Noise model for uncertainties that do not change during the 
///        steam optimization problem.
template <int MEAS_DIM>
class StaticNoiseModel : public BaseNoiseModel<MEAS_DIM>
{
 public:

  /// Convenience typedefs
  typedef boost::shared_ptr<StaticNoiseModel<MEAS_DIM> > Ptr;
  typedef boost::shared_ptr<const StaticNoiseModel<MEAS_DIM> > ConstPtr;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Default constructor
  //////////////////////////////////////////////////////////////////////////////////////////////
  StaticNoiseModel();

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief General constructor
  //////////////////////////////////////////////////////////////////////////////////////////////
  StaticNoiseModel(const Eigen::Matrix<double,MEAS_DIM,MEAS_DIM>& matrix,
             MatrixType type = COVARIANCE);


  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Get a reference to the square root information matrix
  //////////////////////////////////////////////////////////////////////////////////////////////
  virtual const Eigen::Matrix<double,MEAS_DIM,MEAS_DIM>& getSqrtInformation() const;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Get the norm of the whitened error vector, sqrt(rawError^T * info * rawError)
  //////////////////////////////////////////////////////////////////////////////////////////////
  virtual double getWhitenedErrorNorm(const Eigen::Matrix<double,MEAS_DIM,1>& rawError) const;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Get the whitened error vector, sqrtInformation*rawError
  //////////////////////////////////////////////////////////////////////////////////////////////
  virtual Eigen::Matrix<double,MEAS_DIM,1> whitenError(
      const Eigen::Matrix<double,MEAS_DIM,1>& rawError) const;

private:

};

/// \brief DynamicNoiseModel Noise model for uncertainties that change during the steam optimization
///        problem.
template <int MEAS_DIM>
class DynamicNoiseModel : public BaseNoiseModel<MEAS_DIM>
{
 public:
  /// \brief Constructor
  /// \param eval a pointer to a noise evaluator.
  DynamicNoiseModel(boost::shared_ptr<NoiseEvaluator<MEAS_DIM>> eval);

  /// \brief Deault destructor.
  ~DynamicNoiseModel()=default;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Get a reference to the square root information matrix
  //////////////////////////////////////////////////////////////////////////////////////////////
  virtual const Eigen::Matrix<double,MEAS_DIM,MEAS_DIM>& getSqrtInformation() const;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Get the norm of the whitened error vector, sqrt(rawError^T * info * rawError)
  //////////////////////////////////////////////////////////////////////////////////////////////
  virtual double getWhitenedErrorNorm(const Eigen::Matrix<double,MEAS_DIM,1>& rawError) const;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Get the whitened error vector, sqrtInformation*rawError
  //////////////////////////////////////////////////////////////////////////////////////////////
  virtual Eigen::Matrix<double,MEAS_DIM,1> whitenError(
      const Eigen::Matrix<double,MEAS_DIM,1>& rawError) const;

 private:
  /// \brief A pointer to a noise evaluator.
  boost::shared_ptr<NoiseEvaluator<MEAS_DIM>> eval_;
};

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Typedef for the general base noise model
//////////////////////////////////////////////////////////////////////////////////////////////
typedef BaseNoiseModel<Eigen::Dynamic> BaseNoiseModelX;

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Typedef for the general static noise model
//////////////////////////////////////////////////////////////////////////////////////////////
typedef StaticNoiseModel<Eigen::Dynamic> StaticNoiseModelX;

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Typedef for the general dynamic noise model
//////////////////////////////////////////////////////////////////////////////////////////////
typedef DynamicNoiseModel<Eigen::Dynamic> DynamicNoiseModelX;

} // steam


#include <steam/problem/NoiseModel-inl.hpp>

#endif // STEAM_NOISE_MODEL_HPP

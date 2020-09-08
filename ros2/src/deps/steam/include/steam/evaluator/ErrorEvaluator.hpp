//////////////////////////////////////////////////////////////////////////////////////////////
/// \file ErrorEvaluator.hpp
///
/// \author Sean Anderson, ASRL
//////////////////////////////////////////////////////////////////////////////////////////////

#ifndef STEAM_ERROR_EVALUATOR_HPP
#define STEAM_ERROR_EVALUATOR_HPP

#include <Eigen/Core>

#include <steam/evaluator/EvaluatorBase.hpp>

namespace steam {

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief The generic error evaluator is simply a typedef of the templated evaluator base
//////////////////////////////////////////////////////////////////////////////////////////////
template<int MEAS_DIM,       // Dimension of the measurement error. Note the meas dim acts as
                             // both the LHS dim and INNER dim (see EvaluatorBase)
         int MAX_STATE_SIZE> // The maximum dimension of a single state variable perturbation
struct ErrorEvaluator{
  typedef EvaluatorBase<Eigen::Matrix<double, MEAS_DIM, 1>,MEAS_DIM,MEAS_DIM,MAX_STATE_SIZE> type;
  typedef boost::shared_ptr<type> Ptr;
  typedef boost::shared_ptr<const type> ConstPtr;
};

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Dynamic-size error evaluator
//////////////////////////////////////////////////////////////////////////////////////////////
typedef ErrorEvaluator<Eigen::Dynamic,Eigen::Dynamic>::type ErrorEvaluatorX;

} // steam

#endif // STEAM_ERROR_EVALUATOR_HPP

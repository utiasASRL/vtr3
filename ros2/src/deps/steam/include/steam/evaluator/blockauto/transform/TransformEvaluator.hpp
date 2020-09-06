//////////////////////////////////////////////////////////////////////////////////////////////
/// \file TransformEvaluator.hpp
///
/// \author Sean Anderson, ASRL
//////////////////////////////////////////////////////////////////////////////////////////////

#ifndef STEAM_TRANSFORM_EVALUATOR_HPP
#define STEAM_TRANSFORM_EVALUATOR_HPP

#include <Eigen/Core>

#include <steam/evaluator/blockauto/BlockAutomaticEvaluator.hpp>
#include <steam/state/LieGroupStateVar.hpp>
#include <steam/state/VectorSpaceStateVar.hpp>
#include <steam/common/Time.hpp>

namespace steam {
namespace se3 {

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Evaluator for a transformation matrix
///
/// *Note that we fix MAX_STATE_DIM to 6. Typically the performance benefits of fixed size
///  matrices begin to die if larger than 6x6. Size 6 allows for transformation matrices
///  and 6D velocities. If you have a state larger than this, consider writing an
///  error evaluator that extends from ErrorEvaluatorX.
//////////////////////////////////////////////////////////////////////////////////////////////
typedef BlockAutomaticEvaluator<lgmath::se3::Transformation, 6, 6> TransformEvaluator;

} // se3
} // steam

#endif // STEAM_TRANSFORM_EVALUATOR_HPP

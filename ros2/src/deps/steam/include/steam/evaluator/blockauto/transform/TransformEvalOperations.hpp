//////////////////////////////////////////////////////////////////////////////////////////////
/// \file TransformEvalOperations.hpp
///
/// \author Sean Anderson, ASRL
//////////////////////////////////////////////////////////////////////////////////////////////

#ifndef STEAM_TRANSFORM_EVALUATOR_OPERATIONS_HPP
#define STEAM_TRANSFORM_EVALUATOR_OPERATIONS_HPP

#include <steam/evaluator/blockauto/transform/ComposeTransformEvaluator.hpp>
#include <steam/evaluator/blockauto/transform/ComposeInverseTransformEvaluator.hpp>
#include <steam/evaluator/blockauto/transform/InverseTransformEvaluator.hpp>
#include <steam/evaluator/blockauto/transform/ComposeLandmarkEvaluator.hpp>
#include <steam/evaluator/blockauto/transform/LogMapEvaluator.hpp>

namespace steam {
namespace se3 {

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Compose two transform evaluators: T_ca = T_cb * T_ba
//////////////////////////////////////////////////////////////////////////////////////////////
static TransformEvaluator::Ptr compose(const TransformEvaluator::ConstPtr& transform_cb,
                                       const TransformEvaluator::ConstPtr& transform_ba) {
  return ComposeTransformEvaluator::MakeShared(transform_cb, transform_ba);
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Compose two transform evaluators, inverting the second: T_ba = T_bx * inv(T_ax)
//////////////////////////////////////////////////////////////////////////////////////////////
static TransformEvaluator::Ptr composeInverse(const TransformEvaluator::ConstPtr& transform_bx,
                                              const TransformEvaluator::ConstPtr& transform_ax) {
  return ComposeInverseTransformEvaluator::MakeShared(transform_bx, transform_ax);
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Compose a transform evaluator and landmark state variable: p_b = T_ba * p_a
//////////////////////////////////////////////////////////////////////////////////////////////
static ComposeLandmarkEvaluator::Ptr compose(const TransformEvaluator::ConstPtr& transform_ba,
                                             const se3::LandmarkStateVar::Ptr& landmark_a) {
  return ComposeLandmarkEvaluator::MakeShared(transform_ba, landmark_a);
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Invert a transform evaluator: T_ba = inv(T_ab)
//////////////////////////////////////////////////////////////////////////////////////////////
static TransformEvaluator::Ptr inverse(const TransformEvaluator::ConstPtr& transform) {
  return InverseTransformEvaluator::MakeShared(transform);
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Take the 'logarithmic map' of a transformation evaluator:  vec = ln(T_ba)
//////////////////////////////////////////////////////////////////////////////////////////////
static LogMapEvaluator::Ptr tran2vec(const TransformEvaluator::ConstPtr& transform) {
  return LogMapEvaluator::MakeShared(transform);
}

} // se3
} // steam

#endif // STEAM_TRANSFORM_EVALUATOR_OPERATIONS_HPP

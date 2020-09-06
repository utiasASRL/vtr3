//////////////////////////////////////////////////////////////////////////////////////////////
/// \file steam.hpp
/// \brief Convenience Header
///
/// \author Sean Anderson
//////////////////////////////////////////////////////////////////////////////////////////////

#ifndef STEAM_ESTIMATION_LIBRARY_HPP
#define STEAM_ESTIMATION_LIBRARY_HPP

// blkmat
#include <steam/blockmat/BlockMatrix.hpp>
#include <steam/blockmat/BlockVector.hpp>
#include <steam/blockmat/BlockSparseMatrix.hpp>

// common
#include <steam/common/Time.hpp>
#include <steam/common/Timer.hpp>

// evaluator
#include <steam/evaluator/ErrorEvaluator.hpp>

// evaluator - block automatic
#include <steam/evaluator/blockauto/BlockAutomaticEvaluator.hpp>
#include <steam/evaluator/blockauto/transform/TransformEvaluator.hpp>
#include <steam/evaluator/blockauto/transform/PositionEvaluator.hpp>
#include <steam/evaluator/blockauto/transform/TransformStateEvaluator.hpp>
#include <steam/evaluator/blockauto/transform/FixedTransformEvaluator.hpp>
#include <steam/evaluator/blockauto/transform/TransformEvalOperations.hpp>

// evaluator - samples (sample functions)
#include <steam/evaluator/samples/StereoCameraErrorEval.hpp>
#include <steam/evaluator/samples/StereoCameraErrorEvalX.hpp>
#include <steam/evaluator/samples/TransformErrorEval.hpp>
#include <steam/evaluator/samples/PositionErrorEval.hpp>
#include <steam/evaluator/samples/VectorSpaceErrorEval.hpp>

// evaluator - block auto diff
#include <steam/evaluator/blockauto/EvalTreeNode.hpp>

// problem
#include <steam/problem/WeightedLeastSqCostTerm.hpp>
#include <steam/problem/ParallelizedCostTermCollection.hpp>
#include <steam/problem/NoiseModel.hpp>
#include <steam/problem/LossFunctions.hpp>
#include <steam/problem/OptimizationProblem.hpp>

// solver
#include <steam/solver/VanillaGaussNewtonSolver.hpp>
#include <steam/solver/LineSearchGaussNewtonSolver.hpp>
#include <steam/solver/LevMarqGaussNewtonSolver.hpp>
#include <steam/solver/DoglegGaussNewtonSolver.hpp>

// state
#include <steam/state/StateVariable.hpp>
#include <steam/state/VectorSpaceStateVar.hpp>
#include <steam/state/LieGroupStateVar.hpp>
#include <steam/state/LandmarkStateVar.hpp>

// trajectory
#include <steam/trajectory/SteamTrajInterface.hpp>

#endif // STEAM_ESTIMATION_LIBRARY_HPP

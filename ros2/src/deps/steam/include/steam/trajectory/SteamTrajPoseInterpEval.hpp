//////////////////////////////////////////////////////////////////////////////////////////////
/// \file SteamTrajPoseInterpEval.hpp
///
/// \author Sean Anderson, ASRL
//////////////////////////////////////////////////////////////////////////////////////////////

#ifndef STEAM_TRAJECTORY_POSE_INTERP_EVAL_HPP
#define STEAM_TRAJECTORY_POSE_INTERP_EVAL_HPP

#include <Eigen/Core>

#include <steam/trajectory/SteamTrajInterface.hpp>
#include <steam/evaluator/blockauto/transform/TransformEvaluator.hpp>

namespace steam {
namespace se3 {

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Simple transform evaluator for a transformation state variable
//////////////////////////////////////////////////////////////////////////////////////////////
class SteamTrajPoseInterpEval : public TransformEvaluator
{
 public:

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Constructor
  //////////////////////////////////////////////////////////////////////////////////////////////
  SteamTrajPoseInterpEval(const Time& time,
                          const SteamTrajVar::ConstPtr& knot1,
                          const SteamTrajVar::ConstPtr& knot2);

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Pseudo constructor - return a shared pointer to a new instance
  //////////////////////////////////////////////////////////////////////////////////////////////
  static Ptr MakeShared(const Time& time,
                        const SteamTrajVar::ConstPtr& knot1,
                        const SteamTrajVar::ConstPtr& knot2);

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Returns whether or not an evaluator contains unlocked state variables
  //////////////////////////////////////////////////////////////////////////////////////////////
  virtual bool isActive() const;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Adds references (shared pointers) to active state variables to the map output
  //////////////////////////////////////////////////////////////////////////////////////////////
  virtual void getActiveStateVariables(
      std::map<unsigned int, steam::StateVariableBase::Ptr>* outStates) const;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Evaluate the transformation matrix
  //////////////////////////////////////////////////////////////////////////////////////////////
  virtual lgmath::se3::Transformation evaluate() const;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Evaluate the transformation matrix tree
  ///
  /// ** Note that the returned pointer belongs to the memory pool EvalTreeNode<TYPE>::pool,
  ///    and should be given back to the pool, rather than being deleted.
  //////////////////////////////////////////////////////////////////////////////////////////////
  virtual EvalTreeNode<lgmath::se3::Transformation>* evaluateTree() const;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Evaluate the Jacobian tree
  //////////////////////////////////////////////////////////////////////////////////////////////
  virtual void appendBlockAutomaticJacobians(
      const Eigen::MatrixXd& lhs,
      EvalTreeNode<lgmath::se3::Transformation>* evaluationTree,
      std::vector<Jacobian<> >* outJacobians) const;

  virtual void appendBlockAutomaticJacobians(
      const Eigen::Matrix<double,1,6>& lhs,
      EvalTreeNode<lgmath::se3::Transformation>* evaluationTree,
      std::vector<Jacobian<1,6> >* outJacobians) const;

  virtual void appendBlockAutomaticJacobians(
      const Eigen::Matrix<double,2,6>& lhs,
      EvalTreeNode<lgmath::se3::Transformation>* evaluationTree,
      std::vector<Jacobian<2,6> >* outJacobians) const;

  virtual void appendBlockAutomaticJacobians(
      const Eigen::Matrix<double,3,6>& lhs,
      EvalTreeNode<lgmath::se3::Transformation>* evaluationTree,
      std::vector<Jacobian<3,6> >* outJacobians) const;

  virtual void appendBlockAutomaticJacobians(
      const Eigen::Matrix<double,4,6>& lhs,
      EvalTreeNode<lgmath::se3::Transformation>* evaluationTree,
      std::vector<Jacobian<4,6> >* outJacobians) const;

  virtual void appendBlockAutomaticJacobians(
      const Eigen::Matrix<double,6,6>& lhs,
      EvalTreeNode<lgmath::se3::Transformation>* evaluationTree,
      std::vector<Jacobian<6,6> >* outJacobians) const;

 private:

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Implementation for Block Automatic Differentiation
  //////////////////////////////////////////////////////////////////////////////////////////////
  template<int LHS_DIM, int INNER_DIM, int MAX_STATE_SIZE>
  void appendJacobiansImpl(const Eigen::Matrix<double,LHS_DIM,INNER_DIM>& lhs,
                           EvalTreeNode<lgmath::se3::Transformation>* evaluationTree,
                           std::vector<Jacobian<LHS_DIM,MAX_STATE_SIZE> >* outJacobians) const;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief First (earlier) knot
  //////////////////////////////////////////////////////////////////////////////////////////////
  SteamTrajVar::ConstPtr knot1_;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Second (later) knot
  //////////////////////////////////////////////////////////////////////////////////////////////
  SteamTrajVar::ConstPtr knot2_;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Interpolation coefficients
  //////////////////////////////////////////////////////////////////////////////////////////////
  double psi11_;
  double psi12_;
  double psi21_;
  double psi22_;
  double lambda11_;
  double lambda12_;
  double lambda21_;
  double lambda22_;
};

} // se3
} // steam

#endif // STEAM_TRAJECTORY_POSE_INTERP_EVAL_HPP

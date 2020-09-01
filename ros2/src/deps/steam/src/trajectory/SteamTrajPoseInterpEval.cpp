//////////////////////////////////////////////////////////////////////////////////////////////
/// \file SteamTrajPoseInterpEval.cpp
///
/// \author Sean Anderson, ASRL
//////////////////////////////////////////////////////////////////////////////////////////////

#include <steam/trajectory/SteamTrajPoseInterpEval.hpp>

#include <lgmath.hpp>

namespace steam {
namespace se3 {

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Constructor
//////////////////////////////////////////////////////////////////////////////////////////////
SteamTrajPoseInterpEval::SteamTrajPoseInterpEval(const Time& time,
                                   const SteamTrajVar::ConstPtr& knot1,
                                   const SteamTrajVar::ConstPtr& knot2) :
  knot1_(knot1), knot2_(knot2) {

  // Calculate time constants
  double tau = (time - knot1->getTime()).seconds();
  double T = (knot2->getTime() - knot1->getTime()).seconds();
  double ratio = tau/T;
  double ratio2 = ratio*ratio;
  double ratio3 = ratio2*ratio;

  // Calculate 'psi' interpolation values
  psi11_ = 3.0*ratio2 - 2.0*ratio3;
  psi12_ = tau*(ratio2 - ratio);
  psi21_ = 6.0*(ratio - ratio2)/T;
  psi22_ = 3.0*ratio2 - 2.0*ratio;

  // Calculate 'lambda' interpolation values
  lambda11_ = 1.0 - psi11_;
  lambda12_ = tau - T*psi11_ - psi12_;
  lambda21_ = -psi21_;
  lambda22_ = 1.0 - T*psi21_ - psi22_;
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Pseudo constructor - return a shared pointer to a new instance
//////////////////////////////////////////////////////////////////////////////////////////////
SteamTrajPoseInterpEval::Ptr SteamTrajPoseInterpEval::MakeShared(const Time& time,
                                                   const SteamTrajVar::ConstPtr& knot1,
                                                   const SteamTrajVar::ConstPtr& knot2) {
  return SteamTrajPoseInterpEval::Ptr(new SteamTrajPoseInterpEval(time, knot1, knot2));
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Returns whether or not an evaluator contains unlocked state variables
//////////////////////////////////////////////////////////////////////////////////////////////
bool SteamTrajPoseInterpEval::isActive() const {
  return knot1_->getPose()->isActive()  ||
         !knot1_->getVelocity()->isLocked() ||
         knot2_->getPose()->isActive()  ||
         !knot2_->getVelocity()->isLocked();
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Adds references (shared pointers) to active state variables to the map output
//////////////////////////////////////////////////////////////////////////////////////////////
void SteamTrajPoseInterpEval::getActiveStateVariables(
    std::map<unsigned int, steam::StateVariableBase::Ptr>* outStates) const {

  knot1_->getPose()->getActiveStateVariables(outStates);
  knot2_->getPose()->getActiveStateVariables(outStates);
  if (!knot1_->getVelocity()->isLocked()) {
    (*outStates)[knot1_->getVelocity()->getKey().getID()] = knot1_->getVelocity();
  }
  if (!knot2_->getVelocity()->isLocked()) {
    (*outStates)[knot2_->getVelocity()->getKey().getID()] = knot2_->getVelocity();
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Evaluate the transformation matrix
//////////////////////////////////////////////////////////////////////////////////////////////
lgmath::se3::Transformation SteamTrajPoseInterpEval::evaluate() const {

  // Get relative matrix info
  lgmath::se3::Transformation T_21 = knot2_->getPose()->evaluate()/knot1_->getPose()->evaluate();

  // Get se3 algebra of relative matrix
  Eigen::Matrix<double,6,1> xi_21 = T_21.vec();

  // Calculate the 6x6 associated Jacobian
  Eigen::Matrix<double,6,6> J_21_inv = lgmath::se3::vec2jacinv(xi_21);

  // Calculate interpolated relative se3 algebra
  Eigen::Matrix<double,6,1> xi_i1 = lambda12_*knot1_->getVelocity()->getValue() +
                                    psi11_*xi_21 +
                                    psi12_*J_21_inv*knot2_->getVelocity()->getValue();

  // Calculate interpolated relative transformation matrix
  lgmath::se3::Transformation T_i1(xi_i1);

  // Return `global' interpolated transform
  return T_i1*knot1_->getPose()->evaluate();
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Evaluate the transformation matrix tree
//////////////////////////////////////////////////////////////////////////////////////////////
EvalTreeNode<lgmath::se3::Transformation>* SteamTrajPoseInterpEval::evaluateTree() const {

  // Evaluate sub-trees
  EvalTreeNode<lgmath::se3::Transformation>* transform1 = knot1_->getPose()->evaluateTree();
  EvalTreeNode<lgmath::se3::Transformation>* transform2 = knot2_->getPose()->evaluateTree();

  // Get relative matrix info
  lgmath::se3::Transformation T_21 = transform2->getValue()/transform1->getValue();

  // Get se3 algebra of relative matrix
  Eigen::Matrix<double,6,1> xi_21 = T_21.vec();

  // Calculate the 6x6 associated Jacobian
  Eigen::Matrix<double,6,6> J_21_inv = lgmath::se3::vec2jacinv(xi_21);

  // Calculate interpolated relative se3 algebra
  Eigen::Matrix<double,6,1> xi_i1 = lambda12_*knot1_->getVelocity()->getValue() +
                                    psi11_*xi_21 +
                                    psi12_*J_21_inv*knot2_->getVelocity()->getValue();

  // Calculate interpolated relative transformation matrix
  lgmath::se3::Transformation T_i1(xi_i1);

  // Interpolated relative transform - new root node (using pool memory)
  EvalTreeNode<lgmath::se3::Transformation>* root = EvalTreeNode<lgmath::se3::Transformation>::pool.getObj();
  root->setValue(T_i1*transform1->getValue());

  // Add children
  root->addChild(transform1);
  root->addChild(transform2);

  // Return new root node
  return root;
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Implementation for Block Automatic Differentiation
//////////////////////////////////////////////////////////////////////////////////////////////
template<int LHS_DIM, int INNER_DIM, int MAX_STATE_SIZE>
void SteamTrajPoseInterpEval::appendJacobiansImpl(
    const Eigen::Matrix<double,LHS_DIM,INNER_DIM>& lhs,
    EvalTreeNode<lgmath::se3::Transformation>* evaluationTree,
    std::vector<Jacobian<LHS_DIM,MAX_STATE_SIZE> >* outJacobians) const {

  // Cast back to transformations
  EvalTreeNode<lgmath::se3::Transformation>* transform1 =
      static_cast<EvalTreeNode<lgmath::se3::Transformation>*>(evaluationTree->childAt(0));
  EvalTreeNode<lgmath::se3::Transformation>* transform2 =
      static_cast<EvalTreeNode<lgmath::se3::Transformation>*>(evaluationTree->childAt(1));

  // Get relative matrix info
  lgmath::se3::Transformation T_21 = transform2->getValue()/transform1->getValue();

  // Get se3 algebra of relative matrix
  Eigen::Matrix<double,6,1> xi_21 = T_21.vec();

  // Calculate the 6x6 associated Jacobian
  Eigen::Matrix<double,6,6> J_21_inv = lgmath::se3::vec2jacinv(xi_21);

  // Calculate interpolated relative se3 algebra
  Eigen::Matrix<double,6,1> xi_i1 = lambda12_*knot1_->getVelocity()->getValue() +
                                    psi11_*xi_21 +
                                    psi12_*J_21_inv*knot2_->getVelocity()->getValue();

  // Calculate interpolated relative transformation matrix
  lgmath::se3::Transformation T_i1(xi_i1);

  // Calculate the 6x6 Jacobian associated with the interpolated relative transformation matrix
  Eigen::Matrix<double,6,6> J_i1 = lgmath::se3::vec2jac(xi_i1);

  // Check if evaluator is active
  if (this->isActive()) {

    // Pose Jacobians
    if (knot1_->getPose()->isActive() || knot2_->getPose()->isActive()) {

      // Precompute matrix
      Eigen::Matrix<double,6,6> w = psi11_*J_i1*J_21_inv +
        0.5*psi12_*J_i1*lgmath::se3::curlyhat(knot2_->getVelocity()->getValue())*J_21_inv;

      // Check if transform1 is active
      if (knot1_->getPose()->isActive()) {
        Eigen::Matrix<double,6,6> jacobian = (-1) * w * T_21.adjoint() + T_i1.adjoint();
        knot1_->getPose()->appendBlockAutomaticJacobians(lhs*jacobian, transform1, outJacobians);
      }

      // Get index of split between left and right-hand-side of Jacobians
      unsigned int hintIndex = outJacobians->size();

      // Check if transform2 is active
      if (knot2_->getPose()->isActive()) {
        knot2_->getPose()->appendBlockAutomaticJacobians(lhs*w, transform2, outJacobians);
      }

      // Merge jacobians
      Jacobian<LHS_DIM,MAX_STATE_SIZE>::merge(outJacobians, hintIndex);
    }

    // 6 x 6 Velocity Jacobian 1
    if(!knot1_->getVelocity()->isLocked()) {

      // Add Jacobian
      outJacobians->push_back(Jacobian<LHS_DIM,MAX_STATE_SIZE>(knot1_->getVelocity()->getKey(), lhs*lambda12_*J_i1));
    }

    // 6 x 6 Velocity Jacobian 2
    if(!knot2_->getVelocity()->isLocked()) {

      // Add Jacobian
      Eigen::Matrix<double,6,6> jacobian = psi12_*J_i1*J_21_inv;
      outJacobians->push_back(Jacobian<LHS_DIM,MAX_STATE_SIZE>(knot2_->getVelocity()->getKey(), lhs*jacobian));
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Evaluate the Jacobian tree
//////////////////////////////////////////////////////////////////////////////////////////////
void SteamTrajPoseInterpEval::appendBlockAutomaticJacobians(
    const Eigen::MatrixXd& lhs,
    EvalTreeNode<lgmath::se3::Transformation>* evaluationTree,
    std::vector<Jacobian<> >* outJacobians) const {
  this->appendJacobiansImpl(lhs,evaluationTree, outJacobians);
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Fixed-size evaluations of the Jacobian tree
//////////////////////////////////////////////////////////////////////////////////////////////
void SteamTrajPoseInterpEval::appendBlockAutomaticJacobians(
    const Eigen::Matrix<double,1,6>& lhs,
    EvalTreeNode<lgmath::se3::Transformation>* evaluationTree,
    std::vector<Jacobian<1,6> >* outJacobians) const {
  this->appendJacobiansImpl(lhs,evaluationTree, outJacobians);
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Fixed-size evaluations of the Jacobian tree
//////////////////////////////////////////////////////////////////////////////////////////////
void SteamTrajPoseInterpEval::appendBlockAutomaticJacobians(
    const Eigen::Matrix<double,2,6>& lhs,
    EvalTreeNode<lgmath::se3::Transformation>* evaluationTree,
    std::vector<Jacobian<2,6> >* outJacobians) const {
  this->appendJacobiansImpl(lhs,evaluationTree, outJacobians);
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Fixed-size evaluations of the Jacobian tree
//////////////////////////////////////////////////////////////////////////////////////////////
void SteamTrajPoseInterpEval::appendBlockAutomaticJacobians(
    const Eigen::Matrix<double,3,6>& lhs,
    EvalTreeNode<lgmath::se3::Transformation>* evaluationTree,
    std::vector<Jacobian<3,6> >* outJacobians) const {
  this->appendJacobiansImpl(lhs,evaluationTree, outJacobians);
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Fixed-size evaluations of the Jacobian tree
//////////////////////////////////////////////////////////////////////////////////////////////
void SteamTrajPoseInterpEval::appendBlockAutomaticJacobians(
    const Eigen::Matrix<double,4,6>& lhs,
    EvalTreeNode<lgmath::se3::Transformation>* evaluationTree,
    std::vector<Jacobian<4,6> >* outJacobians) const {
  this->appendJacobiansImpl(lhs,evaluationTree, outJacobians);
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Fixed-size evaluations of the Jacobian tree
//////////////////////////////////////////////////////////////////////////////////////////////
void SteamTrajPoseInterpEval::appendBlockAutomaticJacobians(
    const Eigen::Matrix<double,6,6>& lhs,
    EvalTreeNode<lgmath::se3::Transformation>* evaluationTree,
    std::vector<Jacobian<6,6> >* outJacobians) const {
  this->appendJacobiansImpl(lhs,evaluationTree, outJacobians);
}

} // se3
} // steam

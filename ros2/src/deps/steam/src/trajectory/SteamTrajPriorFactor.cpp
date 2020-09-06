//////////////////////////////////////////////////////////////////////////////////////////////
/// \file SteamTrajPriorFactor.cpp
///
/// \author Sean Anderson, ASRL
//////////////////////////////////////////////////////////////////////////////////////////////

#include <steam/trajectory/SteamTrajPriorFactor.hpp>

#include <lgmath.hpp>

namespace steam {
namespace se3 {

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Constructor
//////////////////////////////////////////////////////////////////////////////////////////////
SteamTrajPriorFactor::SteamTrajPriorFactor(const SteamTrajVar::ConstPtr& knot1,
                                           const SteamTrajVar::ConstPtr& knot2) :
  knot1_(knot1), knot2_(knot2) {
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Returns whether or not an evaluator contains unlocked state variables
//////////////////////////////////////////////////////////////////////////////////////////////
bool SteamTrajPriorFactor::isActive() const {
  return knot1_->getPose()->isActive()  || !knot1_->getVelocity()->isLocked() ||
         knot2_->getPose()->isActive()  || !knot2_->getVelocity()->isLocked();
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Evaluate the GP prior factor
//////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXd SteamTrajPriorFactor::evaluate() const {

  // Precompute values
  lgmath::se3::Transformation T_21 = knot2_->getPose()->evaluate()/knot1_->getPose()->evaluate();
  Eigen::Matrix<double,6,1> xi_21 = T_21.vec();
  Eigen::Matrix<double,6,6> J_21_inv = lgmath::se3::vec2jacinv(xi_21);

  // Compute error
  Eigen::Matrix<double,12,1> error;
  error.head<6>() = xi_21 - (knot2_->getTime() - knot1_->getTime()).seconds()*knot1_->getVelocity()->getValue();
  error.tail<6>() = J_21_inv * knot2_->getVelocity()->getValue() - knot1_->getVelocity()->getValue();
  return error;
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Evaluate the GP prior factor and Jacobians
//////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXd SteamTrajPriorFactor::evaluate(const Eigen::MatrixXd& lhs,
                                            std::vector<Jacobian<> >* jacs) const {

  // Check and initialize jacobian array
  if (jacs == NULL) {
    throw std::invalid_argument("Null pointer provided to return-input 'jacs' in evaluate");
  }
  jacs->clear();

  // Get evaluation trees
  EvalTreeNode<lgmath::se3::Transformation>* evaluationTree1 = knot1_->getPose()->evaluateTree();
  EvalTreeNode<lgmath::se3::Transformation>* evaluationTree2 = knot2_->getPose()->evaluateTree();

  // Compute intermediate values
  lgmath::se3::Transformation T_21 = evaluationTree2->getValue()/evaluationTree1->getValue();
  Eigen::Matrix<double,6,1> xi_21 = T_21.vec();
  Eigen::Matrix<double,6,6> J_21_inv = lgmath::se3::vec2jacinv(xi_21);
  double deltaTime = (knot2_->getTime() - knot1_->getTime()).seconds();

  // Knot 1 transform
  if(knot1_->getPose()->isActive()) {
    Eigen::Matrix<double,6,6> Jinv_12 = J_21_inv*T_21.adjoint();

    // Construct jacobian
    Eigen::Matrix<double,12,6> jacobian;
    jacobian.topRows<6>() = -Jinv_12;
    jacobian.bottomRows<6>() = -0.5*lgmath::se3::curlyhat(knot2_->getVelocity()->getValue())*Jinv_12;

    // Get Jacobians
    knot1_->getPose()->appendBlockAutomaticJacobians(lhs * jacobian, evaluationTree1, jacs);
  }

  // Get index of split between left and right-hand-side of pose Jacobians
  unsigned int hintIndex = jacs->size();

  // Knot 2 transform
  if(knot2_->getPose()->isActive()) {

    // Construct jacobian
    Eigen::Matrix<double,12,6> jacobian;
    jacobian.topRows<6>() = J_21_inv;
    jacobian.bottomRows<6>() = 0.5*lgmath::se3::curlyhat(knot2_->getVelocity()->getValue())*J_21_inv;

    // Get Jacobians
    knot2_->getPose()->appendBlockAutomaticJacobians(lhs * jacobian, evaluationTree2, jacs);
  }

  // Merge jacobians (transform evaluators from knots 1 and 2 could contain the same
  // state variables, in which case we need to merge)
  Jacobian<>::merge(jacs, hintIndex);

  // Knot 1 velocity
  if(!knot1_->getVelocity()->isLocked()) {

    // Construct Jacobian Object
    jacs->push_back(Jacobian<>());
    Jacobian<>& jacref = jacs->back();
    jacref.key = knot1_->getVelocity()->getKey();

    // Fill in matrix
    Eigen::Matrix<double,12,6> jacobian;
    jacobian.topRows<6>() = -deltaTime*Eigen::Matrix<double,6,6>::Identity();
    jacobian.bottomRows<6>() = -Eigen::Matrix<double,6,6>::Identity();
    jacref.jac = lhs * jacobian;
  }

  // Knot 2 velocity
  if(!knot2_->getVelocity()->isLocked()) {

    // Construct Jacobian Object
    jacs->push_back(Jacobian<>());
    Jacobian<>& jacref = jacs->back();
    jacref.key = knot2_->getVelocity()->getKey();

    // Fill in matrix
    Eigen::Matrix<double,12,6> jacobian;
    jacobian.topRows<6>() = Eigen::Matrix<double,6,6>::Zero();
    jacobian.bottomRows<6>() = J_21_inv;
    jacref.jac = lhs * jacobian;
  }

  // Return tree memory to pool
  EvalTreeNode<lgmath::se3::Transformation>::pool.returnObj(evaluationTree1);
  EvalTreeNode<lgmath::se3::Transformation>::pool.returnObj(evaluationTree2);

  // Return error
  Eigen::Matrix<double,12,1> error;
  error.head<6>() = xi_21 - deltaTime*knot1_->getVelocity()->getValue();
  error.tail<6>() = J_21_inv * knot2_->getVelocity()->getValue() - knot1_->getVelocity()->getValue();
  return error;
}

} // se3
} // steam

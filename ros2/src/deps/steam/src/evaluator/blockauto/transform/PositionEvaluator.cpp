//////////////////////////////////////////////////////////////////////////////////////////////
/// \file PositionEvaluator.cpp
///
/// \author Sean Anderson, ASRL
//////////////////////////////////////////////////////////////////////////////////////////////

#include <steam/evaluator/blockauto/transform/PositionEvaluator.hpp>

namespace steam {
namespace se3 {

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Constructor
//////////////////////////////////////////////////////////////////////////////////////////////
PositionEvaluator::PositionEvaluator(const TransformEvaluator::ConstPtr &transform) : transform_(transform) {
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Pseudo constructor - return a shared pointer to a new instance
//////////////////////////////////////////////////////////////////////////////////////////////
PositionEvaluator::Ptr PositionEvaluator::MakeShared(const TransformEvaluator::ConstPtr &transform) {
  return PositionEvaluator::Ptr(new PositionEvaluator(transform));
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Returns whether or not an evaluator contains unlocked state variables
//////////////////////////////////////////////////////////////////////////////////////////////
bool PositionEvaluator::isActive() const {
  return transform_->isActive();
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Adds references (shared pointers) to active state variables to the map output
//////////////////////////////////////////////////////////////////////////////////////////////
void PositionEvaluator::getActiveStateVariables(
    std::map<unsigned int, steam::StateVariableBase::Ptr> *outStates) const {
  transform_->getActiveStateVariables(outStates);
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Evaluate the resultant 6x1 vector belonging to the se(3) algebra
//////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix<double, 3, 1> PositionEvaluator::evaluate() const {
  return transform_->evaluate().r_ba_ina();
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Evaluate the resultant 6x1 vector belonging to the se(3) algebra and
///        sub-tree of evaluations
//////////////////////////////////////////////////////////////////////////////////////////////
EvalTreeNode<Eigen::Matrix<double, 3, 1> > *PositionEvaluator::evaluateTree() const {

  // Evaluate sub-trees
  EvalTreeNode<lgmath::se3::Transformation> *transform = transform_->evaluateTree();

  // Make new root node -- note we get memory from the pool
  EvalTreeNode<Eigen::Matrix<double, 3, 1> > *root = EvalTreeNode<Eigen::Matrix<double, 3, 1> >::pool.getObj();
  root->setValue(transform->getValue().r_ba_ina());

  // Add children
  root->addChild(transform);

  // Return new root node
  return root;
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Implementation for Block Automatic Differentiation
//////////////////////////////////////////////////////////////////////////////////////////////
template<int LHS_DIM, int INNER_DIM, int MAX_STATE_SIZE>
void PositionEvaluator::appendJacobiansImpl(
    const Eigen::Matrix<double, LHS_DIM, INNER_DIM> &lhs,
    EvalTreeNode<Eigen::Matrix<double, 3, 1> > *evaluationTree,
    std::vector<Jacobian<LHS_DIM, MAX_STATE_SIZE> > *outJacobians) const {

  EvalTreeNode<lgmath::se3::Transformation> *t1 =
      static_cast<EvalTreeNode<lgmath::se3::Transformation> *>(evaluationTree->childAt(0));

  // Check if transform is active
  if (transform_->isActive()) {
    auto tf = t1->getValue();
    Eigen::Matrix<double, 3, 6> newLhs;
    newLhs << -tf.C_ba().transpose(), Eigen::Matrix3d::Zero();  //lgmath::so3::hat(tf.r_ab_inb());
    transform_->appendBlockAutomaticJacobians(lhs * newLhs, t1, outJacobians);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Evaluate the Jacobian tree
//////////////////////////////////////////////////////////////////////////////////////////////
void PositionEvaluator::appendBlockAutomaticJacobians(const Eigen::MatrixXd &lhs,
                                                      EvalTreeNode<Eigen::Matrix<double, 3, 1> > *evaluationTree,
                                                      std::vector<Jacobian<> > *outJacobians) const {
  this->appendJacobiansImpl(lhs, evaluationTree, outJacobians);
}

void PositionEvaluator::appendBlockAutomaticJacobians(const Eigen::Matrix<double, 1, 3> &lhs,
                                                      EvalTreeNode<Eigen::Matrix<double, 3, 1> > *evaluationTree,
                                                      std::vector<Jacobian<1, 6> > *outJacobians) const {
  this->appendJacobiansImpl(lhs, evaluationTree, outJacobians);
}

void PositionEvaluator::appendBlockAutomaticJacobians(const Eigen::Matrix<double, 2, 3> &lhs,
                                                      EvalTreeNode<Eigen::Matrix<double, 3, 1> > *evaluationTree,
                                                      std::vector<Jacobian<2, 6> > *outJacobians) const {
  this->appendJacobiansImpl(lhs, evaluationTree, outJacobians);
}

void PositionEvaluator::appendBlockAutomaticJacobians(const Eigen::Matrix<double, 3, 3> &lhs,
                                                      EvalTreeNode<Eigen::Matrix<double, 3, 1> > *evaluationTree,
                                                      std::vector<Jacobian<3, 6> > *outJacobians) const {
  this->appendJacobiansImpl(lhs, evaluationTree, outJacobians);
}

void PositionEvaluator::appendBlockAutomaticJacobians(const Eigen::Matrix<double, 4, 3> &lhs,
                                                      EvalTreeNode<Eigen::Matrix<double, 3, 1> > *evaluationTree,
                                                      std::vector<Jacobian<4, 6> > *outJacobians) const {
  this->appendJacobiansImpl(lhs, evaluationTree, outJacobians);
}

void PositionEvaluator::appendBlockAutomaticJacobians(const Eigen::Matrix<double, 6, 3> &lhs,
                                                      EvalTreeNode<Eigen::Matrix<double, 3, 1> > *evaluationTree,
                                                      std::vector<Jacobian<6, 6> > *outJacobians) const {
  this->appendJacobiansImpl(lhs, evaluationTree, outJacobians);
}

} // se3
} // steam

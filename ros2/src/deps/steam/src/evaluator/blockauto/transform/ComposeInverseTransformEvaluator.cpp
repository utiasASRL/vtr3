//////////////////////////////////////////////////////////////////////////////////////////////
/// \file ComposeInverseTransformEvaluator.cpp
///
/// \author Sean Anderson, ASRL
//////////////////////////////////////////////////////////////////////////////////////////////

#include <steam/evaluator/blockauto/transform/ComposeInverseTransformEvaluator.hpp>

#include <lgmath.hpp>

namespace steam {
namespace se3 {

/// Compose

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Constructor
//////////////////////////////////////////////////////////////////////////////////////////////
ComposeInverseTransformEvaluator::ComposeInverseTransformEvaluator(
    const TransformEvaluator::ConstPtr& transform_bx,
    const TransformEvaluator::ConstPtr& transform_ax)
  : transform_bx_(transform_bx), transform_ax_(transform_ax) {
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Pseudo constructor - return a shared pointer to a new instance
//////////////////////////////////////////////////////////////////////////////////////////////
ComposeInverseTransformEvaluator::Ptr ComposeInverseTransformEvaluator::MakeShared(
    const TransformEvaluator::ConstPtr& transform_bx,
    const TransformEvaluator::ConstPtr& transform_ax) {
  return ComposeInverseTransformEvaluator::Ptr(
        new ComposeInverseTransformEvaluator(transform_bx, transform_ax));
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Returns whether or not an evaluator contains unlocked state variables
//////////////////////////////////////////////////////////////////////////////////////////////
bool ComposeInverseTransformEvaluator::isActive() const {
  return transform_bx_->isActive() || transform_ax_->isActive();
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Adds references (shared pointers) to active state variables to the map output
//////////////////////////////////////////////////////////////////////////////////////////////
void ComposeInverseTransformEvaluator::getActiveStateVariables(
    std::map<unsigned int, steam::StateVariableBase::Ptr>* outStates) const {
  transform_bx_->getActiveStateVariables(outStates);
  transform_ax_->getActiveStateVariables(outStates);
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Evaluate the resultant transformation matrix (transform_bx*inv(transform_ax))
//////////////////////////////////////////////////////////////////////////////////////////////
lgmath::se3::Transformation ComposeInverseTransformEvaluator::evaluate() const {
  return transform_bx_->evaluate()/transform_ax_->evaluate();
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Evaluate the transformation matrix tree
//////////////////////////////////////////////////////////////////////////////////////////////
EvalTreeNode<lgmath::se3::Transformation>* ComposeInverseTransformEvaluator::evaluateTree() const {

  // Evaluate sub-trees
  EvalTreeNode<lgmath::se3::Transformation>* tf_bx_tree = transform_bx_->evaluateTree();
  EvalTreeNode<lgmath::se3::Transformation>* tf_ax_tree = transform_ax_->evaluateTree();

  // Make new root node -- note we get memory from the pool
  EvalTreeNode<lgmath::se3::Transformation>* root =
      EvalTreeNode<lgmath::se3::Transformation>::pool.getObj();
  root->setValue(tf_bx_tree->getValue()/tf_ax_tree->getValue());

  // Add children
  root->addChild(tf_bx_tree);
  root->addChild(tf_ax_tree);

  // Return new root node
  return root;
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Implementation for Block Automatic Differentiation
//////////////////////////////////////////////////////////////////////////////////////////////
template<int LHS_DIM, int INNER_DIM, int MAX_STATE_SIZE>
void ComposeInverseTransformEvaluator::appendJacobiansImpl(
    const Eigen::Matrix<double,LHS_DIM,INNER_DIM>& lhs,
    EvalTreeNode<lgmath::se3::Transformation>* evaluationTree,
    std::vector<Jacobian<LHS_DIM,MAX_STATE_SIZE> >* outJacobians) const {

  // Cast back to transformation
  EvalTreeNode<lgmath::se3::Transformation>* tf_bx_tree =
      static_cast<EvalTreeNode<lgmath::se3::Transformation>*>(evaluationTree->childAt(0));

  // Check if transform_bx is active
  if (transform_bx_->isActive()) {

    // LHS Jacobian passes through -- identity multiplication
    transform_bx_->appendBlockAutomaticJacobians(lhs, tf_bx_tree, outJacobians);
  }

  // Get index of split between left and right-hand-side of Jacobians
  unsigned int hintIndex = outJacobians->size();

  // Check if transform_ax is active
  if (transform_ax_->isActive()) {

    EvalTreeNode<lgmath::se3::Transformation>* tf_ax_tree =
        static_cast<EvalTreeNode<lgmath::se3::Transformation>*>(evaluationTree->childAt(1));

    lgmath::se3::Transformation tf_ba = tf_bx_tree->getValue()/tf_ax_tree->getValue();
    Eigen::Matrix<double,LHS_DIM,INNER_DIM> newLhs = (-1)*lhs*tf_ba.adjoint();
    transform_ax_->appendBlockAutomaticJacobians(newLhs, tf_ax_tree, outJacobians);
  }

  // Merge jacobians
  Jacobian<LHS_DIM,MAX_STATE_SIZE>::merge(outJacobians, hintIndex);
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Evaluate the Jacobian tree
//////////////////////////////////////////////////////////////////////////////////////////////
void ComposeInverseTransformEvaluator::appendBlockAutomaticJacobians(const Eigen::MatrixXd& lhs,
    EvalTreeNode<lgmath::se3::Transformation>* evaluationTree,
    std::vector<Jacobian<> >* outJacobians) const {
  this->appendJacobiansImpl(lhs,evaluationTree, outJacobians);
}

void ComposeInverseTransformEvaluator::appendBlockAutomaticJacobians(
    const Eigen::Matrix<double,1,6>& lhs,
    EvalTreeNode<lgmath::se3::Transformation>* evaluationTree,
    std::vector<Jacobian<1,6> >* outJacobians) const {
  this->appendJacobiansImpl(lhs,evaluationTree, outJacobians);
}

void ComposeInverseTransformEvaluator::appendBlockAutomaticJacobians(
    const Eigen::Matrix<double,2,6>& lhs,
    EvalTreeNode<lgmath::se3::Transformation>* evaluationTree,
    std::vector<Jacobian<2,6> >* outJacobians) const {
  this->appendJacobiansImpl(lhs,evaluationTree, outJacobians);
}

void ComposeInverseTransformEvaluator::appendBlockAutomaticJacobians(
    const Eigen::Matrix<double,3,6>& lhs,
    EvalTreeNode<lgmath::se3::Transformation>* evaluationTree,
    std::vector<Jacobian<3,6> >* outJacobians) const {
  this->appendJacobiansImpl(lhs,evaluationTree, outJacobians);
}

void ComposeInverseTransformEvaluator::appendBlockAutomaticJacobians(
    const Eigen::Matrix<double,4,6>& lhs,
    EvalTreeNode<lgmath::se3::Transformation>* evaluationTree,
    std::vector<Jacobian<4,6> >* outJacobians) const {
  this->appendJacobiansImpl(lhs,evaluationTree, outJacobians);
}

void ComposeInverseTransformEvaluator::appendBlockAutomaticJacobians(
    const Eigen::Matrix<double,6,6>& lhs,
    EvalTreeNode<lgmath::se3::Transformation>* evaluationTree,
    std::vector<Jacobian<6,6> >* outJacobians) const {
  this->appendJacobiansImpl(lhs,evaluationTree, outJacobians);
}

} // se3
} // steam

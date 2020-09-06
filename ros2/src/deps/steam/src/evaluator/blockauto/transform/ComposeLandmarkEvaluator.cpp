//////////////////////////////////////////////////////////////////////////////////////////////
/// \file ComposeLandmarkEvaluator.cpp
///
/// \author Sean Anderson, ASRL
//////////////////////////////////////////////////////////////////////////////////////////////

#include <steam/evaluator/blockauto/transform/ComposeLandmarkEvaluator.hpp>

// remove these includes once 'relative' landmark is gone
#include <steam/evaluator/blockauto/transform/ComposeTransformEvaluator.hpp>
#include <steam/evaluator/blockauto/transform/InverseTransformEvaluator.hpp>

#include <lgmath.hpp>

namespace steam {
namespace se3 {

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Constructor
//////////////////////////////////////////////////////////////////////////////////////////////
ComposeLandmarkEvaluator::ComposeLandmarkEvaluator(const TransformEvaluator::ConstPtr& transform,
                                                   const se3::LandmarkStateVar::Ptr& landmark)
  : transform_(transform), landmark_(landmark) {
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Pseudo constructor - return a shared pointer to a new instance
//////////////////////////////////////////////////////////////////////////////////////////////
ComposeLandmarkEvaluator::Ptr ComposeLandmarkEvaluator::MakeShared(const TransformEvaluator::ConstPtr& transform,
                                                                   const se3::LandmarkStateVar::Ptr& landmark) {
  return ComposeLandmarkEvaluator::Ptr(new ComposeLandmarkEvaluator(transform, landmark));
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Returns whether or not an evaluator contains unlocked state variables
//////////////////////////////////////////////////////////////////////////////////////////////
bool ComposeLandmarkEvaluator::isActive() const {
  return transform_->isActive() || !landmark_->isLocked();
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Adds references (shared pointers) to active state variables to the map output
//////////////////////////////////////////////////////////////////////////////////////////////
void ComposeLandmarkEvaluator::getActiveStateVariables(
    std::map<unsigned int, steam::StateVariableBase::Ptr>* outStates) const {
  transform_->getActiveStateVariables(outStates);
  if (!landmark_->isLocked()) {
    (*outStates)[landmark_->getKey().getID()] = landmark_;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Evaluate the point transformed by the transform evaluator
//////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Vector4d ComposeLandmarkEvaluator::evaluate() const {
  return transform_->evaluate()*landmark_->getValue();
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Evaluate the point transformed by the transform evaluator and
///        sub-tree of evaluations
//////////////////////////////////////////////////////////////////////////////////////////////
EvalTreeNode<Eigen::Vector4d>* ComposeLandmarkEvaluator::evaluateTree() const {

  // Evaluate transform sub-tree
  EvalTreeNode<lgmath::se3::Transformation>* transform = transform_->evaluateTree();

  // Make new leaf node for landmark state variable -- note we get memory from the pool
  EvalTreeNode<Eigen::Vector4d>* landmarkLeaf = EvalTreeNode<Eigen::Vector4d>::pool.getObj();
  landmarkLeaf->setValue(landmark_->getValue());

  // Make new root node -- note we get memory from the pool
  EvalTreeNode<Eigen::Vector4d>* root = EvalTreeNode<Eigen::Vector4d>::pool.getObj();
  root->setValue(transform->getValue()*landmarkLeaf->getValue());

  // Add children
  root->addChild(transform);
  root->addChild(landmarkLeaf);

  // Return new root node
  return root;
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Implementation for Block Automatic Differentiation
//////////////////////////////////////////////////////////////////////////////////////////////
template<int LHS_DIM, int INNER_DIM, int MAX_STATE_SIZE>
void ComposeLandmarkEvaluator::appendJacobiansImpl(
    const Eigen::Matrix<double,LHS_DIM,INNER_DIM>& lhs,
    EvalTreeNode<Eigen::Vector4d>* evaluationTree,
    std::vector<Jacobian<LHS_DIM,MAX_STATE_SIZE> >* outJacobians) const {

  // Cast back to transform
  EvalTreeNode<lgmath::se3::Transformation>* t1 =
      static_cast<EvalTreeNode<lgmath::se3::Transformation>*>(evaluationTree->childAt(0));

  // Check if transform1 is active
  if (transform_->isActive()) {
    const Eigen::Vector4d& homogeneous = evaluationTree->getValue();
    Eigen::Matrix<double,LHS_DIM,6> newLhs = lhs * lgmath::se3::point2fs(homogeneous.head<3>(), homogeneous[3]);
    transform_->appendBlockAutomaticJacobians(newLhs, t1, outJacobians);
  }

  // Check if state is locked
  if (!landmark_->isLocked()) {

    // Construct Jacobian
    Eigen::Matrix<double,4,6> landJac;
    landJac.block<4,3>(0,0) = t1->getValue().matrix().block<4,3>(0,0);
    Eigen::Matrix<double,LHS_DIM,MAX_STATE_SIZE> lhsLandJac = lhs * landJac;
    outJacobians->push_back(Jacobian<LHS_DIM,MAX_STATE_SIZE>(landmark_->getKey(), lhsLandJac));
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Evaluate the Jacobian tree
//////////////////////////////////////////////////////////////////////////////////////////////
void ComposeLandmarkEvaluator::appendBlockAutomaticJacobians(const Eigen::MatrixXd& lhs,
                                  EvalTreeNode<Eigen::Vector4d>* evaluationTree,
                                  std::vector<Jacobian<> >* outJacobians) const {
  this->appendJacobiansImpl(lhs,evaluationTree, outJacobians);
}

void ComposeLandmarkEvaluator::appendBlockAutomaticJacobians(const Eigen::Matrix<double,1,4>& lhs,
                              EvalTreeNode<Eigen::Vector4d>* evaluationTree,
                              std::vector<Jacobian<1,6> >* outJacobians) const {
  this->appendJacobiansImpl(lhs,evaluationTree, outJacobians);
}

void ComposeLandmarkEvaluator::appendBlockAutomaticJacobians(const Eigen::Matrix<double,2,4>& lhs,
                              EvalTreeNode<Eigen::Vector4d>* evaluationTree,
                              std::vector<Jacobian<2,6> >* outJacobians) const {
  this->appendJacobiansImpl(lhs,evaluationTree, outJacobians);
}

void ComposeLandmarkEvaluator::appendBlockAutomaticJacobians(const Eigen::Matrix<double,3,4>& lhs,
                              EvalTreeNode<Eigen::Vector4d>* evaluationTree,
                              std::vector<Jacobian<3,6> >* outJacobians) const {
  this->appendJacobiansImpl(lhs,evaluationTree, outJacobians);
}

void ComposeLandmarkEvaluator::appendBlockAutomaticJacobians(const Eigen::Matrix<double,4,4>& lhs,
                              EvalTreeNode<Eigen::Vector4d>* evaluationTree,
                              std::vector<Jacobian<4,6> >* outJacobians) const {
  this->appendJacobiansImpl(lhs,evaluationTree, outJacobians);
}

void ComposeLandmarkEvaluator::appendBlockAutomaticJacobians(const Eigen::Matrix<double,6,4>& lhs,
                              EvalTreeNode<Eigen::Vector4d>* evaluationTree,
                              std::vector<Jacobian<6,6> >* outJacobians) const {
  this->appendJacobiansImpl(lhs,evaluationTree, outJacobians);
}

} // se3
} // steam

#pragma once

/**
 * \file MaskEvalOps.hpp
 * \description This file implements boolean logic for mask evaluators, using
 * functors.
 *              The provided operator definitions are for all combinations of
 *              MaskWeightEvaluator::Ptr and bool.  We use pointers as we do
 *              not wish to copy potentially large mask maps.
 */

#include <vtr_pose_graph/evaluator/evaluator_base.hpp>
#include <vtr_pose_graph/evaluator/functors.hpp>

namespace vtr {
namespace pose_graph {
namespace eval {

/**
 * Op Class Definitions ///
 */

/**
 * \brief Evaluator for a binary operation on two evaluators: a
 * {+,-,*,.,<,>,<=,>=,&,|,^} b
 */
template <typename F, class RVAL, class IVAL>
class BinaryOp : public EvalBase<RVAL> {
 public:
  using InBase = EvalBase<IVAL>;

  using SimpleEdge = typename EvalBase<RVAL>::SimpleEdge;
  using SimpleVertex = typename EvalBase<RVAL>::SimpleVertex;
  using EdgeIdType = typename EvalBase<RVAL>::EdgeIdType;
  using VertexIdType = typename EvalBase<RVAL>::VertexIdType;
  using EdgeMap = typename EvalBase<RVAL>::EdgeMap;
  using VertexMap = typename EvalBase<RVAL>::VertexMap;

  PTR_TYPEDEFS(BinaryOp)

  static Ptr MakeShared(const typename InBase::Ptr &eval1,
                        const typename InBase::Ptr &eval2) {
    return Ptr(new BinaryOp(eval1, eval2));
  }

  BinaryOp(const typename InBase::Ptr &eval1, const typename InBase::Ptr &eval2)
      : eval1_(eval1), eval2_(eval2), func_(F()) {
  }
  BinaryOp(BinaryOp &&) = default;
  BinaryOp(const BinaryOp &) = default;

  virtual ~BinaryOp() {
  }

  BinaryOp &operator=(BinaryOp &&) = default;
  BinaryOp &operator=(const BinaryOp &) = default;

  virtual void setGraph(void *graph) {
    eval1_->setGraph(graph);
    eval2_->setGraph(graph);
  }

  /**
   * \brief Non-const accessor methods.
   * \note these actually don't return a reference
   */
  virtual inline RVAL operator[](const SimpleEdge &e) {
    return this->func_(this->eval1_->operator[](e),
                       this->eval2_->operator[](e));
  }

  virtual inline RVAL operator[](const SimpleVertex &v) {
    return this->func_(this->eval1_->operator[](v),
                       this->eval2_->operator[](v));
  }

  virtual inline RVAL operator[](const EdgeIdType &e) {
    return this->func_(this->eval1_->operator[](e),
                       this->eval2_->operator[](e));
  }

  virtual inline RVAL operator[](const VertexIdType &v) {
    return this->func_(this->eval1_->operator[](v),
                       this->eval2_->operator[](v));
  }

  /**
   * \brief Const accessor methods.
   */
  virtual inline RVAL at(const SimpleEdge &e) const {
    return this->func_(this->eval1_->at(e), this->eval2_->at(e));
  }

  virtual inline RVAL at(const SimpleVertex &v) const {
    return this->func_(this->eval1_->at(v), this->eval2_->at(v));
  }

  virtual inline RVAL at(const EdgeIdType &e) const {
    return this->func_(this->eval1_->at(e), this->eval2_->at(e));
  }

  virtual inline RVAL at(const VertexIdType &v) const {
    return this->func_(this->eval1_->at(v), this->eval2_->at(v));
  }

 private:
  typename EvalBase<IVAL>::Ptr eval1_;

  typename EvalBase<IVAL>::Ptr eval2_;

  const F func_;
};

/**
 * \brief Evaluator for a unary operation on an evaluator:
 * {~,-,sgm,sin,cos,tan}a
 */
template <typename F, class RVAL, class IVAL>
class UnaryOp : public EvalBase<RVAL> {
 public:
  using Base = EvalBase<RVAL>;
  using InBase = EvalBase<IVAL>;

  using SimpleEdge = typename Base::SimpleEdge;
  using SimpleVertex = typename Base::SimpleVertex;
  using EdgeIdType = typename Base::EdgeIdType;
  using VertexIdType = typename Base::VertexIdType;
  using EdgeMap = typename Base::EdgeMap;
  using VertexMap = typename Base::VertexMap;

  PTR_TYPEDEFS(UnaryOp)

  static Ptr MakeShared(const typename InBase::Ptr &eval) {
    return Ptr(new UnaryOp(eval));
  }

  UnaryOp(const typename InBase::Ptr &eval) : eval_(eval), func_(F()) {
  }
  UnaryOp(UnaryOp &&) = default;
  UnaryOp(const UnaryOp &) = default;

  UnaryOp &operator=(UnaryOp &&) = default;
  UnaryOp &operator=(const UnaryOp &) = default;

  virtual void setGraph(void *graph) {
    eval_->setGraph(graph);
  }

  virtual ~UnaryOp() {
  }

  /**
   * \brief Non-const accessor methods.
   * \note these actually don't return a reference
   */
  virtual inline RVAL operator[](const SimpleEdge &e) {
    return this->func_(this->eval_->operator[](e));
  }

  virtual inline RVAL operator[](const SimpleVertex &v) {
    return this->func_(this->eval_->operator[](v));
  }

  virtual inline RVAL operator[](const EdgeIdType &e) {
    return this->func_(this->eval_->operator[](e));
  }

  virtual inline RVAL operator[](const VertexIdType &v) {
    return this->func_(this->eval_->operator[](v));
  }

  /** \brief Const accessor methods. */
  virtual inline RVAL at(const SimpleEdge &e) const {
    return this->func_(this->eval_->at(e));
  }

  virtual inline RVAL at(const SimpleVertex &v) const {
    return this->func_(this->eval_->at(v));
  }

  virtual inline RVAL at(const EdgeIdType &e) const {
    return this->func_(this->eval_->at(e));
  }

  virtual inline RVAL at(const VertexIdType &v) const {
    return this->func_(this->eval_->at(v));
  }

 private:
  /** \brief Underlying evaluator */
  typename InBase::Ptr eval_;

  /** \brief Funtor containing function to be applied */
  const F func_;
};

/**
 * \brief Defines individual boolean logic operators for {Eval<Scalar>, Scalar}
 * x {Eval<Scalar>, Scalar}
 */
#define EVALUATOR_AND_FUNC_HEADER(ReturnType, ScalarType) \
  EVALUATOR_BINARY_FUNC_HEADER(ReturnType, ScalarType, andOp, And)
#define EVALUATOR_OR_FUNC_HEADER(ReturnType, ScalarType) \
  EVALUATOR_BINARY_FUNC_HEADER(ReturnType, ScalarType, orOp, Or)
#define EVALUATOR_XOR_FUNC_HEADER(ReturnType, ScalarType) \
  EVALUATOR_BINARY_FUNC_HEADER(ReturnType, ScalarType, xorOp, Xor)
#define EVALUATOR_NOT_FUNC_HEADER(ReturnType, ScalarType) \
  EVALUATOR_UNARY_FUNC_HEADER(ReturnType, ScalarType, notOp, Not)

#define EVALUATOR_AND_FUNC_IMPL(ReturnType, ScalarType) \
  EVALUATOR_BINARY_FUNC_IMPL(ReturnType, ScalarType, andOp, And)
#define EVALUATOR_OR_FUNC_IMPL(ReturnType, ScalarType) \
  EVALUATOR_BINARY_FUNC_IMPL(ReturnType, ScalarType, orOp, Or)
#define EVALUATOR_XOR_FUNC_IMPL(ReturnType, ScalarType) \
  EVALUATOR_BINARY_FUNC_IMPL(ReturnType, ScalarType, xorOp, Xor)
#define EVALUATOR_NOT_FUNC_IMPL(ReturnType, ScalarType) \
  EVALUATOR_UNARY_FUNC_IMPL(ReturnType, ScalarType, notOp, Not)

/**
 * \brief Defines all boolean logic operators for {Eval<Scalar>, Scalar} x
 * {Eval<Scalar>, Scalar}
 */
#define EVALUATOR_BOOLEAN_INTERFACE_HPP(ReturnType, ScalarType) \
  EVALUATOR_AND_FUNC_HEADER(ReturnType, ScalarType)             \
  EVALUATOR_OR_FUNC_HEADER(ReturnType, ScalarType)              \
  EVALUATOR_XOR_FUNC_HEADER(ReturnType, ScalarType)             \
  EVALUATOR_NOT_FUNC_HEADER(ReturnType, ScalarType)

#define EVALUATOR_BOOLEAN_INTERFACE_CPP(ReturnType, ScalarType) \
  EVALUATOR_AND_FUNC_IMPL(ReturnType, ScalarType)               \
  EVALUATOR_OR_FUNC_IMPL(ReturnType, ScalarType)                \
  EVALUATOR_XOR_FUNC_IMPL(ReturnType, ScalarType)               \
  EVALUATOR_NOT_FUNC_IMPL(ReturnType, ScalarType)

/**
 * \brief Defines individual BEDMAS operators for {Eval<Scalar>, Scalar} x
 * {Eval<Scalar>, Scalar}
 */
#define EVALUATOR_ADD_FUNC_HEADER(ReturnType, ScalarType) \
  EVALUATOR_BINARY_FUNC_HEADER(ReturnType, ScalarType, addOp, Add)
#define EVALUATOR_SUB_FUNC_HEADER(ReturnType, ScalarType) \
  EVALUATOR_BINARY_FUNC_HEADER(ReturnType, ScalarType, subOp, Sub)
#define EVALUATOR_MUL_FUNC_HEADER(ReturnType, ScalarType) \
  EVALUATOR_BINARY_FUNC_HEADER(ReturnType, ScalarType, mulOp, Mul)
#define EVALUATOR_DIV_FUNC_HEADER(ReturnType, ScalarType) \
  EVALUATOR_BINARY_FUNC_HEADER(ReturnType, ScalarType, divOp, Div)
#define EVALUATOR_NEG_FUNC_HEADER(ReturnType, ScalarType) \
  EVALUATOR_UNARY_FUNC_HEADER(ReturnType, ScalarType, negOp, Neg)

#define EVALUATOR_ADD_FUNC_IMPL(ReturnType, ScalarType) \
  EVALUATOR_BINARY_FUNC_IMPL(ReturnType, ScalarType, addOp, Add)
#define EVALUATOR_SUB_FUNC_IMPL(ReturnType, ScalarType) \
  EVALUATOR_BINARY_FUNC_IMPL(ReturnType, ScalarType, subOp, Sub)
#define EVALUATOR_MUL_FUNC_IMPL(ReturnType, ScalarType) \
  EVALUATOR_BINARY_FUNC_IMPL(ReturnType, ScalarType, mulOp, Mul)
#define EVALUATOR_DIV_FUNC_IMPL(ReturnType, ScalarType) \
  EVALUATOR_BINARY_FUNC_IMPL(ReturnType, ScalarType, divOp, Div)
#define EVALUATOR_NEG_FUNC_IMPL(ReturnType, ScalarType) \
  EVALUATOR_UNARY_FUNC_IMPL(ReturnType, ScalarType, negOp, Neg)

/**
 * \brief Defines basic math operators for {Eval<Scalar>, Scalar} x {+,-,/,*} x
 * {Eval<Scalar>, Scalar}
 */
#define EVALUATOR_BASIC_MATH_INTERFACE_HPP(ReturnType, ScalarType) \
  EVALUATOR_ADD_FUNC_HEADER(ReturnType, ScalarType)                \
  EVALUATOR_SUB_FUNC_HEADER(ReturnType, ScalarType)                \
  EVALUATOR_MUL_FUNC_HEADER(ReturnType, ScalarType)                \
  EVALUATOR_DIV_FUNC_HEADER(ReturnType, ScalarType)                \
  EVALUATOR_NEG_FUNC_HEADER(ReturnType, ScalarType)

#define EVALUATOR_BASIC_MATH_INTERFACE_CPP(ReturnType, ScalarType) \
  EVALUATOR_ADD_FUNC_IMPL(ReturnType, ScalarType)                  \
  EVALUATOR_SUB_FUNC_IMPL(ReturnType, ScalarType)                  \
  EVALUATOR_MUL_FUNC_IMPL(ReturnType, ScalarType)                  \
  EVALUATOR_DIV_FUNC_IMPL(ReturnType, ScalarType)                  \
  EVALUATOR_NEG_FUNC_IMPL(ReturnType, ScalarType)

/**
 * \brief Defines unary math functions for {Eval<Scalar>, Scalar} x
 * {Eval<Scalar>, Scalar}
 */
#define EVALUATOR_ABS_FUNC_HEADER(ReturnType, ScalarType) \
  EVALUATOR_UNARY_FUNC_HEADER(ReturnType, ScalarType, absOp, Abs)
#define EVALUATOR_POW_FUNC_HEADER(ReturnType, ScalarType) \
  EVALUATOR_BINARY_FUNC_HEADER(ReturnType, ScalarType, powOp, Pow)
#define EVALUATOR_EXP_FUNC_HEADER(ReturnType, ScalarType) \
  EVALUATOR_UNARY_FUNC_HEADER(ReturnType, ScalarType, expOp, Exp)
#define EVALUATOR_LOG_FUNC_HEADER(ReturnType, ScalarType) \
  EVALUATOR_UNARY_FUNC_HEADER(ReturnType, ScalarType, logOp, Log)
#define EVALUATOR_SGM_FUNC_HEADER(ReturnType, ScalarType) \
  EVALUATOR_UNARY_FUNC_HEADER(ReturnType, ScalarType, sgmOp, Sgm)
#define EVALUATOR_ERF_FUNC_HEADER(ReturnType, ScalarType) \
  EVALUATOR_UNARY_FUNC_HEADER(ReturnType, ScalarType, erfOp, Erf)
#define EVALUATOR_ERFC_FUNC_HEADER(ReturnType, ScalarType) \
  EVALUATOR_UNARY_FUNC_HEADER(ReturnType, ScalarType, erfcOp, Erfc)

#define EVALUATOR_ABS_FUNC_IMPL(ReturnType, ScalarType) \
  EVALUATOR_UNARY_FUNC_IMPL(ReturnType, ScalarType, absOp, Abs)
#define EVALUATOR_POW_FUNC_IMPL(ReturnType, ScalarType) \
  EVALUATOR_BINARY_FUNC_IMPL(ReturnType, ScalarType, powOp, Pow)
#define EVALUATOR_EXP_FUNC_IMPL(ReturnType, ScalarType) \
  EVALUATOR_UNARY_FUNC_IMPL(ReturnType, ScalarType, expOp, Exp)
#define EVALUATOR_LOG_FUNC_IMPL(ReturnType, ScalarType) \
  EVALUATOR_UNARY_FUNC_IMPL(ReturnType, ScalarType, logOp, Log)
#define EVALUATOR_SGM_FUNC_IMPL(ReturnType, ScalarType) \
  EVALUATOR_UNARY_FUNC_IMPL(ReturnType, ScalarType, sgmOp, Sgm)
#define EVALUATOR_ERF_FUNC_IMPL(ReturnType, ScalarType) \
  EVALUATOR_UNARY_FUNC_IMPL(ReturnType, ScalarType, erfOp, Erf)
#define EVALUATOR_ERFC_FUNC_IMPL(ReturnType, ScalarType) \
  EVALUATOR_UNARY_FUNC_IMPL(ReturnType, ScalarType, erfcOp, Erfc)

/**
 * \brief Defines all trig functions for {Eval<Scalar>, Scalar} x
 * {Eval<Scalar>, Scalar}
 */
#define EVALUATOR_UNARY_MATH_INTERFACE_HPP(ReturnType, ScalarType) \
  EVALUATOR_ABS_FUNC_HEADER(ReturnType, ScalarType)                \
  EVALUATOR_POW_FUNC_HEADER(ReturnType, ScalarType)                \
  EVALUATOR_EXP_FUNC_HEADER(ReturnType, ScalarType)                \
  EVALUATOR_LOG_FUNC_HEADER(ReturnType, ScalarType)                \
  EVALUATOR_SGM_FUNC_HEADER(ReturnType, ScalarType)                \
  EVALUATOR_ERF_FUNC_HEADER(ReturnType, ScalarType)                \
  EVALUATOR_ERFC_FUNC_HEADER(ReturnType, ScalarType)

#define EVALUATOR_UNARY_MATH_INTERFACE_CPP(ReturnType, ScalarType) \
  EVALUATOR_ABS_FUNC_IMPL(ReturnType, ScalarType)                  \
  EVALUATOR_POW_FUNC_IMPL(ReturnType, ScalarType)                  \
  EVALUATOR_EXP_FUNC_IMPL(ReturnType, ScalarType)                  \
  EVALUATOR_LOG_FUNC_IMPL(ReturnType, ScalarType)                  \
  EVALUATOR_SGM_FUNC_IMPL(ReturnType, ScalarType)                  \
  EVALUATOR_ERF_FUNC_IMPL(ReturnType, ScalarType)                  \
  EVALUATOR_ERFC_FUNC_IMPL(ReturnType, ScalarType)

/**
 * \brief Defines trig operators for {Eval<Scalar>, Scalar} x {Eval<Scalar>,
 * Scalar}
 */
#define EVALUATOR_SIN_FUNC_HEADER(ReturnType, ScalarType) \
  EVALUATOR_UNARY_FUNC_HEADER(ReturnType, ScalarType, sinOp, Sin)
#define EVALUATOR_COS_FUNC_HEADER(ReturnType, ScalarType) \
  EVALUATOR_UNARY_FUNC_HEADER(ReturnType, ScalarType, cosOp, Cos)
#define EVALUATOR_TAN_FUNC_HEADER(ReturnType, ScalarType) \
  EVALUATOR_UNARY_FUNC_HEADER(ReturnType, ScalarType, tanOp, Tan)
#define EVALUATOR_ASIN_FUNC_HEADER(ReturnType, ScalarType) \
  EVALUATOR_UNARY_FUNC_HEADER(ReturnType, ScalarType, asinOp, ASin)
#define EVALUATOR_ACOS_FUNC_HEADER(ReturnType, ScalarType) \
  EVALUATOR_UNARY_FUNC_HEADER(ReturnType, ScalarType, acosOp, ACos)
#define EVALUATOR_ATAN_FUNC_HEADER(ReturnType, ScalarType) \
  EVALUATOR_UNARY_FUNC_HEADER(ReturnType, ScalarType, atanOp, ATan)
#define EVALUATOR_ATAN2_FUNC_HEADER(ReturnType, ScalarType) \
  EVALUATOR_BINARY_FUNC_HEADER(ReturnType, ScalarType, atan2Op, ATan2)

#define EVALUATOR_SIN_FUNC_IMPL(ReturnType, ScalarType) \
  EVALUATOR_UNARY_FUNC_IMPL(ReturnType, ScalarType, sinOp, Sin)
#define EVALUATOR_COS_FUNC_IMPL(ReturnType, ScalarType) \
  EVALUATOR_UNARY_FUNC_IMPL(ReturnType, ScalarType, cosOp, Cos)
#define EVALUATOR_TAN_FUNC_IMPL(ReturnType, ScalarType) \
  EVALUATOR_UNARY_FUNC_IMPL(ReturnType, ScalarType, tanOp, Tan)
#define EVALUATOR_ASIN_FUNC_IMPL(ReturnType, ScalarType) \
  EVALUATOR_UNARY_FUNC_IMPL(ReturnType, ScalarType, asinOp, ASin)
#define EVALUATOR_ACOS_FUNC_IMPL(ReturnType, ScalarType) \
  EVALUATOR_UNARY_FUNC_IMPL(ReturnType, ScalarType, acosOp, ACos)
#define EVALUATOR_ATAN_FUNC_IMPL(ReturnType, ScalarType) \
  EVALUATOR_UNARY_FUNC_IMPL(ReturnType, ScalarType, atanOp, ATan)
#define EVALUATOR_ATAN2_FUNC_IMPL(ReturnType, ScalarType) \
  EVALUATOR_BINARY_FUNC_IMPL(ReturnType, ScalarType, atan2Op, ATan2)

/**
 * \brief Defines all trig functions for {Eval<Scalar>, Scalar} x
 * {Eval<Scalar>, Scalar}
 */
#define EVALUATOR_TRIG_INTERFACE_HPP(ReturnType, ScalarType) \
  EVALUATOR_SIN_FUNC_HEADER(ReturnType, ScalarType)          \
  EVALUATOR_COS_FUNC_HEADER(ReturnType, ScalarType)          \
  EVALUATOR_TAN_FUNC_HEADER(ReturnType, ScalarType)          \
  EVALUATOR_ASIN_FUNC_HEADER(ReturnType, ScalarType)         \
  EVALUATOR_ACOS_FUNC_HEADER(ReturnType, ScalarType)         \
  EVALUATOR_ATAN_FUNC_HEADER(ReturnType, ScalarType)         \
  EVALUATOR_ATAN2_FUNC_HEADER(ReturnType, ScalarType)

#define EVALUATOR_TRIG_INTERFACE_CPP(ReturnType, ScalarType) \
  EVALUATOR_SIN_FUNC_IMPL(ReturnType, ScalarType)            \
  EVALUATOR_COS_FUNC_IMPL(ReturnType, ScalarType)            \
  EVALUATOR_TAN_FUNC_IMPL(ReturnType, ScalarType)            \
  EVALUATOR_ASIN_FUNC_IMPL(ReturnType, ScalarType)           \
  EVALUATOR_ACOS_FUNC_IMPL(ReturnType, ScalarType)           \
  EVALUATOR_ATAN_FUNC_IMPL(ReturnType, ScalarType)           \
  EVALUATOR_ATAN2_FUNC_IMPL(ReturnType, ScalarType)

/**
 * \brief Defines hyperbolic trig operators for {Eval<Scalar>, Scalar} x
 * {Eval<Scalar>, Scalar}
 */
#define EVALUATOR_SINH_FUNC_HEADER(ReturnType, ScalarType) \
  EVALUATOR_UNARY_FUNC_HEADER(ReturnType, ScalarType, sinhOp, Sinh)
#define EVALUATOR_COSH_FUNC_HEADER(ReturnType, ScalarType) \
  EVALUATOR_UNARY_FUNC_HEADER(ReturnType, ScalarType, coshOp, Cosh)
#define EVALUATOR_TANH_FUNC_HEADER(ReturnType, ScalarType) \
  EVALUATOR_UNARY_FUNC_HEADER(ReturnType, ScalarType, tanhOp, Tanh)
#define EVALUATOR_ASINH_FUNC_HEADER(ReturnType, ScalarType) \
  EVALUATOR_UNARY_FUNC_HEADER(ReturnType, ScalarType, asinhOp, ASinh)
#define EVALUATOR_ACOSH_FUNC_HEADER(ReturnType, ScalarType) \
  EVALUATOR_UNARY_FUNC_HEADER(ReturnType, ScalarType, acoshOp, ACosh)
#define EVALUATOR_ATANH_FUNC_HEADER(ReturnType, ScalarType) \
  EVALUATOR_UNARY_FUNC_HEADER(ReturnType, ScalarType, atanhOp, ATanh)

#define EVALUATOR_SINH_FUNC_IMPL(ReturnType, ScalarType) \
  EVALUATOR_UNARY_FUNC_IMPL(ReturnType, ScalarType, sinhOp, Sinh)
#define EVALUATOR_COSH_FUNC_IMPL(ReturnType, ScalarType) \
  EVALUATOR_UNARY_FUNC_IMPL(ReturnType, ScalarType, coshOp, Cosh)
#define EVALUATOR_TANH_FUNC_IMPL(ReturnType, ScalarType) \
  EVALUATOR_UNARY_FUNC_IMPL(ReturnType, ScalarType, tanhOp, Tanh)
#define EVALUATOR_ASINH_FUNC_IMPL(ReturnType, ScalarType) \
  EVALUATOR_UNARY_FUNC_IMPL(ReturnType, ScalarType, asinhOp, ASinh)
#define EVALUATOR_ACOSH_FUNC_IMPL(ReturnType, ScalarType) \
  EVALUATOR_UNARY_FUNC_IMPL(ReturnType, ScalarType, acoshOp, ACosh)
#define EVALUATOR_ATANH_FUNC_IMPL(ReturnType, ScalarType) \
  EVALUATOR_UNARY_FUNC_IMPL(ReturnType, ScalarType, atanhOp, ATanh)

/**
 * \brief Defines all hyperbolic trig functions for {Eval<Scalar>, Scalar} x
 * {Eval<Scalar>, Scalar}
 */
#define EVALUATOR_HTRIG_INTERFACE_HPP(ReturnType, ScalarType) \
  EVALUATOR_SINH_FUNC_HEADER(ReturnType, ScalarType)          \
  EVALUATOR_COSH_FUNC_HEADER(ReturnType, ScalarType)          \
  EVALUATOR_TANH_FUNC_HEADER(ReturnType, ScalarType)          \
  EVALUATOR_ASINH_FUNC_HEADER(ReturnType, ScalarType)         \
  EVALUATOR_ACOSH_FUNC_HEADER(ReturnType, ScalarType)         \
  EVALUATOR_ATANH_FUNC_HEADER(ReturnType, ScalarType)

#define EVALUATOR_HTRIG_INTERFACE_CPP(ReturnType, ScalarType) \
  EVALUATOR_SINH_FUNC_IMPL(ReturnType, ScalarType)            \
  EVALUATOR_COSH_FUNC_IMPL(ReturnType, ScalarType)            \
  EVALUATOR_TANH_FUNC_IMPL(ReturnType, ScalarType)            \
  EVALUATOR_ASINH_FUNC_IMPL(ReturnType, ScalarType)           \
  EVALUATOR_ACOSH_FUNC_IMPL(ReturnType, ScalarType)           \
  EVALUATOR_ATANH_FUNC_IMPL(ReturnType, ScalarType)

/**
 * \brief Defines functions that only work on scalars for {Eval<Scalar>,
 * Scalar} x {Eval<Scalar>, Scalar}
 */
#define EVALUATOR_SCALAR_MATH_INTERFACE_HPP(ReturnType, ScalarType) \
  EVALUATOR_UNARY_MATH_INTERFACE_HPP(ReturnType, ScalarType)        \
  EVALUATOR_TRIG_INTERFACE_HPP(ReturnType, ScalarType)              \
  EVALUATOR_HTRIG_INTERFACE_HPP(ReturnType, ScalarType)

#define EVALUATOR_SCALAR_MATH_INTERFACE_CPP(ReturnType, ScalarType) \
  EVALUATOR_UNARY_MATH_INTERFACE_CPP(ReturnType, ScalarType)        \
  EVALUATOR_TRIG_INTERFACE_CPP(ReturnType, ScalarType)              \
  EVALUATOR_HTRIG_INTERFACE_CPP(ReturnType, ScalarType)

/**
 * \brief Defines individual comparison operators for {Eval<Scalar>, Scalar} x
 * {Eval<Scalar>, Scalar}
 */
#define EVALUATOR_EQ_FUNC_HEADER(ScalarType) \
  EVALUATOR_BINARY_FUNC_HEADER(bool, ScalarType, eqOp, Equal)
#define EVALUATOR_NE_FUNC_HEADER(ScalarType) \
  EVALUATOR_BINARY_FUNC_HEADER(bool, ScalarType, neOp, NEqual)
#define EVALUATOR_LT_FUNC_HEADER(ScalarType) \
  EVALUATOR_BINARY_FUNC_HEADER(bool, ScalarType, ltOp, Less)
#define EVALUATOR_GT_FUNC_HEADER(ScalarType) \
  EVALUATOR_BINARY_FUNC_HEADER(bool, ScalarType, gtOp, Greater)
#define EVALUATOR_LE_FUNC_HEADER(ScalarType) \
  EVALUATOR_BINARY_FUNC_HEADER(bool, ScalarType, leOp, LEqual)
#define EVALUATOR_GE_FUNC_HEADER(ScalarType) \
  EVALUATOR_BINARY_FUNC_HEADER(bool, ScalarType, geOp, GEqual)

#define EVALUATOR_EQ_FUNC_IMPL(ScalarType) \
  EVALUATOR_BINARY_FUNC_IMPL(bool, ScalarType, eqOp, Equal)
#define EVALUATOR_NE_FUNC_IMPL(ScalarType) \
  EVALUATOR_BINARY_FUNC_IMPL(bool, ScalarType, neOp, NEqual)
#define EVALUATOR_LT_FUNC_IMPL(ScalarType) \
  EVALUATOR_BINARY_FUNC_IMPL(bool, ScalarType, ltOp, Less)
#define EVALUATOR_GT_FUNC_IMPL(ScalarType) \
  EVALUATOR_BINARY_FUNC_IMPL(bool, ScalarType, gtOp, Greater)
#define EVALUATOR_LE_FUNC_IMPL(ScalarType) \
  EVALUATOR_BINARY_FUNC_IMPL(bool, ScalarType, leOp, LEqual)
#define EVALUATOR_GE_FUNC_IMPL(ScalarType) \
  EVALUATOR_BINARY_FUNC_IMPL(bool, ScalarType, geOp, GEqual)

/**
 * \brief Defines all comparison operators for Eval<Scalar>, except for == and
 * !=, which are reserved
 */
#define EVALUATOR_COMPARISON_INTERFACE_HPP(ScalarType) \
  EVALUATOR_EQ_FUNC_HEADER(ScalarType)                 \
  EVALUATOR_NE_FUNC_HEADER(ScalarType)                 \
  EVALUATOR_LT_FUNC_HEADER(ScalarType)                 \
  EVALUATOR_GT_FUNC_HEADER(ScalarType)                 \
  EVALUATOR_LE_FUNC_HEADER(ScalarType)                 \
  EVALUATOR_GE_FUNC_HEADER(ScalarType)

#define EVALUATOR_COMPARISON_INTERFACE_CPP(ScalarType) \
  EVALUATOR_EQ_FUNC_IMPL(ScalarType)                   \
  EVALUATOR_NE_FUNC_IMPL(ScalarType)                   \
  EVALUATOR_LT_FUNC_IMPL(ScalarType)                   \
  EVALUATOR_GT_FUNC_IMPL(ScalarType)                   \
  EVALUATOR_LE_FUNC_IMPL(ScalarType)                   \
  EVALUATOR_GE_FUNC_IMPL(ScalarType)

/** \brief Defines all operators that make sense for a scalar number type */
#define EVALUATOR_SCALAR_INTERFACE_HPP(ReturnType, ScalarType) \
  EVALUATOR_BASIC_MATH_INTERFACE_HPP(ReturnType, ScalarType)   \
  EVALUATOR_SCALAR_MATH_INTERFACE_HPP(ReturnType, ScalarType)  \
  EVALUATOR_COMPARISON_INTERFACE_HPP(ScalarType)

#define EVALUATOR_SCALAR_INTERFACE_CPP(ReturnType, ScalarType) \
  EVALUATOR_BASIC_MATH_INTERFACE_CPP(ReturnType, ScalarType)   \
  EVALUATOR_SCALAR_MATH_INTERFACE_CPP(ReturnType, ScalarType)  \
  EVALUATOR_COMPARISON_INTERFACE_CPP(ScalarType)

/**
 * \brief Defines a binary function for combinations of {Eval<Scalar>, Scalar}
 * x {Eval<Scalar>, Scalar}
 */
#define EVALUATOR_BINARY_FUNC_HEADER(ReturnType, ScalarType, Functor, OpName) \
  typename eval::EvalBase<ReturnType>::Ptr OpName(                            \
      const typename eval::EvalBase<ScalarType>::Ptr &lhs,                    \
      const typename eval::EvalBase<ScalarType>::Ptr &rhs);                   \
  typename eval::EvalBase<ReturnType>::Ptr OpName(                            \
      const typename eval::EvalBase<ScalarType>::Ptr &lhs,                    \
      const ScalarType &rhs);                                                 \
  typename eval::EvalBase<ReturnType>::Ptr OpName(                            \
      const ScalarType &lhs,                                                  \
      const typename eval::EvalBase<ScalarType>::Ptr &rhs);

/**
 * \brief Defines a binary function for combinations of {Eval<Scalar>, Scalar}
 * x {Eval<Scalar>, Scalar}
 */
#define EVALUATOR_BINARY_FUNC_IMPL(ReturnType, ScalarType, Functor, OpName)  \
  typename eval::EvalBase<ReturnType>::Ptr OpName(                           \
      const typename eval::EvalBase<ScalarType>::Ptr &lhs,                   \
      const typename eval::EvalBase<ScalarType>::Ptr &rhs) {                 \
    return eval::BinaryOp<eval::Functor<ScalarType>, ReturnType,             \
                          ScalarType>::MakeShared(lhs, rhs);                 \
  }                                                                          \
                                                                             \
  typename eval::EvalBase<ReturnType>::Ptr OpName(                           \
      const typename eval::EvalBase<ScalarType>::Ptr &lhs,                   \
      const ScalarType &rhs) {                                               \
    return OpName(lhs, eval::Base<ScalarType>::Const::MakeShared(rhs, rhs)); \
  }                                                                          \
                                                                             \
  typename eval::EvalBase<ReturnType>::Ptr OpName(                           \
      const ScalarType &lhs,                                                 \
      const typename eval::EvalBase<ScalarType>::Ptr &rhs) {                 \
    return OpName(eval::Base<ScalarType>::Const::MakeShared(lhs, lhs), rhs); \
  }

/** \brief Defines a unary function Eval<Scalar> */
#define EVALUATOR_UNARY_FUNC_HEADER(ReturnType, ScalarType, Functor, OpName) \
  typename eval::EvalBase<ReturnType>::Ptr OpName(                           \
      const typename EvalBase<ScalarType>::Ptr &lhs);

/** \brief Defines a unary function Eval<Scalar> */
#define EVALUATOR_UNARY_FUNC_IMPL(ReturnType, ScalarType, Functor, OpName) \
  typename eval::EvalBase<ReturnType>::Ptr OpName(                         \
      const typename EvalBase<ScalarType>::Ptr &lhs) {                     \
    return eval::UnaryOp<eval::Functor<ScalarType>, ReturnType,            \
                         ScalarType>::MakeShared(lhs);                     \
  }

}  // namespace eval
}  // namespace pose_graph
}  // namespace vtr

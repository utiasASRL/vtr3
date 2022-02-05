// Copyright 2021, Autonomous Space Robotics Lab (ASRL)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * \file evaluator_ops.hpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "vtr_pose_graph/evaluator_base/evaluator_base.hpp"
#include "vtr_pose_graph/evaluator_base/functors.hpp"

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
class BinaryOp : public BaseEval<RVAL> {
 public:
  PTR_TYPEDEFS(BinaryOp);

  using Base = BaseEval<RVAL>;
  using InBase = BaseEval<IVAL>;

  BinaryOp(const typename InBase::Ptr &eval1, const typename InBase::Ptr &eval2)
      : eval1_(eval1), eval2_(eval2), func_(F()) {}

 protected:
  RVAL computeEdge(const EdgeId &e) override {
    return func_(eval1_->operator[](e), eval2_->operator[](e));
  }

  RVAL computeVertex(const VertexId &v) override {
    return func_(eval1_->operator[](v), eval2_->operator[](v));
  }

 private:
  const typename BaseEval<IVAL>::Ptr eval1_;
  const typename BaseEval<IVAL>::Ptr eval2_;
  const F func_;
};

/** \brief Evaluator for unary operations: {~,-,sgm,sin,cos,tan}a */
template <typename F, class RVAL, class IVAL>
class UnaryOp : public BaseEval<RVAL> {
 public:
  PTR_TYPEDEFS(UnaryOp);

  using Base = BaseEval<RVAL>;
  using InBase = BaseEval<IVAL>;

  UnaryOp(const typename InBase::Ptr &eval) : eval_(eval), func_(F()) {}

 protected:
  RVAL computeEdge(const EdgeId &e) override {
    return func_(eval_->operator[](e));
  }

  RVAL computeVertex(const VertexId &v) override {
    return func_(eval_->operator[](v));
  }

 private:
  /** \brief Underlying evaluator */
  const typename InBase::Ptr eval_;

  /** \brief Funtor containing function to be applied */
  const F func_;
};

/**
 * \brief Defines a binary function for combinations of {Eval<Scalar>, Scalar}
 * x {Eval<Scalar>, Scalar}
 */
#define EVALUATOR_BINARY_FUNC(ReturnType, ScalarType, Functor, OpName)        \
  inline typename BaseEval<ReturnType>::Ptr OpName(                           \
      const typename BaseEval<ScalarType>::Ptr &lhs,                          \
      const typename BaseEval<ScalarType>::Ptr &rhs) {                        \
    return std::make_shared<                                                  \
        BinaryOp<Functor<ScalarType>, ReturnType, ScalarType>>(lhs, rhs);     \
  }                                                                           \
                                                                              \
  inline typename BaseEval<ReturnType>::Ptr OpName(                           \
      const typename BaseEval<ScalarType>::Ptr &lhs, const ScalarType &rhs) { \
    return OpName(lhs, std::make_shared<ConstEval<ScalarType>>(rhs, rhs));    \
  }                                                                           \
                                                                              \
  inline typename BaseEval<ReturnType>::Ptr OpName(                           \
      const ScalarType &lhs, const typename BaseEval<ScalarType>::Ptr &rhs) { \
    return OpName(std::make_shared<ConstEval<ScalarType>>(lhs, lhs), rhs);    \
  }

/** \brief Defines a unary function Eval<Scalar> */
#define EVALUATOR_UNARY_FUNC(ReturnType, ScalarType, Functor, OpName) \
  inline typename BaseEval<ReturnType>::Ptr OpName(                   \
      const typename BaseEval<ScalarType>::Ptr &lhs) {                \
    return std::make_shared<                                          \
        UnaryOp<Functor<ScalarType>, ReturnType, ScalarType>>(lhs);   \
  }

/**
 * \brief Defines individual boolean logic operators for {Eval<Scalar>, Scalar}
 * x {Eval<Scalar>, Scalar}
 */
#define EVALUATOR_AND_FUNC(ReturnType, ScalarType) \
  EVALUATOR_BINARY_FUNC(ReturnType, ScalarType, andOp, And)
#define EVALUATOR_OR_FUNC(ReturnType, ScalarType) \
  EVALUATOR_BINARY_FUNC(ReturnType, ScalarType, orOp, Or)
#define EVALUATOR_XOR_FUNC(ReturnType, ScalarType) \
  EVALUATOR_BINARY_FUNC(ReturnType, ScalarType, xorOp, Xor)
#define EVALUATOR_NOT_FUNC(ReturnType, ScalarType) \
  EVALUATOR_UNARY_FUNC(ReturnType, ScalarType, notOp, Not)

/**
 * \brief Defines all boolean logic operators for {Eval<Scalar>, Scalar} x
 * {Eval<Scalar>, Scalar}
 */
#define EVALUATOR_BOOLEAN_INTERFACE(ReturnType, ScalarType) \
  EVALUATOR_AND_FUNC(ReturnType, ScalarType)                \
  EVALUATOR_OR_FUNC(ReturnType, ScalarType)                 \
  EVALUATOR_XOR_FUNC(ReturnType, ScalarType)                \
  EVALUATOR_NOT_FUNC(ReturnType, ScalarType)

/**
 * \brief Defines individual BEDMAS operators for {Eval<Scalar>, Scalar} x
 * {Eval<Scalar>, Scalar}
 */
#define EVALUATOR_ADD_FUNC(ReturnType, ScalarType) \
  EVALUATOR_BINARY_FUNC(ReturnType, ScalarType, addOp, Add)
#define EVALUATOR_SUB_FUNC(ReturnType, ScalarType) \
  EVALUATOR_BINARY_FUNC(ReturnType, ScalarType, subOp, Sub)
#define EVALUATOR_MUL_FUNC(ReturnType, ScalarType) \
  EVALUATOR_BINARY_FUNC(ReturnType, ScalarType, mulOp, Mul)
#define EVALUATOR_DIV_FUNC(ReturnType, ScalarType) \
  EVALUATOR_BINARY_FUNC(ReturnType, ScalarType, divOp, Div)
#define EVALUATOR_NEG_FUNC(ReturnType, ScalarType) \
  EVALUATOR_UNARY_FUNC(ReturnType, ScalarType, negOp, Neg)

/**
 * \brief Defines basic math operators for {Eval<Scalar>, Scalar} x {+,-,/,*} x
 * {Eval<Scalar>, Scalar}
 */
#define EVALUATOR_BASIC_MATH_INTERFACE(ReturnType, ScalarType) \
  EVALUATOR_ADD_FUNC(ReturnType, ScalarType)                   \
  EVALUATOR_SUB_FUNC(ReturnType, ScalarType)                   \
  EVALUATOR_MUL_FUNC(ReturnType, ScalarType)                   \
  EVALUATOR_DIV_FUNC(ReturnType, ScalarType)                   \
  EVALUATOR_NEG_FUNC(ReturnType, ScalarType)

/**
 * \brief Defines unary math functions for {Eval<Scalar>, Scalar} x
 * {Eval<Scalar>, Scalar}
 */
#define EVALUATOR_ABS_FUNC(ReturnType, ScalarType) \
  EVALUATOR_UNARY_FUNC(ReturnType, ScalarType, absOp, Abs)
#define EVALUATOR_POW_FUNC(ReturnType, ScalarType) \
  EVALUATOR_BINARY_FUNC(ReturnType, ScalarType, powOp, Pow)
#define EVALUATOR_EXP_FUNC(ReturnType, ScalarType) \
  EVALUATOR_UNARY_FUNC(ReturnType, ScalarType, expOp, Exp)
#define EVALUATOR_LOG_FUNC(ReturnType, ScalarType) \
  EVALUATOR_UNARY_FUNC(ReturnType, ScalarType, logOp, Log)
#define EVALUATOR_SGM_FUNC(ReturnType, ScalarType) \
  EVALUATOR_UNARY_FUNC(ReturnType, ScalarType, sgmOp, Sgm)
#define EVALUATOR_ERF_FUNC(ReturnType, ScalarType) \
  EVALUATOR_UNARY_FUNC(ReturnType, ScalarType, erfOp, Erf)
#define EVALUATOR_ERFC_FUNC(ReturnType, ScalarType) \
  EVALUATOR_UNARY_FUNC(ReturnType, ScalarType, erfcOp, Erfc)

/**
 * \brief Defines all trig functions for {Eval<Scalar>, Scalar} x
 * {Eval<Scalar>, Scalar}
 */
#define EVALUATOR_UNARY_MATH_INTERFACE(ReturnType, ScalarType) \
  EVALUATOR_ABS_FUNC(ReturnType, ScalarType)                   \
  EVALUATOR_POW_FUNC(ReturnType, ScalarType)                   \
  EVALUATOR_EXP_FUNC(ReturnType, ScalarType)                   \
  EVALUATOR_LOG_FUNC(ReturnType, ScalarType)                   \
  EVALUATOR_SGM_FUNC(ReturnType, ScalarType)                   \
  EVALUATOR_ERF_FUNC(ReturnType, ScalarType)                   \
  EVALUATOR_ERFC_FUNC(ReturnType, ScalarType)

/**
 * \brief Defines trig operators for {Eval<Scalar>, Scalar} x {Eval<Scalar>,
 * Scalar}
 */
#define EVALUATOR_SIN_FUNC(ReturnType, ScalarType) \
  EVALUATOR_UNARY_FUNC(ReturnType, ScalarType, sinOp, Sin)
#define EVALUATOR_COS_FUNC(ReturnType, ScalarType) \
  EVALUATOR_UNARY_FUNC(ReturnType, ScalarType, cosOp, Cos)
#define EVALUATOR_TAN_FUNC(ReturnType, ScalarType) \
  EVALUATOR_UNARY_FUNC(ReturnType, ScalarType, tanOp, Tan)
#define EVALUATOR_ASIN_FUNC(ReturnType, ScalarType) \
  EVALUATOR_UNARY_FUNC(ReturnType, ScalarType, asinOp, ASin)
#define EVALUATOR_ACOS_FUNC(ReturnType, ScalarType) \
  EVALUATOR_UNARY_FUNC(ReturnType, ScalarType, acosOp, ACos)
#define EVALUATOR_ATAN_FUNC(ReturnType, ScalarType) \
  EVALUATOR_UNARY_FUNC(ReturnType, ScalarType, atanOp, ATan)
#define EVALUATOR_ATAN2_FUNC(ReturnType, ScalarType) \
  EVALUATOR_BINARY_FUNC(ReturnType, ScalarType, atan2Op, ATan2)

/**
 * \brief Defines all trig functions for {Eval<Scalar>, Scalar} x
 * {Eval<Scalar>, Scalar}
 */
#define EVALUATOR_TRIG_INTERFACE(ReturnType, ScalarType) \
  EVALUATOR_SIN_FUNC(ReturnType, ScalarType)             \
  EVALUATOR_COS_FUNC(ReturnType, ScalarType)             \
  EVALUATOR_TAN_FUNC(ReturnType, ScalarType)             \
  EVALUATOR_ASIN_FUNC(ReturnType, ScalarType)            \
  EVALUATOR_ACOS_FUNC(ReturnType, ScalarType)            \
  EVALUATOR_ATAN_FUNC(ReturnType, ScalarType)            \
  EVALUATOR_ATAN2_FUNC(ReturnType, ScalarType)

/**
 * \brief Defines hyperbolic trig operators for {Eval<Scalar>, Scalar} x
 * {Eval<Scalar>, Scalar}
 */
#define EVALUATOR_SINH_FUNC(ReturnType, ScalarType) \
  EVALUATOR_UNARY_FUNC(ReturnType, ScalarType, sinhOp, Sinh)
#define EVALUATOR_COSH_FUNC(ReturnType, ScalarType) \
  EVALUATOR_UNARY_FUNC(ReturnType, ScalarType, coshOp, Cosh)
#define EVALUATOR_TANH_FUNC(ReturnType, ScalarType) \
  EVALUATOR_UNARY_FUNC(ReturnType, ScalarType, tanhOp, Tanh)
#define EVALUATOR_ASINH_FUNC(ReturnType, ScalarType) \
  EVALUATOR_UNARY_FUNC(ReturnType, ScalarType, asinhOp, ASinh)
#define EVALUATOR_ACOSH_FUNC(ReturnType, ScalarType) \
  EVALUATOR_UNARY_FUNC(ReturnType, ScalarType, acoshOp, ACosh)
#define EVALUATOR_ATANH_FUNC(ReturnType, ScalarType) \
  EVALUATOR_UNARY_FUNC(ReturnType, ScalarType, atanhOp, ATanh)

/**
 * \brief Defines all hyperbolic trig functions for {Eval<Scalar>, Scalar} x
 * {Eval<Scalar>, Scalar}
 */
#define EVALUATOR_HTRIG_INTERFACE(ReturnType, ScalarType) \
  EVALUATOR_SINH_FUNC(ReturnType, ScalarType)             \
  EVALUATOR_COSH_FUNC(ReturnType, ScalarType)             \
  EVALUATOR_TANH_FUNC(ReturnType, ScalarType)             \
  EVALUATOR_ASINH_FUNC(ReturnType, ScalarType)            \
  EVALUATOR_ACOSH_FUNC(ReturnType, ScalarType)            \
  EVALUATOR_ATANH_FUNC(ReturnType, ScalarType)

/**
 * \brief Defines functions that only work on scalars for {Eval<Scalar>,
 * Scalar} x {Eval<Scalar>, Scalar}
 */
#define EVALUATOR_SCALAR_MATH_INTERFACE(ReturnType, ScalarType) \
  EVALUATOR_UNARY_MATH_INTERFACE(ReturnType, ScalarType)        \
  EVALUATOR_TRIG_INTERFACE(ReturnType, ScalarType)              \
  EVALUATOR_HTRIG_INTERFACE(ReturnType, ScalarType)

/**
 * \brief Defines individual comparison operators for {Eval<Scalar>, Scalar} x
 * {Eval<Scalar>, Scalar}
 */
#define EVALUATOR_EQ_FUNC(ScalarType) \
  EVALUATOR_BINARY_FUNC(bool, ScalarType, eqOp, Equal)
#define EVALUATOR_NE_FUNC(ScalarType) \
  EVALUATOR_BINARY_FUNC(bool, ScalarType, neOp, NEqual)
#define EVALUATOR_LT_FUNC(ScalarType) \
  EVALUATOR_BINARY_FUNC(bool, ScalarType, ltOp, Less)
#define EVALUATOR_GT_FUNC(ScalarType) \
  EVALUATOR_BINARY_FUNC(bool, ScalarType, gtOp, Greater)
#define EVALUATOR_LE_FUNC(ScalarType) \
  EVALUATOR_BINARY_FUNC(bool, ScalarType, leOp, LEqual)
#define EVALUATOR_GE_FUNC(ScalarType) \
  EVALUATOR_BINARY_FUNC(bool, ScalarType, geOp, GEqual)

/**
 * \brief Defines all comparison operators for Eval<Scalar>, except for == and
 * !=, which are reserved
 */
#define EVALUATOR_COMPARISON_INTERFACE(ScalarType) \
  EVALUATOR_EQ_FUNC(ScalarType)                    \
  EVALUATOR_NE_FUNC(ScalarType)                    \
  EVALUATOR_LT_FUNC(ScalarType)                    \
  EVALUATOR_GT_FUNC(ScalarType)                    \
  EVALUATOR_LE_FUNC(ScalarType)                    \
  EVALUATOR_GE_FUNC(ScalarType)

/** \brief Defines all operators that make sense for a scalar number type */
#define EVALUATOR_SCALAR_INTERFACE(ReturnType, ScalarType) \
  EVALUATOR_BASIC_MATH_INTERFACE(ReturnType, ScalarType)   \
  EVALUATOR_SCALAR_MATH_INTERFACE(ReturnType, ScalarType)  \
  EVALUATOR_COMPARISON_INTERFACE(ScalarType)

}  // namespace eval
}  // namespace pose_graph
}  // namespace vtr

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
 * \file functors.hpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <cmath>

namespace vtr {
namespace pose_graph {
namespace eval {

///////////////////////////
/// Floating Point Math ///
///////////////////////////

/** \brief Addition, as a functor */
template <typename T>
struct addOp {
  inline T operator()(const T &a, const T &b) const { return a + b; }
};

/** \brief Subtraction, as a functor */
template <typename T>
struct subOp {
  inline T operator()(const T &a, const T &b) const { return a - b; }
};

/** \brief Multiplication, as a functor */
template <typename T>
struct mulOp {
  inline double operator()(const T &a, const T &b) const { return a * b; }
};

/** \brief Division, as a functor */
template <typename T>
struct divOp {
  inline double operator()(const T &a, const T &b) const { return a / b; }
};

/** \brief Unary negation, as a functor */
template <typename T>
struct negOp {
  inline double operator()(const T &a) const { return -a; }
};

/** \brief Absolute value, as a functor */
template <typename T>
struct absOp {
  inline double operator()(const T &a) const { return std::abs(a); }
};

/** \brief Power, as a functor */
template <typename T>
struct powOp {
  inline double operator()(const T &a, const T &b) const {
    return std::pow(a, b);
  }
};

/** \brief Exponentiation, as a functor */
template <typename T>
struct expOp {
  inline double operator()(const T &a) const { return std::exp(a); }
};

/** \brief Logarithm, as a functor */
template <typename T>
struct logOp {
  inline double operator()(const T &a) const { return std::log(a); }
};

/** \brief Sigmoid, as a functor */
template <typename T>
struct sgmOp {
  inline double operator()(const T &a) const {
    return T(1) / (T(1) + std::exp(-a));
  }
};

/** \brief Erf(x), as a functor */
template <typename T>
struct erfOp {
  inline double operator()(const T &a) const { return std::erf(a); }
};

/** \brief Erfc(x), as a functor */
template <typename T>
struct erfcOp {
  inline double operator()(const T &a) const { return std::erfc(a); }
};

//////////////////////////////
/// Trigonometry Functors  ///
//////////////////////////////

/** \brief sin(x), as a functor */
template <typename T>
struct sinOp {
  inline double operator()(const T &a) const { return std::sin(a); }
};

/** \brief asin(x), as a functor */
template <typename T>
struct asinOp {
  inline double operator()(const T &a) const { return std::asin(a); }
};

/** \brief cos(x), as a functor */
template <typename T>
struct cosOp {
  inline double operator()(const T &a) const { return std::cos(a); }
};

/** \brief acos(x), as a functor */
template <typename T>
struct acosOp {
  inline double operator()(const T &a) const { return std::acos(a); }
};

/** \brief tan(x), as a functor */
template <typename T>
struct tanOp {
  inline double operator()(const T &a) const { return std::tan(a); }
};

/** \brief atan(x), as a functor */
template <typename T>
struct atanOp {
  inline double operator()(const T &a) const { return std::atan(a); }
};

/** \brief atan2(x,y), as a functor */
template <typename T>
struct atan2Op {
  inline double operator()(const T &a, const T &b) const {
    return std::atan2(a, b);
  }
};

/////////////////////////////////////////
/// Hyperbolic Trigonometry Functors  ///
/////////////////////////////////////////

/** \brief sinh(x), as a functor */
template <typename T>
struct sinhOp {
  inline double operator()(const T &a) const { return std::sinh(a); }
};

/** \brief asinh(x), as a functor */
template <typename T>
struct asinhOp {
  inline double operator()(const T &a) const { return std::asinh(a); }
};

/** \brief cosh(x), as a functor */
template <typename T>
struct coshOp {
  inline double operator()(const T &a) const { return std::cosh(a); }
};

/** \brief acosh(x), as a functor */
template <typename T>
struct acoshOp {
  inline double operator()(const T &a) const { return std::acosh(a); }
};

/** \brief tanh(x), as a functor */
template <typename T>
struct tanhOp {
  inline double operator()(const T &a) const { return std::tanh(a); }
};

/** \brief atanh(x), as a functor */
template <typename T>
struct atanhOp {
  inline double operator()(const T &a) const { return std::atanh(a); }
};

//////////////////////////////
/// Boolean Logic Functors ///
//////////////////////////////

/** \brief AND, as a functor */
template <typename T>
struct andOp {
  inline bool operator()(const T &a, const T &b) const { return a && b; }
};

/** \brief OR, as a functor */
template <typename T>
struct orOp {
  inline bool operator()(const T &a, const T &b) const { return a || b; }
};

/** \brief XOR (!=), as a functor */
template <typename T>
struct xorOp {
  inline bool operator()(const T &a, const T &b) const { return a != b; }
};

/** \brief NOT, as a functor */
template <typename T>
struct notOp {
  inline bool operator()(const T &a) const { return !a; }
};

//////////////////
/// Comparison ///
//////////////////

/** \brief ==, as a functor */
template <typename T>
struct eqOp {
  inline bool operator()(const T &a, const T &b) const { return a == b; }
};

/** \brief !=, as a functor */
template <typename T>
struct neOp {
  inline bool operator()(const T &a, const T &b) const { return a != b; }
};

/** \brief <, as a functor */
template <typename T>
struct ltOp {
  inline bool operator()(const T &a, const T &b) const { return a < b; }
};

/** \brief >, as a functor */
template <typename T>
struct gtOp {
  inline bool operator()(const T &a, const T &b) const { return a > b; }
};

/** \brief <=, as a functor */
template <typename T>
struct leOp {
  inline bool operator()(const T &a, const T &b) const { return a <= b; }
};

/** \brief >=, as a functor */
template <typename T>
struct geOp {
  inline bool operator()(const T &a, const T &b) const { return a >= b; }
};

}  // namespace eval
}  // namespace pose_graph
}  // namespace vtr

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
 * \file noise_model_generator.hpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "steam.hpp"

namespace vtr {
namespace pose_graph {

// Base class for noise model generators
template <class T, size_t DIM>
class NoiseModelGeneratorBase {
 public:
  using MatType = Eigen::Matrix<double, DIM, DIM>;
  using ModelType = steam::StaticNoiseModel<DIM>;
  using ModelPtr = typename steam::BaseNoiseModel<DIM>::Ptr;

  NoiseModelGeneratorBase(const MatType& cov = MatType::Identity())
      : default_model_(new ModelType(cov)) {}

  NoiseModelGeneratorBase(const ModelPtr& default_model)
      : default_model_(default_model) {}

  // Return the default model
  virtual ModelPtr operator()(const T&) const = 0;

 protected:
  ModelPtr default_model_;
};

// Template class that generates noise models for a type, or returns a default
// model. The generic template always returns the default model
template <class T, size_t DIM>
class NoiseModelGenerator : public NoiseModelGeneratorBase<T, DIM> {
 public:
  using Base = NoiseModelGeneratorBase<T, DIM>;
  using ModelPtr = typename Base::ModelPtr;
  using Base::default_model_;
  using Base::NoiseModelGeneratorBase;

  // Return the default model
  ModelPtr operator()(const T&) const override { return default_model_; }
};

// Template specialization for transofrms that have a covariance
template <>
class NoiseModelGenerator<lgmath::se3::TransformationWithCovariance, 6>
    : public NoiseModelGeneratorBase<lgmath::se3::TransformationWithCovariance,
                                     6> {
 public:
  using Base =
      NoiseModelGeneratorBase<lgmath::se3::TransformationWithCovariance, 6>;
  using ModelPtr = typename Base::ModelPtr;
  using Base::default_model_;
  using Base::NoiseModelGeneratorBase;

  // Make a new model and return it
  ModelPtr operator()(
      const lgmath::se3::TransformationWithCovariance& T) const override {
    if (T.covarianceSet()) {
      auto cov = T.cov();
      if (cov.norm() > 0) return steam::StaticNoiseModel<6>::MakeShared(cov);
    }
    return default_model_;
  }
};

}  // namespace pose_graph
}  // namespace vtr

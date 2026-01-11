//
// Created by rgrandia on 14.08.20.
//

#pragma once

#include "tbai_mpc/quadruped_mpc/core/SwitchedModel.h"

namespace tbai::mpc::quadruped {

/**
 * This abstract class defines the interface for a signed distance field.
 */
class SignedDistanceField {
 public:
  SignedDistanceField() = default;
  virtual ~SignedDistanceField() = default;
  SignedDistanceField(const SignedDistanceField&) = delete;
  SignedDistanceField& operator=(const SignedDistanceField&) = delete;

  virtual SignedDistanceField* clone() const = 0;
  virtual scalar_t value(const vector3_t& position) const = 0;
  virtual Eigen::Vector3d derivative(const vector3_t& position) const = 0;
  virtual std::pair<scalar_t, vector3_t> valueAndDerivative(const vector3_t& position) const = 0;
};

}  // namespace tbai::mpc::quadruped
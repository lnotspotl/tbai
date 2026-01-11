#pragma once

#include "tbai_mpc/quadruped_arm_mpc/core/SwitchedModel.h"
#include "tbai_mpc/quadruped_arm_mpc/terrain/SignedDistanceField.h"
#include "tbai_mpc/quadruped_arm_mpc/terrain/TerrainModel.h"

namespace tbai::mpc::quadruped_arm {

/**
 * Implements a flat terrain signed distance field
 */
class PlanarSignedDistanceField : public SignedDistanceField {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit PlanarSignedDistanceField(TerrainPlane terrainPlane);

  ~PlanarSignedDistanceField() override = default;
  PlanarSignedDistanceField* clone() const override { return new PlanarSignedDistanceField(*this); };

  scalar_t value(const vector3_t& position) const override;

  vector3_t derivative(const vector3_t& position) const override;

  std::pair<scalar_t, vector3_t> valueAndDerivative(const vector3_t& position) const override;

 protected:
  PlanarSignedDistanceField(const PlanarSignedDistanceField& other);

 private:
  TerrainPlane terrainPlane_;
};

}  // namespace tbai::mpc::quadruped_arm

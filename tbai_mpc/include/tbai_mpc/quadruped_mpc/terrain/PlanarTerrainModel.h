//
// Created by rgrandia on 29.04.20.
//

#pragma once

#include "tbai_mpc/quadruped_mpc/core/SwitchedModel.h"
#include "tbai_mpc/quadruped_mpc/terrain/PlanarSignedDistanceField.h"
#include "tbai_mpc/quadruped_mpc/terrain/TerrainModel.h"

namespace switched_model {

/**
 * This class models the terrain as a single infinite plane.
 */
class PlanarTerrainModel : public TerrainModel {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  explicit PlanarTerrainModel(TerrainPlane terrainPlane);
  ~PlanarTerrainModel() override = default;

  TerrainPlane getLocalTerrainAtPositionInWorldAlongGravity(const vector3_t& positionInWorld,
                                                            std::function<scalar_t(const vector3_t&)> penaltyFunction) const override;

  vector3_t getHighestObstacleAlongLine(const vector3_t& position1InWorld, const vector3_t& position2InWorld) const override;

  std::vector<vector2_t> getHeightProfileAlongLine(const vector3_t& position1InWorld, const vector3_t& position2InWorld) const override;

  const SignedDistanceField* getSignedDistanceField() const override { return &sdf_; }

 private:
  TerrainPlane terrainPlane_;
  PlanarSignedDistanceField sdf_;
};

}  // namespace switched_model

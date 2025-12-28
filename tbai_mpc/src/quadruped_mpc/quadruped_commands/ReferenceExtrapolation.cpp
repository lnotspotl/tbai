//
// Created by rgrandia on 02.03.20.
//

#include "tbai_mpc/quadruped_mpc/quadruped_commands/ReferenceExtrapolation.h"

#include <tbai_mpc/quadruped_mpc/quadruped_commands/TerrainAdaptation.h>
#include <tbai_mpc/quadruped_mpc/core/Rotations.h>

namespace switched_model {

namespace {
void addVelocitiesFromFiniteDifference(BaseReferenceTrajectory& baseRef) {
  auto N = baseRef.time.size();
  if (N <= 1) {
    return;
  }

  baseRef.linearVelocityInWorld.clear();
  baseRef.angularVelocityInWorld.clear();
  baseRef.linearVelocityInWorld.reserve(N);
  baseRef.angularVelocityInWorld.reserve(N);

  for (int k = 0; (k + 1) < baseRef.time.size(); ++k) {
    auto dt = baseRef.time[k + 1] - baseRef.time[k];
    baseRef.angularVelocityInWorld.push_back(rotationErrorInWorldEulerXYZ(baseRef.eulerXyz[k + 1], baseRef.eulerXyz[k]) / dt);
    baseRef.linearVelocityInWorld.push_back((baseRef.positionInWorld[k + 1] - baseRef.positionInWorld[k]) / dt);
  }

  auto dt = baseRef.time[N - 1] - baseRef.time[N - 2];
  baseRef.angularVelocityInWorld.push_back(rotationErrorInWorldEulerXYZ(baseRef.eulerXyz[N - 1], baseRef.eulerXyz[N - 2]) / dt);
  baseRef.linearVelocityInWorld.push_back((baseRef.positionInWorld[N - 1] - baseRef.positionInWorld[N - 2]) / dt);
}
}  // namespace

Eigen::Vector2d velocityCommand2dInWorld(double headingVelocity, double lateralVelocity, double yaw) {
  Eigen::Vector2d commandInBase{headingVelocity, lateralVelocity};
  rotateInPlace2d(commandInBase, yaw);
  return commandInBase;
}

Base2dReferenceTrajectory generate2DExtrapolatedBaseReference(const BaseReferenceHorizon& horizon, const BaseReferenceState& initialState,
                                                              const BaseReferenceCommand& command) {
  const double dt = horizon.dt;

  Base2dReferenceTrajectory baseRef;
  baseRef.time.reserve(horizon.N);
  baseRef.yaw.reserve(horizon.N);
  baseRef.positionInWorld.reserve(horizon.N);

  baseRef.time.push_back(initialState.t0);

  // Project position and orientation to horizontal plane
  baseRef.positionInWorld.emplace_back(initialState.positionInWorld.x(), initialState.positionInWorld.y());
  baseRef.yaw.emplace_back(alignDesiredOrientationToTerrain(initialState.eulerXyz, TerrainPlane()).z());

  for (int k = 1; k < horizon.N; ++k) {
    baseRef.time.push_back(baseRef.time.back() + dt);

    // Express command in world
    const auto commandedLinearVelocityInWorld =
        velocityCommand2dInWorld(command.headingVelocity, command.lateralVelocity, baseRef.yaw.back() + 0.5 * dt * command.yawRate);

    // Advance position
    baseRef.positionInWorld.push_back(baseRef.positionInWorld.back() + dt * commandedLinearVelocityInWorld);

    // Advance orientation
    baseRef.yaw.push_back(baseRef.yaw.back() + dt * command.yawRate);
  }

  return baseRef;
}

BaseReferenceTrajectory generateExtrapolatedBaseReference(const BaseReferenceHorizon& horizon, const BaseReferenceState& initialState,
                                                          const BaseReferenceCommand& command, const TerrainPlane& projectedHeadingFrame) {
  auto reference2d = generate2DExtrapolatedBaseReference(horizon, initialState, command);

  BaseReferenceTrajectory baseRef;
  baseRef.time = std::move(reference2d.time);
  baseRef.eulerXyz.reserve(horizon.N);
  baseRef.positionInWorld.reserve(horizon.N);

  // Adapt poses
  for (int k = 0; k < horizon.N; ++k) {
    baseRef.positionInWorld.push_back(
        adaptDesiredPositionHeightToTerrain(reference2d.positionInWorld[k], projectedHeadingFrame, command.baseHeight));
    baseRef.eulerXyz.emplace_back(alignDesiredOrientationToTerrain({0.0, 0.0, reference2d.yaw[k]}, projectedHeadingFrame));
  }

  addVelocitiesFromFiniteDifference(baseRef);
  return baseRef;
}

}  // namespace switched_model

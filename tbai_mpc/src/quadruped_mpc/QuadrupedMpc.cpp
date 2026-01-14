//
// Created by rgrandia on 06.07.21.
//

#include "tbai_mpc/quadruped_mpc/QuadrupedMpc.h"

#include <ocs2_ddp/GaussNewtonDDP_MPC.h>
#include <ocs2_sqp/SqpMpc.h>

namespace tbai::mpc::quadruped {

std::unique_ptr<ocs2::MPC_BASE> getDdpMpc(const QuadrupedInterface &quadrupedInterface,
                                          const ocs2::mpc::Settings &mpcSettings,
                                          const ocs2::ddp::Settings &ddpSettings) {
    std::unique_ptr<ocs2::MPC_BASE> mpcPtr(new ocs2::GaussNewtonDDP_MPC(
        mpcSettings, ddpSettings, quadrupedInterface.getRollout(), quadrupedInterface.getOptimalControlProblem(),
        quadrupedInterface.getInitializer()));
    mpcPtr->getSolverPtr()->setReferenceManager(quadrupedInterface.getReferenceManagerPtr());
    mpcPtr->getSolverPtr()->setSynchronizedModules(quadrupedInterface.getSynchronizedModules());
    return mpcPtr;
}

std::unique_ptr<ocs2::MPC_BASE> getSqpMpc(const QuadrupedInterface &quadrupedInterface,
                                          const ocs2::mpc::Settings &mpcSettings,
                                          const ocs2::sqp::Settings &sqpSettings) {
    std::unique_ptr<ocs2::MPC_BASE> mpcPtr(new ocs2::SqpMpc(
        mpcSettings, sqpSettings, quadrupedInterface.getOptimalControlProblem(), quadrupedInterface.getInitializer()));
    mpcPtr->getSolverPtr()->setReferenceManager(quadrupedInterface.getReferenceManagerPtr());
    mpcPtr->getSolverPtr()->setSynchronizedModules(quadrupedInterface.getSynchronizedModules());
    return mpcPtr;
}

}  // namespace tbai::mpc::quadruped

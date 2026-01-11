#include "tbai_mpc/quadruped_arm_mpc/initialization/ComKinoInitializer.h"

#include "tbai_mpc/quadruped_arm_mpc/core/SwitchedModel.h"

namespace tbai::mpc::quadruped_arm {

ComKinoInitializer::ComKinoInitializer(const com_model_t &comModel,
                                       const SwitchedModelModeScheduleManager &modeScheduleManager)
    : comModelPtr_(comModel.clone()), modeScheduleManagerPtr_(&modeScheduleManager) {}

ComKinoInitializer::ComKinoInitializer(const ComKinoInitializer &rhs)
    : ocs2::Initializer(rhs),
      comModelPtr_(rhs.comModelPtr_->clone()),
      modeScheduleManagerPtr_(rhs.modeScheduleManagerPtr_) {}

ComKinoInitializer *ComKinoInitializer::clone() const {
    return new ComKinoInitializer(*this);
}

void ComKinoInitializer::compute(scalar_t time, const vector_t &state, scalar_t nextTime, vector_t &input,
                                 vector_t &nextState) {
    const comkino_state_t comkinoState = state;
    const auto basePose = getBasePose(comkinoState);
    const auto contactFlags = modeScheduleManagerPtr_->getContactFlags(time);

    // Initial guess
    input = weightCompensatingInputs(*comModelPtr_, contactFlags, getOrientation(basePose));

    nextState = state;
}

}  // namespace tbai::mpc::quadruped_arm

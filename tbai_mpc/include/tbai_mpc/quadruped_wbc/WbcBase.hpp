#pragma once

// clang-format off
#include <pinocchio/fwd.hpp>
// clang-format on

#include <memory>
#include <string>
#include <vector>

#include <tbai_core/control/Controllers.hpp>
#include <tbai_mpc/quadruped_mpc/quadruped_models/QuadrupedCom.h>
#include <tbai_mpc/quadruped_mpc/quadruped_models/QuadrupedKinematics.h>
#include <tbai_mpc/wbc/Task.hpp>

namespace tbai {
namespace mpc {
namespace quadruped {

class WbcBase {
   public:
    WbcBase(const std::string &configFile, const std::string &urdfString,
            const tbai::mpc::quadruped::ComModelBase<scalar_t> &comModel,
            const tbai::mpc::quadruped::KinematicsModelBase<scalar_t> &kinematics, const std::string &configPrefix);

    virtual ~WbcBase() = default;

    virtual std::vector<tbai::MotorCommand> getMotorCommands(scalar_t currentTime, const vector_t &currentState,
                                                             const vector_t &currentInput, const size_t currentMode,
                                                             const vector_t &desiredState, const vector_t &desiredInput,
                                                             const size_t desiredMode,
                                                             const vector_t &desiredJointAcceleration,
                                                             bool &isStable) = 0;

   protected:
    Task createDynamicsTask();
    Task createContactForceTask();
    Task createStanceFootNoMotionTask();
    Task createTorqueLimitTask();

    Task createBaseAccelerationTask(const vector_t &stateCurrent, const vector_t &stateDesired,
                                    const vector_t &inputDesired, const vector_t &desiredJointAcceleration);
    Task createSwingFootAccelerationTask(const vector_t &stateCurrent, const vector_t &inputCurrent,
                                         const vector_t &stateDesired, const vector_t &inputDesired);
    Task createContactForceMinimizationTask(const vector_t &inputDesired);

    void updateMeasuredState(const vector_t &stateCurrent, const vector_t &inputCurrent);
    void updateKinematicsAndDynamicsCurrent();
    void updateDesiredState(const vector_t &stateDesired, const vector_t &inputDesired);
    void updateKinematicsAndDynamicsDesired(const vector_t &stateDesired, const vector_t &inputDesired);
    void updateContactFlags(const size_t modeCurrent, const size_t modeDesired);

    /* Contact jacobians - stacked on top of each other */
    matrix_t Jcontact_;

    /* Contact jacobians time derivative - stacked on top of each other */
    matrix_t dJcontactdt_;

    /* Measured generalized positions */
    vector_t qMeasured_;

    /* Measured generalized velocities */
    vector_t vMeasured_;

    /* Measured rotation matrix from base to world*/
    matrix_t rMeasured_;

    /* Desired rotation matrix from base to world*/
    matrix_t rDesired_;

    /* Number of decision variables */
    size_t nDecisionVariables_;

    /* Number of generalized coordinates */
    size_t nGeneralizedCoordinates_;

    /* Measured pinocchio interface */
    ocs2::PinocchioInterface pinocchioInterfaceMeasured_;

    /* Foot names*/
    std::vector<std::string> footNames_;

    /* Friction cone matrix*/
    matrix_t muMatrix_;

    /* swing kp and kd*/
    scalar_t swingKp_;
    scalar_t swingKd_;

    /* base kp and kd */
    scalar_t baseKp_;
    scalar_t baseKd_;

    /* euler kp and kd*/
    scalar_t eulerKp_;
    scalar_t eulerKd_;

    /* torque limit */
    scalar_t torqueLimit_;

    /* Desired contact flags */
    tbai::mpc::quadruped::contact_flag_t contactFlags_;
    size_t nContacts_;

    /* whether stance should be enforeced as a constraint  or as a cost (false is more robust (e.g. slips)) */
    bool stanceAsConstraint_ = false;

   private:
    void loadSettings(const std::string &configFile, const std::string &configPrefix);
    void generateFrictionConeMatrix(const scalar_t mu);

    std::unique_ptr<tbai::mpc::quadruped::ComModelBase<scalar_t>> comModelPtr_;
    std::unique_ptr<tbai::mpc::quadruped::KinematicsModelBase<scalar_t>> kinematicsPtr_;
};

}  // namespace quadruped
}  // namespace mpc
}  // namespace tbai
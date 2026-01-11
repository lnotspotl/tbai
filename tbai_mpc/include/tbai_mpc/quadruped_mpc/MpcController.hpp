#pragma once

#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <ocs2_mpc/MPC_MRT_Interface.h>
#include <ocs2_mpc/MRT_BASE.h>
#include <ocs2_mpc/SystemObservation.h>
#include <tbai_core/Logging.hpp>
#include <tbai_core/Types.hpp>
#include <tbai_core/control/Controllers.hpp>
#include <tbai_core/control/Subscribers.hpp>
#include <tbai_mpc/quadruped_mpc/QuadrupedInterface.h>
#include <tbai_mpc/quadruped_mpc/quadruped_reference/ReferenceTrajectoryGenerator.hpp>
#include <tbai_mpc/quadruped_wbc/WbcBase.hpp>
#include <tbai_reference/ReferenceVelocityGenerator.hpp>

namespace tbai::mpc::quadruped {

/**
 * ROS-independent MPC controller base class.
 * Uses MPC_MRT_Interface by default (direct MPC, no ROS communication).
 * Override createMrtInterface() for ROS-based MPC communication.
 */
class MpcController : public tbai::Controller {
   public:
    /**
     * Constructor
     * @param stateSubscriberPtr: State subscriber for getting robot state
     * @param velocityGeneratorPtr: Reference velocity generator
     */
    MpcController(const std::string &robotName, const std::shared_ptr<tbai::StateSubscriber> &stateSubscriberPtr,
                  std::shared_ptr<tbai::reference::ReferenceVelocityGenerator> velocityGeneratorPtr,
                  std::function<scalar_t()> getCurrentTimeFunction);

    virtual ~MpcController();

    std::vector<MotorCommand> getMotorCommands(scalar_t currentTime, scalar_t dt) override;

    std::string getName() const override { return "MpcController"; }

    void changeController(const std::string &controllerType, scalar_t currentTime) override;

    bool isSupported(const std::string &controllerType) override;

    void stopController() override { stopReferenceThread(); }

    scalar_t getRate() const override { return 400.0; }

    bool checkStability() const override;

    void waitTillInitialized() override { stateSubscriberPtr_->waitTillInitialized(); }

    void preStep(scalar_t currentTime, scalar_t dt) override { state_ = stateSubscriberPtr_->getLatestState(); }

    void postStep(scalar_t currentTime, scalar_t dt) override {}

    /**
     * Factory method to create the MRT interface.
     * Default implementation creates MPC_MRT_Interface (direct MPC).
     * Override this in ROS version to create MRT_ROS_Interface.
     */
    virtual std::unique_ptr<ocs2::MRT_BASE> createMrtInterface();

    /**
     * Factory method to create the MPC interface.
     * Default implementation creates MPC_BASE (direct MPC).
     * Override this in ROS version to create MPC_ROS_Interface.
     */
    virtual std::unique_ptr<ocs2::MPC_BASE> createMpcInterface();

   protected:
    /**
     * Initializes the controller components.
     * Called from the constructor or derived class.
     * @param urdfString: Robot URDF string
     * @param taskSettingsFile: Path to task settings file
     * @param frameDeclarationFile: Path to frame declaration file
     * @param controllerConfigFile: Path to controller config file
     * @param targetCommandFile: Path to target command config file
     * @param trajdt: Reference trajectory timestep
     * @param trajKnots: Number of reference trajectory knots
     */
    void initialize(const std::string &urdfString, const std::string &taskSettingsFile,
                    const std::string &frameDeclarationFile, const std::string &controllerConfigFile,
                    const std::string &targetCommandFile, scalar_t trajdt = 0.1, size_t trajKnots = 20);

    void resetMpc();
    void setObservation();
    ocs2::SystemObservation generateSystemObservation() const;

    // Reference thread management
    virtual void referenceThreadLoop();
    void startReferenceThread();
    void stopReferenceThread();

    std::shared_ptr<tbai::StateSubscriber> stateSubscriberPtr_;
    std::shared_ptr<tbai::reference::ReferenceVelocityGenerator> velocityGeneratorPtr_;

    std::unique_ptr<tbai::mpc::quadruped::QuadrupedInterface> quadrupedInterfacePtr_;
    std::unique_ptr<tbai::mpc::quadruped::WbcBase> wbcPtr_;
    std::unique_ptr<ocs2::MRT_BASE> mrtPtr_;
    std::shared_ptr<ocs2::MPC_BASE> mpcPtr_;
    std::unique_ptr<reference::ReferenceTrajectoryGenerator> referenceTrajectoryGeneratorPtr_;

    scalar_t initTime_ = 0.0;
    scalar_t tNow_ = 0.0;

    bool mrt_initialized_ = false;

    scalar_t mpcRate_ = 30.0;
    scalar_t timeSinceLastMpcUpdate_ = 1e5;

    bool isStable_ = true;

    State state_;

    std::string robotName_;

    std::function<scalar_t()> getCurrentTimeFunction_;

    std::shared_ptr<spdlog::logger> logger_;

    // Reference thread
    std::atomic<bool> stopReferenceThread_{false};
    std::thread referenceThread_;
    scalar_t referenceThreadRate_ = 5.0;
};

}  // namespace tbai::mpc::quadruped

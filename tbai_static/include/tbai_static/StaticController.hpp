#pragma once

#include <tbai_core/control/Subscribers.hpp>
#include <tbai_core/control/Controllers.hpp>
#include <tbai_core/Logging.hpp>

namespace tbai {
namespace static_ {

class StaticController : public tbai::Controller {
   public:
    StaticController(std::shared_ptr<tbai::StateSubscriber> stateSubscriberPtr);

    std::vector<MotorCommand> getMotorCommands(scalar_t currentTime, scalar_t dt) override;

    bool isSupported(const std::string &controllerType) override;

    void stopController() override {}

    scalar_t getRate() const override;

    bool checkStability() const override { return true; }

    void changeController(const std::string &controllerType, scalar_t currentTime) override;

   protected:
    void loadSettings();

    /** Get command message during interpolation phase */
    std::vector<MotorCommand> getInterpCommandMessage(scalar_t dt);

    /** Get command message when standing */
    std::vector<MotorCommand> getStandCommandMessage();

    /** Get command message when sitting */
    std::vector<MotorCommand> getSitCommandMessage();

    /** Pack desired joint angles into a command message */
    std::vector<MotorCommand> packCommandMessage(const vector_t &jointAngles);

    /** State subscriber */
    std::shared_ptr<tbai::StateSubscriber> stateSubscriberPtr_;

    /** PD constants */
    scalar_t kp_;
    scalar_t kd_;

    /** Stand joint angles */
    vector_t standJointAngles_;

    /** Sit joint angles */
    vector_t sitJointAngles_;

    /** How long should interpolation take */
    scalar_t interpolationTime_;

    /** Interp from and to */
    vector_t interpFrom_;
    vector_t interpTo_;

    /** Interpolation phase: -1 means interpolation has finished */
    scalar_t alpha_;

    /** Rate at which the controller should be running */
    scalar_t rate_;

    /** Joint names */
    std::vector<std::string> jointNames_;

    /** Current controller type */
    std::string currentControllerType_;

    /** Logger */
    std::shared_ptr<spdlog::logger> logger_;

    bool first_;
};

}  // namespace static_
}  // namespace tbai
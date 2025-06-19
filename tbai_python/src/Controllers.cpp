#include <tbai_core/Logging.hpp>
#include <tbai_core/Utils.hpp>
#include <tbai_python/Controllers.hpp>

namespace tbai {
namespace python {

PyCentralController::PyCentralController(std::shared_ptr<tbai::python::MyPyStateSubscriber> stateSubscriberPtr,
                                         std::shared_ptr<tbai::reference::ReferenceVelocityGenerator> refVelGen,
                                         std::shared_ptr<tbai::CommandPublisher> commandPublisherPtr)
    : ::tbai::CentralController() {
    fallbackControllerType_ = "SIT";
    containsFallbackController_ = false;

    stateSubscriberPtr_ = stateSubscriberPtr;
    refVelGen_ = refVelGen;
    commandPublisherPtr_ = commandPublisherPtr;

    // Load init time
    initTime_ = tbai::readInitTime();
}

void PyCentralController::start() {
    // Make sure there is at least one controller
    if (activeController_ == nullptr) {
        TBAI_LOG_FATAL("No active controller found!");
        return;
    }

    // Check if a fallback controller is available
    containsFallbackController_ = checkForFallbackController();
    if (!containsFallbackController_) {
        TBAI_LOG_WARN("No fallback controller found, no stability checks will be performed!");
        return;
    }

    // Wait for initial state message
    stateSubscriberPtr_->waitTillInitialized();

    // Get rate
    PyRate rate = PyRate(activeController_->getRate());

    scalar_t lastTime_ = getCurrentTime();

    while (true) {
        auto t1 = std::chrono::high_resolution_clock::now();

        stateSubscriberPtr_->updateState();

        if (containsFallbackController_ && !activeController_->checkStability()) {
            switchToFallbackController();
        }

        scalar_t currentTime_ = getCurrentTime();
        scalar_t dt = lastTime_ - currentTime_;

        auto commands = activeController_->getMotorCommands(currentTime_, dt);
        commandPublisherPtr_->publish(commands);

        activeController_->visualize();

        auto t2 = std::chrono::high_resolution_clock::now();
        rate.sleep();
        auto t3 = std::chrono::high_resolution_clock::now();

        auto duration1 = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
        auto duration2 = std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2).count();

        TBAI_LOG_INFO_THROTTLE(2.0, "Time taken: " + std::to_string(duration1) + "ms, " + std::to_string(duration2) + "ms");

        auto sleepPercentage = (duration2 / duration1) * 100.0;
        TBAI_LOG_INFO_THROTTLE(2.0, "Sleep percentage: " + std::to_string(sleepPercentage) + "%");

        lastTime_ = currentTime_;
    }
}

bool PyCentralController::checkForFallbackController() {
    for (const auto &controller : controllers_) {
        if (controller->isSupported(fallbackControllerType_)) {
            return true;
        }
    }
    return false;
}

scalar_t PyCentralController::getCurrentTime() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count() - initTime_;
}

void PyCentralController::switchToFallbackController() {
    switchToController(fallbackControllerType_);
}

void PyCentralController::switchToController(const std::string &controllerType) {
    for (auto &controller : controllers_) {
        if (controller->isSupported(controllerType)) {
            if (activeController_ != controller.get()) {
                activeController_->stopController();  // Stop current controller
            }
            activeController_ = controller.get();  // Set new active controller
            activeController_->changeController(controllerType, getCurrentTime());
            TBAI_LOG_INFO("Controller changed to " + controllerType);
            return;
        }
    }
    TBAI_LOG_WARN("Controller " + controllerType + " not supported");
}

}  // namespace python
}  // namespace tbai
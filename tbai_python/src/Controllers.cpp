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

        rate.sleep();

        // Get current time
        scalar_t currentTime_ = getCurrentTime();
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

}  // namespace python
}  // namespace tbai
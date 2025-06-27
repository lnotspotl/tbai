#pragma once

#include <string>

#include <tbai_core/Logging.hpp>
#include <tbai_core/Throws.hpp>
#include <tbai_core/Types.hpp>
#include <tbai_core/control/Controllers.hpp>
#include <tbai_core/control/Publishers.hpp>
#include <tbai_core/control/Subscribers.hpp>

namespace tbai {

template <typename RATE, typename TIME>
class CentralController {
   public:
    // Constructor
    CentralController(std::shared_ptr<CommandPublisher> commandPublisherPtr,
                      std::shared_ptr<ChangeControllerSubscriber> changeControllerSubscriberPtr)
        : loopRate_(1) {
        // Initialize logger
        logger_ = tbai::getLogger("central_controller");

        commandPublisherPtr_ = commandPublisherPtr;
        changeControllerSubscriberPtr_ = changeControllerSubscriberPtr;

        // Set changeControllerSubscriber callback function
        TBAI_LOG_INFO(logger_, "Setting changeControllerSubscriber callback function");
        changeControllerSubscriberPtr_->setCallbackFunction(
            std::bind(&CentralController<RATE, TIME>::changeController, this, std::placeholders::_1));

        initTime_ = tbai::readInitTime();
        fallbackControllerType_ = "SIT";
        TBAI_LOG_INFO(logger_, "Init time: {} | Fallback controller type: {}", initTime_, fallbackControllerType_);
        controllers_ = std::vector<std::unique_ptr<Controller>>();
    }

    // Add a new controller
    void addController(std::unique_ptr<Controller> controllerPtr, bool makeActive = false) {
        controllers_.push_back(std::move(controllerPtr));
        if (makeActive || controllers_.size() == 1) {
            activeController_ = controllers_.back().get();
        }
    }

    inline scalar_t getCurrentTime() const { return TIME::rightNow() - initTime_; }

    inline void initialize() {
        TBAI_LOG_INFO(logger_, "Starting central controller loop");

        if (activeController_ == nullptr) {
            TBAI_THROW("No active controller found!");
        }

        TBAI_LOG_INFO(logger_, "Active controller found!");

        // Check if fallback controller is available
        containsFallbackController_ = checkForFallbackController();
        if (!containsFallbackController_) {
            TBAI_LOG_WARN(logger_, "Fallback controller not found, no stability checks will be performed!");
        } else {
            TBAI_LOG_INFO(logger_, "Fallback controller: {}", fallbackControllerType_);
        }

        TBAI_LOG_INFO(logger_, "Waiting for controllers to initialize");
        for (auto &controller : controllers_) {
            controller->waitTillInitialized();
        }
        TBAI_LOG_INFO(logger_, "All controllers initialized");
    }

    inline void step(scalar_t currentTime, scalar_t dt) {
        // Trigger all change controller callbacks
        changeControllerSubscriberPtr_->triggerCallbacks();

        // Trigger all callbacks this controller wants to trigger
        activeController_->preStep(currentTime, dt);

        // Check stability and switch to fallback controller if necessary
        if (containsFallbackController_ && !activeController_->checkStability()) {
            TBAI_LOG_WARN(logger_, "Stability check failed, switching to fallback controller: {}",
                          fallbackControllerType_);
            switchToFallbackController();
            activeController_->preStep(currentTime, dt);
        }

        // Step controller
        auto commands = activeController_->getMotorCommands(currentTime, dt);
        commandPublisherPtr_->publish(std::move(commands));

        // Allow controller to visualize stuff and what not
        activeController_->postStep(currentTime, dt);
    }

    inline scalar_t getRate() const { return activeController_->getRate(); }

    void start() {
        // Prepare the central controller
        initialize();

        loopRate_ = RATE(activeController_->getRate());
        TBAI_LOG_INFO(logger_, "Active controller's rate is {} Hz", activeController_->getRate());
        TBAI_LOG_INFO(logger_, "Starting! Current time: {}", getCurrentTime());

        scalar_t lastTime = getCurrentTime();
        while (activeController_->ok() && running_) {
            // Keep track of time for stats
            auto t1 = std::chrono::high_resolution_clock::now();

            // Compute current time and time since last call
            scalar_t currentTime = getCurrentTime();
            scalar_t dt = currentTime - lastTime;

            step(currentTime, dt);

            lastTime = currentTime;

            auto t2 = std::chrono::high_resolution_clock::now();
            loopRate_.sleep();
            auto t3 = std::chrono::high_resolution_clock::now();

            auto duration1 = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();
            auto duration2 = std::chrono::duration_cast<std::chrono::microseconds>(t3 - t2).count();
            auto sleepTimePercentage = 100.0 * duration2 / (duration1 + duration2);

            TBAI_LOG_INFO_THROTTLE(logger_, 10.0,
                                   "Loop duration: {} us, Sleep duration: {} us, Sleep time percentage: {} %",
                                   duration1, duration2, sleepTimePercentage);
        }

        TBAI_LOG_INFO(logger_, "Central controller loop stopped.");
    }

    void startThread() {
        controllerThread_ = std::thread([this]() { start(); });
    }

    void stopThread() {
        running_ = false;
        controllerThread_.join();
    }

   protected:
    bool checkForFallbackController() {
        for (const auto &controller : controllers_) {
            if (controller->isSupported(fallbackControllerType_)) {
                return true;
            }
        }
        return false;
    }

    inline void switchToFallbackController() { changeController(fallbackControllerType_); }

    void changeController(const std::string &controllerType) {
        for (auto &controller : controllers_) {
            if (controller->isSupported(controllerType)) {
                if (activeController_ != controller.get()) {
                    activeController_->stopController();  // Stop current controller
                }
                activeController_ = controller.get();  // Set new active controller
                activeController_->changeController(controllerType, getCurrentTime());
                loopRate_ = RATE(activeController_->getRate());
                TBAI_LOG_INFO(logger_, "Controller changed to {}, loop rate: {} Hz", controllerType,
                              activeController_->getRate());
                return;
            }
        }

        TBAI_LOG_WARN(logger_, "Invalid controller type: {} | Controller was not changed!", controllerType);
    }

    // All the controllers
    std::vector<std::unique_ptr<Controller>> controllers_;
    Controller *activeController_ = nullptr;

    std::shared_ptr<CommandPublisher> commandPublisherPtr_;
    std::shared_ptr<StateSubscriber> stateSubscriberPtr_;
    std::shared_ptr<ChangeControllerSubscriber> changeControllerSubscriberPtr_;

    scalar_t initTime_;
    bool containsFallbackController_ = false;

    RATE loopRate_;

    std::string fallbackControllerType_;
    std::shared_ptr<spdlog::logger> logger_;

    std::thread controllerThread_;
    bool running_ = true;
};

}  // namespace tbai
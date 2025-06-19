#pragma once

#include <tbai_bob/BobController.hpp>
#include <tbai_core/Types.hpp>
#include <tbai_core/control/CentralController.hpp>
#include <tbai_core/control/StateSubscriber.hpp>
#include <tbai_reference/ReferenceVelocityGenerator.hpp>

namespace tbai {
namespace python {

using ::tbai::vector_t;
using ::tbai::scalar_t;
using ::tbai::matrix_t;

class PyBlindBobController : public ::tbai::BobController {
   public:
    PyBlindBobController(const std::shared_ptr<tbai::StateSubscriber> &stateSubscriberPtr,
                         std::shared_ptr<tbai::reference::ReferenceVelocityGenerator> refVelGen)
        : tbai::BobController(stateSubscriberPtr, refVelGen) {};

    virtual void visualize() override {
        // Do nothing
    };

    virtual void atPositions(const matrix_t &positions) override {
        // Do nothing
    };

    virtual void changeController(const std::string &controllerType, scalar_t currentTime) override {
        // Do nothing - this controller doesn't support changing to other controllers
    };
};

class MyPyStateSubscriber : public ::tbai::StateSubscriber {
   public:
    using ::tbai::StateSubscriber::StateSubscriber;
    virtual void updateState() = 0;
    virtual void waitTillInitialized() = 0;
    virtual const ::tbai::vector_t &getLatestRbdState() = 0;
    virtual const ::tbai::scalar_t getLatestRbdStamp() = 0;
    virtual const std::vector<bool> getContactFlags() = 0;
};

class PyCentralController : public ::tbai::CentralController {
   public:
    PyCentralController(std::shared_ptr<tbai::python::MyPyStateSubscriber> stateSubscriberPtr,
                        std::shared_ptr<tbai::reference::ReferenceVelocityGenerator> refVelGen,
                        std::shared_ptr<tbai::CommandPublisher> commandPublisherPtr);

    virtual void start() override;

    void addBobController() {
        auto bobController = std::unique_ptr<tbai::BobController>(new PyBlindBobController(stateSubscriberPtr_, refVelGen_));
        addController(std::move(bobController));
    }

   private:
    std::string fallbackControllerType_;
    bool containsFallbackController_;

    bool checkForFallbackController();

    scalar_t initTime_;

    std::shared_ptr<tbai::python::MyPyStateSubscriber> stateSubscriberPtr_;
    std::shared_ptr<tbai::reference::ReferenceVelocityGenerator> refVelGen_;
    std::shared_ptr<tbai::CommandPublisher> commandPublisherPtr_;

    scalar_t getCurrentTime();

    void switchToFallbackController();

    void switchToController(const std::string &controllerType);
};

class PyRate {
   public:
    PyRate(scalar_t rate) : rate_(rate), last_time_(std::chrono::steady_clock::now()) {}

    void sleep() {
        auto expected_next = last_time_ + std::chrono::duration<double>(1.0 / rate_);
        auto now = std::chrono::steady_clock::now();

        if (expected_next > now) {
            std::this_thread::sleep_until(expected_next);
        }

        last_time_ = std::chrono::steady_clock::now();
    }

   private:
    scalar_t rate_;
    std::chrono::steady_clock::time_point last_time_;
};

}  // namespace python
}  // namespace tbai
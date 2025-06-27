#pragma once

#include <mutex>
#include <vector>

#include <tbai_core/Types.hpp>

namespace tbai {

struct State {
    vector_t x;
    scalar_t timestamp;
    std::vector<bool> contactFlags;
};
class StateSubscriber {
   public:
    virtual ~StateSubscriber() = default;

    virtual void waitTillInitialized() = 0;
    virtual State getLatestState() = 0;
};

class ThreadedStateSubscriber : public StateSubscriber {
   public:
    State getLatestState() override {
        std::lock_guard<std::mutex> lock(stateMutex_);
        return state_;
    }

    virtual void startThreads() = 0;
    virtual void stopThreads() = 0;

   protected:
    State state_;
    std::mutex stateMutex_;
};

class ChangeControllerSubscriber {
   public:
    virtual ~ChangeControllerSubscriber() = default;

    virtual void setCallbackFunction(std::function<void(const std::string &controllerType)> callbackFunction) {
        callbackFunction_ = callbackFunction;
    }

    virtual void triggerCallbacks() = 0;

   protected:
    std::function<void(const std::string &controllerType)> callbackFunction_;
};

}  // namespace tbai
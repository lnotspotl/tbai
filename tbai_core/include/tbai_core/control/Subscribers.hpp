#pragma once

#include <vector>

#include <tbai_core/Types.hpp>

namespace tbai {

class StateSubscriber {
   public:
    virtual ~StateSubscriber() = default;

    virtual void waitTillInitialized() = 0;
    virtual const vector_t &getLatestRbdState() = 0;
    virtual const scalar_t getLatestRbdStamp() = 0;
    virtual const std::vector<bool> getContactFlags() = 0;
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
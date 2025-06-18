#pragma once

#include <vector>

#include <tbai_core/Types.hpp>

namespace tbai {

class StateSubscriber {
   public:
    virtual ~StateSubscriber() = default;

    virtual void waitTillInitialized() const = 0;
    virtual vector_t &getLatestRbdState() const = 0;
    virtual scalar_t getLatestRbdStamp() const = 0;
    virtual std::vector<bool> &getContactFlags() const = 0;
};

}  // namespace tbai
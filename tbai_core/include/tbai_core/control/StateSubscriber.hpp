#pragma once

#include <vector>

#include <tbai_core/Types.hpp>

namespace tbai {

class StateSubscriber {
   public:
    virtual ~StateSubscriber() = default;

    virtual void waitTillInitialized() = 0;
    virtual const vector_t &getLatestRbdState() const = 0;
    virtual const scalar_t getLatestRbdStamp() const = 0;
    virtual const std::vector<bool> &getContactFlags() const = 0;
};

}  // namespace tbai
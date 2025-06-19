#pragma once

#include <chrono>
#include <thread>

#include <tbai_core/Types.hpp>
#include <tbai_core/Utils.hpp>

namespace tbai {

template <typename SCALAR_T>
class SystemRate {
   public:
    SystemRate(SCALAR_T rate) : rate_(rate) {
        timeInterval_ = 1.0 / rate;
        lastTime_ = std::chrono::high_resolution_clock::now();
    }

    SCALAR_T getRate() const { return rate_; }
    void sleep() {
        auto now = std::chrono::high_resolution_clock::now();
        auto expected = lastTime_ + std::chrono::duration<SCALAR_T>(timeInterval_);

        if (now < expected) {
            std::this_thread::sleep_until(expected);
        }

        lastTime_ = std::chrono::high_resolution_clock::now();
    }

   private:
    std::chrono::high_resolution_clock::time_point lastTime_;
    SCALAR_T rate_;
    SCALAR_T timeInterval_;
};

template <typename CLOCK>
struct SystemTime {
    static inline scalar_t rightNow() {
        return convertToScalar(CLOCK::now());
    }
};

}  // namespace tbai
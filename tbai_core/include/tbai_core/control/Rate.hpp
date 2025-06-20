#pragma once

#include <chrono>
#include <thread>

#include <tbai_core/Types.hpp>
#include <tbai_core/Utils.hpp>

namespace tbai {

template <typename SCALAR_T, typename CLOCK_T = std::chrono::high_resolution_clock>
class SystemRate {
   public:
    SystemRate(SCALAR_T rate) : rate_(rate) {
        timeInterval_ = 1.0 / rate;
        lastTime_ = CLOCK_T::now();
    }

    SCALAR_T getRate() const { return rate_; }
    void sleep() {
        auto now = CLOCK_T::now();
        auto expected = lastTime_ + std::chrono::duration<SCALAR_T>(timeInterval_);

        if (now < expected) {
            std::this_thread::sleep_until(expected);
        }

        lastTime_ = CLOCK_T::now();
    }

   private:
    typename CLOCK_T::time_point lastTime_;
    SCALAR_T rate_;
    SCALAR_T timeInterval_;
};

template <typename CLOCK_T = std::chrono::high_resolution_clock>
struct SystemTime {
    static inline scalar_t rightNow() { return convertToScalar(CLOCK_T::now()); }
};

}  // namespace tbai
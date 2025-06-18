#pragma once

#include <tbai_core/Types.hpp>
#include <tbai_reference/ReferenceVelocity.hpp>

namespace tbai {
namespace reference {

class ReferenceVelocityGenerator {
   public:
    /** Destructor*/
    virtual ~ReferenceVelocityGenerator() = default;

    /**
     * @brief Generate reference velocity
     * @param time Current time
     * @param dt Time since the last call
     * @return Reference velocity
     */
    virtual ReferenceVelocity getReferenceVelocity(scalar_t time, scalar_t dt) = 0;
};

}  // namespace reference
}  // namespace tbai
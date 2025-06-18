#pragma once

#include <tbai_core/Types.hpp>

namespace tbai {
namespace reference {

/** Reference velocity */
struct ReferenceVelocity {
    /** Velocity in the x direction, [m/s] */
    scalar_t velocity_x = 0.0;

    /** Velocity in the y direction, [m/s] */
    scalar_t velocity_y = 0.0;

    /** Angular velocity around the z axis, [rad/s] */
    scalar_t yaw_rate = 0.0;
};

}  // namespace reference
}  // namespace tbai
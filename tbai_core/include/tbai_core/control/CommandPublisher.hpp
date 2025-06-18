#pragma once

#include <string>
#include <vector>

#include <tbai_core/Types.hpp>

namespace tbai {

struct MotorCommand {
    scalar_t kp;
    scalar_t desired_position;
    scalar_t kd;
    scalar_t desired_velocity;
    scalar_t torque_ff;
    std::string joint_name;
};

class CommandPublisher {
   public:
    virtual ~CommandPublisher() = default;

    virtual void publish(const std::vector<MotorCommand> &commands) = 0;
};

}  // namespace tbai
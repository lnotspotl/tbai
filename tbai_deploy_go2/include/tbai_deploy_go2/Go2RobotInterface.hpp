#pragma once

#include <tbai_core/control/RobotInterface.hpp>

namespace tbai {

class Go2RobotInterface : public RobotInterface {
   public:
    // virtual methods from RobotInterface
    void publish(std::vector<MotorCommand> commands) override;
    void waitTillInitialized() override;
    State getLatestState() override;
};

}  // namespace tbai
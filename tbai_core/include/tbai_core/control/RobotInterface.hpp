#pragma once

#include <tbai_core/control/Publishers.hpp>
#include <tbai_core/control/Subscribers.hpp>

namespace tbai {

class RobotInterface : public CommandPublisher, public StateSubscriber {};

}  // namespace tbai
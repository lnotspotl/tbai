#include <tbai_core/control/CentralController.hpp>

namespace tbai {

void CentralController::addController(std::unique_ptr<Controller> controller, bool makeActive) {
    controllers_.push_back(std::move(controller));
    if (makeActive) activeController_ = controllers_.back().get();
}

}  // namespace tbai
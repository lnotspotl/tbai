#pragma once

#include <memory>
#include <string>
#include <vector>

#include <tbai_core/Types.hpp>
#include <tbai_core/control/Controller.hpp>

namespace tbai {

class CentralController {
   public:
    CentralController() = default;
    ~CentralController() = default;

    /**
     * @brief Start main control loop
     *
     */
    virtual void start() = 0;

    /**
     * @brief Add controller to central controller
     *
     * @param controller : controller to add
     * @param makeActive : whether to make this controller active
     */
    void addController(std::unique_ptr<Controller> controller, bool makeActive = false);

   protected:
    std::vector<std::unique_ptr<Controller>> controllers_;

    Controller *activeController_ = nullptr;
};

}  // namespace tbai
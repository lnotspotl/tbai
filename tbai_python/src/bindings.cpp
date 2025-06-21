#include <iostream>

#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <tbai_bob/BobController.hpp>
#include <tbai_core/Rotations.hpp>
#include <tbai_core/Types.hpp>
#include <tbai_core/Utils.hpp>
#include <tbai_core/control/CentralController.hpp>
#include <tbai_core/control/Controllers.hpp>
#include <tbai_core/control/Rate.hpp>
#include <tbai_core/control/Subscribers.hpp>
#include <tbai_reference/ReferenceVelocityGenerator.hpp>
#include <tbai_static/StaticController.hpp>

// Python wrappers around virtual classes
namespace tbai {

class PyStateSubscriber : public tbai::StateSubscriber {
   public:
    using StateSubscriber::StateSubscriber;

    void waitTillInitialized() override { PYBIND11_OVERRIDE_PURE(void, tbai::StateSubscriber, waitTillInitialized); }
    const vector_t &getLatestRbdState() override {
        PYBIND11_OVERRIDE_PURE(const vector_t &, tbai::StateSubscriber, getLatestRbdState);
    }
    const scalar_t getLatestRbdStamp() override {
        PYBIND11_OVERRIDE_PURE(const scalar_t, tbai::StateSubscriber, getLatestRbdStamp);
    }
    const std::vector<bool> getContactFlags() override {
        PYBIND11_OVERRIDE_PURE(const std::vector<bool>, tbai::StateSubscriber, getContactFlags);
    }
};

class PyCommandPublisher : public tbai::CommandPublisher {
   public:
    using CommandPublisher::CommandPublisher;

    void publish(const std::vector<MotorCommand> &commands) override {
        PYBIND11_OVERRIDE_PURE(void, tbai::CommandPublisher, publish, commands);
    }
};

class PyChangeControllerSubscriber : public tbai::ChangeControllerSubscriber {
   public:
    using ChangeControllerSubscriber::ChangeControllerSubscriber;

    void setCallbackFunction(std::function<void(const std::string &controllerType)> callbackFunction) override {
        PYBIND11_OVERRIDE(void, tbai::ChangeControllerSubscriber, setCallbackFunction, callbackFunction);
    }
    void triggerCallbacks() override {
        PYBIND11_OVERRIDE_PURE(void, tbai::ChangeControllerSubscriber, triggerCallbacks);
    }
};

class PyStaticController : public tbai::static_::StaticController {
   public:
    using static_::StaticController::StaticController;

    void visualize(scalar_t currentTime, scalar_t dt) override {
        // Do nothing
    }

    bool ok() const override { return true; }

    void triggerCallbacks() override {
        // Do nothing
    }
};

class PyBobController : public tbai::BobController {
   public:
    using BobController::BobController;

    void visualize(scalar_t currentTime, scalar_t dt) override {
        // Do nothing
    }

    void changeController(const std::string &controllerType, scalar_t currentTime) override {
        // Do nothing
    }

    void atPositions(matrix_t &positions) override {
        // Do nothing
    }

    bool ok() const override { return true; }

    void triggerCallbacks() override {
        // Do nothing
    }
};

class PyReferenceVelocityGenerator : public tbai::reference::ReferenceVelocityGenerator {
   public:
    using reference::ReferenceVelocityGenerator::ReferenceVelocityGenerator;

    virtual tbai::reference::ReferenceVelocity getReferenceVelocity(scalar_t time, scalar_t dt) override {
        PYBIND11_OVERRIDE_PURE(tbai::reference::ReferenceVelocity, tbai::reference::ReferenceVelocityGenerator,
                               getReferenceVelocity, time, dt);
    }
};

typedef tbai::CentralController<tbai::SystemRate<scalar_t>, tbai::SystemTime<std::chrono::high_resolution_clock>>
    CentralControllerPython;

class PyCentralController : public CentralControllerPython {
   public:
    using CentralControllerPython::CentralController;
};

}  // namespace tbai

namespace py = pybind11;

PYBIND11_MODULE(tbai_python, m) {
    m.def("write_init_time", py::overload_cast<>(&tbai::writeInitTime));
    m.def("write_init_time", py::overload_cast<const long, const long>(&tbai::writeInitTime));
    m.def("write_init_time", py::overload_cast<const double>(&tbai::writeInitTime));
    m.def("read_init_time", &tbai::readInitTime);

    py::class_<tbai::MotorCommand>(m, "MotorCommand")
        .def_readwrite("kp", &tbai::MotorCommand::kp)
        .def_readwrite("desired_position", &tbai::MotorCommand::desired_position)
        .def_readwrite("kd", &tbai::MotorCommand::kd)
        .def_readwrite("desired_velocity", &tbai::MotorCommand::desired_velocity)
        .def_readwrite("torque_ff", &tbai::MotorCommand::torque_ff)
        .def_readwrite("joint_name", &tbai::MotorCommand::joint_name);

    py::class_<tbai::StateSubscriber, tbai::PyStateSubscriber, std::shared_ptr<tbai::StateSubscriber>>(
        m, "StateSubscriber")
        .def(py::init<>())
        .def("waitTillInitialized", &tbai::StateSubscriber::waitTillInitialized)
        .def("get_latest_rbd_state", &tbai::StateSubscriber::getLatestRbdState)
        .def("get_latest_rbd_stamp", &tbai::StateSubscriber::getLatestRbdStamp)
        .def("get_contact_flags", &tbai::StateSubscriber::getContactFlags);

    py::class_<tbai::CommandPublisher, tbai::PyCommandPublisher, std::shared_ptr<tbai::CommandPublisher>>(
        m, "CommandPublisher")
        .def(py::init<>())
        .def("publish", &tbai::CommandPublisher::publish);

    py::class_<tbai::ChangeControllerSubscriber, tbai::PyChangeControllerSubscriber,
               std::shared_ptr<tbai::ChangeControllerSubscriber>>(m, "ChangeControllerSubscriber")
        .def(py::init<>())
        .def("setCallbackFunction", &tbai::ChangeControllerSubscriber::setCallbackFunction)
        .def("triggerCallbacks", &tbai::ChangeControllerSubscriber::triggerCallbacks);

    py::class_<tbai::reference::ReferenceVelocity>(m, "ReferenceVelocity")
        .def(py::init<>())
        .def_readwrite("velocity_x", &tbai::reference::ReferenceVelocity::velocity_x)
        .def_readwrite("velocity_y", &tbai::reference::ReferenceVelocity::velocity_y)
        .def_readwrite("yaw_rate", &tbai::reference::ReferenceVelocity::yaw_rate);

    py::class_<tbai::reference::ReferenceVelocityGenerator, tbai::PyReferenceVelocityGenerator,
               std::shared_ptr<tbai::reference::ReferenceVelocityGenerator>>(m, "ReferenceVelocityGenerator")
        .def(py::init<>())
        .def("getReferenceVelocity", &tbai::reference::ReferenceVelocityGenerator::getReferenceVelocity);

    py::class_<tbai::CentralControllerPython, std::shared_ptr<tbai::CentralControllerPython>>(m, "CentralController")
        .def_static("create",
                    [](std::shared_ptr<tbai::StateSubscriber> stateSubscriberPtr,
                       std::shared_ptr<tbai::CommandPublisher> commandPublisherPtr,
                       std::shared_ptr<tbai::ChangeControllerSubscriber> changeControllerSubscriberPtr) {
                        return std::make_shared<tbai::CentralControllerPython>(stateSubscriberPtr, commandPublisherPtr,
                                                                               changeControllerSubscriberPtr);
                    })
        .def("start", &tbai::CentralControllerPython::start)
        .def("startThread", &tbai::CentralControllerPython::startThread)
        .def("stopThread", &tbai::CentralControllerPython::stopThread)
        .def("add_bob_controller",
             [](tbai::CentralControllerPython *self, std::shared_ptr<tbai::StateSubscriber> stateSubscriberPtr,
                std::shared_ptr<tbai::reference::ReferenceVelocityGenerator> refVelGen) {
                 self->addController(std::make_unique<tbai::PyBobController>(stateSubscriberPtr, refVelGen));
             })
        .def("add_static_controller",
             [](tbai::CentralControllerPython *self, std::shared_ptr<tbai::StateSubscriber> stateSubscriberPtr) {
                 self->addController(std::make_unique<tbai::PyStaticController>(stateSubscriberPtr));
             });

    // Bind rotation helper functions
    py::module rotations_module = m.def_submodule("rotations");
    rotations_module.def(
        "rpy2quat",
        [](const tbai::vector3_t &rpy) {
            tbai::quaternion_t q = tbai::rpy2quat(rpy);
            return tbai::vector4_t(q.x(), q.y(), q.z(), q.w());
        },
        "Convert roll-pitch-yaw euler angles to quaternion (returns xyzw vector)");
    rotations_module.def(
        "quat2mat",
        [](const tbai::vector4_t &q) {
            tbai::matrix3_t mat = tbai::quat2mat(tbai::quaternion_t(q[3], q[0], q[1], q[2]));
            return tbai::matrix3_t(mat);
        },
        "Convert quaternion to rotation matrix, expects xyzw vector");
    rotations_module.def("mat2rpy", &tbai::mat2rpy, "Convert rotation matrix to roll-pitch-yaw euler angles");
    rotations_module.def("mat2ocs2rpy", &tbai::mat2oc2rpy,
                         "Convert rotation matrix to ocs2-style roll-pitch-yaw euler angles");
    rotations_module.def(
        "ocs2rpy2quat",
        [](const tbai::vector3_t &rpy) {
            tbai::quaternion_t q = tbai::ocs2rpy2quat(rpy);
            return tbai::vector4_t(q.x(), q.y(), q.z(), q.w());
        },
        "Convert ocs2-style roll-pitch-yaw angles to quaternion (returns xyzw vector)");
    rotations_module.def("rpy2mat", &tbai::rpy2mat, "Convert roll-pitch-yaw euler angles to rotation matrix");
    rotations_module.def("mat2aa", &tbai::mat2aa, "Convert rotation matrix to axis-angle representation");
}
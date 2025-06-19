#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <tbai_core/Types.hpp>
#include <tbai_core/control/CommandPublisher.hpp>
#include <tbai_core/control/StateSubscriber.hpp>
#include <tbai_python/Controllers.hpp>
#include <tbai_reference/ReferenceVelocity.hpp>
#include <tbai_reference/ReferenceVelocityGenerator.hpp>

namespace py = pybind11;

namespace tbai {
namespace python {

class PyStateSubscriber : public ::tbai::python::MyPyStateSubscriber {
   public:
    using MyPyStateSubscriber::MyPyStateSubscriber;

    void updateState() override { PYBIND11_OVERRIDE_PURE(void, MyPyStateSubscriber, updateState); }

    void waitTillInitialized() override { PYBIND11_OVERRIDE_PURE(void, MyPyStateSubscriber, waitTillInitialized); }

    const vector_t &getLatestRbdState() override {
        PYBIND11_OVERRIDE_PURE(const vector_t &, MyPyStateSubscriber, getLatestRbdState);
    }

    const scalar_t getLatestRbdStamp() override {
        PYBIND11_OVERRIDE_PURE(scalar_t, MyPyStateSubscriber, getLatestRbdStamp);
    }

    const std::vector<bool> getContactFlags() override {
        PYBIND11_OVERRIDE_PURE(const std::vector<bool>, MyPyStateSubscriber, getContactFlags);
    }
};

class PyCommandPublisher : public tbai::CommandPublisher {
   public:
    using CommandPublisher::CommandPublisher;

    void publish(const std::vector<tbai::MotorCommand> &commands) override {
        PYBIND11_OVERRIDE_PURE(void, CommandPublisher, publish, commands);
    }
};

class PyReferenceVelocityGenerator : public tbai::reference::ReferenceVelocityGenerator {
   public:
    using ReferenceVelocityGenerator::ReferenceVelocityGenerator;

    tbai::reference::ReferenceVelocity getReferenceVelocity(scalar_t time, scalar_t dt) override {
        PYBIND11_OVERRIDE_PURE(tbai::reference::ReferenceVelocity, ReferenceVelocityGenerator, getReferenceVelocity,
                               time, dt);
    }
};

}  // namespace python
}  // namespace tbai

PYBIND11_MODULE(tbai_python, m) {
    py::class_<tbai::python::MyPyStateSubscriber, std::shared_ptr<tbai::python::MyPyStateSubscriber>>(
        m, "StateSubscriber")
        .def("waitTillInitialized", &tbai::python::MyPyStateSubscriber::waitTillInitialized)
        .def("getLatestRbdState", &tbai::python::MyPyStateSubscriber::getLatestRbdState, py::return_value_policy::reference)
        .def("getLatestRbdStamp", &tbai::python::MyPyStateSubscriber::getLatestRbdStamp)
        .def("getContactFlags", &tbai::python::MyPyStateSubscriber::getContactFlags, py::return_value_policy::reference);

    // Bind MotorCommand struct
    py::class_<tbai::MotorCommand>(m, "MotorCommand")
        .def(py::init<>())
        .def_readwrite("kp", &tbai::MotorCommand::kp)
        .def_readwrite("desired_position", &tbai::MotorCommand::desired_position)
        .def_readwrite("kd", &tbai::MotorCommand::kd)
        .def_readwrite("desired_velocity", &tbai::MotorCommand::desired_velocity)
        .def_readwrite("torque_ff", &tbai::MotorCommand::torque_ff)
        .def_readwrite("joint_name", &tbai::MotorCommand::joint_name);

    // Bind CommandPublisher class
    py::class_<tbai::CommandPublisher, std::shared_ptr<tbai::CommandPublisher>, tbai::python::PyCommandPublisher>(
        m, "CommandPublisher")
        .def(py::init<>())
        .def("publish", &tbai::CommandPublisher::publish);

    // Bind ReferenceVelocity struct
    py::class_<tbai::reference::ReferenceVelocity>(m, "ReferenceVelocity")
        .def(py::init<>())
        .def_readwrite("velocity_x", &tbai::reference::ReferenceVelocity::velocity_x)
        .def_readwrite("velocity_y", &tbai::reference::ReferenceVelocity::velocity_y)
        .def_readwrite("yaw_rate", &tbai::reference::ReferenceVelocity::yaw_rate);

    // Bind ReferenceVelocityGenerator class
    py::class_<tbai::reference::ReferenceVelocityGenerator,
               std::shared_ptr<tbai::reference::ReferenceVelocityGenerator>,
               tbai::python::PyReferenceVelocityGenerator>(m, "ReferenceVelocityGenerator")
        .def(py::init<>())
        .def("getReferenceVelocity", &tbai::reference::ReferenceVelocityGenerator::getReferenceVelocity);
}

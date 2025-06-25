#include <tbai_core/Utils.hpp>
#include <tbai_muse/TbaiEstimator.hpp>

int main(int argc, char **argv) {
    tbai::writeInitTime();

    std::vector<std::string> footNames = {"FL_foot", "FR_foot", "RL_foot", "RR_foot"};
    tbai::TbaiEstimator estimator(footNames);

    tbai::vector4_t quatBase = {0.0, 0.0, 0.0, 1.0};
    tbai::vector_t jointPositions =
        (tbai::vector_t(12) << 0.0, 0.806, -1.802, 0.0, 0.996, -1.802, 0.0, 0.806, -1.802, 0.0, 0.996, -1.802)
            .finished();
    tbai::vector_t jointVelocities = tbai::vector_t::Zero(12);
    tbai::vector3_t linearAccBase = {0.0, 0.0, 8.81};
    tbai::vector3_t angularVelBase = {0.0, 0.0, 0.0};

    tbai::scalar_t currentTime = tbai::readInitTime();
    tbai::scalar_t dt = 0.01;

    std::cout << "Base position: " << estimator.getBasePosition().transpose() << std::endl;
    std::cout << "Base velocity: " << estimator.getBaseVelocity().transpose() << std::endl;

    std::cout << "--------------------------------" << std::endl;

    std::vector<bool> contacts = {true, true, true, true};

    estimator.update(currentTime + 0.01, dt, quatBase, jointPositions, jointVelocities, linearAccBase, angularVelBase, contacts);
    estimator.update(currentTime + 0.02, dt, quatBase, jointPositions, jointVelocities, linearAccBase, angularVelBase, contacts);
    estimator.update(currentTime + 0.03, dt, quatBase, jointPositions, jointVelocities, linearAccBase, angularVelBase, contacts);
    estimator.update(currentTime + 0.04, dt, quatBase, jointPositions, jointVelocities, linearAccBase, angularVelBase, contacts);

    std::cout << "Base position: " << estimator.getBasePosition().transpose() << std::endl;
    std::cout << "Base velocity: " << estimator.getBaseVelocity().transpose() << std::endl;

    return 0;
}
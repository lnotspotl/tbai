#include <tbai_bob/CentralPatternGenerator.hpp>
#include <tbai_core/config/Config.hpp>

namespace tbai {

/**********************************************************************************************************************/
/**********************************************************************************************************************/
/**********************************************************************************************************************/
CentralPatternGenerator::CentralPatternGenerator(scalar_t period, scalar_t swingHeight, const vector_t &initial_offset)
    : period_(period), timeOffsets_(initial_offset), swingHeight_(swingHeight) {
    reset();
}

/**********************************************************************************************************************/
/**********************************************************************************************************************/
/**********************************************************************************************************************/
void CentralPatternGenerator::reset() {
    time_ = 0.0;
}
/**********************************************************************************************************************/
/**********************************************************************************************************************/
/**********************************************************************************************************************/
void CentralPatternGenerator::step(const scalar_t dt) {
    time_ += dt;
}

/**********************************************************************************************************************/
/**********************************************************************************************************************/
/**********************************************************************************************************************/
vector_t CentralPatternGenerator::computePhases() {
    vector_t phases = timeOffsets_;

    // Essentially compute ((time + offset) / period % 1) * 2 * pi for all legs
    phases.array() += time_;
    phases.array() /= period_;
    phases = phases.unaryExpr([](scalar_t x) { return std::fmod(x, static_cast<scalar_t>(1.0)); });
    phases.array() *= 2 * M_PI;
    return phases;
}

/**********************************************************************************************************************/
/**********************************************************************************************************************/
/**********************************************************************************************************************/
vector_t CentralPatternGenerator::computePhases(vector_t &phase_offsets) {
    auto phases = computePhases();
    phases = phases.array() + phase_offsets.array();
    phases = phases.unaryExpr([](scalar_t x) { return std::fmod(x, static_cast<scalar_t>(2 * M_PI)); });
    return phases;
}

/**********************************************************************************************************************/
/**********************************************************************************************************************/
/**********************************************************************************************************************/
vector_t CentralPatternGenerator::getObservation() {
    auto phases = computePhases();
    vector_t observation(8);
    observation.head<4>() = phases.array().cos();
    observation.tail<4>() = phases.array().sin();
    return observation;
}

/**********************************************************************************************************************/
/**********************************************************************************************************************/
/**********************************************************************************************************************/
vector_t CentralPatternGenerator::legHeights() {
    auto phases = computePhases();
    return computeLegHeights(phases);
}

/**********************************************************************************************************************/
/**********************************************************************************************************************/
/**********************************************************************************************************************/
vector_t CentralPatternGenerator::legHeights(vector_t &phase_offsets) {
    auto phases = computePhases(phase_offsets);
    return computeLegHeights(phases);
}

/**********************************************************************************************************************/
/**********************************************************************************************************************/
/**********************************************************************************************************************/
vector_t CentralPatternGenerator::computeLegHeights(vector_t &phases) {
    vector_t leg_heights(4);
    for (int i = 0; i < 4; ++i) {
        auto phase = phases[i];
        if (phase <= M_PI / 2) {  // swing - move up
            scalar_t time_up = phase * (2 / M_PI);
            leg_heights[i] = swingHeight_ * (-2 * std::pow(time_up, 3) + 3 * std::pow(time_up, 2));
        } else if (phase <= M_PI) {  // swing - move down
            scalar_t time_down = phase * (2 / M_PI) - 1;
            leg_heights[i] = swingHeight_ * (2 * std::pow(time_down, 3) - 3 * std::pow(time_down, 2) + 1.0);
        } else {  // stance
            leg_heights[i] = 0.0;
        }
    }

    return leg_heights;
}

/**********************************************************************************************************************/
/**********************************************************************************************************************/
/**********************************************************************************************************************/
std::unique_ptr<CentralPatternGenerator> getCentralPatternGeneratorUnique() {
    auto period = tbai::fromGlobalConfig<scalar_t>("bob_controller/cpg/period");
    auto time_offsets = tbai::fromGlobalConfig<vector_t>("bob_controller/cpg/time_offsets");
    auto swingHeight = tbai::fromGlobalConfig<scalar_t>("bob_controller/cpg/swing_height");

    return std::unique_ptr<CentralPatternGenerator>(new CentralPatternGenerator(period, swingHeight, time_offsets));
}

/**********************************************************************************************************************/
/**********************************************************************************************************************/
/**********************************************************************************************************************/
std::shared_ptr<CentralPatternGenerator> getCentralPatternGeneratorShared() {
    return std::shared_ptr<CentralPatternGenerator>(getCentralPatternGeneratorUnique().release());
}
}  // namespace tbai
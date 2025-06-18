#pragma once

#include <memory>

#include <tbai_core/Types.hpp>

namespace tbai {

class CentralPatternGenerator {
   public:
    CentralPatternGenerator(scalar_t period, scalar_t swingHeight, const vector_t &initial_offset);

    void reset();
    void step(const scalar_t dt);
    vector_t computePhases();
    vector_t computePhases(vector_t &phase_offsets);
    vector_t getObservation();

    vector_t legHeights();
    vector_t legHeights(vector_t &phase_offsets);

   private:
    vector_t computeLegHeights(vector_t &phases);

    scalar_t time_;
    scalar_t period_;
    scalar_t swingHeight_;
    vector_t timeOffsets_;
};

std::unique_ptr<CentralPatternGenerator> getCentralPatternGeneratorUnique();

std::shared_ptr<CentralPatternGenerator> getCentralPatternGeneratorShared();

}  // namespace tbai
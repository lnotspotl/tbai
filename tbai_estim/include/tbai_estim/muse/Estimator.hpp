#pragma once

#include <tbai_core/Types.hpp>

namespace tbai {
namespace muse {

class EstimatorBase {
   public:
    EstimatorBase() : EstimatorBase(std::string("")) {}
    EstimatorBase(std::string name) : name_(name) {}

    std::string getName() const { return name_; }

    void setName(std::string name) { this->name_ = name; }

    virtual unsigned int getNumStates() const { return 0; }
    virtual bool hasP() const { return false; }
    virtual unsigned int getNumMeasurments() const { return 0; }
    virtual unsigned int getNumOutputs() const { return 0; }

    virtual matrix_t getPgen() const {
        matrix_t ret;
        return ret;
    }

    virtual matrix_t getXgen() const {
        matrix_t ret;
        return ret;
    }

    virtual void updateGen(scalar_t t, matrix_t &u, matrix_t &z) {}

   protected:
    std::string name_;
};

template <typename T, unsigned int N>
class Estimator : public EstimatorBase {
   public:
    virtual unsigned int getNumStates() const override { return N; }
};

}  // namespace muse
}  // namespace tbai

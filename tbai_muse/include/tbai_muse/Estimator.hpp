#ifndef ESTIMATOR_HPP
#define ESTIMATOR_HPP

#include <tbai_muse/lib.hpp>

namespace state_estimator {

class EstimatorBase {
public:

	EstimatorBase() : EstimatorBase(std::string("")) { }
	EstimatorBase(std::string name) : name_(name) { }
	
	std::string getName() const {
                return name_;
	}

	void setName(std::string name) {
                this->name_=name;
	}

	virtual unsigned int getNumStates() const { return 0; }
	virtual bool hasP() const { return false; }
	virtual unsigned int getNumMeasurments() const { return 0; }
	virtual unsigned int getNumOutputs() const { return 0; }

	
	
	virtual Eigen::MatrixXd getPgen() const {
		Eigen::MatrixXd ret;
		return ret;
	}

	virtual Eigen::MatrixXd getXgen() const {
		Eigen::MatrixXd ret;
		return ret;
	}

virtual void updateGen(double t, Eigen::MatrixXd &u, Eigen::MatrixXd &z) { }

protected:

        std::string name_;
};

template <typename T, unsigned int N> class Estimator : public EstimatorBase{
public:
	virtual unsigned int getNumStates() const override { return N; }
 };

} //namespace state_estimator


#endif

#pragma once

#include <Eigen/Dense>
#include <tbai_muse/Observer.hpp>

namespace tbai {
namespace muse {

// N1: length of state vector (x)
// N2: length of input vector (u)
// N3: length of output vector (y or z)
// N4: length of 'internal' state vector (x)
// N5: length of 'internal' ouput vector (y or z)
// N6: length of covariance matrix (P)
// N7: length of covariance matrix (Q)
// Note that typically N1=N4=N6=N7, and N3=N5 except for a few special cases such as when using quaternions
// template <typename T, int N1, int N2, int N3, int N4, int N5, int N6, int N7> class EKFBase;

// template <typename T, int N, int M, int O> class EKF : public EKFBase<T,N,M,O,N,O,N,N> {
// public:
// 	EKF(T t0, const Eigen::Matrix<T,N,1> &xhat0, const Eigen::Matrix<T,N,N> &P0, const Eigen::Matrix<T,N,N> &Qi,
// const Eigen::Matrix<T,N,N> &Ri) : 		EKFBase<T,N,M,O,N,N,N,N>(t0,xhat0,P0,Qi,Ri) { }
// };

template <typename T, unsigned int N1, unsigned int N2, unsigned int N3, unsigned int N4, unsigned int N5, unsigned int N6, unsigned int N7>
class EKFBase : public Observer<T, N1, N2, N3> {
   public:
    EKFBase(T t0, const Eigen::Matrix<T, N1, 1> &xhat0, const Eigen::Matrix<T, N6, N6> &P0,
            const Eigen::Matrix<T, N7, N7> &Qi, const Eigen::Matrix<T, N5, N5> &Ri)
        : Observer<T, N1, N2, N3>(t0, xhat0) {
        P = P0;
        Q = Qi;
        R = Ri;
        I = Eigen::Matrix<T, N4, N4>::Identity();
        Estimator<T, N1>::name_ = std::string("EKF");
    }

    // xhatdot = f
    // Pdot = FP+PF^T+Q
    virtual void predict(T t, const Eigen::Matrix<T, N2, 1> &u) override {}

    // K = PH'(HPH'+R)^-1
    // xhat = xhat + K(z-h)
    // P = (I-KH)P
    virtual void update(T t, const Eigen::Matrix<T, N3, 1> &z) override {}

    virtual Eigen::Matrix<T, N1, 1> calc_f(T t, const Eigen::Matrix<T, N1, 1> &x, const Eigen::Matrix<T, N2, 1> &u) = 0;
    virtual Eigen::Matrix<T, N4, N4> calc_F(T t, const Eigen::Matrix<T, N1, 1> &x,
                                            const Eigen::Matrix<T, N2, 1> &u) = 0;
    virtual Eigen::Matrix<T, N3, 1> calc_h(T t, const Eigen::Matrix<T, N1, 1> &x) = 0;
    virtual Eigen::Matrix<T, N5, N4> calc_H(T t, const Eigen::Matrix<T, N1, 1> &x) = 0;

    virtual Eigen::Matrix<T, N6, N6> getP() const {
        // ROS_ERROR_STREAM("sei dentro P");
        return this->P;
    }

    virtual Eigen::MatrixXd getPgen() const override {
        Eigen::MatrixXd ret;
        ret = this->P;
        return ret;
    }

    Eigen::Matrix<T, N7, N7> getQ() const { return Q; }
    Eigen::Matrix<T, N5, N5> getR() const { return R; }

    // P must be symmetric and positive definite
    // IMPORTANT the one line version below does NOT work!!!
    inline void fixP() {
        Eigen::Matrix<T, N6, N6> tmp = 0.5 * (this->P + this->P.transpose());
        this->P = tmp;
        //		this->P = 0.5*(this->P+this->P.transpose());
    }

    virtual bool hasP() const override { return true; }

    void setP(const Eigen::Matrix<T, N6, N6> &P) { this->P = P; }

    void setQ(const Eigen::Matrix<T, N7, N7> &Q) { this->Q = Q; }

    void setR(const Eigen::Matrix<T, N5, N5> &R) { this->R = R; }

   protected:
    Eigen::Matrix<T, N6, N6> P;
    Eigen::Matrix<T, N7, N7> Q;
    Eigen::Matrix<T, N5, N5> R;
    Eigen::Matrix<T, N4, N4> I;
};

}  // namespace muse
}  // namespace tbai

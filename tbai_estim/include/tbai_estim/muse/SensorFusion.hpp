#pragma once

#include <tbai_estim/muse/EKF.hpp>
#include <unsupported/Eigen/MatrixFunctions>

namespace tbai {
namespace muse {

//   N1,N2,N3,N4,N5,N6,N7
class KFSensorFusion : public EKFBase<double, 6, 3, 3, 6, 3, 6, 6> {
   public:
    KFSensorFusion(const Eigen::Matrix<double, 6, 1> &xhat0, const Eigen::Matrix<double, 6, 6> &P0,
                   const Eigen::Matrix<double, 6, 6> &Q, const Eigen::Matrix<double, 3, 3> &R, bool lidar,
                   bool slippage);
    ~KFSensorFusion();

    void predict(double dt, const Eigen::Matrix<double, 3, 1> &u) override;
    void update(double dt, const Eigen::Matrix<double, 3, 1> &z);
    void setMatricesSF(bool slippage);

   protected:
    Eigen::Matrix<double, 6, 1> calc_f(double dt, const Eigen::Matrix<double, 6, 1> &x,
                                       const Eigen::Matrix<double, 3, 1> &u) override;
    Eigen::Matrix<double, 6, 6> calc_F(double dt, const Eigen::Matrix<double, 6, 1> &x,
                                       const Eigen::Matrix<double, 3, 1> &u) override;
    Eigen::Matrix<double, 3, 1> calc_h(double dt, const Eigen::Matrix<double, 6, 1> &x);
    Eigen::Matrix<double, 3, 6> calc_H(double dt, const Eigen::Matrix<double, 6, 1> &x);

    Eigen::Matrix<double, 6, 1> fx{Eigen::Matrix<double, 6, 1>::Zero()};
    Eigen::Matrix<double, 6, 6> F{Eigen::Matrix<double, 6, 6>::Zero()};
    Eigen::Matrix<double, 6, 6> G{Eigen::Matrix<double, 6, 6>::Identity()};
    Eigen::Matrix<double, 12, 12> M{Eigen::Matrix<double, 12, 12>::Zero()};
    Eigen::Matrix<double, 12, 12> s{Eigen::Matrix<double, 12, 12>::Zero()};
    Eigen::Matrix<double, 6, 6> Phi{Eigen::Matrix<double, 6, 6>::Zero()};
    Eigen::Matrix<double, 6, 6> Gamma{Eigen::Matrix<double, 6, 6>::Zero()};

    // update-related members
    Eigen::Matrix<double, 6, 1> xtilde{Eigen::Matrix<double, 6, 1>::Zero()};
    Eigen::Matrix<double, 3, 1> yhat{Eigen::Matrix<double, 3, 1>::Zero()};
    Eigen::Matrix<double, 3, 6> H{Eigen::Matrix<double, 3, 6>::Zero()};
    Eigen::Matrix<double, 6, 3> K{Eigen::Matrix<double, 6, 3>::Zero()};

    // calc_f helper member
    Eigen::Matrix<double, 3, 1> v{Eigen::Matrix<double, 3, 1>::Zero()};
};

// --- Inline implementations ---

inline KFSensorFusion::KFSensorFusion(const Eigen::Matrix<double, 6, 1> &xhat0, const Eigen::Matrix<double, 6, 6> &P0,
                                      const Eigen::Matrix<double, 6, 6> &Q, const Eigen::Matrix<double, 3, 3> &R,
                                      bool lidar, bool slippage)
    : EKFBase(xhat0, P0, Q, R) {
    // Optionally call setMatricesSF(slippage) if needed:
    // setMatricesSF(slippage);
}

inline KFSensorFusion::~KFSensorFusion() {}

inline void KFSensorFusion::predict(double dt, const Eigen::Matrix<double, 3, 1> &u) {
    fx = calc_f(dt, this->xhat, u);
    F = calc_F(dt, this->xhat, u);

    // Set up G: first three diagonal elements to -1, rest 1 (from Identity initialization)
    for (int i = 0; i < 3; i++) {
        G(i, i) = -1;
    }

    // Build the M matrix (concatenation of F and G blocks)
    M.block(0, 0, 6, 6) = F * dt;
    M.block(0, 6, 6, 6) = G * dt;
    M.block(6, 0, 6, 12) = Eigen::Matrix<double, 6, 12>::Zero();

    // Exponentiate the matrix M
    s = M.exp();
    Phi = s.block(0, 0, 6, 6);
    Gamma = s.block(0, 6, 6, 6);

    // Zero-order hold update for the covariance matrix
    this->P = Phi * this->P * Phi.transpose() + Gamma * this->Q * Gamma.transpose();
    this->xhat = this->xhat + fx * dt;
    this->fixP();
}

inline void KFSensorFusion::update(double dt, const Eigen::Matrix<double, 3, 1> &z) {
    yhat = calc_h(dt, this->xhat);
    H = calc_H(dt, this->xhat);

    K = this->P * H.transpose() * (H * this->P * H.transpose() + this->R).inverse();

    xtilde = K * (z - yhat);  // z from leg odometry

    this->xhat = this->xhat + xtilde;
    this->P = (this->I - K * H) * this->P;

    this->fixP();
}

inline Eigen::Matrix<double, 6, 1> KFSensorFusion::calc_f(double dt, const Eigen::Matrix<double, 6, 1> &x,
                                                          const Eigen::Matrix<double, 3, 1> &u) {
    // x is composed of [position; velocity], so extract velocity:
    v = x.tail(3);
    Eigen::Matrix<double, 6, 1> fx_local;
    fx_local << v, u;
    return fx_local;
}

inline Eigen::Matrix<double, 6, 6> KFSensorFusion::calc_F(double dt, const Eigen::Matrix<double, 6, 1> &x,
                                                          const Eigen::Matrix<double, 3, 1> &u) {
    Eigen::Matrix<double, 6, 6> F_local;
    F_local << 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    return F_local;
}

inline Eigen::Matrix<double, 3, 1> KFSensorFusion::calc_h(double dt, const Eigen::Matrix<double, 6, 1> &x) {
    // Estimated velocity measurement from the state vector
    return x.tail(3);
}

inline Eigen::Matrix<double, 3, 6> KFSensorFusion::calc_H(double dt, const Eigen::Matrix<double, 6, 1> &x) {
    Eigen::Matrix<double, 3, 6> H_local;
    H_local << 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
    return H_local;
}

// Optional: define setMatricesSF if needed
inline void KFSensorFusion::setMatricesSF(bool slippage) {
    // Implementation details depending on what you need to do when 'slippage' is true/false.
}
}  // namespace muse
}  // namespace tbai

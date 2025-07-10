#pragma once

#include <tbai_core/Types.hpp>
#include <torch/torch.h>

namespace tbai {
namespace np3o {

/** Torch -> Eigen*/
vector_t torch2vector(const torch::Tensor &t) {
    const size_t rows = t.size(0);
    float *t_data = t.data_ptr<float>();

    vector_t out(rows);
    Eigen::Map<Eigen::VectorXf> map(t_data, rows);
    out = map.cast<scalar_t>();
    return out;
}

matrix_t torch2matrix(const torch::Tensor &t) {
    const size_t rows = t.size(0);
    const size_t cols = t.size(1);
    float *t_data = t.data_ptr<float>();

    matrix_t out(cols, rows);
    Eigen::Map<Eigen::MatrixXf> map(t_data, cols, rows);
    out = map.cast<scalar_t>();
    return out.transpose();
}

/** Eigen -> Torch */
torch::Tensor vector2torch(const vector_t &v) {
    const long rows = static_cast<long>(v.rows());
    auto out = torch::empty({rows});
    float *data = out.data_ptr<float>();

    Eigen::Map<Eigen::VectorXf> map(data, rows);
    map = v.cast<float>();

    return out;
}

torch::Tensor matrix2torch(const matrix_t &m) {
    const long rows = static_cast<long>(m.rows());
    const long cols = static_cast<long>(m.cols());
    auto out = torch::empty({rows, cols});
    float *data = out.data_ptr<float>();

    Eigen::Map<Eigen::MatrixXf> map(data, cols, rows);  // "view" it as a [cols x rows] matrix
    map = m.transpose().cast<float>();

    return out;
}

}  // namespace np3o
}  // namespace tbai
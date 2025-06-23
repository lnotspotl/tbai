#include <gtest/gtest.h>
#include <tbai_wtw/EigenTorch.hpp>

using namespace tbai;

TEST(EigTorch, VectorConversion) {
    // Test vector Torch -> Eigen
    torch::Tensor t = torch::tensor({1.0f, 2.0f, 3.0f});
    vector_t v = torch2vector(t);
    EXPECT_EQ(v.size(), 3);
    EXPECT_NEAR(v[0], 1.0, 1e-6);
    EXPECT_NEAR(v[1], 2.0, 1e-6);
    EXPECT_NEAR(v[2], 3.0, 1e-6);

    // Test vector Eigen -> Torch
    vector_t v2(3);
    v2 << 4.0, 5.0, 6.0;
    torch::Tensor t2 = vector2torch(v2);
    EXPECT_EQ(t2.size(0), 3);
    EXPECT_NEAR(t2[0].item<float>(), 4.0f, 1e-6f);
    EXPECT_NEAR(t2[1].item<float>(), 5.0f, 1e-6f);
    EXPECT_NEAR(t2[2].item<float>(), 6.0f, 1e-6f);
}

TEST(EigTorch, MatrixConversion) {
    // Test matrix Torch -> Eigen
    torch::Tensor t = torch::tensor({{1.0f, 2.0f}, {3.0f, 4.0f}});
    matrix_t m = torch2matrix(t);
    EXPECT_EQ(m.rows(), 2);
    EXPECT_EQ(m.cols(), 2);
    EXPECT_NEAR(m(0, 0), 1.0, 1e-6);
    EXPECT_NEAR(m(0, 1), 2.0, 1e-6);
    EXPECT_NEAR(m(1, 0), 3.0, 1e-6);
    EXPECT_NEAR(m(1, 1), 4.0, 1e-6);

    // Test matrix Eigen -> Torch
    matrix_t m2(3, 2);
    m2 << 5.0, 6.0, 7.0, 8.0, 9.0, 10.0;
    torch::Tensor t2 = matrix2torch(m2);
    EXPECT_EQ(t2.size(0), 3);
    EXPECT_EQ(t2.size(1), 2);
    EXPECT_NEAR(t2[0][0].item<float>(), 5.0f, 1e-6f);
    EXPECT_NEAR(t2[0][1].item<float>(), 6.0f, 1e-6f);
    EXPECT_NEAR(t2[1][0].item<float>(), 7.0f, 1e-6f);
    EXPECT_NEAR(t2[1][1].item<float>(), 8.0f, 1e-6f);
    EXPECT_NEAR(t2[2][0].item<float>(), 9.0f, 1e-6f);
    EXPECT_NEAR(t2[2][1].item<float>(), 10.0f, 1e-6f);
}

TEST(EigTorch, VectorRoundTrip) {
    // Test vector Eigen -> Torch
    vector_t v(10);
    v << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0;
    torch::Tensor t = vector2torch(v);
    vector_t v2 = torch2vector(t);
    EXPECT_EQ(v2.size(), 10);
    for (size_t i = 0; i < 10; i++) {
        EXPECT_NEAR(v2[i], v[i], 1e-6);
    }

    // Test vector Torch -> Eigen
    torch::Tensor t2 = torch::tensor({1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f, 10.0f});
    vector_t v3 = torch2vector(t2);
    torch::Tensor t3 = vector2torch(v3);
    EXPECT_EQ(v3.size(), 10);
    for (size_t i = 0; i < 10; i++) {
        EXPECT_NEAR(t3[i].item<float>(), t2[i].item<float>(), 1e-6);
    }
}

TEST(EigTorch, MatrixRoundTrip) {
    // Test matrix Eigen -> Torch
    matrix_t m(2, 10);
    m << 1.0, 2.0, 3.12, 4.0, 5.0, 6.0, 7.0, 8.0, 9.923, 10.0, 3.0, 2.0, 3.33, 4.0, 5.0, 16.0, 7.0, 8.0, 9.0, 10.0;
    torch::Tensor t = matrix2torch(m);
    matrix_t m2 = torch2matrix(t);
    EXPECT_EQ(m2.rows(), 2);
    EXPECT_EQ(m2.cols(), 10);
    for (size_t i = 0; i < 10; i++) {
        EXPECT_NEAR(m2(0, i), m(0, i), 1e-6);
        EXPECT_NEAR(m2(1, i), m(1, i), 1e-6);
    }

    // Test matrix Torch -> Eigen
    torch::Tensor t2 = torch::tensor({{1.0f, 2.0f},
                                      {3.0f, 4.0f},
                                      {5.0f, 6.0f},
                                      {7.0f, 8.0f},
                                      {9.0f, 10.0f},
                                      {11.0f, 12.0f},
                                      {13.0f, 14.0f},
                                      {15.0f, 16.0f},
                                      {17.0f, 18.0f},
                                      {19.0f, 20.0f}});
    matrix_t m3 = torch2matrix(t2);
    torch::Tensor t3 = matrix2torch(m3);
    EXPECT_EQ(m3.rows(), 10);
    EXPECT_EQ(m3.cols(), 2);
    for (size_t i = 0; i < 10; i++) {
        EXPECT_NEAR(t3[i][0].item<float>(), t2[i][0].item<float>(), 1e-6);
        EXPECT_NEAR(t3[i][1].item<float>(), t2[i][1].item<float>(), 1e-6);
    }
}
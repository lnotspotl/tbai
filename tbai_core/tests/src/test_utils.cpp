#include <filesystem>
#include <fstream>
#include <thread>

#include <gtest/gtest.h>
#include <tbai_core/Utils.hpp>

using namespace tbai;

TEST(UtilsTest, InitTimeReadWrite) {
    // Write init time
    tbai::writeInitTime(1234, 567);

    // Read init time and verify
    tbai::scalar_t time = tbai::readInitTime();
    ASSERT_NEAR(time, 1234.000000567, 1e-9);
}

TEST(UtilsTest, TimeProgression) {
    tbai::writeInitTime();

    std::this_thread::sleep_for(std::chrono::seconds(1));

    auto currentTime = tbai::convertToScalar(std::chrono::system_clock::now());
    ASSERT_NEAR(currentTime, tbai::readInitTime() + 1.0, 1e-2);
}

TEST(VvstackTest, EmptyVectorInput) {
    vector_t result = vvstack({});
    EXPECT_EQ(result.size(), 0);
}

TEST(VvstackTest, SingleEmptyVector) {
    vector_t v1(0);
    std::vector<std::reference_wrapper<const vector_t>> vectors{v1};
    vector_t result = vvstack(vectors);
    EXPECT_EQ(result.size(), 0);
}

TEST(VvstackTest, SingleNonEmptyVector) {
    vector_t v1(3);
    v1 << 1.0, 2.0, 3.0;
    vector_t result = vvstack({v1});
    EXPECT_EQ(result.size(), 3);
    EXPECT_TRUE(result.isApprox(v1));
}

TEST(VvstackTest, MultipleVectors) {
    vector_t v1(2);
    v1 << 1.0, 2.0;
    vector_t v2(3);
    v2 << 3.0, 4.0, 5.0;
    vector_t v3(1);
    v3 << 6.0;
    vector_t result = vvstack({v1, v2, v3});
    EXPECT_EQ(result.size(), 6);
    vector_t expected(6);
    expected << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0;
    EXPECT_TRUE(result.isApprox(expected));
}

TEST(VvstackTest, MixedEmptyAndNonEmptyVectors) {
    vector_t v1(2);
    v1 << 1.0, 2.0;
    vector_t v2(0);
    vector_t v3(2);
    v3 << 3.0, 4.0;
    vector_t result = vvstack({v1, v2, v3});
    EXPECT_EQ(result.size(), 4);
    vector_t expected(4);
    expected << 1.0, 2.0, 3.0, 4.0;
    EXPECT_TRUE(result.isApprox(expected));
}

TEST(VvstackTest, MultipleEmptyVectors) {
    vector_t v1(0);
    vector_t v2(0);
    vector_t result = vvstack({v1, v2});
    EXPECT_EQ(result.size(), 0);
}

TEST(VvstackTest, MultipleVectorsWithDifferentSizes) {
    vector_t v1(2);
    v1 << 1.0, 2.0;
    vector_t v2(3);
    v2 << 3.0, 4.0, 5.0;
    vector_t result = vvstack({v1, v2});
    EXPECT_EQ(result.size(), 5);
    vector_t expected(5);
    expected << 1.0, 2.0, 3.0, 4.0, 5.0;
    EXPECT_TRUE(result.isApprox(expected));
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
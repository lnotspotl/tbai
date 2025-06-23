#include <gtest/gtest.h>
#include <tbai_wtw/HistoryBuffer.hpp>

static void testEq(const tbai::vector_t &a, const tbai::vector_t &b) {
    EXPECT_EQ(a.size(), b.size());
    for (size_t i = 0; i < a.size(); i++) {
        EXPECT_EQ(a[i], b[i]);
    }
}

TEST(HistoryBufferTest, TestHistoryBuffer) {
    tbai::wtw::HistoryBuffer historyBuffer(2, 3);
    tbai::vector_t finalObservation = historyBuffer.getFinalObservation();
    testEq(finalObservation, ((tbai::vector_t(6) << 0, 0, 0, 0, 0, 0).finished()));

    historyBuffer.addObservation((tbai::vector_t(2) << 1, 2).finished());
    finalObservation = historyBuffer.getFinalObservation();
    std::cout << finalObservation.transpose() << std::endl;
    testEq(finalObservation, ((tbai::vector_t(6) << 1, 2, 0, 0, 0, 0).finished()));

    historyBuffer.addObservation((tbai::vector_t(2) << 3, 4).finished());
    finalObservation = historyBuffer.getFinalObservation();
    std::cout << finalObservation.transpose() << std::endl;
    testEq(finalObservation, ((tbai::vector_t(6) << 3, 4, 1, 2, 0, 0).finished()));

    historyBuffer.addObservation((tbai::vector_t(2) << 5, 6).finished());
    finalObservation = historyBuffer.getFinalObservation();
    std::cout << finalObservation.transpose() << std::endl;
    testEq(finalObservation, ((tbai::vector_t(6) << 5, 6, 3, 4, 1, 2).finished()));

    historyBuffer.addObservation((tbai::vector_t(2) << 7, 8).finished());
    finalObservation = historyBuffer.getFinalObservation();
    std::cout << finalObservation.transpose() << std::endl;
    testEq(finalObservation, ((tbai::vector_t(6) << 7, 8, 5, 6, 3, 4).finished()));
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
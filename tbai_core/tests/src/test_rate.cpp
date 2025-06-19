#include <chrono>
#include <thread>

#include <gtest/gtest.h>
#include <tbai_core/control/Rate.hpp>

TEST(RateTest, BasicRate) {
    tbai::SystemRate<double> rate(100.0);
    EXPECT_DOUBLE_EQ(rate.getRate(), 100.0);
}

TEST(RateTest, SleepTiming) {
    tbai::SystemRate<double> rate(10.0);

    for (int i = 0; i < 3; i++) {
        auto start = std::chrono::high_resolution_clock::now();
        rate.sleep();
        auto end = std::chrono::high_resolution_clock::now();

        // Sleep should take approximately 100ms
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        EXPECT_NEAR(duration, 100, 10);  // Allow 10ms tolerance
    }
}

TEST(RateTest, SystemTime) {
    auto now1 = tbai::SystemTime<std::chrono::high_resolution_clock>::rightNow();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    auto now2 = tbai::SystemTime<std::chrono::high_resolution_clock>::rightNow();

    // Time difference should be approximately 100ms
    EXPECT_NEAR(now2 - now1, 0.1, 0.01);  // Allow 10ms tolerance
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

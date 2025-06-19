#include <filesystem>
#include <fstream>
#include <thread>

#include <gtest/gtest.h>
#include <tbai_core/Utils.hpp>

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

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
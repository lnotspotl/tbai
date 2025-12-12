#include <fstream>
#include <string>

#include <gtest/gtest.h>
#include <tbai_core/Env.hpp>
#include <tbai_core/Types.hpp>

TEST(EnvTest, standard_dtypes) {
    tbai::unsetEnv("RANDOM_ENV_VAR");
    EXPECT_THROW(tbai::getEnvAs<std::string>("RANDOM_ENV_VAR"), std::runtime_error);

    tbai::setEnv("RANDOM_ENV_VAR", "hello");
    EXPECT_EQ(tbai::getEnvAs<std::string>("RANDOM_ENV_VAR"), "hello");

    tbai::unsetEnv("RANDOM_ENV_VAR");
    EXPECT_THROW(tbai::getEnvAs<std::string>("RANDOM_ENV_VAR"), std::runtime_error);

    EXPECT_EQ(tbai::getEnvAs<std::string>("RANDOM_ENV_VAR", true, "default"), "default");

    tbai::setEnv("RANDOM_ENV_VAR", "123");
    EXPECT_EQ(tbai::getEnvAs<int>("RANDOM_ENV_VAR"), 123);

    tbai::unsetEnv("RANDOM_ENV_VAR");
    EXPECT_THROW(tbai::getEnvAs<int>("RANDOM_ENV_VAR"), std::runtime_error);

    EXPECT_EQ(tbai::getEnvAs<int>("RANDOM_ENV_VAR", true, 42), 42);

    tbai::setEnv("RANDOM_ENV_VAR", "123.456");
    EXPECT_EQ(tbai::getEnvAs<double>("RANDOM_ENV_VAR"), 123.456);

    tbai::unsetEnv("RANDOM_ENV_VAR");
    EXPECT_THROW(tbai::getEnvAs<double>("RANDOM_ENV_VAR"), std::runtime_error);

    EXPECT_EQ(tbai::getEnvAs<double>("RANDOM_ENV_VAR", true, 42.0), 42.0);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

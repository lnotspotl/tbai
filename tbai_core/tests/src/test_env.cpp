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
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

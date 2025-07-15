#include <fstream>
#include <string>

#include <gtest/gtest.h>
#include <tbai_core/Types.hpp>
#include <tbai_core/config/Config.hpp>

#define DUMMY_CONFIG_PATH "/tmp/tbai_test_config_abc123.yaml"

class YamlConfigTest : public ::testing::Test {
   protected:
    static void SetUpTestSuite() {
        const std::string configPath = DUMMY_CONFIG_PATH;
        std::ofstream file(configPath);
        std::string content =
            "a:\n  b: hello\n  c: 1\n  d: 3.14\n  e:\n    f: "
            "28\nvec: [1,2,3] \nmat: [[1,2],[3,4]]\n";
        content = content + "joint_names: [joint1, joint2, joint3]\n";
        file << content;
        file.close();
    }

    static void TearDownTestSuite() {
        const std::string configPath = DUMMY_CONFIG_PATH;
        std::remove(configPath.c_str());
    }
};

TEST_F(YamlConfigTest, standard_dtypes) {
    const std::string configPath = DUMMY_CONFIG_PATH;

    ASSERT_EQ(tbai::fromConfig<std::string>("a/b", DUMMY_CONFIG_PATH), "hello");
    ASSERT_EQ(tbai::fromConfig<int>("a/c", DUMMY_CONFIG_PATH), 1);
    ASSERT_EQ(tbai::fromConfig<double>("a/d", DUMMY_CONFIG_PATH), 3.14);
    ASSERT_EQ(tbai::fromConfig<int>("a/e/f", DUMMY_CONFIG_PATH), 28);
}

TEST_F(YamlConfigTest, tbai_vector) {
    auto vec = tbai::fromConfig<tbai::vector_t>("vec", DUMMY_CONFIG_PATH);
    ASSERT_EQ(vec.size(), 3);
    ASSERT_EQ(vec(0), 1.0);
    ASSERT_EQ(vec(1), 2.0);
    ASSERT_EQ(vec(2), 3.0);
}

TEST_F(YamlConfigTest, tbai_matrix) {
    auto mat = tbai::fromConfig<tbai::matrix_t>("mat", DUMMY_CONFIG_PATH);
    ASSERT_EQ(mat.rows(), 2);
    ASSERT_EQ(mat.cols(), 2);
    ASSERT_EQ(mat(0, 0), 1.0);
    ASSERT_EQ(mat(0, 1), 2.0);
    ASSERT_EQ(mat(1, 0), 3.0);
    ASSERT_EQ(mat(1, 1), 4.0);
}

TEST_F(YamlConfigTest, vector_of_strings) {
    auto vec = tbai::fromConfig<std::vector<std::string>>("joint_names", DUMMY_CONFIG_PATH);
    ASSERT_EQ(vec.size(), 3);
    ASSERT_EQ(vec[0], "joint1");
    ASSERT_EQ(vec[1], "joint2");
    ASSERT_EQ(vec[2], "joint3");
}

TEST_F(YamlConfigTest, test_global_config) {
    tbai::setEnv("TBAI_GLOBAL_CONFIG_PATH", DUMMY_CONFIG_PATH);

    ASSERT_EQ(tbai::fromGlobalConfig<std::string>("a/b"), "hello");
    ASSERT_EQ(tbai::fromGlobalConfig<int>("a/c"), 1);
    ASSERT_EQ(tbai::fromGlobalConfig<double>("a/d"), 3.14);
    ASSERT_EQ(tbai::fromGlobalConfig<int>("a/e/f"), 28);
}

TEST_F(YamlConfigTest, test_global_config_with_default) {
    tbai::setEnv("TBAI_GLOBAL_CONFIG_PATH", DUMMY_CONFIG_PATH);

    // Test present values
    ASSERT_EQ(tbai::fromGlobalConfig<std::string>("a/b", "default"), "hello");
    ASSERT_EQ(tbai::fromGlobalConfig<int>("a/c", 0), 1);
    ASSERT_EQ(tbai::fromGlobalConfig<double>("a/d", 0.0), 3.14);
    ASSERT_EQ(tbai::fromGlobalConfig<int>("a/e/f", 0), 28);

    // Test absent values
    ASSERT_EQ(tbai::fromGlobalConfig<std::string>("a/b/c", "default"), "default");
    ASSERT_EQ(tbai::fromGlobalConfig<int>("a/c/d", 0), 0);
    ASSERT_EQ(tbai::fromGlobalConfig<double>("a/d/e", 0.0), 0.0);
    ASSERT_EQ(tbai::fromGlobalConfig<int>("a/e/f/g", 0), 0);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
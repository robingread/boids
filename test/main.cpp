#define CATCH_CONFIG_RUNNER // Prevent Catch2 from defining main()
#include <catch2/catch.hpp>
#include <gtest/gtest.h>

int main(int argc, char** argv) {
    // Initialize Google Test
    ::testing::InitGoogleTest(&argc, argv);
    int gtest_result = RUN_ALL_TESTS();

    // Run Catch2 tests
    int catch_result = Catch::Session().run(argc, argv);

    // Return 0 only if both test frameworks pass
    return (gtest_result == 0 && catch_result == 0) ? 0 : 1;
}
#include "pico/stdlib.h"
#include "utils/assert/assert.hpp"
/**
 * This file contains basic tests for the ASSERT macro.
 * Note: These tests are designed to be run in an environment
 */
namespace test {
    bool test_assert_failure()
    {
        // This test is expected to trigger an assertion failure.
        // In a real unit test framework, you would catch this.
        ASSERT(false && "Intentional assertion failure for testing.");
        return true; // This line should never be reached.
    }

    bool test_assert_success()
    {
        // This test should pass without triggering an assertion.
        ASSERT(true);
        return true;
    }
}
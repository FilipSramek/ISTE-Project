#include "pico/stdlib.h"
#include "utils/assert/assert.hpp"
/*
 * This file contains basic tests for the ASSERT macro.
 * Note: These tests are designed to be run in an environment
 */
namespace example {
    int example_assert() {
        // Example usage of ASSERT macro
        int a = 5;
        int b = 10;

        ASSERT(a + b == 15); // This should pass

        // Uncommenting the following line will trigger an assertion failure
        // ASSERT(a - b == 0); // This should fail
    }
}
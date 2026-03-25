#ifndef UTILS_ASSERT_HPP
#define UTILS_ASSERT_HPP

#include <cstdint>

namespace utils
{
    inline void assert_failed(const char* file, uint32_t line, const char* expression)
    {
        // Handle assertion failure - can be customized for your platform
        // Options: halt, log, blink LED, etc.
        (void)file;
        (void)line;
        (void)expression;
        
        while (1)
        {
            // Infinite loop or platform-specific halt
        }
    }
} // namespace utils

#ifdef NDEBUG
    #define ASSERT(expr) ((void)0)
#else
    #define ASSERT(expr) \
        do { \
            if (!(expr)) { \
                utils::assert_failed(__FILE__, __LINE__, #expr); \
            } \
        } while (0)
#endif

#endif // UTILS_ASSERT_HPP
#ifndef UTILS_ASSERT_HPP
#define UTILS_ASSERT_HPP

#include <cstdint>

namespace utils
{
    void assert_failed(const char* file, uint32_t line, const char* expression);
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
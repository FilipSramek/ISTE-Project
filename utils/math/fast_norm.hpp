#pragma once
#include "inv_sqrt.hpp"
#include "vec3.hpp"

namespace math
{
    /**
     * @brief Length squared of a vector
     * @param v Input vector
     * @return Length squared of the vector
     */
    inline float length_squared(Vec3 v) {
        return (v.x * v.x) + (v.y * v.y) + (v.z * v.z);
    }

    /**
     * @brief Normalized vector
     * @param v Input vector
     * @return Normalized vector
     */
    inline Vec3 norm(Vec3 v) {
        float len_sq = length_squared(v);
        float inv_len = inv_sqrt(len_sq);

        return Vec3(v.x * inv_len, v.y * inv_len, v.z * inv_len);
    }
} // namespace math
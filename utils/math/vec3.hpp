#pragma once

#include <cstdint>
#include <cmath>

namespace math {

/**
 * @brief 3D vector class
 * 
 * Provides basic vector operations for 3D space
 */
class Vec3 {
public:
    float x;
    float y;
    float z;

    // ========== Constructors ==========

    /**
     * @brief Default constructor - zero vector
     */
    constexpr Vec3() : x(0.0f), y(0.0f), z(0.0f) {}

    /**
     * @brief Constructor from components
     */
    constexpr Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

    // ========== Basic Operations ==========

    /**
     * @brief Calculate magnitude squared
     * @return Squared magnitude of the vector
     */
    float magnitude_squared() const {
        return x * x + y * y + z * z;
    }

    /**
     * @brief Calculate magnitude
     * @return Magnitude of the vector
     */
    float magnitude() const {
        return sqrtf(magnitude_squared());
    }

    /**
     * @brief Normalize the vector in place
     * @return true if successful, false if magnitude too small
     */
    bool normalize() {
        float mag_sq = magnitude_squared();
        
        constexpr float EPSILON = 1e-8f;
        if (mag_sq < EPSILON) {
            return false;
        }

        float mag = sqrtf(mag_sq);
        x /= mag;
        y /= mag;
        z /= mag;

        return true;
    }

    /**
     * @brief Get normalized vector without modifying this one
     * @param result Output normalized vector
     * @return true if successful, false if magnitude too small
     */
    bool normalized(Vec3& result) const {
        result = *this;
        return result.normalize();
    }

    /**
     * @brief Calculate dot product with another vector
     */
    float dot(const Vec3& v) const {
        return x * v.x + y * v.y + z * v.z;
    }

    /**
     * @brief Calculate cross product with another vector
     */
    Vec3 cross(const Vec3& v) const {
        Vec3 result;
        result.x = y * v.z - z * v.y;
        result.y = z * v.x - x * v.z;
        result.z = x * v.y - y * v.x;
        return result;
    }

    /**
     * @brief Calculate distance to another vector
     */
    float distance(const Vec3& v) const {
        float dx = x - v.x;
        float dy = y - v.y;
        float dz = z - v.z;
        return sqrtf(dx * dx + dy * dy + dz * dz);
    }

    /**
     * @brief Calculate squared distance to another vector
     */
    float distance_squared(const Vec3& v) const {
        float dx = x - v.x;
        float dy = y - v.y;
        float dz = z - v.z;
        return dx * dx + dy * dy + dz * dz;
    }

    // ========== Operator Overloads ==========

    /**
     * @brief Vector addition
     */
    Vec3 operator+(const Vec3& v) const {
        return Vec3(x + v.x, y + v.y, z + v.z);
    }

    /**
     * @brief Vector addition assignment
     */
    Vec3& operator+=(const Vec3& v) {
        x += v.x;
        y += v.y;
        z += v.z;
        return *this;
    }

    /**
     * @brief Vector subtraction
     */
    Vec3 operator-(const Vec3& v) const {
        return Vec3(x - v.x, y - v.y, z - v.z);
    }

    /**
     * @brief Vector subtraction assignment
     */
    Vec3& operator-=(const Vec3& v) {
        x -= v.x;
        y -= v.y;
        z -= v.z;
        return *this;
    }

    /**
     * @brief Unary negation
     */
    Vec3 operator-() const {
        return Vec3(-x, -y, -z);
    }

    /**
     * @brief Scalar multiplication
     */
    Vec3 operator*(float scalar) const {
        return Vec3(x * scalar, y * scalar, z * scalar);
    }

    /**
     * @brief Scalar multiplication assignment
     */
    Vec3& operator*=(float scalar) {
        x *= scalar;
        y *= scalar;
        z *= scalar;
        return *this;
    }

    /**
     * @brief Scalar division
     */
    Vec3 operator/(float scalar) const {
        return Vec3(x / scalar, y / scalar, z / scalar);
    }

    /**
     * @brief Scalar division assignment
     */
    Vec3& operator/=(float scalar) {
        x /= scalar;
        y /= scalar;
        z /= scalar;
        return *this;
    }

    /**
     * @brief Element-wise multiplication
     */
    Vec3 operator*(const Vec3& v) const {
        return Vec3(x * v.x, y * v.y, z * v.z);
    }

    /**
     * @brief Element-wise multiplication assignment
     */
    Vec3& operator*=(const Vec3& v) {
        x *= v.x;
        y *= v.y;
        z *= v.z;
        return *this;
    }

    // ========== Comparison Operators ==========

    /**
     * @brief Equality comparison (exact)
     */
    bool operator==(const Vec3& v) const {
        return x == v.x && y == v.y && z == v.z;
    }

    /**
     * @brief Inequality comparison (exact)
     */
    bool operator!=(const Vec3& v) const {
        return !(*this == v);
    }

    /**
     * @brief Approximate equality comparison
     * @param v Vector to compare with
     * @param epsilon Tolerance for comparison
     */
    bool equals(const Vec3& v, float epsilon) const {
        float dx = x - v.x;
        float dy = y - v.y;
        float dz = z - v.z;
        
        if (dx < 0.0f) dx = -dx;
        if (dy < 0.0f) dy = -dy;
        if (dz < 0.0f) dz = -dz;
        
        return dx <= epsilon && dy <= epsilon && dz <= epsilon;
    }

    // ========== Interpolation ==========

    /**
     * @brief Linear interpolation between two vectors
     * @param v0 Start vector
     * @param v1 End vector
     * @param t Interpolation parameter [0, 1]
     * @param result Output interpolated vector
     * @return true if successful
     */
    static bool lerp(const Vec3& v0, const Vec3& v1, float t, Vec3& result) {
        if (t < 0.0f || t > 1.0f) {
            return false;
        }

        result.x = v0.x + t * (v1.x - v0.x);
        result.y = v0.y + t * (v1.y - v0.y);
        result.z = v0.z + t * (v1.z - v0.z);

        return true;
    }

    // ========== Utility Functions ==========

    /**
     * @brief Set all components to zero
     */
    void zero() {
        x = 0.0f;
        y = 0.0f;
        z = 0.0f;
    }

    /**
     * @brief Set vector components
     */
    void set(float x_, float y_, float z_) {
        x = x_;
        y = y_;
        z = z_;
    }

    /**
     * @brief Calculate angle between two vectors (in radians)
     * @param v Other vector
     * @param result Output angle in radians
     * @return true if successful, false if either vector has near-zero magnitude
     */
    bool angle_to(const Vec3& v, float& result) const {
        float mag1_sq = magnitude_squared();
        float mag2_sq = v.magnitude_squared();
        
        constexpr float EPSILON = 1e-8f;
        if (mag1_sq < EPSILON || mag2_sq < EPSILON) {
            return false;
        }

        float mag_product = sqrtf(mag1_sq * mag2_sq);
        float cos_angle = dot(v) / mag_product;

        // Clamp to valid range for acos
        if (cos_angle > 1.0f) {
            cos_angle = 1.0f;
        } else if (cos_angle < -1.0f) {
            cos_angle = -1.0f;
        }

        result = acosf(cos_angle);
        return true;
    }

    /**
     * @brief Project this vector onto another vector
     * @param v Vector to project onto
     * @param result Output projected vector
     * @return true if successful
     */
    bool project_onto(const Vec3& v, Vec3& result) const {
        float v_mag_sq = v.magnitude_squared();
        
        constexpr float EPSILON = 1e-8f;
        if (v_mag_sq < EPSILON) {
            return false;
        }

        float scalar = dot(v) / v_mag_sq;
        result = v * scalar;
        return true;
    }

    /**
     * @brief Reflect this vector across a normal
     * @param normal Surface normal (should be normalized)
     * @param result Output reflected vector
     */
    void reflect(const Vec3& normal, Vec3& result) const {
        float dot_product = dot(normal);
        result.x = x - 2.0f * dot_product * normal.x;
        result.y = y - 2.0f * dot_product * normal.y;
        result.z = z - 2.0f * dot_product * normal.z;
    }

    // ========== Common Vectors ==========

    static constexpr Vec3 ZERO() { return Vec3(0.0f, 0.0f, 0.0f); }
    static constexpr Vec3 ONE() { return Vec3(1.0f, 1.0f, 1.0f); }
    static constexpr Vec3 UNIT_X() { return Vec3(1.0f, 0.0f, 0.0f); }
    static constexpr Vec3 UNIT_Y() { return Vec3(0.0f, 1.0f, 0.0f); }
    static constexpr Vec3 UNIT_Z() { return Vec3(0.0f, 0.0f, 1.0f); }
};

/**
 * @brief Scalar multiplication (scalar on left side)
 */
inline Vec3 operator*(float scalar, const Vec3& v) {
    return v * scalar;
}

}  // namespace math
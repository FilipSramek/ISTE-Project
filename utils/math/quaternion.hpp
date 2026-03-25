#pragma once

#include <cstdint>
#include <cmath>

namespace math {

/**
 * @brief Quaternion class for 3D orientation representation
 * 
 * Quaternion format: q = w + xi + yj + zk
 * where w is the scalar part and (x, y, z) is the vector part
 */
class Quaternion {
public:
    float w;  // Scalar part
    float x;  // i component
    float y;  // j component
    float z;  // k component

    // ========== Constructors ==========

    /**
     * @brief Default constructor - identity quaternion
     */
    constexpr Quaternion() : w(1.0f), x(0.0f), y(0.0f), z(0.0f) {}

    /**
     * @brief Constructor from components
     */
    constexpr Quaternion(float w_, float x_, float y_, float z_)
        : w(w_), x(x_), y(y_), z(z_) {}

    // ========== Basic Operations ==========

    /**
     * @brief Calculate magnitude squared
     * @return Squared magnitude of the quaternion
     */
    float magnitude_squared() const {
        float result = w * w + x * x + y * y + z * z;
        return result;
    }

    /**
     * @brief Calculate magnitude
     * @return Magnitude of the quaternion
     */
    float magnitude() const {
        return sqrtf(magnitude_squared());
    }

    /**
     * @brief Normalize the quaternion in place
     * @return true if successful, false if magnitude too small
     */
    bool normalize() {
        float mag_sq = magnitude_squared();
        
        // Check for near-zero magnitude
        constexpr float EPSILON = 1e-8f;
        if (mag_sq < EPSILON) {
            return false;
        }

        float mag = sqrtf(mag_sq);
        w /= mag;
        x /= mag;
        y /= mag;
        z /= mag;

        return true;
    }

    /**
     * @brief Get normalized quaternion without modifying this one
     * @param result Output normalized quaternion
     * @return true if successful, false if magnitude too small
     */
    bool normalized(Quaternion& result) const {
        result = *this;
        return result.normalize();
    }

    /**
     * @brief Get conjugate of the quaternion
     * @return Conjugate quaternion (w, -x, -y, -z)
     */
    Quaternion conjugate() const {
        return Quaternion(w, -x, -y, -z);
    }

    /**
     * @brief Get inverse of the quaternion
     * @param result Output inverse quaternion
     * @return true if successful, false if magnitude too small
     */
    bool inverse(Quaternion& result) const {
        float mag_sq = magnitude_squared();
        
        constexpr float EPSILON = 1e-8f;
        if (mag_sq < EPSILON) {
            return false;
        }

        Quaternion conj = conjugate();
        result.w = conj.w / mag_sq;
        result.x = conj.x / mag_sq;
        result.y = conj.y / mag_sq;
        result.z = conj.z / mag_sq;

        return true;
    }

    // ========== Operator Overloads ==========

    /**
     * @brief Quaternion multiplication
     */
    Quaternion operator*(const Quaternion& q) const {
        Quaternion result;
        result.w = w * q.w - x * q.x - y * q.y - z * q.z;
        result.x = w * q.x + x * q.w + y * q.z - z * q.y;
        result.y = w * q.y - x * q.z + y * q.w + z * q.x;
        result.z = w * q.z + x * q.y - y * q.x + z * q.w;
        return result;
    }

    /**
     * @brief Quaternion multiplication assignment
     */
    Quaternion& operator*=(const Quaternion& q) {
        *this = *this * q;
        return *this;
    }

    /**
     * @brief Quaternion addition
     */
    Quaternion operator+(const Quaternion& q) const {
        return Quaternion(w + q.w, x + q.x, y + q.y, z + q.z);
    }

    /**
     * @brief Quaternion addition assignment
     */
    Quaternion& operator+=(const Quaternion& q) {
        w += q.w;
        x += q.x;
        y += q.y;
        z += q.z;
        return *this;
    }

    /**
     * @brief Quaternion subtraction
     */
    Quaternion operator-(const Quaternion& q) const {
        return Quaternion(w - q.w, x - q.x, y - q.y, z - q.z);
    }

    /**
     * @brief Quaternion subtraction assignment
     */
    Quaternion& operator-=(const Quaternion& q) {
        w -= q.w;
        x -= q.x;
        y -= q.y;
        z -= q.z;
        return *this;
    }

    /**
     * @brief Scalar multiplication
     */
    Quaternion operator*(float scalar) const {
        return Quaternion(w * scalar, x * scalar, y * scalar, z * scalar);
    }

    /**
     * @brief Scalar multiplication assignment
     */
    Quaternion& operator*=(float scalar) {
        w *= scalar;
        x *= scalar;
        y *= scalar;
        z *= scalar;
        return *this;
    }

    /**
     * @brief Scalar division
     */
    Quaternion operator/(float scalar) const {
        return Quaternion(w / scalar, x / scalar, y / scalar, z / scalar);
    }

    /**
     * @brief Scalar division assignment
     */
    Quaternion& operator/=(float scalar) {
        w /= scalar;
        x /= scalar;
        y /= scalar;
        z /= scalar;
        return *this;
    }

    // ========== Rotation Operations ==========

    /**
     * @brief Rotate a 3D vector using this quaternion
     * @param vx X component of vector
     * @param vy Y component of vector
     * @param vz Z component of vector
     * @param rx Output rotated X component
     * @param ry Output rotated Y component
     * @param rz Output rotated Z component
     */
    void rotate_vector(float vx, float vy, float vz,
                      float& rx, float& ry, float& rz) const {
        // v' = q * v * q^-1
        // Optimized form without creating quaternions
        float t0 = 2.0f * (w * vx + y * vz - z * vy);
        float t1 = 2.0f * (w * vy + z * vx - x * vz);
        float t2 = 2.0f * (w * vz + x * vy - y * vx);

        rx = vx + w * t0 + (y * t2 - z * t1);
        ry = vy + w * t1 + (z * t0 - x * t2);
        rz = vz + w * t2 + (x * t1 - y * t0);
    }

    // ========== Euler Angle Conversions ==========

    /**
     * @brief Convert to Euler angles (ZYX convention)
     * @param roll Output roll angle (radians)
     * @param pitch Output pitch angle (radians)
     * @param yaw Output yaw angle (radians)
     */
    void to_euler(float& roll, float& pitch, float& yaw) const {
        // Roll (x-axis rotation)
        float sinr_cosp = 2.0f * (w * x + y * z);
        float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
        roll = atan2f(sinr_cosp, cosr_cosp);

        // Pitch (y-axis rotation)
        float sinp = 2.0f * (w * y - z * x);
        if (sinp >= 1.0f) {
            pitch = 1.57079632679f;  // π/2
        } else if (sinp <= -1.0f) {
            pitch = -1.57079632679f;  // -π/2
        } else {
            pitch = asinf(sinp);
        }

        // Yaw (z-axis rotation)
        float siny_cosp = 2.0f * (w * z + x * y);
        float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
        yaw = atan2f(siny_cosp, cosy_cosp);
    }

    /**
     * @brief Create quaternion from Euler angles (ZYX convention)
     * @param roll Roll angle (radians)
     * @param pitch Pitch angle (radians)
     * @param yaw Yaw angle (radians)
     * @param result Output quaternion
     */
    static void from_euler(float roll, float pitch, float yaw,
                          Quaternion& result) {
        float cr = cosf(roll * 0.5f);
        float sr = sinf(roll * 0.5f);
        float cp = cosf(pitch * 0.5f);
        float sp = sinf(pitch * 0.5f);
        float cy = cosf(yaw * 0.5f);
        float sy = sinf(yaw * 0.5f);

        result.w = cr * cp * cy + sr * sp * sy;
        result.x = sr * cp * cy - cr * sp * sy;
        result.y = cr * sp * cy + sr * cp * sy;
        result.z = cr * cp * sy - sr * sp * cy;
    }

    /**
     * @brief Create quaternion from axis-angle representation
     * @param axis_x X component of rotation axis (must be normalized)
     * @param axis_y Y component of rotation axis (must be normalized)
     * @param axis_z Z component of rotation axis (must be normalized)
     * @param angle Rotation angle in radians
     * @param result Output quaternion
     */
    static void from_axis_angle(float axis_x, float axis_y, float axis_z,
                               float angle, Quaternion& result) {
        float half_angle = angle * 0.5f;
        float s = sinf(half_angle);
        
        result.w = cosf(half_angle);
        result.x = axis_x * s;
        result.y = axis_y * s;
        result.z = axis_z * s;
    }

    // ========== Angular Rate Integration ==========

    /**
     * @brief Update quaternion using angular rates (gyroscope integration)
     * @param gx Angular rate around X axis (rad/s)
     * @param gy Angular rate around Y axis (rad/s)
     * @param gz Angular rate around Z axis (rad/s)
     * @param dt Time step (seconds)
     * @return true if successful
     * 
     * Uses first-order integration:
     * q(t+dt) = q(t) + 0.5 * q(t) * [0, gx, gy, gz] * dt
     */
    bool update_from_gyro(float gx, float gy, float gz, float dt) {
        constexpr float MAX_DT = 1.0f;
        if (dt <= 0.0f || dt > MAX_DT) {
            return false;
        }

        // Quaternion derivative: dq/dt = 0.5 * q * omega
        // where omega = [0, gx, gy, gz]
        float half_dt = 0.5f * dt;
        
        float dw = -half_dt * (x * gx + y * gy + z * gz);
        float dx =  half_dt * (w * gx + y * gz - z * gy);
        float dy =  half_dt * (w * gy + z * gx - x * gz);
        float dz =  half_dt * (w * gz + x * gy - y * gx);

        w += dw;
        x += dx;
        y += dy;
        z += dz;

        // Normalize to prevent drift
        return normalize();
    }

    /**
     * @brief Update quaternion using angular rates with improved integration
     * @param gx Angular rate around X axis (rad/s)
     * @param gy Angular rate around Y axis (rad/s)
     * @param gz Angular rate around Z axis (rad/s)
     * @param dt Time step (seconds)
     * @return true if successful
     * 
     * Uses more accurate method for larger rotation rates
     */
    bool update_from_gyro_accurate(float gx, float gy, float gz, float dt) {
        constexpr float MAX_DT = 1.0f;
        if (dt <= 0.0f || dt > MAX_DT) {
            return false;
        }

        // Calculate rotation magnitude
        float angle = sqrtf(gx * gx + gy * gy + gz * gz) * dt;
        
        constexpr float MIN_ANGLE = 1e-6f;
        if (angle < MIN_ANGLE) {
            // Use simple integration for small angles
            return update_from_gyro(gx, gy, gz, dt);
        }

        // Normalize axis
        float angle_inv = dt / angle;
        float axis_x = gx * angle_inv;
        float axis_y = gy * angle_inv;
        float axis_z = gz * angle_inv;

        // Create rotation quaternion
        Quaternion rotation;
        from_axis_angle(axis_x, axis_y, axis_z, angle, rotation);

        // Apply rotation: q_new = rotation * q_old
        *this = rotation * (*this);

        return normalize();
    }

    // ========== Interpolation ==========

    /**
     * @brief Spherical linear interpolation (SLERP) between two quaternions
     * @param q0 Start quaternion
     * @param q1 End quaternion
     * @param t Interpolation parameter [0, 1]
     * @param result Output interpolated quaternion
     * @return true if successful
     */
    static bool slerp(const Quaternion& q0, const Quaternion& q1, 
                     float t, Quaternion& result) {
        constexpr float EPSILON = 1e-6f;
        if (t < 0.0f || t > 1.0f) {
            return false;
        }

        // Calculate dot product
        float dot = q0.w * q1.w + q0.x * q1.x + q0.y * q1.y + q0.z * q1.z;

        // If quaternions are close, use linear interpolation
        if (dot > 1.0f - EPSILON) {
            result.w = q0.w + t * (q1.w - q0.w);
            result.x = q0.x + t * (q1.x - q0.x);
            result.y = q0.y + t * (q1.y - q0.y);
            result.z = q0.z + t * (q1.z - q0.z);
            return result.normalize();
        }

        // Ensure shortest path
        Quaternion q1_adj = q1;
        if (dot < 0.0f) {
            q1_adj.w = -q1.w;
            q1_adj.x = -q1.x;
            q1_adj.y = -q1.y;
            q1_adj.z = -q1.z;
            dot = -dot;
        }

        // Clamp dot product
        if (dot > 1.0f) {
            dot = 1.0f;
        }

        float theta = acosf(dot);
        float sin_theta = sinf(theta);

        if (sin_theta < EPSILON) {
            return false;
        }

        float w0 = sinf((1.0f - t) * theta) / sin_theta;
        float w1 = sinf(t * theta) / sin_theta;

        result.w = w0 * q0.w + w1 * q1_adj.w;
        result.x = w0 * q0.x + w1 * q1_adj.x;
        result.y = w0 * q0.y + w1 * q1_adj.y;
        result.z = w0 * q0.z + w1 * q1_adj.z;

        return true;
    }

    // ========== Utility Functions ==========

    /**
     * @brief Calculate dot product with another quaternion
     */
    float dot(const Quaternion& q) const {
        return w * q.w + x * q.x + y * q.y + z * q.z;
    }
};

}  // namespace math
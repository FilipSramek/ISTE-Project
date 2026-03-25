#pragma once

#include <cstdint>
#include <cmath>
#include "vec3.hpp"

namespace math {

/**
 * @brief 3x3 matrix class
 * 
 * Matrix stored in row-major order:
 * [ m00  m01  m02 ]
 * [ m10  m11  m12 ]
 * [ m20  m21  m22 ]
 */
class Mat3 {
public:
    float m00, m01, m02;
    float m10, m11, m12;
    float m20, m21, m22;

    // ========== Constructors ==========

    /**
     * @brief Default constructor - identity matrix
     */
    constexpr Mat3()
        : m00(1.0f), m01(0.0f), m02(0.0f),
          m10(0.0f), m11(1.0f), m12(0.0f),
          m20(0.0f), m21(0.0f), m22(1.0f) {}

    /**
     * @brief Constructor from individual elements (row-major)
     */
    constexpr Mat3(float m00_, float m01_, float m02_,
                   float m10_, float m11_, float m12_,
                   float m20_, float m21_, float m22_)
        : m00(m00_), m01(m01_), m02(m02_),
          m10(m10_), m11(m11_), m12(m12_),
          m20(m20_), m21(m21_), m22(m22_) {}

    /**
     * @brief Constructor from column vectors
     */
    constexpr Mat3(const Vec3& col0, const Vec3& col1, const Vec3& col2)
        : m00(col0.x), m01(col1.x), m02(col2.x),
          m10(col0.y), m11(col1.y), m12(col2.y),
          m20(col0.z), m21(col1.z), m22(col2.z) {}

    // ========== Basic Operations ==========

    /**
     * @brief Set matrix to identity
     */
    void set_identity() {
        m00 = 1.0f; m01 = 0.0f; m02 = 0.0f;
        m10 = 0.0f; m11 = 1.0f; m12 = 0.0f;
        m20 = 0.0f; m21 = 0.0f; m22 = 1.0f;
    }

    /**
     * @brief Set matrix to zero
     */
    void set_zero() {
        m00 = 0.0f; m01 = 0.0f; m02 = 0.0f;
        m10 = 0.0f; m11 = 0.0f; m12 = 0.0f;
        m20 = 0.0f; m21 = 0.0f; m22 = 0.0f;
    }

    /**
     * @brief Get transpose of the matrix
     */
    Mat3 transpose() const {
        return Mat3(m00, m10, m20,
                   m01, m11, m21,
                   m02, m12, m22);
    }

    /**
     * @brief Calculate determinant
     */
    float determinant() const {
        float det = m00 * (m11 * m22 - m12 * m21)
                  - m01 * (m10 * m22 - m12 * m20)
                  + m02 * (m10 * m21 - m11 * m20);
        return det;
    }

    /**
     * @brief Calculate inverse of the matrix
     * @param result Output inverse matrix
     * @return true if successful, false if matrix is singular
     */
    bool inverse(Mat3& result) const {
        float det = determinant();
        
        constexpr float EPSILON = 1e-8f;
        if (det > -EPSILON && det < EPSILON) {
            return false;
        }

        float inv_det = 1.0f / det;

        result.m00 = (m11 * m22 - m12 * m21) * inv_det;
        result.m01 = (m02 * m21 - m01 * m22) * inv_det;
        result.m02 = (m01 * m12 - m02 * m11) * inv_det;

        result.m10 = (m12 * m20 - m10 * m22) * inv_det;
        result.m11 = (m00 * m22 - m02 * m20) * inv_det;
        result.m12 = (m02 * m10 - m00 * m12) * inv_det;

        result.m20 = (m10 * m21 - m11 * m20) * inv_det;
        result.m21 = (m01 * m20 - m00 * m21) * inv_det;
        result.m22 = (m00 * m11 - m01 * m10) * inv_det;

        return true;
    }

    /**
     * @brief Calculate trace (sum of diagonal elements)
     */
    float trace() const {
        return m00 + m11 + m22;
    }

    // ========== Operator Overloads ==========

    /**
     * @brief Matrix multiplication
     */
    Mat3 operator*(const Mat3& mat) const {
        Mat3 result;
        
        result.m00 = m00 * mat.m00 + m01 * mat.m10 + m02 * mat.m20;
        result.m01 = m00 * mat.m01 + m01 * mat.m11 + m02 * mat.m21;
        result.m02 = m00 * mat.m02 + m01 * mat.m12 + m02 * mat.m22;

        result.m10 = m10 * mat.m00 + m11 * mat.m10 + m12 * mat.m20;
        result.m11 = m10 * mat.m01 + m11 * mat.m11 + m12 * mat.m21;
        result.m12 = m10 * mat.m02 + m11 * mat.m12 + m12 * mat.m22;

        result.m20 = m20 * mat.m00 + m21 * mat.m10 + m22 * mat.m20;
        result.m21 = m20 * mat.m01 + m21 * mat.m11 + m22 * mat.m21;
        result.m22 = m20 * mat.m02 + m21 * mat.m12 + m22 * mat.m22;

        return result;
    }

    /**
     * @brief Matrix multiplication assignment
     */
    Mat3& operator*=(const Mat3& mat) {
        *this = *this * mat;
        return *this;
    }

    /**
     * @brief Matrix-vector multiplication
     */
    Vec3 operator*(const Vec3& v) const {
        Vec3 result;
        result.x = m00 * v.x + m01 * v.y + m02 * v.z;
        result.y = m10 * v.x + m11 * v.y + m12 * v.z;
        result.z = m20 * v.x + m21 * v.y + m22 * v.z;
        return result;
    }

    /**
     * @brief Matrix addition
     */
    Mat3 operator+(const Mat3& mat) const {
        return Mat3(m00 + mat.m00, m01 + mat.m01, m02 + mat.m02,
                   m10 + mat.m10, m11 + mat.m11, m12 + mat.m12,
                   m20 + mat.m20, m21 + mat.m21, m22 + mat.m22);
    }

    /**
     * @brief Matrix addition assignment
     */
    Mat3& operator+=(const Mat3& mat) {
        m00 += mat.m00; m01 += mat.m01; m02 += mat.m02;
        m10 += mat.m10; m11 += mat.m11; m12 += mat.m12;
        m20 += mat.m20; m21 += mat.m21; m22 += mat.m22;
        return *this;
    }

    /**
     * @brief Matrix subtraction
     */
    Mat3 operator-(const Mat3& mat) const {
        return Mat3(m00 - mat.m00, m01 - mat.m01, m02 - mat.m02,
                   m10 - mat.m10, m11 - mat.m11, m12 - mat.m12,
                   m20 - mat.m20, m21 - mat.m21, m22 - mat.m22);
    }

    /**
     * @brief Matrix subtraction assignment
     */
    Mat3& operator-=(const Mat3& mat) {
        m00 -= mat.m00; m01 -= mat.m01; m02 -= mat.m02;
        m10 -= mat.m10; m11 -= mat.m11; m12 -= mat.m12;
        m20 -= mat.m20; m21 -= mat.m21; m22 -= mat.m22;
        return *this;
    }

    /**
     * @brief Scalar multiplication
     */
    Mat3 operator*(float scalar) const {
        return Mat3(m00 * scalar, m01 * scalar, m02 * scalar,
                   m10 * scalar, m11 * scalar, m12 * scalar,
                   m20 * scalar, m21 * scalar, m22 * scalar);
    }

    /**
     * @brief Scalar multiplication assignment
     */
    Mat3& operator*=(float scalar) {
        m00 *= scalar; m01 *= scalar; m02 *= scalar;
        m10 *= scalar; m11 *= scalar; m12 *= scalar;
        m20 *= scalar; m21 *= scalar; m22 *= scalar;
        return *this;
    }

    /**
     * @brief Scalar division
     */
    Mat3 operator/(float scalar) const {
        return Mat3(m00 / scalar, m01 / scalar, m02 / scalar,
                   m10 / scalar, m11 / scalar, m12 / scalar,
                   m20 / scalar, m21 / scalar, m22 / scalar);
    }

    /**
     * @brief Scalar division assignment
     */
    Mat3& operator/=(float scalar) {
        m00 /= scalar; m01 /= scalar; m02 /= scalar;
        m10 /= scalar; m11 /= scalar; m12 /= scalar;
        m20 /= scalar; m21 /= scalar; m22 /= scalar;
        return *this;
    }

    // ========== Row and Column Access ==========

    /**
     * @brief Get row as vector
     */
    Vec3 get_row(uint32_t row) const {
        if (row == 0) {
            return Vec3(m00, m01, m02);
        } else if (row == 1) {
            return Vec3(m10, m11, m12);
        } else {
            return Vec3(m20, m21, m22);
        }
    }

    /**
     * @brief Get column as vector
     */
    Vec3 get_column(uint32_t col) const {
        if (col == 0) {
            return Vec3(m00, m10, m20);
        } else if (col == 1) {
            return Vec3(m01, m11, m21);
        } else {
            return Vec3(m02, m12, m22);
        }
    }

    /**
     * @brief Set row from vector
     */
    void set_row(uint32_t row, const Vec3& v) {
        if (row == 0) {
            m00 = v.x; m01 = v.y; m02 = v.z;
        } else if (row == 1) {
            m10 = v.x; m11 = v.y; m12 = v.z;
        } else {
            m20 = v.x; m21 = v.y; m22 = v.z;
        }
    }

    /**
     * @brief Set column from vector
     */
    void set_column(uint32_t col, const Vec3& v) {
        if (col == 0) {
            m00 = v.x; m10 = v.y; m20 = v.z;
        } else if (col == 1) {
            m01 = v.x; m11 = v.y; m21 = v.z;
        } else {
            m02 = v.x; m12 = v.y; m22 = v.z;
        }
    }

    // ========== Rotation Matrices ==========

    /**
     * @brief Create rotation matrix around X axis
     * @param angle Rotation angle in radians
     * @param result Output rotation matrix
     */
    static void rotation_x(float angle, Mat3& result) {
        float c = cosf(angle);
        float s = sinf(angle);

        result.m00 = 1.0f; result.m01 = 0.0f; result.m02 = 0.0f;
        result.m10 = 0.0f; result.m11 = c;    result.m12 = -s;
        result.m20 = 0.0f; result.m21 = s;    result.m22 = c;
    }

    /**
     * @brief Create rotation matrix around Y axis
     * @param angle Rotation angle in radians
     * @param result Output rotation matrix
     */
    static void rotation_y(float angle, Mat3& result) {
        float c = cosf(angle);
        float s = sinf(angle);

        result.m00 = c;    result.m01 = 0.0f; result.m02 = s;
        result.m10 = 0.0f; result.m11 = 1.0f; result.m12 = 0.0f;
        result.m20 = -s;   result.m21 = 0.0f; result.m22 = c;
    }

    /**
     * @brief Create rotation matrix around Z axis
     * @param angle Rotation angle in radians
     * @param result Output rotation matrix
     */
    static void rotation_z(float angle, Mat3& result) {
        float c = cosf(angle);
        float s = sinf(angle);

        result.m00 = c;    result.m01 = -s;   result.m02 = 0.0f;
        result.m10 = s;    result.m11 = c;    result.m12 = 0.0f;
        result.m20 = 0.0f; result.m21 = 0.0f; result.m22 = 1.0f;
    }

    /**
     * @brief Create rotation matrix from axis-angle
     * @param axis Rotation axis (must be normalized)
     * @param angle Rotation angle in radians
     * @param result Output rotation matrix
     */
    static void rotation_axis_angle(const Vec3& axis, float angle, Mat3& result) {
        float c = cosf(angle);
        float s = sinf(angle);
        float t = 1.0f - c;

        float x = axis.x;
        float y = axis.y;
        float z = axis.z;

        result.m00 = t * x * x + c;
        result.m01 = t * x * y - s * z;
        result.m02 = t * x * z + s * y;

        result.m10 = t * x * y + s * z;
        result.m11 = t * y * y + c;
        result.m12 = t * y * z - s * x;

        result.m20 = t * x * z - s * y;
        result.m21 = t * y * z + s * x;
        result.m22 = t * z * z + c;
    }

    /**
     * @brief Create rotation matrix from Euler angles (ZYX convention)
     * @param roll Roll angle in radians
     * @param pitch Pitch angle in radians
     * @param yaw Yaw angle in radians
     * @param result Output rotation matrix
     */
    static void from_euler(float roll, float pitch, float yaw, Mat3& result) {
        float cr = cosf(roll);
        float sr = sinf(roll);
        float cp = cosf(pitch);
        float sp = sinf(pitch);
        float cy = cosf(yaw);
        float sy = sinf(yaw);

        result.m00 = cy * cp;
        result.m01 = cy * sp * sr - sy * cr;
        result.m02 = cy * sp * cr + sy * sr;

        result.m10 = sy * cp;
        result.m11 = sy * sp * sr + cy * cr;
        result.m12 = sy * sp * cr - cy * sr;

        result.m20 = -sp;
        result.m21 = cp * sr;
        result.m22 = cp * cr;
    }

    // ========== Scaling Matrices ==========

    /**
     * @brief Create uniform scaling matrix
     */
    static Mat3 scaling(float s) {
        return Mat3(s,    0.0f, 0.0f,
                   0.0f, s,    0.0f,
                   0.0f, 0.0f, s);
    }

    /**
     * @brief Create non-uniform scaling matrix
     */
    static Mat3 scaling(float sx, float sy, float sz) {
        return Mat3(sx,   0.0f, 0.0f,
                   0.0f, sy,   0.0f,
                   0.0f, 0.0f, sz);
    }

    // ========== Utility Functions ==========

    /**
     * @brief Create skew-symmetric matrix from vector (for cross product)
     */
    static Mat3 skew_symmetric(const Vec3& v) {
        return Mat3(0.0f,  -v.z,   v.y,
                    v.z,   0.0f,  -v.x,
                   -v.y,   v.x,   0.0f);
    }

    /**
     * @brief Create outer product of two vectors
     */
    static Mat3 outer_product(const Vec3& a, const Vec3& b) {
        return Mat3(a.x * b.x, a.x * b.y, a.x * b.z,
                   a.y * b.x, a.y * b.y, a.y * b.z,
                   a.z * b.x, a.z * b.y, a.z * b.z);
    }

    // ========== Common Matrices ==========

    static constexpr Mat3 identity() {
        return Mat3(1.0f, 0.0f, 0.0f,
                   0.0f, 1.0f, 0.0f,
                   0.0f, 0.0f, 1.0f);
    }

    static constexpr Mat3 zero() {
        return Mat3(0.0f, 0.0f, 0.0f,
                   0.0f, 0.0f, 0.0f,
                   0.0f, 0.0f, 0.0f);
    }
};

/**
 * @brief Scalar multiplication (scalar on left side)
 */
inline Mat3 operator*(float scalar, const Mat3& mat) {
    return mat * scalar;
}

}  // namespace math
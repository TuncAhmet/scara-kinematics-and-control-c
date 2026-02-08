/**
 * @file math_utils.h
 * @brief Matrix and vector mathematical utilities
 */

#ifndef SCARA_MATH_UTILS_H
#define SCARA_MATH_UTILS_H

#include "types.h"

/* ============================================================================
 * Matrix Operations
 * ========================================================================== */

/**
 * @brief Create an identity 4x4 matrix
 * @return Identity matrix
 */
Matrix4x4 matrix_identity(void);

/**
 * @brief Create a zero 4x4 matrix
 * @return Zero matrix
 */
Matrix4x4 matrix_zeros(void);

/**
 * @brief Multiply two 4x4 matrices: C = A * B
 * @param A First matrix
 * @param B Second matrix
 * @return Result matrix
 */
Matrix4x4 matrix_multiply(const Matrix4x4* A, const Matrix4x4* B);

/**
 * @brief Multiply a 4x4 matrix by a 4x1 vector: v_out = M * v
 * @param M Matrix
 * @param v Vector
 * @return Result vector
 */
Vector4 matrix_vector_multiply(const Matrix4x4* M, const Vector4* v);

/**
 * @brief Copy a 4x4 matrix
 * @param src Source matrix
 * @param dst Destination matrix
 */
void matrix_copy(const Matrix4x4* src, Matrix4x4* dst);

/**
 * @brief Check if two matrices are approximately equal
 * @param A First matrix
 * @param B Second matrix
 * @param tolerance Comparison tolerance
 * @return true if matrices are equal within tolerance
 */
bool matrix_equals(const Matrix4x4* A, const Matrix4x4* B, double tolerance);

/**
 * @brief Extract the rotation matrix (3x3) from a 4x4 transformation matrix
 * @param T Transformation matrix
 * @param R Output 3x3 rotation (stored in R[0..2][0..2])
 */
void matrix_extract_rotation(const Matrix4x4* T, double R[3][3]);

/**
 * @brief Extract the translation vector from a 4x4 transformation matrix
 * @param T Transformation matrix
 * @return Translation as Vector3
 */
Vector3 matrix_extract_translation(const Matrix4x4* T);

/* ============================================================================
 * Vector Operations
 * ========================================================================== */

/**
 * @brief Create a Vector4 with specified values
 * @param x, y, z, w Vector components
 * @return Vector4
 */
Vector4 vector4_create(double x, double y, double z, double w);

/**
 * @brief Create a Vector3 with specified values
 * @param x, y, z Vector components
 * @return Vector3
 */
Vector3 vector3_create(double x, double y, double z);

/**
 * @brief Compute the Euclidean norm of a Vector3
 * @param v Vector
 * @return Euclidean norm
 */
double vector3_norm(const Vector3* v);

/**
 * @brief Compute the dot product of two Vector3
 * @param a First vector
 * @param b Second vector
 * @return Dot product
 */
double vector3_dot(const Vector3* a, const Vector3* b);

/* ============================================================================
 * Angle Utilities
 * ========================================================================== */

/**
 * @brief Convert degrees to radians
 * @param deg Angle in degrees
 * @return Angle in radians
 */
double deg_to_rad(double deg);

/**
 * @brief Convert radians to degrees
 * @param rad Angle in radians
 * @return Angle in degrees
 */
double rad_to_deg(double rad);

/**
 * @brief Normalize angle to range [-PI, PI]
 * @param angle Angle in radians
 * @return Normalized angle
 */
double normalize_angle(double angle);

/**
 * @brief Compute the shortest angular difference between two angles
 * @param angle1 First angle [rad]
 * @param angle2 Second angle [rad]
 * @return Shortest difference (angle2 - angle1) in [-PI, PI]
 */
double angle_difference(double angle1, double angle2);

/* ============================================================================
 * General Utilities
 * ========================================================================== */

/**
 * @brief Clamp a value to a range
 * @param value Value to clamp
 * @param min Minimum value
 * @param max Maximum value
 * @return Clamped value
 */
double clamp(double value, double min, double max);

/**
 * @brief Linear interpolation
 * @param a Start value
 * @param b End value
 * @param t Interpolation factor [0, 1]
 * @return Interpolated value
 */
double lerp(double a, double b, double t);

/**
 * @brief Sign function
 * @param x Input value
 * @return -1, 0, or 1
 */
double sign(double x);

/**
 * @brief Check if a value is within range (inclusive)
 * @param value Value to check
 * @param min Minimum
 * @param max Maximum
 * @return true if min <= value <= max
 */
bool in_range(double value, double min, double max);

/**
 * @brief Check if two doubles are approximately equal
 * @param a First value
 * @param b Second value
 * @param tolerance Comparison tolerance
 * @return true if |a - b| <= tolerance
 */
bool double_equals(double a, double b, double tolerance);

#endif /* SCARA_MATH_UTILS_H */

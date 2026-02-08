/**
 * @file math_utils.c
 * @brief Matrix and vector mathematical utilities implementation
 */

#include "scara/math_utils.h"
#include <math.h>
#include <string.h>

/* ============================================================================
 * Matrix Operations
 * ========================================================================== */

Matrix4x4 matrix_identity(void) {
    Matrix4x4 result = {{{0}}};
    result.m[0][0] = 1.0;
    result.m[1][1] = 1.0;
    result.m[2][2] = 1.0;
    result.m[3][3] = 1.0;
    return result;
}

Matrix4x4 matrix_zeros(void) {
    Matrix4x4 result = {{{0}}};
    return result;
}

Matrix4x4 matrix_multiply(const Matrix4x4* A, const Matrix4x4* B) {
    Matrix4x4 result = {{{0}}};
    
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            double sum = 0.0;
            for (int k = 0; k < 4; k++) {
                sum += A->m[i][k] * B->m[k][j];
            }
            result.m[i][j] = sum;
        }
    }
    
    return result;
}

Vector4 matrix_vector_multiply(const Matrix4x4* M, const Vector4* v) {
    Vector4 result = {{0}};
    
    for (int i = 0; i < 4; i++) {
        double sum = 0.0;
        for (int j = 0; j < 4; j++) {
            sum += M->m[i][j] * v->v[j];
        }
        result.v[i] = sum;
    }
    
    return result;
}

void matrix_copy(const Matrix4x4* src, Matrix4x4* dst) {
    memcpy(dst->m, src->m, sizeof(double) * 16);
}

bool matrix_equals(const Matrix4x4* A, const Matrix4x4* B, double tolerance) {
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            if (fabs(A->m[i][j] - B->m[i][j]) > tolerance) {
                return false;
            }
        }
    }
    return true;
}

void matrix_extract_rotation(const Matrix4x4* T, double R[3][3]) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            R[i][j] = T->m[i][j];
        }
    }
}

Vector3 matrix_extract_translation(const Matrix4x4* T) {
    Vector3 result;
    result.x = T->m[0][3];
    result.y = T->m[1][3];
    result.z = T->m[2][3];
    return result;
}

/* ============================================================================
 * Vector Operations
 * ========================================================================== */

Vector4 vector4_create(double x, double y, double z, double w) {
    Vector4 result;
    result.v[0] = x;
    result.v[1] = y;
    result.v[2] = z;
    result.v[3] = w;
    return result;
}

Vector3 vector3_create(double x, double y, double z) {
    Vector3 result;
    result.x = x;
    result.y = y;
    result.z = z;
    return result;
}

double vector3_norm(const Vector3* v) {
    return sqrt(v->x * v->x + v->y * v->y + v->z * v->z);
}

double vector3_dot(const Vector3* a, const Vector3* b) {
    return a->x * b->x + a->y * b->y + a->z * b->z;
}

/* ============================================================================
 * Angle Utilities
 * ========================================================================== */

double deg_to_rad(double deg) {
    return deg * SCARA_DEG_TO_RAD;
}

double rad_to_deg(double rad) {
    return rad * SCARA_RAD_TO_DEG;
}

double normalize_angle(double angle) {
    while (angle > SCARA_PI) {
        angle -= SCARA_TWO_PI;
    }
    while (angle < -SCARA_PI) {
        angle += SCARA_TWO_PI;
    }
    return angle;
}

double angle_difference(double angle1, double angle2) {
    double diff = angle2 - angle1;
    return normalize_angle(diff);
}

/* ============================================================================
 * General Utilities
 * ========================================================================== */

double clamp(double value, double min, double max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

double lerp(double a, double b, double t) {
    return a + (b - a) * t;
}

double sign(double x) {
    if (x > 0.0) return 1.0;
    if (x < 0.0) return -1.0;
    return 0.0;
}

bool in_range(double value, double min, double max) {
    return (value >= min) && (value <= max);
}

bool double_equals(double a, double b, double tolerance) {
    return fabs(a - b) <= tolerance;
}

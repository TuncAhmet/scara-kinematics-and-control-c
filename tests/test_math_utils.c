/**
 * @file test_math_utils.c
 * @brief Unit tests for math utilities
 */

#include "unity/unity.h"
#include "scara/math_utils.h"
#include "scara/types.h"
#include <math.h>

void setUp(void) {}
void tearDown(void) {}

/* ============================================================================
 * Matrix Tests
 * ========================================================================== */

void test_matrix_identity(void) {
    Matrix4x4 I = matrix_identity();
    
    TEST_ASSERT_DOUBLE_WITHIN(1e-10, 1.0, I.m[0][0]);
    TEST_ASSERT_DOUBLE_WITHIN(1e-10, 1.0, I.m[1][1]);
    TEST_ASSERT_DOUBLE_WITHIN(1e-10, 1.0, I.m[2][2]);
    TEST_ASSERT_DOUBLE_WITHIN(1e-10, 1.0, I.m[3][3]);
    
    /* Off-diagonal should be zero */
    TEST_ASSERT_DOUBLE_WITHIN(1e-10, 0.0, I.m[0][1]);
    TEST_ASSERT_DOUBLE_WITHIN(1e-10, 0.0, I.m[1][0]);
    TEST_ASSERT_DOUBLE_WITHIN(1e-10, 0.0, I.m[2][3]);
}

void test_matrix_multiply_identity(void) {
    Matrix4x4 I = matrix_identity();
    Matrix4x4 A = matrix_identity();
    A.m[0][3] = 1.0;  /* Add some translation */
    A.m[1][3] = 2.0;
    A.m[2][3] = 3.0;
    
    Matrix4x4 result = matrix_multiply(&A, &I);
    
    /* A * I should equal A */
    TEST_ASSERT_TRUE(matrix_equals(&result, &A, 1e-10));
}

void test_matrix_multiply_rotation(void) {
    /* Test Rz(90°) * Rz(-90°) = I */
    Matrix4x4 Rz_pos, Rz_neg;
    double angle = SCARA_PI / 2.0;
    
    Rz_pos = matrix_identity();
    Rz_pos.m[0][0] = cos(angle);
    Rz_pos.m[0][1] = -sin(angle);
    Rz_pos.m[1][0] = sin(angle);
    Rz_pos.m[1][1] = cos(angle);
    
    Rz_neg = matrix_identity();
    Rz_neg.m[0][0] = cos(-angle);
    Rz_neg.m[0][1] = -sin(-angle);
    Rz_neg.m[1][0] = sin(-angle);
    Rz_neg.m[1][1] = cos(-angle);
    
    Matrix4x4 result = matrix_multiply(&Rz_pos, &Rz_neg);
    Matrix4x4 I = matrix_identity();
    
    TEST_ASSERT_TRUE(matrix_equals(&result, &I, 1e-10));
}

void test_matrix_vector_multiply(void) {
    Matrix4x4 T = matrix_identity();
    T.m[0][3] = 1.0;  /* Translate by (1, 2, 3) */
    T.m[1][3] = 2.0;
    T.m[2][3] = 3.0;
    
    Vector4 v = vector4_create(0.0, 0.0, 0.0, 1.0);
    Vector4 result = matrix_vector_multiply(&T, &v);
    
    TEST_ASSERT_DOUBLE_WITHIN(1e-10, 1.0, result.v[0]);
    TEST_ASSERT_DOUBLE_WITHIN(1e-10, 2.0, result.v[1]);
    TEST_ASSERT_DOUBLE_WITHIN(1e-10, 3.0, result.v[2]);
    TEST_ASSERT_DOUBLE_WITHIN(1e-10, 1.0, result.v[3]);
}

/* ============================================================================
 * Angle Tests
 * ========================================================================== */

void test_deg_to_rad(void) {
    TEST_ASSERT_DOUBLE_WITHIN(1e-10, 0.0, deg_to_rad(0.0));
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, SCARA_PI, deg_to_rad(180.0));
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, SCARA_PI / 2.0, deg_to_rad(90.0));
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, -SCARA_PI / 4.0, deg_to_rad(-45.0));
}

void test_rad_to_deg(void) {
    TEST_ASSERT_DOUBLE_WITHIN(1e-10, 0.0, rad_to_deg(0.0));
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 180.0, rad_to_deg(SCARA_PI));
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 90.0, rad_to_deg(SCARA_PI / 2.0));
}

void test_normalize_angle(void) {
    TEST_ASSERT_DOUBLE_WITHIN(1e-10, 0.0, normalize_angle(0.0));
    TEST_ASSERT_DOUBLE_WITHIN(1e-10, 0.0, normalize_angle(SCARA_TWO_PI));
    TEST_ASSERT_DOUBLE_WITHIN(1e-10, 0.0, normalize_angle(-SCARA_TWO_PI));
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, -SCARA_PI + 0.1, normalize_angle(SCARA_PI + 0.1));
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, SCARA_PI - 0.1, normalize_angle(-SCARA_PI - 0.1));
}

void test_angle_difference(void) {
    /* Same angle */
    TEST_ASSERT_DOUBLE_WITHIN(1e-10, 0.0, angle_difference(0.5, 0.5));
    
    /* Simple difference */
    TEST_ASSERT_DOUBLE_WITHIN(1e-10, 0.5, angle_difference(0.0, 0.5));
    TEST_ASSERT_DOUBLE_WITHIN(1e-10, -0.5, angle_difference(0.5, 0.0));
    
    /* Wrap around */
    double diff = angle_difference(-SCARA_PI + 0.1, SCARA_PI - 0.1);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, -0.2, diff);
}

/* ============================================================================
 * Utility Tests
 * ========================================================================== */

void test_clamp(void) {
    TEST_ASSERT_DOUBLE_WITHIN(1e-10, 0.5, clamp(0.5, 0.0, 1.0));
    TEST_ASSERT_DOUBLE_WITHIN(1e-10, 0.0, clamp(-0.5, 0.0, 1.0));
    TEST_ASSERT_DOUBLE_WITHIN(1e-10, 1.0, clamp(1.5, 0.0, 1.0));
}

void test_lerp(void) {
    TEST_ASSERT_DOUBLE_WITHIN(1e-10, 0.0, lerp(0.0, 1.0, 0.0));
    TEST_ASSERT_DOUBLE_WITHIN(1e-10, 1.0, lerp(0.0, 1.0, 1.0));
    TEST_ASSERT_DOUBLE_WITHIN(1e-10, 0.5, lerp(0.0, 1.0, 0.5));
    TEST_ASSERT_DOUBLE_WITHIN(1e-10, 5.0, lerp(0.0, 10.0, 0.5));
}

void test_sign(void) {
    TEST_ASSERT_DOUBLE_WITHIN(1e-10, 1.0, sign(5.0));
    TEST_ASSERT_DOUBLE_WITHIN(1e-10, -1.0, sign(-5.0));
    TEST_ASSERT_DOUBLE_WITHIN(1e-10, 0.0, sign(0.0));
}

void test_in_range(void) {
    TEST_ASSERT_TRUE(in_range(0.5, 0.0, 1.0));
    TEST_ASSERT_TRUE(in_range(0.0, 0.0, 1.0));
    TEST_ASSERT_TRUE(in_range(1.0, 0.0, 1.0));
    TEST_ASSERT_FALSE(in_range(-0.1, 0.0, 1.0));
    TEST_ASSERT_FALSE(in_range(1.1, 0.0, 1.0));
}

void test_double_equals(void) {
    TEST_ASSERT_TRUE(double_equals(1.0, 1.0, 1e-10));
    TEST_ASSERT_TRUE(double_equals(1.0, 1.0 + 1e-11, 1e-10));
    TEST_ASSERT_FALSE(double_equals(1.0, 1.1, 0.05));
}

/* ============================================================================
 * Main
 * ========================================================================== */

int main(void) {
    UnityBegin("test_math_utils.c");
    
    RUN_TEST(test_matrix_identity);
    RUN_TEST(test_matrix_multiply_identity);
    RUN_TEST(test_matrix_multiply_rotation);
    RUN_TEST(test_matrix_vector_multiply);
    
    RUN_TEST(test_deg_to_rad);
    RUN_TEST(test_rad_to_deg);
    RUN_TEST(test_normalize_angle);
    RUN_TEST(test_angle_difference);
    
    RUN_TEST(test_clamp);
    RUN_TEST(test_lerp);
    RUN_TEST(test_sign);
    RUN_TEST(test_in_range);
    RUN_TEST(test_double_equals);
    
    return UnityEnd();
}

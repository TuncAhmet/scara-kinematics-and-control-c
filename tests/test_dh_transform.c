/**
 * @file test_dh_transform.c
 * @brief Unit tests for D-H transformation
 */

#include "unity/unity.h"
#include "scara/dh_transform.h"
#include "scara/math_utils.h"
#include <math.h>

void setUp(void) {}
void tearDown(void) {}

/* ============================================================================
 * D-H Transform Tests
 * ========================================================================== */

void test_dh_identity(void) {
    /* D-H with all zeros should give identity */
    DHParams params = dh_params_create(0.0, 0.0, 0.0, 0.0);
    Matrix4x4 T = dh_transform(&params);
    Matrix4x4 I = matrix_identity();
    
    TEST_ASSERT_TRUE(matrix_equals(&T, &I, 1e-10));
}

void test_dh_pure_translation_z(void) {
    /* Pure translation along Z (d parameter) */
    DHParams params = dh_params_create(0.0, 0.0, 1.0, 0.0);
    Matrix4x4 T = dh_transform(&params);
    
    /* Should have translation in Z */
    TEST_ASSERT_DOUBLE_WITHIN(1e-10, 0.0, T.m[0][3]);
    TEST_ASSERT_DOUBLE_WITHIN(1e-10, 0.0, T.m[1][3]);
    TEST_ASSERT_DOUBLE_WITHIN(1e-10, 1.0, T.m[2][3]);
}

void test_dh_pure_translation_x(void) {
    /* Pure translation along X (a parameter) */
    DHParams params = dh_params_create(1.0, 0.0, 0.0, 0.0);
    Matrix4x4 T = dh_transform(&params);
    
    /* Should have translation in X */
    TEST_ASSERT_DOUBLE_WITHIN(1e-10, 1.0, T.m[0][3]);
    TEST_ASSERT_DOUBLE_WITHIN(1e-10, 0.0, T.m[1][3]);
    TEST_ASSERT_DOUBLE_WITHIN(1e-10, 0.0, T.m[2][3]);
}

void test_dh_rotation_z_90(void) {
    /* Rotation about Z by 90 degrees */
    DHParams params = dh_params_create(0.0, 0.0, 0.0, SCARA_PI / 2.0);
    Matrix4x4 T = dh_transform(&params);
    
    /* Rotation matrix should be Rz(90°) */
    TEST_ASSERT_DOUBLE_WITHIN(1e-10, 0.0, T.m[0][0]);   /* cos(90) */
    TEST_ASSERT_DOUBLE_WITHIN(1e-10, -1.0, T.m[0][1]);  /* -sin(90) */
    TEST_ASSERT_DOUBLE_WITHIN(1e-10, 1.0, T.m[1][0]);   /* sin(90) */
    TEST_ASSERT_DOUBLE_WITHIN(1e-10, 0.0, T.m[1][1]);   /* cos(90) */
}

void test_dh_combined(void) {
    /* Combined: a=1, alpha=0, d=0.5, theta=45° */
    double theta = deg_to_rad(45.0);
    DHParams params = dh_params_create(1.0, 0.0, 0.5, theta);
    Matrix4x4 T = dh_transform(&params);
    
    /* Position should be (a*cos(θ), a*sin(θ), d) */
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, cos(theta), T.m[0][3]);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, sin(theta), T.m[1][3]);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 0.5, T.m[2][3]);
}

/* ============================================================================
 * SCARA Config Tests
 * ========================================================================== */

void test_scara_config_default(void) {
    SCARAConfig cfg = scara_config_default();
    
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 0.30, cfg.L1);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 0.25, cfg.L2);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 0.40, cfg.d1);
    TEST_ASSERT_TRUE(cfg.theta1_max > cfg.theta1_min);
    TEST_ASSERT_TRUE(cfg.d3_max > cfg.d3_min);
}

void test_scara_joint_limits(void) {
    SCARAConfig cfg = scara_config_default();
    
    /* Valid joint state */
    JointState valid = {0.0, 0.0, 0.1, 0.0};
    TEST_ASSERT_TRUE(scara_check_joint_limits(&cfg, &valid));
    
    /* Invalid joint state (theta1 out of range) */
    JointState invalid = {10.0, 0.0, 0.1, 0.0};  /* 10 rad > limit */
    TEST_ASSERT_FALSE(scara_check_joint_limits(&cfg, &invalid));
    
    /* Invalid d3 (negative) */
    JointState invalid_d3 = {0.0, 0.0, -0.1, 0.0};
    TEST_ASSERT_FALSE(scara_check_joint_limits(&cfg, &invalid_d3));
}

void test_scara_clamp_joint_limits(void) {
    SCARAConfig cfg = scara_config_default();
    
    JointState joints = {10.0, -10.0, -0.1, 20.0};
    scara_clamp_joint_limits(&cfg, &joints);
    
    TEST_ASSERT_TRUE(scara_check_joint_limits(&cfg, &joints));
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, cfg.theta1_max, joints.theta1);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, cfg.theta2_min, joints.theta2);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, cfg.d3_min, joints.d3);
}

/* ============================================================================
 * Main
 * ========================================================================== */

int main(void) {
    UnityBegin("test_dh_transform.c");
    
    RUN_TEST(test_dh_identity);
    RUN_TEST(test_dh_pure_translation_z);
    RUN_TEST(test_dh_pure_translation_x);
    RUN_TEST(test_dh_rotation_z_90);
    RUN_TEST(test_dh_combined);
    
    RUN_TEST(test_scara_config_default);
    RUN_TEST(test_scara_joint_limits);
    RUN_TEST(test_scara_clamp_joint_limits);
    
    return UnityEnd();
}

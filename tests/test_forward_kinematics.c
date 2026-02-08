/**
 * @file test_forward_kinematics.c
 * @brief Unit tests for forward kinematics
 */

#include "unity/unity.h"
#include "scara/forward_kinematics.h"
#include "scara/dh_transform.h"
#include "scara/math_utils.h"
#include <math.h>

static SCARAConfig cfg;

void setUp(void) {
    cfg = scara_config_default();
}

void tearDown(void) {}

/* ============================================================================
 * Forward Kinematics Tests
 * ========================================================================== */

void test_fk_home_position(void) {
    /* Home position: all joints at zero */
    JointState joints = {0.0, 0.0, 0.0, 0.0};
    
    EndEffectorPose pose = fk_compute_pose_analytical(&cfg, &joints);
    
    /* At home: x = L1 + L2, y = 0, z = d1 */
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, cfg.L1 + cfg.L2, pose.x);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 0.0, pose.y);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, cfg.d1, pose.z);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 0.0, pose.yaw);
}

void test_fk_extended_along_y(void) {
    /* Theta1 = 90°, theta2 = 0: arm extends along +Y */
    JointState joints = {SCARA_PI / 2.0, 0.0, 0.0, 0.0};
    
    EndEffectorPose pose = fk_compute_pose_analytical(&cfg, &joints);
    
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 0.0, pose.x);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, cfg.L1 + cfg.L2, pose.y);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, cfg.d1, pose.z);
}

void test_fk_folded(void) {
    /* Theta2 = 180°: arm folds back */
    JointState joints = {0.0, SCARA_PI, 0.0, 0.0};
    
    EndEffectorPose pose = fk_compute_pose_analytical(&cfg, &joints);
    
    /* When folded: x = L1 - L2 */
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, cfg.L1 - cfg.L2, pose.x);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 0.0, pose.y);
}

void test_fk_prismatic_extension(void) {
    /* Test Z axis movement via d3 */
    JointState joints = {0.0, 0.0, 0.1, 0.0};
    
    EndEffectorPose pose = fk_compute_pose_analytical(&cfg, &joints);
    
    /* z = d1 - d3 */
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, cfg.d1 - 0.1, pose.z);
}

void test_fk_yaw_rotation(void) {
    /* Test that yaw = theta1 + theta2 + theta4 */
    JointState joints = {0.5, 0.3, 0.0, 0.2};
    
    EndEffectorPose pose = fk_compute_pose_analytical(&cfg, &joints);
    
    double expected_yaw = normalize_angle(0.5 + 0.3 + 0.2);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, expected_yaw, pose.yaw);
}

void test_fk_matrix_vs_analytical(void) {
    /* Compare matrix-based FK with analytical FK */
    JointState joints = {0.5, -0.3, 0.05, 0.1};
    
    EndEffectorPose pose_matrix = fk_compute_pose(&cfg, &joints);
    EndEffectorPose pose_analytical = fk_compute_pose_analytical(&cfg, &joints);
    
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, pose_matrix.x, pose_analytical.x);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, pose_matrix.y, pose_analytical.y);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, pose_matrix.z, pose_analytical.z);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, pose_matrix.yaw, pose_analytical.yaw);
}

void test_fk_joint_positions(void) {
    JointState joints = {0.0, SCARA_PI / 2.0, 0.0, 0.0};
    
    /* Base position */
    Vector3 p0 = fk_get_joint_position(&cfg, &joints, 0);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 0.0, p0.x);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 0.0, p0.y);
    
    /* Elbow position: (L1, 0) when theta1 = 0 */
    Vector3 p1 = fk_get_joint_position(&cfg, &joints, 1);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, cfg.L1, p1.x);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 0.0, p1.y);
    
    /* Wrist position when theta2 = 90° */
    Vector3 p2 = fk_get_joint_position(&cfg, &joints, 2);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, cfg.L1, p2.x);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, cfg.L2, p2.y);
}

/* ============================================================================
 * Main
 * ========================================================================== */

int main(void) {
    UnityBegin("test_forward_kinematics.c");
    
    RUN_TEST(test_fk_home_position);
    RUN_TEST(test_fk_extended_along_y);
    RUN_TEST(test_fk_folded);
    RUN_TEST(test_fk_prismatic_extension);
    RUN_TEST(test_fk_yaw_rotation);
    RUN_TEST(test_fk_matrix_vs_analytical);
    RUN_TEST(test_fk_joint_positions);
    
    return UnityEnd();
}

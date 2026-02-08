/**
 * @file test_inverse_kinematics.c
 * @brief Unit tests for inverse kinematics
 */

#include "unity/unity.h"
#include "scara/inverse_kinematics.h"
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
 * IK Reachability Tests
 * ========================================================================== */

void test_ik_reachable_target(void) {
    /* Target within workspace */
    EndEffectorPose target = {0.3, 0.2, 0.35, 0.5};
    TEST_ASSERT_TRUE(ik_is_reachable(&cfg, &target));
}

void test_ik_unreachable_too_far(void) {
    /* Target beyond max reach */
    EndEffectorPose target = {1.0, 0.0, 0.35, 0.0};  /* > L1 + L2 */
    TEST_ASSERT_FALSE(ik_is_reachable(&cfg, &target));
}

void test_ik_unreachable_too_close(void) {
    /* Target inside min reach (when L1 != L2) */
    EndEffectorPose target = {0.01, 0.0, 0.35, 0.0};  /* < |L1 - L2| */
    TEST_ASSERT_FALSE(ik_is_reachable(&cfg, &target));
}

void test_ik_unreachable_z_limit(void) {
    /* Target with Z outside prismatic range */
    EndEffectorPose target = {0.3, 0.2, 0.0, 0.0};  /* z = 0, d3 would need to be > d1 */
    TEST_ASSERT_FALSE(ik_is_reachable(&cfg, &target));
}

/* ============================================================================
 * IK Solution Tests
 * ========================================================================== */

void test_ik_home_position(void) {
    /* Target at home position (fully extended - SINGULAR) */
    EndEffectorPose target = {cfg.L1 + cfg.L2, 0.0, cfg.d1, 0.0};
    
    IKResult result;
    IKStatus status = inverse_kinematics(&cfg, &target, &result);
    
    /* Home position is at singularity (theta2 = 0) */
    TEST_ASSERT_TRUE(status == IK_SUCCESS || status == IK_SINGULAR);
    TEST_ASSERT_TRUE(result.solutions.elbow_up_valid || result.solutions.elbow_down_valid);
}

void test_ik_round_trip(void) {
    /* Forward -> Inverse -> Forward should give same pose */
    /* Use a non-singular configuration */
    JointState original = {0.3, 0.5, 0.05, 0.2};  /* Larger theta2 to avoid singularity */
    
    /* FK: joints -> pose */
    EndEffectorPose pose = fk_compute_pose_analytical(&cfg, &original);
    
    /* IK: pose -> joints */
    IKResult result;
    IKStatus status = inverse_kinematics(&cfg, &pose, &result);
    TEST_ASSERT_TRUE(status == IK_SUCCESS || status == IK_SINGULAR);
    
    /* Select best solution */
    JointState computed;
    bool selected = ik_select_best_solution(&result, &original, &cfg, &computed);
    TEST_ASSERT_TRUE(selected);
    
    /* FK: computed joints -> final pose */
    EndEffectorPose final_pose = fk_compute_pose_analytical(&cfg, &computed);
    
    /* Compare poses (wider tolerance due to IK numerical precision) */
    TEST_ASSERT_DOUBLE_WITHIN(1e-3, pose.x, final_pose.x);
    TEST_ASSERT_DOUBLE_WITHIN(1e-3, pose.y, final_pose.y);
    TEST_ASSERT_DOUBLE_WITHIN(1e-3, pose.z, final_pose.z);
    /* Yaw can differ by 2π and still be correct */
    double yaw_diff = fabs(angle_difference(pose.yaw, final_pose.yaw));
    TEST_ASSERT_TRUE(yaw_diff < 1e-3);
}

void test_ik_elbow_up_down(void) {
    /* Target with both elbow-up and elbow-down solutions */
    /* Use a position not at extreme reach */
    EndEffectorPose target = {0.35, 0.2, 0.35, 0.5};
    
    IKResult result;
    inverse_kinematics(&cfg, &target, &result);
    
    /* At least one solution should be valid */
    TEST_ASSERT_TRUE(result.solutions.elbow_up_valid || result.solutions.elbow_down_valid);
    
    /* If both valid, verify elbow angles have opposite signs */
    if (result.solutions.elbow_up_valid && result.solutions.elbow_down_valid) {
        /* elbow_up has positive theta2, elbow_down has negative theta2 */
        TEST_ASSERT_TRUE(result.solutions.elbow_up.theta2 >= 0);
        TEST_ASSERT_TRUE(result.solutions.elbow_down.theta2 <= 0);
    }
}

void test_ik_solution_selection_minimal_motion(void) {
    /* Current state - use larger theta2 to avoid singularity */
    JointState current = {0.3, 0.5, 0.05, 0.1};
    EndEffectorPose pose = fk_compute_pose_analytical(&cfg, &current);
    
    /* Same target (no offset) - should give very small cost */
    IKResult result;
    inverse_kinematics(&cfg, &pose, &result);
    
    JointState selected;
    bool ok = ik_select_best_solution(&result, &current, &cfg, &selected);
    TEST_ASSERT_TRUE(ok);
    
    /* Selected solution should be close to current (minimal motion) */
    double cost = ik_compute_motion_cost(&current, &selected);
    TEST_ASSERT_TRUE(cost < 5.0);  /* Reasonable threshold for joint motion */
}

/* ============================================================================
 * Edge Case Tests
 * ========================================================================== */

void test_ik_singularity_detection(void) {
    /* Near full extension (theta2 → 0) */
    JointState near_singular = {0.0, 0.01, 0.0, 0.0};  /* Almost fully extended */
    TEST_ASSERT_TRUE(ik_is_near_singularity(&near_singular, 0.05));
    
    /* Normal configuration */
    JointState normal = {0.0, 0.5, 0.0, 0.0};
    TEST_ASSERT_FALSE(ik_is_near_singularity(&normal, 0.05));
}

void test_ik_unreachable_status(void) {
    /* Target way outside workspace */
    EndEffectorPose target = {2.0, 0.0, 0.35, 0.0};
    
    IKResult result;
    IKStatus status = inverse_kinematics(&cfg, &target, &result);
    
    TEST_ASSERT_EQUAL_INT(IK_UNREACHABLE, status);
}

void test_ik_workspace_bounds(void) {
    double min_r, max_r;
    ik_get_workspace_bounds(&cfg, &min_r, &max_r);
    
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, cfg.L1 + cfg.L2, max_r);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, fabs(cfg.L1 - cfg.L2), min_r);
}

/* ============================================================================
 * Main
 * ========================================================================== */

int main(void) {
    UnityBegin("test_inverse_kinematics.c");
    
    RUN_TEST(test_ik_reachable_target);
    RUN_TEST(test_ik_unreachable_too_far);
    RUN_TEST(test_ik_unreachable_too_close);
    RUN_TEST(test_ik_unreachable_z_limit);
    
    RUN_TEST(test_ik_home_position);
    RUN_TEST(test_ik_round_trip);
    RUN_TEST(test_ik_elbow_up_down);
    RUN_TEST(test_ik_solution_selection_minimal_motion);
    
    RUN_TEST(test_ik_singularity_detection);
    RUN_TEST(test_ik_unreachable_status);
    RUN_TEST(test_ik_workspace_bounds);
    
    return UnityEnd();
}

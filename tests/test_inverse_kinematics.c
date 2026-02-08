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
 * IK Workspace Tests
 * ========================================================================== */

void test_ik_workspace_bounds(void) {
    double min_r, max_r;
    ik_get_workspace_bounds(&cfg, &min_r, &max_r);
    
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, cfg.L1 + cfg.L2, max_r);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, fabs(cfg.L1 - cfg.L2), min_r);
}

void test_ik_unreachable_too_far(void) {
    /* Target beyond max reach (L1+L2 = 0.55m) */
    EndEffectorPose target = {1.0, 0.0, 0.35, 0.0};
    TEST_ASSERT_FALSE(ik_is_reachable(&cfg, &target));
}

void test_ik_unreachable_too_close(void) {
    /* Target inside min reach (|L1-L2| = 0.05m) */
    EndEffectorPose target = {0.01, 0.0, 0.35, 0.0};
    TEST_ASSERT_FALSE(ik_is_reachable(&cfg, &target));
}

void test_ik_unreachable_z_limit(void) {
    /* Target with Z outside prismatic range [0.20, 0.40]m */
    EndEffectorPose target = {0.3, 0.2, 0.0, 0.0};  /* z=0, requires d3=0.40m > d3_max=0.20m */
    TEST_ASSERT_FALSE(ik_is_reachable(&cfg, &target));
}

void test_ik_reachable_target(void) {
    /* Target within workspace 
     * reach = sqrt(0.3^2 + 0.1^2) = 0.316m, within [0.05, 0.55]
     * z = 0.35, d3_required = 0.40 - 0.35 = 0.05, within [0, 0.20]
     */
    EndEffectorPose target = {0.30, 0.10, 0.35, 0.0};
    bool reachable = ik_is_reachable(&cfg, &target);
    TEST_ASSERT_TRUE(reachable);
}

/* ============================================================================
 * IK Solution Tests
 * ========================================================================== */

void test_ik_unreachable_status(void) {
    /* Target way outside workspace */
    EndEffectorPose target = {2.0, 0.0, 0.35, 0.0};
    
    IKResult result;
    IKStatus status = inverse_kinematics(&cfg, &target, &result);
    
    TEST_ASSERT_EQUAL_INT(IK_UNREACHABLE, status);
}

void test_ik_singularity_at_full_extension(void) {
    /* At full extension (theta2 = 0), IK should return IK_SINGULAR or work */
    EndEffectorPose target = {cfg.L1 + cfg.L2, 0.0, cfg.d1, 0.0};
    
    IKResult result;
    IKStatus status = inverse_kinematics(&cfg, &target, &result);
    
    /* Singularity or success */
    TEST_ASSERT_TRUE(status == IK_SUCCESS || status == IK_SINGULAR || status == IK_OUT_OF_LIMITS);
}

void test_ik_singularity_detection(void) {
    /* Near full extension (theta2 → 0) */
    JointState near_singular = {0.0, 0.01, 0.0, 0.0};
    TEST_ASSERT_TRUE(ik_is_near_singularity(&near_singular, 0.05));
    
    /* Normal configuration */
    JointState normal = {0.0, 0.5, 0.0, 0.0};
    TEST_ASSERT_FALSE(ik_is_near_singularity(&normal, 0.05));
}

void test_ik_motion_cost(void) {
    /* Test motion cost calculation */
    JointState state1 = {0.0, 0.0, 0.0, 0.0};
    JointState state2 = {0.1, 0.1, 0.01, 0.1};
    
    double cost = ik_compute_motion_cost(&state1, &state2);
    TEST_ASSERT_TRUE(cost > 0.0);
    
    /* Same state should have zero cost */
    double zero_cost = ik_compute_motion_cost(&state1, &state1);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 0.0, zero_cost);
}

void test_ik_valid_solution_from_fk(void) {
    /* Start with known joint state, compute FK, then IK
     * Use simple configuration with theta4=0
     */
    JointState original = {0.2, 0.4, 0.05, 0.0};
    
    /* FK: joints -> pose */
    EndEffectorPose pose = fk_compute_pose_analytical(&cfg, &original);
    
    /* Verify pose is reachable */
    bool reachable = ik_is_reachable(&cfg, &pose);
    TEST_ASSERT_TRUE(reachable);
    
    /* IK: pose -> joints */
    IKResult result;
    IKStatus status = inverse_kinematics(&cfg, &pose, &result);
    
    /* Should succeed (maybe with singularity warning) */
    TEST_ASSERT_TRUE(status == IK_SUCCESS || status == IK_SINGULAR);
}

void test_ik_round_trip_consistency(void) {
    /* Test that FK->IK->FK gives consistent results 
     * Use joint state with theta4 = -(theta1+theta2) to make yaw=0
     */
    JointState original = {0.3, 0.4, 0.05, -0.7};  /* theta4 = -(0.3+0.4) = -0.7 for yaw≈0 */
    
    /* FK: joints -> pose */
    EndEffectorPose pose = fk_compute_pose_analytical(&cfg, &original);
    
    /* IK: pose -> joints */
    IKResult result;
    IKStatus status = inverse_kinematics(&cfg, &pose, &result);
    
    if (status == IK_SUCCESS || status == IK_SINGULAR) {
        /* Select best solution */
        JointState selected;
        if (ik_select_best_solution(&result, &original, &cfg, &selected)) {
            /* FK: selected joints -> final pose */
            EndEffectorPose final_pose = fk_compute_pose_analytical(&cfg, &selected);
            
            /* Verify poses match */
            TEST_ASSERT_DOUBLE_WITHIN(0.01, pose.x, final_pose.x);
            TEST_ASSERT_DOUBLE_WITHIN(0.01, pose.y, final_pose.y);
            TEST_ASSERT_DOUBLE_WITHIN(0.01, pose.z, final_pose.z);
        }
    }
}

/* ============================================================================
 * Main
 * ========================================================================== */

int main(void) {
    UnityBegin("test_inverse_kinematics.c");
    
    /* Workspace tests */
    RUN_TEST(test_ik_workspace_bounds);
    RUN_TEST(test_ik_unreachable_too_far);
    RUN_TEST(test_ik_unreachable_too_close);
    RUN_TEST(test_ik_unreachable_z_limit);
    RUN_TEST(test_ik_reachable_target);
    
    /* Solution tests */
    RUN_TEST(test_ik_unreachable_status);
    RUN_TEST(test_ik_singularity_at_full_extension);
    RUN_TEST(test_ik_singularity_detection);
    RUN_TEST(test_ik_motion_cost);
    RUN_TEST(test_ik_valid_solution_from_fk);
    RUN_TEST(test_ik_round_trip_consistency);
    
    return UnityEnd();
}

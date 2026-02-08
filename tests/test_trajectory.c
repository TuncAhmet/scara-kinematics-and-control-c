/**
 * @file test_trajectory.c
 * @brief Unit tests for trajectory generation
 */

#include "unity/unity.h"
#include "scara/trajectory.h"
#include "scara/dh_transform.h"
#include "scara/math_utils.h"
#include <math.h>

static SCARAConfig cfg;

void setUp(void) {
    cfg = scara_config_default();
}

void tearDown(void) {}

/* ============================================================================
 * Basic Trajectory Tests
 * ========================================================================== */

void test_trajectory_zero_motion(void) {
    /* Start and end at same position */
    JointState start = {0.0, 0.0, 0.0, 0.0};
    JointState end = {0.0, 0.0, 0.0, 0.0};
    
    TrajectoryProfile traj = trajectory_plan_with_config(&cfg, &start, &end);
    
    TEST_ASSERT_TRUE(traj.is_complete);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 0.0, traj.duration);
}

void test_trajectory_single_axis(void) {
    /* Move only theta1 */
    JointState start = {0.0, 0.0, 0.0, 0.0};
    JointState end = {1.0, 0.0, 0.0, 0.0};
    
    TrajectoryProfile traj = trajectory_plan_with_config(&cfg, &start, &end);
    
    TEST_ASSERT_TRUE(traj.is_valid);
    TEST_ASSERT_TRUE(traj.duration > 0.0);
    TEST_ASSERT_FALSE(traj.is_complete);
}

void test_trajectory_start_point(void) {
    JointState start = {0.0, 0.5, 0.1, 0.0};
    JointState end = {1.0, 0.0, 0.0, 0.5};
    
    TrajectoryProfile traj = trajectory_plan_with_config(&cfg, &start, &end);
    
    /* At t=0, should be at start */
    TrajectoryPoint point = trajectory_evaluate(&traj, 0.0);
    
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, start.theta1, point.position.theta1);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, start.theta2, point.position.theta2);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, start.d3, point.position.d3);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, start.theta4, point.position.theta4);
}

void test_trajectory_end_point(void) {
    JointState start = {0.0, 0.5, 0.1, 0.0};
    JointState end = {1.0, 0.0, 0.0, 0.5};
    
    TrajectoryProfile traj = trajectory_plan_with_config(&cfg, &start, &end);
    
    /* At t=duration, should be at end */
    TrajectoryPoint point = trajectory_evaluate(&traj, traj.duration);
    
    TEST_ASSERT_DOUBLE_WITHIN(1e-4, end.theta1, point.position.theta1);
    TEST_ASSERT_DOUBLE_WITHIN(1e-4, end.theta2, point.position.theta2);
    TEST_ASSERT_DOUBLE_WITHIN(1e-4, end.d3, point.position.d3);
    TEST_ASSERT_DOUBLE_WITHIN(1e-4, end.theta4, point.position.theta4);
}

void test_trajectory_velocity_at_endpoints(void) {
    JointState start = {0.0, 0.0, 0.0, 0.0};
    JointState end = {1.0, 0.5, 0.1, 0.3};
    
    TrajectoryProfile traj = trajectory_plan_with_config(&cfg, &start, &end);
    
    /* Velocity should be zero at start */
    TrajectoryPoint p_start = trajectory_evaluate(&traj, 0.0);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 0.0, p_start.velocity.theta1);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 0.0, p_start.velocity.theta2);
    
    /* Velocity should be zero at end */
    TrajectoryPoint p_end = trajectory_evaluate(&traj, traj.duration);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 0.0, p_end.velocity.theta1);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 0.0, p_end.velocity.theta2);
}

/* ============================================================================
 * Profile Shape Tests
 * ========================================================================== */

void test_trajectory_monotonic(void) {
    JointState start = {0.0, 0.0, 0.0, 0.0};
    JointState end = {1.0, 0.0, 0.0, 0.0};
    
    TrajectoryProfile traj = trajectory_plan_with_config(&cfg, &start, &end);
    
    double prev_pos = 0.0;
    int steps = 100;
    
    for (int i = 1; i <= steps; i++) {
        double t = (double)i / steps * traj.duration;
        TrajectoryPoint point = trajectory_evaluate(&traj, t);
        
        /* Position should be monotonically increasing */
        TEST_ASSERT_TRUE(point.position.theta1 >= prev_pos - 1e-6);
        prev_pos = point.position.theta1;
    }
}

void test_trajectory_velocity_constraints(void) {
    JointState start = {0.0, 0.0, 0.0, 0.0};
    JointState end = {2.0, 1.0, 0.1, 1.0};  /* Large motion */
    
    TrajectoryProfile traj = trajectory_plan_with_config(&cfg, &start, &end);
    
    int steps = 100;
    
    for (int i = 0; i <= steps; i++) {
        double t = (double)i / steps * traj.duration;
        TrajectoryPoint point = trajectory_evaluate(&traj, t);
        
        /* Velocities should be within limits */
        TEST_ASSERT_TRUE(fabs(point.velocity.theta1) <= cfg.vel_theta1_max + 1e-6);
        TEST_ASSERT_TRUE(fabs(point.velocity.theta2) <= cfg.vel_theta2_max + 1e-6);
        TEST_ASSERT_TRUE(fabs(point.velocity.d3) <= cfg.vel_d3_max + 1e-6);
        TEST_ASSERT_TRUE(fabs(point.velocity.theta4) <= cfg.vel_theta4_max + 1e-6);
    }
}

void test_trajectory_phases(void) {
    JointState start = {0.0, 0.0, 0.0, 0.0};
    JointState end = {2.0, 0.0, 0.0, 0.0};  /* Large single-axis motion */
    
    TrajectoryProfile traj = trajectory_plan_with_config(&cfg, &start, &end);
    
    /* At start: acceleration phase */
    TrajectoryPoint p1 = trajectory_evaluate(&traj, 0.01);
    TEST_ASSERT_EQUAL_INT(TRAJ_PHASE_ACCEL, p1.phase);
    
    /* At end: complete */
    TrajectoryPoint p3 = trajectory_evaluate(&traj, traj.duration);
    TEST_ASSERT_EQUAL_INT(TRAJ_PHASE_COMPLETE, p3.phase);
}

/* ============================================================================
 * Trajectory Step/Progress Tests
 * ========================================================================== */

void test_trajectory_step(void) {
    JointState start = {0.0, 0.0, 0.0, 0.0};
    JointState end = {1.0, 0.0, 0.0, 0.0};
    
    TrajectoryProfile traj = trajectory_plan_with_config(&cfg, &start, &end);
    
    double dt = 0.01;
    int steps = (int)(traj.duration / dt) + 1;
    
    for (int i = 0; i < steps; i++) {
        trajectory_step(&traj, dt);
    }
    
    TEST_ASSERT_TRUE(trajectory_is_complete(&traj));
}

void test_trajectory_remaining_time(void) {
    JointState start = {0.0, 0.0, 0.0, 0.0};
    JointState end = {1.0, 0.0, 0.0, 0.0};
    
    TrajectoryProfile traj = trajectory_plan_with_config(&cfg, &start, &end);
    
    double initial_remaining = trajectory_remaining_time(&traj);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, traj.duration, initial_remaining);
    
    trajectory_step(&traj, 0.1);
    
    double new_remaining = trajectory_remaining_time(&traj);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, traj.duration - 0.1, new_remaining);
}

void test_trajectory_reset(void) {
    JointState start = {0.0, 0.0, 0.0, 0.0};
    JointState end = {1.0, 0.0, 0.0, 0.0};
    
    TrajectoryProfile traj = trajectory_plan_with_config(&cfg, &start, &end);
    
    /* Advance trajectory */
    trajectory_step(&traj, traj.duration);
    TEST_ASSERT_TRUE(trajectory_is_complete(&traj));
    
    /* Reset */
    trajectory_reset(&traj);
    TEST_ASSERT_FALSE(trajectory_is_complete(&traj));
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, traj.duration, trajectory_remaining_time(&traj));
}

void test_trajectory_compute_time(void) {
    double t = trajectory_compute_time(1.0, 1.0, 2.0);
    
    /* For trapezoidal: 
     * t_accel = v_max / a_max = 1 / 2 = 0.5
     * d_accel_decel = v_max * t_accel = 1 * 0.5 = 0.5
     * Since distance = 1 > 0.5, we have cruise phase
     * d_cruise = 1 - 0.5 = 0.5
     * t_cruise = d_cruise / v_max = 0.5 / 1 = 0.5
     * total = 2 * 0.5 + 0.5 = 1.5
     */
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 1.5, t);
}

/* ============================================================================
 * Main
 * ========================================================================== */

int main(void) {
    UnityBegin("test_trajectory.c");
    
    RUN_TEST(test_trajectory_zero_motion);
    RUN_TEST(test_trajectory_single_axis);
    RUN_TEST(test_trajectory_start_point);
    RUN_TEST(test_trajectory_end_point);
    RUN_TEST(test_trajectory_velocity_at_endpoints);
    
    RUN_TEST(test_trajectory_monotonic);
    RUN_TEST(test_trajectory_velocity_constraints);
    RUN_TEST(test_trajectory_phases);
    
    RUN_TEST(test_trajectory_step);
    RUN_TEST(test_trajectory_remaining_time);
    RUN_TEST(test_trajectory_reset);
    RUN_TEST(test_trajectory_compute_time);
    
    return UnityEnd();
}

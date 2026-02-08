/**
 * @file test_pid_controller.c
 * @brief Unit tests for PID controller
 */

#include "unity/unity.h"
#include "scara/pid_controller.h"
#include "scara/math_utils.h"
#include <math.h>

void setUp(void) {}
void tearDown(void) {}

/* ============================================================================
 * Basic PID Tests
 * ========================================================================== */

void test_pid_create(void) {
    PIDController pid = pid_create(1.0, 0.1, 0.01);
    
    TEST_ASSERT_DOUBLE_WITHIN(1e-10, 1.0, pid.Kp);
    TEST_ASSERT_DOUBLE_WITHIN(1e-10, 0.1, pid.Ki);
    TEST_ASSERT_DOUBLE_WITHIN(1e-10, 0.01, pid.Kd);
    TEST_ASSERT_DOUBLE_WITHIN(1e-10, 0.0, pid.integral);
    TEST_ASSERT_FALSE(pid.initialized);
}

void test_pid_proportional_only(void) {
    PIDController pid = pid_create(2.0, 0.0, 0.0);
    
    double output = pid_update(&pid, 10.0, 5.0, 0.01);
    
    /* Error = 10 - 5 = 5, output = Kp * error = 2 * 5 = 10 */
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 10.0, output);
}

void test_pid_integral_accumulation(void) {
    PIDController pid = pid_create(0.0, 10.0, 0.0);
    pid.integral_max = 100.0;  /* High limit */
    
    double dt = 0.1;
    double error = 1.0;  /* setpoint - measurement */
    
    pid_update(&pid, 1.0, 0.0, dt);  /* First update */
    pid_update(&pid, 1.0, 0.0, dt);  /* Second update */
    double output = pid_update(&pid, 1.0, 0.0, dt);  /* Third update */
    
    /* Integral should be 3 * error * dt = 3 * 1 * 0.1 = 0.3 */
    /* Output = Ki * integral = 10 * 0.3 = 3.0 */
    TEST_ASSERT_DOUBLE_WITHIN(1e-4, 3.0, output);
}

void test_pid_derivative_action(void) {
    PIDController pid = pid_create(0.0, 0.0, 1.0);
    
    double dt = 0.1;
    
    pid_update(&pid, 0.0, 0.0, dt);  /* First update (no derivative yet) */
    double output = pid_update(&pid, 1.0, 0.0, dt);  /* Error changes: 0 -> 1 */
    
    /* Derivative = (error - prev_error) / dt = (1 - 0) / 0.1 = 10 */
    /* Output = Kd * derivative = 1 * 10 = 10 */
    TEST_ASSERT_DOUBLE_WITHIN(1e-4, 10.0, output);
}

void test_pid_reset(void) {
    PIDController pid = pid_create(1.0, 1.0, 1.0);
    
    pid_update(&pid, 10.0, 0.0, 0.1);
    pid_update(&pid, 10.0, 5.0, 0.1);
    
    TEST_ASSERT_TRUE(pid.initialized);
    TEST_ASSERT_TRUE(pid.integral != 0.0);
    
    pid_reset(&pid);
    
    TEST_ASSERT_FALSE(pid.initialized);
    TEST_ASSERT_DOUBLE_WITHIN(1e-10, 0.0, pid.integral);
    TEST_ASSERT_DOUBLE_WITHIN(1e-10, 0.0, pid.prev_error);
}

/* ============================================================================
 * Saturation and Anti-Windup Tests
 * ========================================================================== */

void test_pid_output_saturation(void) {
    PIDController pid = pid_create_full(10.0, 0.0, 0.0, -5.0, 5.0, 10.0, 0.0);
    
    /* Large error should saturate output */
    double output = pid_update(&pid, 100.0, 0.0, 0.1);
    
    TEST_ASSERT_DOUBLE_WITHIN(1e-10, 5.0, output);
    TEST_ASSERT_TRUE(pid.is_saturated);
    
    /* Negative saturation */
    output = pid_update(&pid, -100.0, 0.0, 0.1);
    TEST_ASSERT_DOUBLE_WITHIN(1e-10, -5.0, output);
}

void test_pid_anti_windup(void) {
    PIDController pid = pid_create_full(0.0, 1.0, 0.0, -10.0, 10.0, 2.0, 0.0);
    
    double dt = 0.1;
    
    /* Accumulate integral */
    for (int i = 0; i < 100; i++) {
        pid_update(&pid, 100.0, 0.0, dt);
    }
    
    /* Integral should be clamped */
    TEST_ASSERT_TRUE(fabs(pid.integral) <= 2.0 + 1e-6);
}

void test_pid_rate_limiting(void) {
    PIDController pid = pid_create_full(100.0, 0.0, 0.0, -100.0, 100.0, 10.0, 50.0);
    
    double dt = 0.1;
    
    /* First update: from 0 */
    pid_update(&pid, 0.0, 0.0, dt);
    
    /* Second update: large step */
    double output = pid_update(&pid, 10.0, 0.0, dt);
    
    /* Rate limit: max change = 50 * 0.1 = 5 per step */
    TEST_ASSERT_TRUE(fabs(output) <= 5.0 + 1e-6);
}

/* ============================================================================
 * Step Response Test
 * ========================================================================== */

void test_pid_step_response_convergence(void) {
    PIDController pid = pid_create_full(5.0, 1.0, 0.5, -10.0, 10.0, 5.0, 0.0);
    
    double dt = 0.01;
    double setpoint = 1.0;
    double measurement = 0.0;
    double tau = 0.1;  /* Time constant for first-order plant */
    
    /* Simulate closed-loop for 2 seconds */
    for (int i = 0; i < 200; i++) {
        double output = pid_update(&pid, setpoint, measurement, dt);
        
        /* Simple first-order plant model */
        measurement += (output - measurement) * dt / tau;
    }
    
    /* Should converge close to setpoint */
    TEST_ASSERT_DOUBLE_WITHIN(0.1, setpoint, measurement);
}

/* ============================================================================
 * Joint PID Controllers Tests
 * ========================================================================== */

void test_joint_pid_create(void) {
    JointPIDControllers pids = joint_pid_create_default();
    
    TEST_ASSERT_TRUE(pids.theta1.Kp > 0.0);
    TEST_ASSERT_TRUE(pids.theta2.Kp > 0.0);
    TEST_ASSERT_TRUE(pids.d3.Kp > 0.0);
    TEST_ASSERT_TRUE(pids.theta4.Kp > 0.0);
}

void test_joint_pid_update(void) {
    JointPIDControllers pids = joint_pid_create_default();
    
    JointState setpoint = {1.0, 0.5, 0.1, 0.3};
    JointState measurement = {0.0, 0.0, 0.0, 0.0};
    double dt = 0.01;
    
    JointState output = joint_pid_update(&pids, &setpoint, &measurement, dt);
    
    /* All outputs should be positive (trying to increase values) */
    TEST_ASSERT_TRUE(output.theta1 > 0.0);
    TEST_ASSERT_TRUE(output.theta2 > 0.0);
    TEST_ASSERT_TRUE(output.d3 > 0.0);
    TEST_ASSERT_TRUE(output.theta4 > 0.0);
}

void test_joint_pid_reset(void) {
    JointPIDControllers pids = joint_pid_create_default();
    
    JointState setpoint = {1.0, 0.5, 0.1, 0.3};
    JointState measurement = {0.0, 0.0, 0.0, 0.0};
    
    joint_pid_update(&pids, &setpoint, &measurement, 0.01);
    
    joint_pid_reset(&pids);
    
    TEST_ASSERT_FALSE(pids.theta1.initialized);
    TEST_ASSERT_FALSE(pids.theta2.initialized);
    TEST_ASSERT_FALSE(pids.d3.initialized);
    TEST_ASSERT_FALSE(pids.theta4.initialized);
}

/* ============================================================================
 * Main
 * ========================================================================== */

int main(void) {
    UnityBegin("test_pid_controller.c");
    
    RUN_TEST(test_pid_create);
    RUN_TEST(test_pid_proportional_only);
    RUN_TEST(test_pid_integral_accumulation);
    RUN_TEST(test_pid_derivative_action);
    RUN_TEST(test_pid_reset);
    
    RUN_TEST(test_pid_output_saturation);
    RUN_TEST(test_pid_anti_windup);
    RUN_TEST(test_pid_rate_limiting);
    
    RUN_TEST(test_pid_step_response_convergence);
    
    RUN_TEST(test_joint_pid_create);
    RUN_TEST(test_joint_pid_update);
    RUN_TEST(test_joint_pid_reset);
    
    return UnityEnd();
}

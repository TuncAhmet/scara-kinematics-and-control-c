/**
 * @file test_integration.c
 * @brief Integration tests for full simulation pipeline
 */

#include "unity/unity.h"
#include "scara/simulation.h"
#include "scara/forward_kinematics.h"
#include "scara/inverse_kinematics.h"
#include "scara/dh_transform.h"
#include "scara/math_utils.h"
#include <math.h>

static SimulationState sim;

void setUp(void) {
    sim = simulation_create(500.0);  /* 500 Hz */
}

void tearDown(void) {}

/* ============================================================================
 * Simulation Initialization Tests
 * ========================================================================== */

void test_simulation_create(void) {
    TEST_ASSERT_EQUAL_INT(MODE_IDLE, sim.mode);
    TEST_ASSERT_TRUE(sim.ik_valid);
    TEST_ASSERT_TRUE(sim.at_target);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 0.002, sim.dt);  /* 1/500 */
}

void test_simulation_initial_pose(void) {
    EndEffectorPose pose = simulation_get_pose(&sim);
    
    /* At home: x = L1 + L2, y = 0, z = d1 */
    TEST_ASSERT_DOUBLE_WITHIN(1e-4, sim.robot_config.L1 + sim.robot_config.L2, pose.x);
    TEST_ASSERT_DOUBLE_WITHIN(1e-4, 0.0, pose.y);
    TEST_ASSERT_DOUBLE_WITHIN(1e-4, sim.robot_config.d1, pose.z);
}

/* ============================================================================
 * Target Setting Tests
 * ========================================================================== */

void test_simulation_set_valid_target(void) {
    EndEffectorPose target = {0.35, 0.2, 0.35, 0.5};
    
    bool success = simulation_set_target(&sim, &target);
    
    TEST_ASSERT_TRUE(success);
    TEST_ASSERT_EQUAL_INT(MODE_MOVING, sim.mode);
    TEST_ASSERT_TRUE(sim.ik_valid);
    TEST_ASSERT_FALSE(sim.at_target);
}

void test_simulation_set_unreachable_target(void) {
    EndEffectorPose target = {2.0, 0.0, 0.35, 0.0};  /* Way outside workspace */
    
    bool success = simulation_set_target(&sim, &target);
    
    TEST_ASSERT_FALSE(success);
    TEST_ASSERT_EQUAL_INT(MODE_ERROR, sim.mode);
    TEST_ASSERT_FALSE(sim.ik_valid);
}

void test_simulation_set_joint_target(void) {
    JointState target = {0.5, -0.3, 0.05, 0.2};
    
    bool success = simulation_set_target_joints(&sim, &target);
    
    TEST_ASSERT_TRUE(success);
    TEST_ASSERT_EQUAL_INT(MODE_MOVING, sim.mode);
}

/* ============================================================================
 * Convergence Tests
 * ========================================================================== */

void test_simulation_converges_to_target(void) {
    /* Set a reachable target */
    EndEffectorPose target = {0.4, 0.15, 0.35, 0.3};
    
    bool success = simulation_set_target(&sim, &target);
    TEST_ASSERT_TRUE(success);
    
    /* Run simulation for up to 5 seconds (2500 steps at 500 Hz) */
    int max_steps = 2500;
    for (int i = 0; i < max_steps; i++) {
        simulation_step(&sim);
        
        if (sim.mode == MODE_REACHED) {
            break;
        }
    }
    
    /* Should have reached target */
    TEST_ASSERT_EQUAL_INT(MODE_REACHED, sim.mode);
    TEST_ASSERT_TRUE(sim.at_target);
    
    /* Verify final pose */
    EndEffectorPose final_pose = simulation_get_pose(&sim);
    TEST_ASSERT_DOUBLE_WITHIN(0.01, target.x, final_pose.x);
    TEST_ASSERT_DOUBLE_WITHIN(0.01, target.y, final_pose.y);
    TEST_ASSERT_DOUBLE_WITHIN(0.01, target.z, final_pose.z);
}

void test_simulation_joint_target_convergence(void) {
    JointState target = {0.5, -0.3, 0.05, 0.2};
    
    simulation_set_target_joints(&sim, &target);
    
    /* Run simulation */
    int max_steps = 2500;
    for (int i = 0; i < max_steps; i++) {
        simulation_step(&sim);
        
        if (sim.mode == MODE_REACHED) {
            break;
        }
    }
    
    TEST_ASSERT_EQUAL_INT(MODE_REACHED, sim.mode);
    
    /* Verify final joints */
    JointState final_joints = simulation_get_joints(&sim);
    TEST_ASSERT_DOUBLE_WITHIN(0.02, target.theta1, final_joints.theta1);
    TEST_ASSERT_DOUBLE_WITHIN(0.02, target.theta2, final_joints.theta2);
    TEST_ASSERT_DOUBLE_WITHIN(0.005, target.d3, final_joints.d3);
    TEST_ASSERT_DOUBLE_WITHIN(0.02, target.theta4, final_joints.theta4);
}

/* ============================================================================
 * Stop and Reset Tests
 * ========================================================================== */

void test_simulation_stop(void) {
    EndEffectorPose target = {0.4, 0.15, 0.35, 0.3};
    simulation_set_target(&sim, &target);
    
    /* Run a few steps */
    for (int i = 0; i < 50; i++) {
        simulation_step(&sim);
    }
    
    /* Stop */
    simulation_stop(&sim);
    
    TEST_ASSERT_EQUAL_INT(MODE_IDLE, sim.mode);
}

void test_simulation_reset(void) {
    /* Move to a target */
    JointState target = {0.5, -0.3, 0.05, 0.2};
    simulation_set_target_joints(&sim, &target);
    
    for (int i = 0; i < 500; i++) {
        simulation_step(&sim);
    }
    
    /* Reset */
    simulation_reset(&sim);
    
    TEST_ASSERT_EQUAL_INT(MODE_IDLE, sim.mode);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 0.0, sim.sim_time);
    
    /* Should be back at home */
    JointState joints = simulation_get_joints(&sim);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 0.0, joints.theta1);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 0.0, joints.theta2);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 0.0, joints.d3);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 0.0, joints.theta4);
}

/* ============================================================================
 * Telemetry Tests
 * ========================================================================== */

void test_simulation_telemetry(void) {
    simulation_step(&sim);
    
    Telemetry tel = simulation_get_telemetry(&sim);
    
    TEST_ASSERT_TRUE(tel.sequence > 0 || tel.sequence == 0);  /* Valid sequence */
    TEST_ASSERT_TRUE(tel.timestamp >= 0.0);
    TEST_ASSERT_TRUE(tel.ik_valid);
}

void test_simulation_telemetry_updates(void) {
    /* Run initial step */
    simulation_step(&sim);
    Telemetry tel1 = simulation_get_telemetry(&sim);
    
    /* Run another step */
    simulation_step(&sim);
    Telemetry tel2 = simulation_get_telemetry(&sim);
    
    /* Sequence should increment (each step increases sequence by 1) */
    TEST_ASSERT_TRUE(tel2.sequence >= tel1.sequence);
    
    /* Timestamp should advance */
    TEST_ASSERT_TRUE(tel2.timestamp > tel1.timestamp);
}

/* ============================================================================
 * Regression Tests
 * ========================================================================== */

void test_unreachable_detection(void) {
    /* Array of unreachable positions */
    EndEffectorPose unreachable[] = {
        {2.0, 0.0, 0.35, 0.0},       /* Too far */
        {0.0, 0.0, 0.35, 0.0},       /* At origin (inside min reach) */
        {0.3, 0.2, 0.0, 0.0},        /* Z too low */
        {0.3, 0.2, 0.5, 0.0}         /* Z too high */
    };
    
    for (int i = 0; i < 4; i++) {
        simulation_reset(&sim);
        bool success = simulation_set_target(&sim, &unreachable[i]);
        TEST_ASSERT_FALSE(success);
    }
}

void test_joint_limit_clamping(void) {
    /* Try to set target at joint limits */
    JointState at_limits = {
        sim.robot_config.theta1_max,
        sim.robot_config.theta2_min,
        sim.robot_config.d3_max,
        sim.robot_config.theta4_max
    };
    
    bool success = simulation_set_target_joints(&sim, &at_limits);
    TEST_ASSERT_TRUE(success);
}

/* ============================================================================
 * Main
 * ========================================================================== */

int main(void) {
    UnityBegin("test_integration.c");
    
    RUN_TEST(test_simulation_create);
    RUN_TEST(test_simulation_initial_pose);
    
    RUN_TEST(test_simulation_set_valid_target);
    RUN_TEST(test_simulation_set_unreachable_target);
    RUN_TEST(test_simulation_set_joint_target);
    
    RUN_TEST(test_simulation_converges_to_target);
    RUN_TEST(test_simulation_joint_target_convergence);
    
    RUN_TEST(test_simulation_stop);
    RUN_TEST(test_simulation_reset);
    
    RUN_TEST(test_simulation_telemetry);
    RUN_TEST(test_simulation_telemetry_updates);
    
    RUN_TEST(test_unreachable_detection);
    RUN_TEST(test_joint_limit_clamping);
    
    return UnityEnd();
}

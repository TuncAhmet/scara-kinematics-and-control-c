/**
 * @file simulation.c
 * @brief Real-time simulation loop and state management
 */

#include "scara/simulation.h"
#include "scara/forward_kinematics.h"
#include "scara/inverse_kinematics.h"
#include "scara/dh_transform.h"
#include "scara/math_utils.h"
#include <string.h>
#include <math.h>

/* Convergence tolerances for "at target" detection */
#define DEFAULT_POSITION_TOL    0.001   /* 1 mm */
#define DEFAULT_ANGLE_TOL       0.01    /* ~0.5 degrees */

SimulationState simulation_create(double loop_hz) {
    SCARAConfig cfg = scara_config_default();
    return simulation_create_with_config(&cfg, loop_hz);
}

SimulationState simulation_create_with_config(const SCARAConfig* cfg, double loop_hz) {
    SimulationState sim;
    memset(&sim, 0, sizeof(sim));
    
    sim.robot_config = *cfg;
    sim.plant = plant_create_default();
    sim.pid = joint_pid_create_default();
    
    sim.loop_rate_hz = loop_hz;
    sim.dt = 1.0 / loop_hz;
    sim.sim_time = 0.0;
    
    sim.mode = MODE_IDLE;
    sim.has_target = false;
    sim.ik_valid = true;
    sim.at_target = true;
    
    sim.telemetry_seq = 0;
    
    /* Initialize trajectory as empty */
    sim.trajectory.is_valid = false;
    sim.trajectory.is_complete = true;
    
    return sim;
}

bool simulation_set_target(SimulationState* sim, const EndEffectorPose* target) {
    /* Compute IK */
    IKResult ik_result;
    IKStatus status = inverse_kinematics(&sim->robot_config, target, &ik_result);
    
    if (status == IK_UNREACHABLE || status == IK_NO_VALID_SOLUTION) {
        sim->ik_valid = false;
        sim->mode = MODE_ERROR;
        return false;
    }
    
    /* Get current joint state */
    JointState current = plant_get_position(&sim->plant);
    
    /* Select best IK solution */
    JointState selected;
    if (!ik_select_best_solution(&ik_result, &current, &sim->robot_config, &selected)) {
        sim->ik_valid = false;
        sim->mode = MODE_ERROR;
        return false;
    }
    
    sim->ik_valid = true;
    sim->target_pose = *target;
    sim->target_joints = selected;
    sim->has_target = true;
    
    /* Plan trajectory */
    sim->trajectory = trajectory_plan_with_config(&sim->robot_config, &current, &selected);
    
    /* Reset PID controllers for new motion */
    joint_pid_reset(&sim->pid);
    
    sim->mode = MODE_MOVING;
    sim->at_target = false;
    
    return true;
}

bool simulation_set_target_joints(SimulationState* sim, const JointState* target) {
    /* Check joint limits */
    if (!scara_check_joint_limits(&sim->robot_config, target)) {
        sim->mode = MODE_ERROR;
        return false;
    }
    
    /* Get current joint state */
    JointState current = plant_get_position(&sim->plant);
    
    sim->target_joints = *target;
    sim->has_target = true;
    sim->ik_valid = true;  /* Not applicable for direct joint control */
    
    /* Compute target pose using FK */
    sim->target_pose = fk_compute_pose_analytical(&sim->robot_config, target);
    
    /* Plan trajectory */
    sim->trajectory = trajectory_plan_with_config(&sim->robot_config, &current, target);
    
    /* Reset PID controllers */
    joint_pid_reset(&sim->pid);
    
    sim->mode = MODE_MOVING;
    sim->at_target = false;
    
    return true;
}

void simulation_step(SimulationState* sim) {
    /* Get current plant state */
    JointState current_joints = plant_get_position(&sim->plant);
    
    /* Initialize setpoint based on mode */
    JointState setpoint;
    
    if (sim->mode == MODE_REACHED && sim->has_target) {
        /* Hold at target position to prevent drift */
        setpoint = sim->target_joints;
    } else {
        /* Default to current position (for IDLE/ERROR modes) */
        setpoint = current_joints;
    }
    
    if (sim->mode == MODE_MOVING && sim->trajectory.is_valid) {
        /* Advance trajectory time */
        trajectory_step(&sim->trajectory, sim->dt);
        
        /* Get trajectory setpoint */
        TrajectoryPoint traj_point = trajectory_get_current(&sim->trajectory);
        setpoint = traj_point.position;
        
        /* Update telemetry trajectory info */
        sim->telemetry.traj_phase = traj_point.phase;
        if (sim->trajectory.duration > 0.0) {
            sim->telemetry.traj_progress = sim->trajectory.elapsed / sim->trajectory.duration;
        } else {
            sim->telemetry.traj_progress = 1.0;
        }
        
        /* Check if trajectory complete */
        if (trajectory_is_complete(&sim->trajectory)) {
            setpoint = sim->target_joints;  /* Use exact target at end */
        }
    }
    
    /* Compute PID outputs */
    JointState pid_output = joint_pid_update(&sim->pid, &setpoint, &current_joints, sim->dt);
    
    /* Update plant with PID output (velocity command mode) */
    plant_update(&sim->plant, &pid_output, sim->dt);
    
    /* Get updated state */
    current_joints = plant_get_position(&sim->plant);
    
    /* Compute forward kinematics for telemetry */
    EndEffectorPose current_pose = fk_compute_pose_analytical(&sim->robot_config, &current_joints);
    
    /* Compute tracking errors */
    JointState errors;
    errors.theta1 = angle_difference(current_joints.theta1, setpoint.theta1);
    errors.theta2 = angle_difference(current_joints.theta2, setpoint.theta2);
    errors.d3 = setpoint.d3 - current_joints.d3;
    errors.theta4 = angle_difference(current_joints.theta4, setpoint.theta4);
    
    /* Check if at target */
    if (sim->mode == MODE_MOVING && trajectory_is_complete(&sim->trajectory)) {
        bool at_target = simulation_at_target(sim, DEFAULT_POSITION_TOL, DEFAULT_ANGLE_TOL);
        if (at_target) {
            sim->mode = MODE_REACHED;
            sim->at_target = true;
        }
    }
    
    /* Update telemetry */
    sim->telemetry.sequence = sim->telemetry_seq++;
    sim->telemetry.timestamp = sim->sim_time;
    sim->telemetry.joints = current_joints;
    sim->telemetry.joint_velocities = plant_get_velocity(&sim->plant);
    sim->telemetry.pose = current_pose;
    sim->telemetry.setpoints = setpoint;
    sim->telemetry.errors = errors;
    sim->telemetry.pid_outputs = pid_output;
    sim->telemetry.mode = sim->mode;
    sim->telemetry.ik_valid = sim->ik_valid;
    sim->telemetry.at_target = sim->at_target;
    sim->telemetry.saturated[0] = sim->pid.theta1.is_saturated;
    sim->telemetry.saturated[1] = sim->pid.theta2.is_saturated;
    sim->telemetry.saturated[2] = sim->pid.d3.is_saturated;
    sim->telemetry.saturated[3] = sim->pid.theta4.is_saturated;
    
    /* Advance simulation time */
    sim->sim_time += sim->dt;
}

Telemetry simulation_get_telemetry(SimulationState* sim) {
    return sim->telemetry;
}

void simulation_stop(SimulationState* sim) {
    sim->mode = MODE_IDLE;
    sim->trajectory.is_complete = true;
    joint_pid_reset(&sim->pid);
}

void simulation_reset(SimulationState* sim) {
    /* Reset to home position (all zeros) */
    JointState home = {0.0, 0.0, 0.0, 0.0};
    plant_reset(&sim->plant, &home);
    joint_pid_reset(&sim->pid);
    
    sim->mode = MODE_IDLE;
    sim->has_target = false;
    sim->ik_valid = true;
    sim->at_target = true;
    sim->sim_time = 0.0;
    sim->telemetry_seq = 0;
    
    sim->trajectory.is_valid = false;
    sim->trajectory.is_complete = true;
}

bool simulation_at_target(const SimulationState* sim, 
                          double position_tolerance, double angle_tolerance) {
    if (!sim->has_target) {
        return true;  /* No target = at target */
    }
    
    JointState current = plant_get_position(&sim->plant);
    
    /* Check angular joints */
    if (fabs(angle_difference(current.theta1, sim->target_joints.theta1)) > angle_tolerance) {
        return false;
    }
    if (fabs(angle_difference(current.theta2, sim->target_joints.theta2)) > angle_tolerance) {
        return false;
    }
    if (fabs(angle_difference(current.theta4, sim->target_joints.theta4)) > angle_tolerance) {
        return false;
    }
    
    /* Check prismatic joint */
    if (fabs(current.d3 - sim->target_joints.d3) > position_tolerance) {
        return false;
    }
    
    return true;
}

EndEffectorPose simulation_get_pose(const SimulationState* sim) {
    JointState joints = plant_get_position(&sim->plant);
    return fk_compute_pose_analytical(&sim->robot_config, &joints);
}

JointState simulation_get_joints(const SimulationState* sim) {
    return plant_get_position(&sim->plant);
}

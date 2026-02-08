/**
 * @file simulation.h
 * @brief Real-time simulation loop and state management
 */

#ifndef SCARA_SIMULATION_H
#define SCARA_SIMULATION_H

#include "types.h"
#include "pid_controller.h"
#include "trajectory.h"
#include "plant_model.h"

/**
 * @brief Telemetry data structure (streamed to UI)
 */
typedef struct {
    uint32_t sequence;              /**< Sequence number */
    double timestamp;               /**< Simulation time [seconds] */
    
    /* State */
    JointState joints;              /**< Current joint positions */
    JointState joint_velocities;    /**< Current joint velocities */
    EndEffectorPose pose;           /**< Current end-effector pose */
    
    /* Control */
    JointState setpoints;           /**< Current trajectory setpoints */
    JointState errors;              /**< Joint tracking errors */
    JointState pid_outputs;         /**< PID controller outputs */
    
    /* Status */
    SimMode mode;                   /**< Current simulation mode */
    bool ik_valid;                  /**< IK solution validity */
    bool at_target;                 /**< True if at target position */
    bool saturated[4];              /**< PID saturation flags */
    TrajectoryPhase traj_phase;     /**< Current trajectory phase */
    double traj_progress;           /**< Trajectory progress [0, 1] */
} Telemetry;

/**
 * @brief Main simulation state
 */
typedef struct {
    /* Robot configuration */
    SCARAConfig robot_config;
    
    /* Control components */
    PlantState plant;
    JointPIDControllers pid;
    TrajectoryProfile trajectory;
    
    /* Target */
    EndEffectorPose target_pose;
    JointState target_joints;
    bool has_target;
    
    /* Timing */
    double sim_time;
    double dt;                      /**< Fixed time step (e.g., 1/500 for 500 Hz) */
    double loop_rate_hz;
    
    /* Status */
    SimMode mode;
    bool ik_valid;
    bool at_target;
    
    /* Telemetry */
    Telemetry telemetry;
    uint32_t telemetry_seq;
} SimulationState;

/**
 * @brief Initialize simulation with default configuration
 * @param loop_hz Control loop rate [Hz]
 * @return Initialized simulation state
 */
SimulationState simulation_create(double loop_hz);

/**
 * @brief Initialize simulation with custom robot configuration
 * @param cfg Robot configuration
 * @param loop_hz Control loop rate [Hz]
 * @return Initialized simulation state
 */
SimulationState simulation_create_with_config(const SCARAConfig* cfg, double loop_hz);

/**
 * @brief Set target pose (commands IK + trajectory planning)
 * 
 * This function:
 * 1. Computes IK for the target pose
 * 2. Selects the best solution based on current position
 * 3. Plans a trajectory from current to target
 * 4. Sets mode to MODE_MOVING
 * 
 * @param sim Simulation state
 * @param target Target end-effector pose
 * @return true if target is reachable and trajectory planned
 */
bool simulation_set_target(SimulationState* sim, const EndEffectorPose* target);

/**
 * @brief Set target joint state directly (bypasses IK)
 * @param sim Simulation state
 * @param target Target joint state
 * @return true if target is within limits
 */
bool simulation_set_target_joints(SimulationState* sim, const JointState* target);

/**
 * @brief Execute one simulation step
 * 
 * This function runs at the control loop rate and:
 * 1. Evaluates the trajectory for current setpoint
 * 2. Updates PID controllers
 * 3. Updates plant model
 * 4. Computes forward kinematics
 * 5. Updates telemetry
 * 
 * @param sim Simulation state
 */
void simulation_step(SimulationState* sim);

/**
 * @brief Get current telemetry
 * @param sim Simulation state
 * @return Current telemetry data
 */
Telemetry simulation_get_telemetry(SimulationState* sim);

/**
 * @brief Stop current motion and hold position
 * @param sim Simulation state
 */
void simulation_stop(SimulationState* sim);

/**
 * @brief Reset simulation to home position
 * @param sim Simulation state
 */
void simulation_reset(SimulationState* sim);

/**
 * @brief Check if simulation has reached target
 * @param sim Simulation state
 * @param position_tolerance Position tolerance [m]
 * @param angle_tolerance Angle tolerance [rad]
 * @return true if at target within tolerances
 */
bool simulation_at_target(const SimulationState* sim, 
                          double position_tolerance, double angle_tolerance);

/**
 * @brief Get current end-effector pose
 * @param sim Simulation state
 * @return Current pose
 */
EndEffectorPose simulation_get_pose(const SimulationState* sim);

/**
 * @brief Get current joint state
 * @param sim Simulation state
 * @return Current joint positions
 */
JointState simulation_get_joints(const SimulationState* sim);

#endif /* SCARA_SIMULATION_H */

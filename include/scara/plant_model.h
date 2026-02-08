/**
 * @file plant_model.h
 * @brief First-order motor/actuator dynamics simulation
 */

#ifndef SCARA_PLANT_MODEL_H
#define SCARA_PLANT_MODEL_H

#include "types.h"

/**
 * @brief Plant state for simulating motor dynamics
 */
typedef struct {
    JointState position;            /**< Current joint positions */
    JointState velocity;            /**< Current joint velocities */
    double time_constants[4];       /**< Time constants for each joint [seconds] */
    double damping[4];              /**< Damping coefficients */
} PlantState;

/**
 * @brief Initialize plant state with default parameters
 * @return Initialized plant state
 */
PlantState plant_create_default(void);

/**
 * @brief Initialize plant state with custom time constants
 * @param time_constants Array of time constants [theta1, theta2, d3, theta4]
 * @return Initialized plant state
 */
PlantState plant_create(const double time_constants[4]);

/**
 * @brief Update plant state based on command input
 * 
 * Simulates first-order dynamics for each joint:
 *   τ · dq/dt + q = q_cmd
 *   q(k+1) = q(k) + (q_cmd - q(k)) · dt / τ
 * 
 * @param plant Plant state
 * @param command Commanded joint velocities/torques
 * @param dt Time step [seconds]
 */
void plant_update(PlantState* plant, const JointState* command, double dt);

/**
 * @brief Update plant with position command (position servo mode)
 * 
 * Simulates position tracking with first-order lag:
 *   q(k+1) = q(k) + (q_target - q(k)) · dt / τ
 * 
 * @param plant Plant state
 * @param target Target joint positions
 * @param dt Time step [seconds]
 */
void plant_update_position(PlantState* plant, const JointState* target, double dt);

/**
 * @brief Reset plant to specific position
 * @param plant Plant state
 * @param position Initial position
 */
void plant_reset(PlantState* plant, const JointState* position);

/**
 * @brief Get current plant position
 * @param plant Plant state
 * @return Current joint positions
 */
JointState plant_get_position(const PlantState* plant);

/**
 * @brief Get current plant velocity
 * @param plant Plant state
 * @return Current joint velocities
 */
JointState plant_get_velocity(const PlantState* plant);

#endif /* SCARA_PLANT_MODEL_H */

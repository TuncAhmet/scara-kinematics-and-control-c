/**
 * @file pid_controller.h
 * @brief PID controller with anti-windup and rate limiting
 */

#ifndef SCARA_PID_CONTROLLER_H
#define SCARA_PID_CONTROLLER_H

#include "types.h"

/**
 * @brief PID controller state
 */
typedef struct {
    /* Gains */
    double Kp;                  /**< Proportional gain */
    double Ki;                  /**< Integral gain */
    double Kd;                  /**< Derivative gain */
    
    /* State */
    double integral;            /**< Integrated error */
    double prev_error;          /**< Previous error (for derivative) */
    double prev_output;         /**< Previous output (for rate limiting) */
    bool initialized;           /**< True after first update */
    
    /* Limits */
    double output_min;          /**< Minimum output */
    double output_max;          /**< Maximum output */
    double integral_max;        /**< Maximum integral magnitude (anti-windup) */
    double rate_limit;          /**< Maximum output change per second */
    
    /* Diagnostics */
    double last_p_term;         /**< Last proportional term */
    double last_i_term;         /**< Last integral term */
    double last_d_term;         /**< Last derivative term */
    bool is_saturated;          /**< True if output is saturated */
} PIDController;

/**
 * @brief Initialize a PID controller with default settings
 * @param Kp Proportional gain
 * @param Ki Integral gain
 * @param Kd Derivative gain
 * @return Initialized PID controller
 */
PIDController pid_create(double Kp, double Ki, double Kd);

/**
 * @brief Initialize a PID controller with full settings
 * @param Kp Proportional gain
 * @param Ki Integral gain
 * @param Kd Derivative gain
 * @param output_min Minimum output value
 * @param output_max Maximum output value
 * @param integral_max Maximum integral magnitude
 * @param rate_limit Maximum output change per second (0 = disabled)
 * @return Initialized PID controller
 */
PIDController pid_create_full(double Kp, double Ki, double Kd,
                              double output_min, double output_max,
                              double integral_max, double rate_limit);

/**
 * @brief Update PID controller and compute output
 * @param pid PID controller state
 * @param setpoint Desired value
 * @param measurement Current measured value
 * @param dt Time step [seconds]
 * @return Control output
 */
double pid_update(PIDController* pid, double setpoint, double measurement, double dt);

/**
 * @brief Reset PID controller state
 * @param pid PID controller to reset
 */
void pid_reset(PIDController* pid);

/**
 * @brief Set PID gains
 * @param pid PID controller
 * @param Kp Proportional gain
 * @param Ki Integral gain
 * @param Kd Derivative gain
 */
void pid_set_gains(PIDController* pid, double Kp, double Ki, double Kd);

/**
 * @brief Set output limits
 * @param pid PID controller
 * @param min Minimum output
 * @param max Maximum output
 */
void pid_set_output_limits(PIDController* pid, double min, double max);

/**
 * @brief Set anti-windup limit
 * @param pid PID controller
 * @param max Maximum integral magnitude
 */
void pid_set_integral_limit(PIDController* pid, double max);

/**
 * @brief Set rate limit
 * @param pid PID controller
 * @param rate_limit Maximum output change per second (0 to disable)
 */
void pid_set_rate_limit(PIDController* pid, double rate_limit);

/* ============================================================================
 * Joint PID Controllers (convenience wrappers for 4-DOF SCARA)
 * ========================================================================== */

/**
 * @brief Array of 4 PID controllers for SCARA joints
 */
typedef struct {
    PIDController theta1;       /**< PID for joint 1 (base rotation) */
    PIDController theta2;       /**< PID for joint 2 (elbow rotation) */
    PIDController d3;           /**< PID for joint 3 (prismatic) */
    PIDController theta4;       /**< PID for joint 4 (wrist rotation) */
} JointPIDControllers;

/**
 * @brief Initialize joint PID controllers with default gains
 * @return Initialized joint PID controllers
 */
JointPIDControllers joint_pid_create_default(void);

/**
 * @brief Update all joint PID controllers and compute outputs
 * @param pids Joint PID controllers
 * @param setpoint Target joint state
 * @param measurement Current joint state
 * @param dt Time step [seconds]
 * @return Control outputs for each joint
 */
JointState joint_pid_update(JointPIDControllers* pids, 
                            const JointState* setpoint,
                            const JointState* measurement,
                            double dt);

/**
 * @brief Reset all joint PID controllers
 * @param pids Joint PID controllers to reset
 */
void joint_pid_reset(JointPIDControllers* pids);

#endif /* SCARA_PID_CONTROLLER_H */

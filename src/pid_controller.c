/**
 * @file pid_controller.c
 * @brief PID controller implementation with anti-windup and rate limiting
 */

#include "scara/pid_controller.h"
#include "scara/math_utils.h"
#include <math.h>

/* Default PID limits */
#define DEFAULT_OUTPUT_MIN      -100.0
#define DEFAULT_OUTPUT_MAX      100.0
#define DEFAULT_INTEGRAL_MAX    50.0
#define DEFAULT_RATE_LIMIT      0.0     /* Disabled by default */

/* Default gains for joint controllers */
#define DEFAULT_KP_THETA        50.0
#define DEFAULT_KI_THETA        5.0
#define DEFAULT_KD_THETA        2.0
#define DEFAULT_KP_D3           100.0
#define DEFAULT_KI_D3           10.0
#define DEFAULT_KD_D3           5.0

PIDController pid_create(double Kp, double Ki, double Kd) {
    return pid_create_full(Kp, Ki, Kd,
                           DEFAULT_OUTPUT_MIN, DEFAULT_OUTPUT_MAX,
                           DEFAULT_INTEGRAL_MAX, DEFAULT_RATE_LIMIT);
}

PIDController pid_create_full(double Kp, double Ki, double Kd,
                              double output_min, double output_max,
                              double integral_max, double rate_limit) {
    PIDController pid;
    
    pid.Kp = Kp;
    pid.Ki = Ki;
    pid.Kd = Kd;
    
    pid.integral = 0.0;
    pid.prev_error = 0.0;
    pid.prev_output = 0.0;
    pid.initialized = false;
    
    pid.output_min = output_min;
    pid.output_max = output_max;
    pid.integral_max = integral_max;
    pid.rate_limit = rate_limit;
    
    pid.last_p_term = 0.0;
    pid.last_i_term = 0.0;
    pid.last_d_term = 0.0;
    pid.is_saturated = false;
    
    return pid;
}

double pid_update(PIDController* pid, double setpoint, double measurement, double dt) {
    if (dt <= 0.0) {
        return pid->prev_output;
    }
    
    /* Compute error */
    double error = setpoint - measurement;
    
    /* Proportional term */
    double p_term = pid->Kp * error;
    
    /* Integral term with anti-windup */
    pid->integral += error * dt;
    
    /* Clamp integral (anti-windup) */
    if (pid->integral_max > 0.0) {
        pid->integral = clamp(pid->integral, -pid->integral_max, pid->integral_max);
    }
    
    double i_term = pid->Ki * pid->integral;
    
    /* Derivative term (on error) */
    double d_term = 0.0;
    if (pid->initialized) {
        double derivative = (error - pid->prev_error) / dt;
        d_term = pid->Kd * derivative;
    }
    
    /* Compute raw output */
    double output = p_term + i_term + d_term;
    
    /* Apply output saturation */
    double saturated_output = clamp(output, pid->output_min, pid->output_max);
    pid->is_saturated = (saturated_output != output);
    
    /* Conditional integration (back-calculation anti-windup) */
    if (pid->is_saturated && pid->Ki > 0.0) {
        /* If saturated, prevent integral from growing in the saturating direction */
        if ((output > pid->output_max && error > 0.0) ||
            (output < pid->output_min && error < 0.0)) {
            /* Remove the contribution that caused saturation */
            pid->integral -= error * dt;
        }
    }
    
    output = saturated_output;
    
    /* Apply rate limiting */
    if (pid->rate_limit > 0.0 && pid->initialized) {
        double max_change = pid->rate_limit * dt;
        double change = output - pid->prev_output;
        if (fabs(change) > max_change) {
            output = pid->prev_output + sign(change) * max_change;
        }
    }
    
    /* Store state for next iteration */
    pid->prev_error = error;
    pid->prev_output = output;
    pid->initialized = true;
    
    /* Store diagnostics */
    pid->last_p_term = p_term;
    pid->last_i_term = i_term;
    pid->last_d_term = d_term;
    
    return output;
}

void pid_reset(PIDController* pid) {
    pid->integral = 0.0;
    pid->prev_error = 0.0;
    pid->prev_output = 0.0;
    pid->initialized = false;
    pid->last_p_term = 0.0;
    pid->last_i_term = 0.0;
    pid->last_d_term = 0.0;
    pid->is_saturated = false;
}

void pid_set_gains(PIDController* pid, double Kp, double Ki, double Kd) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
}

void pid_set_output_limits(PIDController* pid, double min, double max) {
    pid->output_min = min;
    pid->output_max = max;
}

void pid_set_integral_limit(PIDController* pid, double max) {
    pid->integral_max = max;
}

void pid_set_rate_limit(PIDController* pid, double rate_limit) {
    pid->rate_limit = rate_limit;
}

/* ============================================================================
 * Joint PID Controllers
 * ========================================================================== */

JointPIDControllers joint_pid_create_default(void) {
    JointPIDControllers pids;
    
    /* Revolute joints (theta1, theta2, theta4) - angular control */
    pids.theta1 = pid_create_full(DEFAULT_KP_THETA, DEFAULT_KI_THETA, DEFAULT_KD_THETA,
                                  -10.0, 10.0, 5.0, 20.0);
    pids.theta2 = pid_create_full(DEFAULT_KP_THETA, DEFAULT_KI_THETA, DEFAULT_KD_THETA,
                                  -10.0, 10.0, 5.0, 20.0);
    pids.theta4 = pid_create_full(DEFAULT_KP_THETA, DEFAULT_KI_THETA, DEFAULT_KD_THETA,
                                  -10.0, 10.0, 5.0, 20.0);
    
    /* Prismatic joint (d3) - linear control */
    pids.d3 = pid_create_full(DEFAULT_KP_D3, DEFAULT_KI_D3, DEFAULT_KD_D3,
                              -5.0, 5.0, 2.0, 10.0);
    
    return pids;
}

JointState joint_pid_update(JointPIDControllers* pids,
                            const JointState* setpoint,
                            const JointState* measurement,
                            double dt) {
    JointState output;
    
    /* Use angle difference for revolute joints to handle wrap-around */
    double error_theta1 = angle_difference(measurement->theta1, setpoint->theta1);
    double error_theta2 = angle_difference(measurement->theta2, setpoint->theta2);
    double error_theta4 = angle_difference(measurement->theta4, setpoint->theta4);
    
    /* Temporarily set measurement to create correct error internally */
    output.theta1 = pid_update(&pids->theta1, error_theta1 + measurement->theta1, 
                               measurement->theta1, dt);
    output.theta2 = pid_update(&pids->theta2, error_theta2 + measurement->theta2,
                               measurement->theta2, dt);
    output.d3 = pid_update(&pids->d3, setpoint->d3, measurement->d3, dt);
    output.theta4 = pid_update(&pids->theta4, error_theta4 + measurement->theta4,
                               measurement->theta4, dt);
    
    return output;
}

void joint_pid_reset(JointPIDControllers* pids) {
    pid_reset(&pids->theta1);
    pid_reset(&pids->theta2);
    pid_reset(&pids->d3);
    pid_reset(&pids->theta4);
}

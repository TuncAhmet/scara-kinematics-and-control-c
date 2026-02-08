/**
 * @file trajectory.h
 * @brief Trajectory generation with trapezoidal velocity profiles
 */

#ifndef SCARA_TRAJECTORY_H
#define SCARA_TRAJECTORY_H

#include "types.h"

/**
 * @brief Trapezoidal profile phases
 */
typedef enum {
    TRAJ_PHASE_ACCEL,           /**< Acceleration phase */
    TRAJ_PHASE_CRUISE,          /**< Constant velocity phase */
    TRAJ_PHASE_DECEL,           /**< Deceleration phase */
    TRAJ_PHASE_COMPLETE         /**< Motion complete */
} TrajectoryPhase;

/**
 * @brief Single-axis trapezoidal profile parameters
 */
typedef struct {
    double start;               /**< Start position */
    double end;                 /**< End position */
    double distance;            /**< Total distance (signed) */
    double direction;           /**< Direction (-1 or +1) */
    
    double v_max;               /**< Maximum (cruise) velocity */
    double a_max;               /**< Maximum acceleration/deceleration */
    
    double t_accel;             /**< Acceleration phase duration */
    double t_cruise;            /**< Cruise phase duration */
    double t_decel;             /**< Deceleration phase duration */
    double t_total;             /**< Total motion duration */
    
    bool is_triangular;         /**< True if no cruise phase (short move) */
} AxisProfile;

/**
 * @brief Multi-joint trajectory profile
 */
typedef struct {
    JointState start;           /**< Start joint state */
    JointState end;             /**< End (target) joint state */
    
    AxisProfile profiles[4];    /**< Per-joint profiles [theta1, theta2, d3, theta4] */
    
    double duration;            /**< Synchronized total duration */
    double elapsed;             /**< Elapsed time since start */
    
    bool is_valid;              /**< True if trajectory is valid */
    bool is_complete;           /**< True if motion is complete */
} TrajectoryProfile;

/**
 * @brief Trajectory point (position and velocity at a given time)
 */
typedef struct {
    JointState position;        /**< Joint positions */
    JointState velocity;        /**< Joint velocities */
    double time;                /**< Time stamp */
    TrajectoryPhase phase;      /**< Current phase (based on slowest axis) */
} TrajectoryPoint;

/**
 * @brief Plan a trapezoidal trajectory from start to end
 * 
 * This plans a time-synchronized trajectory where all joints
 * start and finish at the same time. Each joint uses the maximum
 * velocity and acceleration allowed while ensuring synchronization.
 * 
 * @param start Start joint state
 * @param end End joint state
 * @param max_vel Maximum velocities [theta1, theta2, d3, theta4]
 * @param max_acc Maximum accelerations [theta1, theta2, d3, theta4]
 * @return Planned trajectory profile
 */
TrajectoryProfile trajectory_plan(const JointState* start, const JointState* end,
                                  const double max_vel[4], const double max_acc[4]);

/**
 * @brief Plan a trajectory using robot configuration limits
 * @param cfg Robot configuration
 * @param start Start joint state
 * @param end End joint state
 * @return Planned trajectory profile
 */
TrajectoryProfile trajectory_plan_with_config(const SCARAConfig* cfg,
                                              const JointState* start,
                                              const JointState* end);

/**
 * @brief Evaluate trajectory at a given time
 * @param profile Trajectory profile
 * @param t Time since trajectory start [seconds]
 * @return Trajectory point (position, velocity)
 */
TrajectoryPoint trajectory_evaluate(const TrajectoryProfile* profile, double t);

/**
 * @brief Update trajectory elapsed time
 * @param profile Trajectory profile
 * @param dt Time step [seconds]
 */
void trajectory_step(TrajectoryProfile* profile, double dt);

/**
 * @brief Get current trajectory point based on elapsed time
 * @param profile Trajectory profile
 * @return Current trajectory point
 */
TrajectoryPoint trajectory_get_current(const TrajectoryProfile* profile);

/**
 * @brief Reset trajectory (back to start)
 * @param profile Trajectory profile
 */
void trajectory_reset(TrajectoryProfile* profile);

/**
 * @brief Check if trajectory is complete
 * @param profile Trajectory profile
 * @return true if elapsed >= duration
 */
bool trajectory_is_complete(const TrajectoryProfile* profile);

/**
 * @brief Get remaining time
 * @param profile Trajectory profile
 * @return Remaining time [seconds]
 */
double trajectory_remaining_time(const TrajectoryProfile* profile);

/**
 * @brief Compute time required to travel a distance with trapezoidal profile
 * @param distance Distance to travel (positive)
 * @param v_max Maximum velocity
 * @param a_max Maximum acceleration
 * @return Required time
 */
double trajectory_compute_time(double distance, double v_max, double a_max);

#endif /* SCARA_TRAJECTORY_H */

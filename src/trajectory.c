/**
 * @file trajectory.c
 * @brief Trajectory generation with trapezoidal velocity profiles
 */

#include "scara/trajectory.h"
#include "scara/dh_transform.h"
#include "scara/math_utils.h"
#include <math.h>
#include <string.h>

/* Minimum motion threshold */
#define MIN_MOTION_THRESHOLD    1e-6

/**
 * @brief Plan a single-axis trapezoidal profile
 */
static AxisProfile plan_axis_profile(double start, double end, double v_max, double a_max) {
    AxisProfile profile;
    
    profile.start = start;
    profile.end = end;
    profile.distance = end - start;
    profile.direction = sign(profile.distance);
    profile.v_max = v_max;
    profile.a_max = a_max;
    
    double d = fabs(profile.distance);
    
    if (d < MIN_MOTION_THRESHOLD) {
        /* No motion needed */
        profile.t_accel = 0.0;
        profile.t_cruise = 0.0;
        profile.t_decel = 0.0;
        profile.t_total = 0.0;
        profile.is_triangular = false;
        return profile;
    }
    
    /* Time to reach v_max */
    double t_accel = v_max / a_max;
    
    /* Distance covered during accel + decel phases */
    double d_accel_decel = v_max * t_accel;  /* = v_max^2 / a_max */
    
    if (d_accel_decel >= d) {
        /* Triangular profile: can't reach v_max */
        profile.is_triangular = true;
        
        /* Peak velocity: v_peak = sqrt(d * a_max) */
        double v_peak = sqrt(d * a_max);
        
        profile.t_accel = v_peak / a_max;
        profile.t_cruise = 0.0;
        profile.t_decel = profile.t_accel;
        profile.t_total = 2.0 * profile.t_accel;
    } else {
        /* Trapezoidal profile: has cruise phase */
        profile.is_triangular = false;
        
        double d_cruise = d - d_accel_decel;
        
        profile.t_accel = t_accel;
        profile.t_cruise = d_cruise / v_max;
        profile.t_decel = t_accel;
        profile.t_total = profile.t_accel + profile.t_cruise + profile.t_decel;
    }
    
    return profile;
}

/**
 * @brief Evaluate a single-axis profile at time t
 */
static void evaluate_axis_profile(const AxisProfile* profile, double t,
                                  double* position, double* velocity) {
    if (fabs(profile->distance) < MIN_MOTION_THRESHOLD || t <= 0.0) {
        *position = profile->start;
        *velocity = 0.0;
        return;
    }
    
    if (t >= profile->t_total) {
        *position = profile->end;
        *velocity = 0.0;
        return;
    }
    
    double dir = profile->direction;
    double a = profile->a_max;
    double v_max;
    
    if (profile->is_triangular) {
        v_max = sqrt(fabs(profile->distance) * a);
    } else {
        v_max = profile->v_max;
    }
    
    double t1 = profile->t_accel;
    double t2 = profile->t_accel + profile->t_cruise;
    
    if (t < t1) {
        /* Acceleration phase */
        *velocity = dir * a * t;
        *position = profile->start + dir * 0.5 * a * t * t;
    } else if (t < t2) {
        /* Cruise phase */
        double dt = t - t1;
        *velocity = dir * v_max;
        double p1 = profile->start + dir * 0.5 * a * t1 * t1;
        *position = p1 + dir * v_max * dt;
    } else {
        /* Deceleration phase */
        double dt = t - t2;
        *velocity = dir * (v_max - a * dt);
        double p1 = profile->start + dir * 0.5 * a * t1 * t1;
        double p2 = p1 + dir * v_max * profile->t_cruise;
        *position = p2 + dir * (v_max * dt - 0.5 * a * dt * dt);
    }
}

/**
 * @brief Scale an axis profile to match a target duration
 */
static void scale_axis_profile(AxisProfile* profile, double target_duration) {
    if (profile->t_total < MIN_MOTION_THRESHOLD || 
        target_duration < MIN_MOTION_THRESHOLD) {
        return;
    }
    
    double scale = target_duration / profile->t_total;
    
    /* Scale times */
    profile->t_accel *= scale;
    profile->t_cruise *= scale;
    profile->t_decel *= scale;
    profile->t_total = target_duration;
    
    /* Velocities and accelerations are inversely scaled */
    profile->v_max /= scale;
    profile->a_max /= (scale * scale);
}

TrajectoryProfile trajectory_plan(const JointState* start, const JointState* end,
                                  const double max_vel[4], const double max_acc[4]) {
    TrajectoryProfile traj;
    memset(&traj, 0, sizeof(traj));
    
    traj.start = *start;
    traj.end = *end;
    traj.elapsed = 0.0;
    traj.is_valid = true;
    traj.is_complete = false;
    
    /* Compute distances (use angle_difference for revolute joints) */
    double distances[4];
    distances[0] = angle_difference(start->theta1, end->theta1);
    distances[1] = angle_difference(start->theta2, end->theta2);
    distances[2] = end->d3 - start->d3;
    distances[3] = angle_difference(start->theta4, end->theta4);
    
    /* Plan each axis independently */
    traj.profiles[0] = plan_axis_profile(start->theta1, start->theta1 + distances[0],
                                         max_vel[0], max_acc[0]);
    traj.profiles[1] = plan_axis_profile(start->theta2, start->theta2 + distances[1],
                                         max_vel[1], max_acc[1]);
    traj.profiles[2] = plan_axis_profile(start->d3, end->d3, max_vel[2], max_acc[2]);
    traj.profiles[3] = plan_axis_profile(start->theta4, start->theta4 + distances[3],
                                         max_vel[3], max_acc[3]);
    
    /* Find the longest duration (slowest axis) */
    double max_duration = 0.0;
    for (int i = 0; i < 4; i++) {
        if (traj.profiles[i].t_total > max_duration) {
            max_duration = traj.profiles[i].t_total;
        }
    }
    
    traj.duration = max_duration;
    
    /* Synchronize all axes to the longest duration */
    for (int i = 0; i < 4; i++) {
        if (traj.profiles[i].t_total > MIN_MOTION_THRESHOLD) {
            scale_axis_profile(&traj.profiles[i], max_duration);
        }
    }
    
    /* Check for zero motion */
    if (max_duration < MIN_MOTION_THRESHOLD) {
        traj.is_complete = true;
    }
    
    return traj;
}

TrajectoryProfile trajectory_plan_with_config(const SCARAConfig* cfg,
                                              const JointState* start,
                                              const JointState* end) {
    double max_vel[4] = {
        cfg->vel_theta1_max,
        cfg->vel_theta2_max,
        cfg->vel_d3_max,
        cfg->vel_theta4_max
    };
    
    double max_acc[4] = {
        cfg->acc_theta1_max,
        cfg->acc_theta2_max,
        cfg->acc_d3_max,
        cfg->acc_theta4_max
    };
    
    return trajectory_plan(start, end, max_vel, max_acc);
}

TrajectoryPoint trajectory_evaluate(const TrajectoryProfile* profile, double t) {
    TrajectoryPoint point;
    point.time = t;
    
    if (!profile->is_valid) {
        point.position = profile->start;
        point.velocity.theta1 = 0.0;
        point.velocity.theta2 = 0.0;
        point.velocity.d3 = 0.0;
        point.velocity.theta4 = 0.0;
        point.phase = TRAJ_PHASE_COMPLETE;
        return point;
    }
    
    /* Clamp time to valid range */
    t = clamp(t, 0.0, profile->duration);
    
    /* Evaluate each axis */
    double pos[4], vel[4];
    for (int i = 0; i < 4; i++) {
        evaluate_axis_profile(&profile->profiles[i], t, &pos[i], &vel[i]);
    }
    
    point.position.theta1 = pos[0];
    point.position.theta2 = pos[1];
    point.position.d3 = pos[2];
    point.position.theta4 = pos[3];
    
    point.velocity.theta1 = vel[0];
    point.velocity.theta2 = vel[1];
    point.velocity.d3 = vel[2];
    point.velocity.theta4 = vel[3];
    
    /* Determine phase based on slowest axis */
    int slowest = 0;
    double max_t = 0.0;
    for (int i = 0; i < 4; i++) {
        if (profile->profiles[i].t_total > max_t) {
            max_t = profile->profiles[i].t_total;
            slowest = i;
        }
    }
    
    const AxisProfile* slow = &profile->profiles[slowest];
    if (t >= slow->t_total) {
        point.phase = TRAJ_PHASE_COMPLETE;
    } else if (t < slow->t_accel) {
        point.phase = TRAJ_PHASE_ACCEL;
    } else if (t < slow->t_accel + slow->t_cruise) {
        point.phase = TRAJ_PHASE_CRUISE;
    } else {
        point.phase = TRAJ_PHASE_DECEL;
    }
    
    return point;
}

void trajectory_step(TrajectoryProfile* profile, double dt) {
    profile->elapsed += dt;
    if (profile->elapsed >= profile->duration) {
        profile->elapsed = profile->duration;
        profile->is_complete = true;
    }
}

TrajectoryPoint trajectory_get_current(const TrajectoryProfile* profile) {
    return trajectory_evaluate(profile, profile->elapsed);
}

void trajectory_reset(TrajectoryProfile* profile) {
    profile->elapsed = 0.0;
    profile->is_complete = false;
}

bool trajectory_is_complete(const TrajectoryProfile* profile) {
    return profile->is_complete;
}

double trajectory_remaining_time(const TrajectoryProfile* profile) {
    return profile->duration - profile->elapsed;
}

double trajectory_compute_time(double distance, double v_max, double a_max) {
    double d = fabs(distance);
    
    if (d < MIN_MOTION_THRESHOLD) {
        return 0.0;
    }
    
    /* Time to reach v_max */
    double t_accel = v_max / a_max;
    
    /* Distance during accel + decel */
    double d_accel_decel = v_max * t_accel;
    
    if (d_accel_decel >= d) {
        /* Triangular profile */
        return 2.0 * sqrt(d / a_max);
    } else {
        /* Trapezoidal profile */
        double d_cruise = d - d_accel_decel;
        return 2.0 * t_accel + d_cruise / v_max;
    }
}

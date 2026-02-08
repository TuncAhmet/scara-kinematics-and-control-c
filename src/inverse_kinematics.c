/**
 * @file inverse_kinematics.c
 * @brief Inverse kinematics implementation for SCARA robot
 */

#include "scara/inverse_kinematics.h"
#include "scara/dh_transform.h"
#include "scara/math_utils.h"
#include <math.h>

/* Singularity detection threshold (radians from 0 or ±π) */
#define SINGULARITY_THRESHOLD   0.05

/* Motion cost weights for solution selection */
#define WEIGHT_THETA1   1.0
#define WEIGHT_THETA2   1.0
#define WEIGHT_D3       5.0     /* Higher weight for prismatic (different units) */
#define WEIGHT_THETA4   0.5     /* Lower weight for wrist rotation */

void ik_get_workspace_bounds(const SCARAConfig* cfg, double* min_radius, double* max_radius) {
    *max_radius = cfg->L1 + cfg->L2;
    *min_radius = fabs(cfg->L1 - cfg->L2);
}

bool ik_is_reachable(const SCARAConfig* cfg, const EndEffectorPose* target) {
    double reach_sq = target->x * target->x + target->y * target->y;
    double max_reach = cfg->L1 + cfg->L2;
    double min_reach = fabs(cfg->L1 - cfg->L2);
    
    /* Check XY plane reachability */
    if (reach_sq > (max_reach * max_reach) + IK_TOLERANCE) {
        return false;
    }
    if (reach_sq < (min_reach * min_reach) - IK_TOLERANCE) {
        return false;
    }
    
    /* Check Z reachability */
    double d3_required = cfg->d1 - target->z;
    if (d3_required < cfg->d3_min - IK_TOLERANCE || d3_required > cfg->d3_max + IK_TOLERANCE) {
        return false;
    }
    
    return true;
}

bool ik_is_near_singularity(const JointState* joints, double tolerance) {
    /* Full extension: θ2 ≈ 0 */
    if (fabs(joints->theta2) < tolerance) {
        return true;
    }
    
    /* Full retraction: θ2 ≈ ±π */
    if (fabs(fabs(joints->theta2) - SCARA_PI) < tolerance) {
        return true;
    }
    
    return false;
}

/**
 * @brief Validate joint solution and check limits
 */
static bool validate_solution(const SCARAConfig* cfg, JointState* joints) {
    /* Normalize angles to [-π, π] */
    joints->theta1 = normalize_angle(joints->theta1);
    joints->theta2 = normalize_angle(joints->theta2);
    joints->theta4 = normalize_angle(joints->theta4);
    
    /* Check joint limits */
    return scara_check_joint_limits(cfg, joints);
}

IKStatus inverse_kinematics(const SCARAConfig* cfg, const EndEffectorPose* target, IKResult* result) {
    /* Initialize result */
    result->status = IK_SUCCESS;
    result->solutions.elbow_up_valid = false;
    result->solutions.elbow_down_valid = false;
    result->is_singular = false;
    
    double max_reach = cfg->L1 + cfg->L2;
    double min_reach = fabs(cfg->L1 - cfg->L2);
    result->max_reach = max_reach;
    result->min_reach = min_reach;
    
    /* Compute reach in XY plane */
    double x = target->x;
    double y = target->y;
    double reach_sq = x * x + y * y;
    double reach = sqrt(reach_sq);
    result->reach = reach;
    
    /* Check reachability in XY plane */
    if (reach > max_reach + IK_TOLERANCE) {
        result->status = IK_UNREACHABLE;
        return IK_UNREACHABLE;
    }
    if (reach < min_reach - IK_TOLERANCE) {
        result->status = IK_UNREACHABLE;
        return IK_UNREACHABLE;
    }
    
    /* Compute d3 (prismatic joint) */
    double d3 = cfg->d1 - target->z;
    
    /* Check d3 limits */
    if (d3 < cfg->d3_min - IK_TOLERANCE || d3 > cfg->d3_max + IK_TOLERANCE) {
        result->status = IK_UNREACHABLE;
        return IK_UNREACHABLE;
    }
    
    /* Clamp d3 to valid range */
    d3 = clamp(d3, cfg->d3_min, cfg->d3_max);
    
    /* Compute θ2 using law of cosines */
    double L1 = cfg->L1;
    double L2 = cfg->L2;
    double cos_theta2 = (reach_sq - L1*L1 - L2*L2) / (2.0 * L1 * L2);
    
    /* Clamp cos_theta2 to valid range (numerical safety) */
    cos_theta2 = clamp(cos_theta2, -1.0, 1.0);
    
    /* Check for singularity */
    if (fabs(cos_theta2) > 1.0 - IK_TOLERANCE) {
        result->is_singular = true;
        result->status = IK_SINGULAR;
    }
    
    double theta2_up = acos(cos_theta2);    /* Elbow up (positive θ2) */
    double theta2_down = -theta2_up;         /* Elbow down (negative θ2) */
    
    /* Compute θ1 for both configurations */
    double sin_theta2_up = sin(theta2_up);
    double sin_theta2_down = sin(theta2_down);
    
    double k1_up = L1 + L2 * cos_theta2;
    double k2_up = L2 * sin_theta2_up;
    double theta1_up = atan2(y, x) - atan2(k2_up, k1_up);
    
    double k1_down = L1 + L2 * cos_theta2;  /* Same as k1_up */
    double k2_down = L2 * sin_theta2_down;
    double theta1_down = atan2(y, x) - atan2(k2_down, k1_down);
    
    /* Compute θ4 for both configurations */
    double theta4_up = target->yaw - theta1_up - theta2_up;
    double theta4_down = target->yaw - theta1_down - theta2_down;
    
    /* Store elbow-up solution */
    result->solutions.elbow_up.theta1 = theta1_up;
    result->solutions.elbow_up.theta2 = theta2_up;
    result->solutions.elbow_up.d3 = d3;
    result->solutions.elbow_up.theta4 = theta4_up;
    
    /* Store elbow-down solution */
    result->solutions.elbow_down.theta1 = theta1_down;
    result->solutions.elbow_down.theta2 = theta2_down;
    result->solutions.elbow_down.d3 = d3;
    result->solutions.elbow_down.theta4 = theta4_down;
    
    /* Validate solutions against joint limits */
    result->solutions.elbow_up_valid = validate_solution(cfg, &result->solutions.elbow_up);
    result->solutions.elbow_down_valid = validate_solution(cfg, &result->solutions.elbow_down);
    
    /* Update status based on validity */
    if (!result->solutions.elbow_up_valid && !result->solutions.elbow_down_valid) {
        result->status = IK_OUT_OF_LIMITS;
        return IK_OUT_OF_LIMITS;
    }
    
    /* Check for singularities in valid solutions */
    if (result->solutions.elbow_up_valid) {
        if (ik_is_near_singularity(&result->solutions.elbow_up, SINGULARITY_THRESHOLD)) {
            result->is_singular = true;
        }
    }
    if (result->solutions.elbow_down_valid) {
        if (ik_is_near_singularity(&result->solutions.elbow_down, SINGULARITY_THRESHOLD)) {
            result->is_singular = true;
        }
    }
    
    return result->status;
}

double ik_compute_motion_cost(const JointState* current, const JointState* target) {
    double cost = 0.0;
    
    /* Angular differences (wrapped) */
    double d_theta1 = fabs(angle_difference(current->theta1, target->theta1));
    double d_theta2 = fabs(angle_difference(current->theta2, target->theta2));
    double d_d3 = fabs(target->d3 - current->d3);
    double d_theta4 = fabs(angle_difference(current->theta4, target->theta4));
    
    /* Weighted cost */
    cost = WEIGHT_THETA1 * d_theta1 +
           WEIGHT_THETA2 * d_theta2 +
           WEIGHT_D3 * d_d3 +
           WEIGHT_THETA4 * d_theta4;
    
    return cost;
}

bool ik_select_best_solution(const IKResult* result, const JointState* current,
                             const SCARAConfig* cfg, JointState* selected) {
    (void)cfg;  /* Unused but kept for API consistency */
    
    bool has_up = result->solutions.elbow_up_valid;
    bool has_down = result->solutions.elbow_down_valid;
    
    if (!has_up && !has_down) {
        return false;
    }
    
    if (has_up && !has_down) {
        *selected = result->solutions.elbow_up;
        return true;
    }
    
    if (!has_up && has_down) {
        *selected = result->solutions.elbow_down;
        return true;
    }
    
    /* Both solutions valid - select by minimum motion cost */
    double cost_up = ik_compute_motion_cost(current, &result->solutions.elbow_up);
    double cost_down = ik_compute_motion_cost(current, &result->solutions.elbow_down);
    
    if (cost_up <= cost_down) {
        *selected = result->solutions.elbow_up;
    } else {
        *selected = result->solutions.elbow_down;
    }
    
    return true;
}

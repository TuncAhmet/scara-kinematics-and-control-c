/**
 * @file inverse_kinematics.h
 * @brief Inverse kinematics for SCARA robot
 */

#ifndef SCARA_INVERSE_KINEMATICS_H
#define SCARA_INVERSE_KINEMATICS_H

#include "types.h"

/**
 * @brief IK solution pair (elbow-up and elbow-down configurations)
 */
typedef struct {
    JointState elbow_up;        /**< Elbow-up configuration (θ2 > 0) */
    JointState elbow_down;      /**< Elbow-down configuration (θ2 < 0) */
    bool elbow_up_valid;        /**< True if elbow-up solution is valid and within limits */
    bool elbow_down_valid;      /**< True if elbow-down solution is valid and within limits */
} IKSolutions;

/**
 * @brief Extended IK result with status and diagnostics
 */
typedef struct {
    IKStatus status;            /**< Overall IK status */
    IKSolutions solutions;      /**< Solution pair */
    double reach;               /**< Distance from base to target in XY plane */
    double max_reach;           /**< Maximum reach (L1 + L2) */
    double min_reach;           /**< Minimum reach |L1 - L2| */
    bool is_singular;           /**< True if near singular configuration */
} IKResult;

/**
 * @brief Compute inverse kinematics for SCARA robot
 * 
 * Given a desired end-effector pose (x, y, z, yaw), compute joint angles
 * (θ1, θ2, d3, θ4) that achieve the pose.
 * 
 * Algorithm:
 * 1. d3 = d1 - z (vertical position from prismatic joint)
 * 2. θ2 = ±acos((x² + y² - L1² - L2²) / (2·L1·L2))
 *    (+) for elbow-up, (-) for elbow-down
 * 3. θ1 = atan2(y, x) - atan2(L2·sin(θ2), L1 + L2·cos(θ2))
 * 4. θ4 = yaw - θ1 - θ2
 * 
 * @param cfg Robot configuration
 * @param target Desired end-effector pose
 * @param result Output IK result with solutions and status
 * @return IK status code
 */
IKStatus inverse_kinematics(const SCARAConfig* cfg, const EndEffectorPose* target, IKResult* result);

/**
 * @brief Select the best IK solution based on criteria
 * 
 * Selection criteria:
 * 1. Solution must be valid (within joint limits)
 * 2. Prefer solution closest to current joint state (minimal motion)
 * 
 * @param result IK result containing both solutions
 * @param current Current joint state (for minimal motion heuristic)
 * @param cfg Robot configuration
 * @param selected Output: selected joint state
 * @return true if a valid solution was selected, false otherwise
 */
bool ik_select_best_solution(const IKResult* result, const JointState* current,
                             const SCARAConfig* cfg, JointState* selected);

/**
 * @brief Compute the "cost" of transitioning from current to target joint state
 * 
 * Cost is weighted sum of joint displacements:
 *   cost = w1*|Δθ1| + w2*|Δθ2| + w3*|Δd3| + w4*|Δθ4|
 * 
 * @param current Current joint state
 * @param target Target joint state
 * @return Motion cost
 */
double ik_compute_motion_cost(const JointState* current, const JointState* target);

/**
 * @brief Check if target pose is reachable
 * @param cfg Robot configuration
 * @param target Target pose
 * @return true if target is potentially reachable (in workspace)
 */
bool ik_is_reachable(const SCARAConfig* cfg, const EndEffectorPose* target);

/**
 * @brief Check if a configuration is near a singularity
 * 
 * Singularities for SCARA:
 * - Full extension: θ2 ≈ 0 (arm fully extended)
 * - Full retraction: θ2 ≈ ±π (arm fully folded)
 * 
 * @param joints Joint state to check
 * @param tolerance Angle tolerance for singularity detection [rad]
 * @return true if near singular configuration
 */
bool ik_is_near_singularity(const JointState* joints, double tolerance);

/**
 * @brief Compute workspace boundaries at given height
 * @param cfg Robot configuration
 * @param min_radius Output: minimum reach radius
 * @param max_radius Output: maximum reach radius
 */
void ik_get_workspace_bounds(const SCARAConfig* cfg, double* min_radius, double* max_radius);

#endif /* SCARA_INVERSE_KINEMATICS_H */

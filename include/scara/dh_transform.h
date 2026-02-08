/**
 * @file dh_transform.h
 * @brief Denavit-Hartenberg transformation matrix generation
 */

#ifndef SCARA_DH_TRANSFORM_H
#define SCARA_DH_TRANSFORM_H

#include "types.h"

/**
 * @brief Compute a standard D-H transformation matrix
 * 
 * Standard D-H convention transformation matrix:
 * T = Rz(theta) * Tz(d) * Tx(a) * Rx(alpha)
 * 
 * Where:
 * - Rz(theta): Rotation about z-axis by theta
 * - Tz(d): Translation along z-axis by d
 * - Tx(a): Translation along x-axis by a
 * - Rx(alpha): Rotation about x-axis by alpha
 * 
 * @param params D-H parameters (a, alpha, d, theta)
 * @return 4x4 homogeneous transformation matrix
 */
Matrix4x4 dh_transform(const DHParams* params);

/**
 * @brief Create D-H parameters structure
 * @param a Link length
 * @param alpha Link twist
 * @param d Link offset
 * @param theta Joint angle
 * @return DHParams structure
 */
DHParams dh_params_create(double a, double alpha, double d, double theta);

/**
 * @brief Initialize default SCARA robot configuration
 * 
 * Default parameters:
 * - L1 = 0.3 m, L2 = 0.25 m
 * - d1 = 0.4 m (base height)
 * - Joint limits: ±150° for revolute joints
 * - d3 range: 0 to 0.2 m
 * 
 * @return Default SCARA configuration
 */
SCARAConfig scara_config_default(void);

/**
 * @brief Create a custom SCARA robot configuration
 * @param L1 Link 1 length [m]
 * @param L2 Link 2 length [m]
 * @param d1 Base height offset [m]
 * @return SCARA configuration with default limits
 */
SCARAConfig scara_config_create(double L1, double L2, double d1);

/**
 * @brief Check if a joint state is within the robot's joint limits
 * @param cfg Robot configuration
 * @param joints Joint state to check
 * @return true if all joints are within limits
 */
bool scara_check_joint_limits(const SCARAConfig* cfg, const JointState* joints);

/**
 * @brief Clamp joint state to robot's joint limits
 * @param cfg Robot configuration
 * @param joints Joint state to clamp (modified in place)
 */
void scara_clamp_joint_limits(const SCARAConfig* cfg, JointState* joints);

#endif /* SCARA_DH_TRANSFORM_H */

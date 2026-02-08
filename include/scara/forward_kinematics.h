/**
 * @file forward_kinematics.h
 * @brief Forward kinematics for SCARA robot
 */

#ifndef SCARA_FORWARD_KINEMATICS_H
#define SCARA_FORWARD_KINEMATICS_H

#include "types.h"

/**
 * @brief Compute transformation matrix T0_1 (base to joint 1)
 * @param cfg Robot configuration
 * @param theta1 Joint 1 angle [rad]
 * @return Transformation matrix T0_1
 */
Matrix4x4 fk_compute_T01(const SCARAConfig* cfg, double theta1);

/**
 * @brief Compute transformation matrix T1_2 (joint 1 to joint 2)
 * @param cfg Robot configuration
 * @param theta2 Joint 2 angle [rad]
 * @return Transformation matrix T1_2
 */
Matrix4x4 fk_compute_T12(const SCARAConfig* cfg, double theta2);

/**
 * @brief Compute transformation matrix T2_3 (joint 2 to joint 3)
 * @param d3 Prismatic joint extension [m]
 * @return Transformation matrix T2_3
 */
Matrix4x4 fk_compute_T23(double d3);

/**
 * @brief Compute transformation matrix T3_4 (joint 3 to end-effector)
 * @param theta4 Joint 4 angle [rad]
 * @return Transformation matrix T3_4
 */
Matrix4x4 fk_compute_T34(double theta4);

/**
 * @brief Compute forward kinematics: T0_4 transformation matrix
 * @param cfg Robot configuration
 * @param joints Current joint state
 * @return Homogeneous transformation matrix T0_4
 */
Matrix4x4 forward_kinematics(const SCARAConfig* cfg, const JointState* joints);

/**
 * @brief Extract end-effector pose from transformation matrix
 * @param T04 Transformation matrix from base to end-effector
 * @return End-effector pose (x, y, z, yaw)
 */
EndEffectorPose fk_extract_pose(const Matrix4x4* T04);

/**
 * @brief Compute end-effector pose directly from joint state
 * @param cfg Robot configuration
 * @param joints Current joint state
 * @return End-effector pose (x, y, z, yaw)
 */
EndEffectorPose fk_compute_pose(const SCARAConfig* cfg, const JointState* joints);

/**
 * @brief Compute analytical forward kinematics (optimized, no matrix multiplications)
 * 
 * For SCARA robot, the FK equations can be computed directly:
 *   x = L1*cos(θ1) + L2*cos(θ1 + θ2)
 *   y = L1*sin(θ1) + L2*sin(θ1 + θ2)
 *   z = d1 - d3
 *   yaw = θ1 + θ2 + θ4
 * 
 * @param cfg Robot configuration
 * @param joints Current joint state
 * @return End-effector pose (x, y, z, yaw)
 */
EndEffectorPose fk_compute_pose_analytical(const SCARAConfig* cfg, const JointState* joints);

/**
 * @brief Get individual joint position in Cartesian space
 * @param cfg Robot configuration
 * @param joints Current joint state
 * @param joint_index Joint index (0=base, 1=elbow, 2=wrist, 3=end-effector)
 * @return Position of the specified joint
 */
Vector3 fk_get_joint_position(const SCARAConfig* cfg, const JointState* joints, int joint_index);

#endif /* SCARA_FORWARD_KINEMATICS_H */

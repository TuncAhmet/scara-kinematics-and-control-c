/**
 * @file forward_kinematics.c
 * @brief Forward kinematics implementation for SCARA robot
 */

#include "scara/forward_kinematics.h"
#include "scara/dh_transform.h"
#include "scara/math_utils.h"
#include <math.h>

/*
 * SCARA D-H Parameter Table (Standard Convention)
 * ================================================
 * | Joint | a     | α   | d   | θ   |
 * |-------|-------|-----|-----|-----|
 * |   1   | L1    | 0   | d1  | θ1  |
 * |   2   | L2    | 0   | 0   | θ2  |
 * |   3   | 0     | 0   | d3  | 0   |  (prismatic: d3 is variable)
 * |   4   | 0     | 0   | 0   | θ4  |
 */

Matrix4x4 fk_compute_T01(const SCARAConfig* cfg, double theta1) {
    DHParams params = dh_params_create(cfg->L1, 0.0, cfg->d1, theta1);
    return dh_transform(&params);
}

Matrix4x4 fk_compute_T12(const SCARAConfig* cfg, double theta2) {
    DHParams params = dh_params_create(cfg->L2, 0.0, 0.0, theta2);
    return dh_transform(&params);
}

Matrix4x4 fk_compute_T23(double d3) {
    /* Prismatic joint: d varies, theta = 0 
     * For SCARA: prismatic extends downward, so negative d
     */
    DHParams params = dh_params_create(0.0, 0.0, -d3, 0.0);
    return dh_transform(&params);
}

Matrix4x4 fk_compute_T34(double theta4) {
    DHParams params = dh_params_create(0.0, 0.0, 0.0, theta4);
    return dh_transform(&params);
}

Matrix4x4 forward_kinematics(const SCARAConfig* cfg, const JointState* joints) {
    /* Compute individual transformation matrices */
    Matrix4x4 T01 = fk_compute_T01(cfg, joints->theta1);
    Matrix4x4 T12 = fk_compute_T12(cfg, joints->theta2);
    Matrix4x4 T23 = fk_compute_T23(joints->d3);
    Matrix4x4 T34 = fk_compute_T34(joints->theta4);
    
    /* Chain multiply: T04 = T01 * T12 * T23 * T34 */
    Matrix4x4 T02 = matrix_multiply(&T01, &T12);
    Matrix4x4 T03 = matrix_multiply(&T02, &T23);
    Matrix4x4 T04 = matrix_multiply(&T03, &T34);
    
    return T04;
}

EndEffectorPose fk_extract_pose(const Matrix4x4* T04) {
    EndEffectorPose pose;
    
    /* Extract position from last column */
    pose.x = T04->m[0][3];
    pose.y = T04->m[1][3];
    pose.z = T04->m[2][3];
    
    /* 
     * Extract yaw (rotation about Z-axis) from rotation matrix.
     * For SCARA, the end-effector only rotates about Z, so:
     *   R = Rz(yaw) = | cos(yaw)  -sin(yaw)  0 |
     *                 | sin(yaw)   cos(yaw)  0 |
     *                 |    0          0      1 |
     * 
     * Therefore: yaw = atan2(R[1][0], R[0][0])
     */
    pose.yaw = atan2(T04->m[1][0], T04->m[0][0]);
    
    return pose;
}

EndEffectorPose fk_compute_pose(const SCARAConfig* cfg, const JointState* joints) {
    Matrix4x4 T04 = forward_kinematics(cfg, joints);
    return fk_extract_pose(&T04);
}

EndEffectorPose fk_compute_pose_analytical(const SCARAConfig* cfg, const JointState* joints) {
    EndEffectorPose pose;
    
    double theta12 = joints->theta1 + joints->theta2;
    double cos_theta1 = cos(joints->theta1);
    double sin_theta1 = sin(joints->theta1);
    double cos_theta12 = cos(theta12);
    double sin_theta12 = sin(theta12);
    
    /* Direct FK equations for SCARA */
    pose.x = cfg->L1 * cos_theta1 + cfg->L2 * cos_theta12;
    pose.y = cfg->L1 * sin_theta1 + cfg->L2 * sin_theta12;
    pose.z = cfg->d1 - joints->d3;  /* Prismatic joint extends downward */
    pose.yaw = normalize_angle(theta12 + joints->theta4);
    
    return pose;
}

Vector3 fk_get_joint_position(const SCARAConfig* cfg, const JointState* joints, int joint_index) {
    Vector3 pos = {0.0, 0.0, 0.0};
    
    double cos_t1 = cos(joints->theta1);
    double sin_t1 = sin(joints->theta1);
    double theta12 = joints->theta1 + joints->theta2;
    double cos_t12 = cos(theta12);
    double sin_t12 = sin(theta12);
    
    switch (joint_index) {
        case 0: /* Base (origin) */
            pos.x = 0.0;
            pos.y = 0.0;
            pos.z = 0.0;
            break;
            
        case 1: /* Elbow (end of L1) */
            pos.x = cfg->L1 * cos_t1;
            pos.y = cfg->L1 * sin_t1;
            pos.z = cfg->d1;
            break;
            
        case 2: /* Wrist (end of L2, before prismatic) */
            pos.x = cfg->L1 * cos_t1 + cfg->L2 * cos_t12;
            pos.y = cfg->L1 * sin_t1 + cfg->L2 * sin_t12;
            pos.z = cfg->d1;
            break;
            
        case 3: /* End-effector */
        default:
            pos.x = cfg->L1 * cos_t1 + cfg->L2 * cos_t12;
            pos.y = cfg->L1 * sin_t1 + cfg->L2 * sin_t12;
            pos.z = cfg->d1 - joints->d3;
            break;
    }
    
    return pos;
}

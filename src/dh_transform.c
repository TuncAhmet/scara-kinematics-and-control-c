/**
 * @file dh_transform.c
 * @brief Denavit-Hartenberg transformation matrix generation implementation
 */

#include "scara/dh_transform.h"
#include "scara/math_utils.h"
#include <math.h>

Matrix4x4 dh_transform(const DHParams* params) {
    Matrix4x4 T;
    
    double cos_theta = cos(params->theta);
    double sin_theta = sin(params->theta);
    double cos_alpha = cos(params->alpha);
    double sin_alpha = sin(params->alpha);
    
    /*
     * Standard D-H transformation matrix:
     * T = | cos(θ)  -sin(θ)cos(α)   sin(θ)sin(α)   a·cos(θ) |
     *     | sin(θ)   cos(θ)cos(α)  -cos(θ)sin(α)   a·sin(θ) |
     *     |   0        sin(α)          cos(α)          d     |
     *     |   0          0               0             1     |
     */
    
    /* Row 0 */
    T.m[0][0] = cos_theta;
    T.m[0][1] = -sin_theta * cos_alpha;
    T.m[0][2] = sin_theta * sin_alpha;
    T.m[0][3] = params->a * cos_theta;
    
    /* Row 1 */
    T.m[1][0] = sin_theta;
    T.m[1][1] = cos_theta * cos_alpha;
    T.m[1][2] = -cos_theta * sin_alpha;
    T.m[1][3] = params->a * sin_theta;
    
    /* Row 2 */
    T.m[2][0] = 0.0;
    T.m[2][1] = sin_alpha;
    T.m[2][2] = cos_alpha;
    T.m[2][3] = params->d;
    
    /* Row 3 */
    T.m[3][0] = 0.0;
    T.m[3][1] = 0.0;
    T.m[3][2] = 0.0;
    T.m[3][3] = 1.0;
    
    return T;
}

DHParams dh_params_create(double a, double alpha, double d, double theta) {
    DHParams params;
    params.a = a;
    params.alpha = alpha;
    params.d = d;
    params.theta = theta;
    return params;
}

SCARAConfig scara_config_default(void) {
    SCARAConfig cfg;
    
    /* Link lengths */
    cfg.L1 = 0.30;              /* 30 cm */
    cfg.L2 = 0.25;              /* 25 cm */
    
    /* Offsets */
    cfg.d1 = 0.40;              /* 40 cm base height */
    cfg.tool_offset = 0.0;
    
    /* Joint limits (±150 degrees for revolute, 0-20cm for prismatic) */
    cfg.theta1_min = deg_to_rad(-150.0);
    cfg.theta1_max = deg_to_rad(150.0);
    cfg.theta2_min = deg_to_rad(-150.0);
    cfg.theta2_max = deg_to_rad(150.0);
    cfg.d3_min = 0.0;
    cfg.d3_max = 0.20;          /* 20 cm stroke */
    cfg.theta4_min = deg_to_rad(-180.0);
    cfg.theta4_max = deg_to_rad(180.0);
    
    /* Velocity limits */
    cfg.vel_theta1_max = deg_to_rad(180.0);  /* 180 deg/s */
    cfg.vel_theta2_max = deg_to_rad(180.0);
    cfg.vel_d3_max = 0.2;                     /* 20 cm/s */
    cfg.vel_theta4_max = deg_to_rad(360.0);  /* 360 deg/s */
    
    /* Acceleration limits */
    cfg.acc_theta1_max = deg_to_rad(360.0);  /* 360 deg/s^2 */
    cfg.acc_theta2_max = deg_to_rad(360.0);
    cfg.acc_d3_max = 0.5;                     /* 50 cm/s^2 */
    cfg.acc_theta4_max = deg_to_rad(720.0);  /* 720 deg/s^2 */
    
    return cfg;
}

SCARAConfig scara_config_create(double L1, double L2, double d1) {
    SCARAConfig cfg = scara_config_default();
    cfg.L1 = L1;
    cfg.L2 = L2;
    cfg.d1 = d1;
    return cfg;
}

bool scara_check_joint_limits(const SCARAConfig* cfg, const JointState* joints) {
    if (!in_range(joints->theta1, cfg->theta1_min, cfg->theta1_max)) return false;
    if (!in_range(joints->theta2, cfg->theta2_min, cfg->theta2_max)) return false;
    if (!in_range(joints->d3, cfg->d3_min, cfg->d3_max)) return false;
    if (!in_range(joints->theta4, cfg->theta4_min, cfg->theta4_max)) return false;
    return true;
}

void scara_clamp_joint_limits(const SCARAConfig* cfg, JointState* joints) {
    joints->theta1 = clamp(joints->theta1, cfg->theta1_min, cfg->theta1_max);
    joints->theta2 = clamp(joints->theta2, cfg->theta2_min, cfg->theta2_max);
    joints->d3 = clamp(joints->d3, cfg->d3_min, cfg->d3_max);
    joints->theta4 = clamp(joints->theta4, cfg->theta4_min, cfg->theta4_max);
}

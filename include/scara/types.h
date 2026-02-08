/**
 * @file types.h
 * @brief Common type definitions for SCARA robot kinematics and control
 */

#ifndef SCARA_TYPES_H
#define SCARA_TYPES_H

#include <stdbool.h>
#include <stdint.h>

/* ============================================================================
 * Mathematical Types
 * ========================================================================== */

/**
 * @brief 4x4 homogeneous transformation matrix
 */
typedef struct {
    double m[4][4];
} Matrix4x4;

/**
 * @brief 4x1 homogeneous vector
 */
typedef struct {
    double v[4];
} Vector4;

/**
 * @brief 3x1 vector for positions
 */
typedef struct {
    double x, y, z;
} Vector3;

/* ============================================================================
 * Robot State Types
 * ========================================================================== */

/**
 * @brief Joint state for 4-DOF SCARA robot
 * 
 * - theta1: Base rotation [rad]
 * - theta2: Elbow rotation [rad]
 * - d3: Vertical extension [m]
 * - theta4: Wrist rotation [rad]
 */
typedef struct {
    double theta1;
    double theta2;
    double d3;
    double theta4;
} JointState;

/**
 * @brief End-effector pose in Cartesian space
 * 
 * - x, y, z: Position [m]
 * - yaw: Orientation about Z-axis [rad]
 */
typedef struct {
    double x;
    double y;
    double z;
    double yaw;
} EndEffectorPose;

/* ============================================================================
 * D-H Parameter Types
 * ========================================================================== */

/**
 * @brief Standard Denavit-Hartenberg parameters for a single joint
 * 
 * Standard D-H convention:
 * - a: Link length (along x_{i})
 * - alpha: Link twist (about x_{i})
 * - d: Link offset (along z_{i-1})
 * - theta: Joint angle (about z_{i-1})
 */
typedef struct {
    double a;
    double alpha;
    double d;
    double theta;
} DHParams;

/**
 * @brief SCARA robot configuration parameters
 */
typedef struct {
    /* Link lengths */
    double L1;              /**< Length of link 1 [m] */
    double L2;              /**< Length of link 2 [m] */
    
    /* Offsets */
    double d1;              /**< Base height offset [m] */
    double tool_offset;     /**< End-effector tool offset [m] */
    
    /* Joint limits */
    double theta1_min;      /**< Min limit for theta1 [rad] */
    double theta1_max;      /**< Max limit for theta1 [rad] */
    double theta2_min;      /**< Min limit for theta2 [rad] */
    double theta2_max;      /**< Max limit for theta2 [rad] */
    double d3_min;          /**< Min limit for d3 [m] */
    double d3_max;          /**< Max limit for d3 [m] */
    double theta4_min;      /**< Min limit for theta4 [rad] */
    double theta4_max;      /**< Max limit for theta4 [rad] */
    
    /* Velocity limits */
    double vel_theta1_max;  /**< Max angular velocity for theta1 [rad/s] */
    double vel_theta2_max;  /**< Max angular velocity for theta2 [rad/s] */
    double vel_d3_max;      /**< Max linear velocity for d3 [m/s] */
    double vel_theta4_max;  /**< Max angular velocity for theta4 [rad/s] */
    
    /* Acceleration limits */
    double acc_theta1_max;  /**< Max angular acceleration for theta1 [rad/s^2] */
    double acc_theta2_max;  /**< Max angular acceleration for theta2 [rad/s^2] */
    double acc_d3_max;      /**< Max linear acceleration for d3 [m/s^2] */
    double acc_theta4_max;  /**< Max angular acceleration for theta4 [rad/s^2] */
} SCARAConfig;

/* ============================================================================
 * Status and Error Types
 * ========================================================================== */

/**
 * @brief Inverse kinematics status codes
 */
typedef enum {
    IK_SUCCESS = 0,         /**< Valid solution found */
    IK_UNREACHABLE,         /**< Target outside workspace */
    IK_SINGULAR,            /**< Near singular configuration */
    IK_OUT_OF_LIMITS,       /**< Solution violates joint limits */
    IK_NO_VALID_SOLUTION    /**< No valid solution after all checks */
} IKStatus;

/**
 * @brief Simulation mode states
 */
typedef enum {
    MODE_IDLE = 0,          /**< Robot idle, waiting for commands */
    MODE_MOVING,            /**< Robot executing trajectory */
    MODE_REACHED,           /**< Target position reached */
    MODE_ERROR              /**< Error state (IK failure, limit violation) */
} SimMode;

/* ============================================================================
 * Numeric Constants
 * ========================================================================== */

#define SCARA_PI            3.14159265358979323846
#define SCARA_TWO_PI        6.28318530717958647692
#define SCARA_DEG_TO_RAD    0.01745329251994329577
#define SCARA_RAD_TO_DEG    57.29577951308232087680

#define IK_TOLERANCE        1e-6    /**< Numerical tolerance for IK */
#define POSITION_TOLERANCE  1e-4    /**< Position convergence tolerance [m] */
#define ANGLE_TOLERANCE     1e-3    /**< Angle convergence tolerance [rad] */

#endif /* SCARA_TYPES_H */

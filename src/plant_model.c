/**
 * @file plant_model.c
 * @brief First-order motor/actuator dynamics simulation
 */

#include "scara/plant_model.h"
#include "scara/math_utils.h"
#include <math.h>

/* Default time constants (seconds) */
#define DEFAULT_TAU_THETA1      0.05    /* 50ms for base rotation */
#define DEFAULT_TAU_THETA2      0.04    /* 40ms for elbow */
#define DEFAULT_TAU_D3          0.03    /* 30ms for prismatic */
#define DEFAULT_TAU_THETA4      0.02    /* 20ms for wrist */

/* Default damping coefficients */
#define DEFAULT_DAMPING         0.1

PlantState plant_create_default(void) {
    double time_constants[4] = {
        DEFAULT_TAU_THETA1,
        DEFAULT_TAU_THETA2,
        DEFAULT_TAU_D3,
        DEFAULT_TAU_THETA4
    };
    return plant_create(time_constants);
}

PlantState plant_create(const double time_constants[4]) {
    PlantState plant;
    
    plant.position.theta1 = 0.0;
    plant.position.theta2 = 0.0;
    plant.position.d3 = 0.0;
    plant.position.theta4 = 0.0;
    
    plant.velocity.theta1 = 0.0;
    plant.velocity.theta2 = 0.0;
    plant.velocity.d3 = 0.0;
    plant.velocity.theta4 = 0.0;
    
    for (int i = 0; i < 4; i++) {
        plant.time_constants[i] = time_constants[i];
        plant.damping[i] = DEFAULT_DAMPING;
    }
    
    return plant;
}

void plant_update(PlantState* plant, const JointState* command, double dt) {
    if (dt <= 0.0) return;
    
    double tau[4] = {
        plant->time_constants[0],
        plant->time_constants[1],
        plant->time_constants[2],
        plant->time_constants[3]
    };
    
    /* Store old velocities for position update */
    double old_vel[4] = {
        plant->velocity.theta1,
        plant->velocity.theta2,
        plant->velocity.d3,
        plant->velocity.theta4
    };
    
    /* Update velocities (first-order response to command) */
    /* v(k+1) = v(k) + (cmd - v(k)) * dt / tau */
    double cmd[4] = {command->theta1, command->theta2, command->d3, command->theta4};
    double new_vel[4];
    
    for (int i = 0; i < 4; i++) {
        double alpha = dt / tau[i];
        if (alpha > 1.0) alpha = 1.0;  /* Prevent instability */
        new_vel[i] = old_vel[i] + (cmd[i] - old_vel[i]) * alpha;
    }
    
    plant->velocity.theta1 = new_vel[0];
    plant->velocity.theta2 = new_vel[1];
    plant->velocity.d3 = new_vel[2];
    plant->velocity.theta4 = new_vel[3];
    
    /* Update positions using average velocity (trapezoidal integration) */
    plant->position.theta1 += 0.5 * (old_vel[0] + new_vel[0]) * dt;
    plant->position.theta2 += 0.5 * (old_vel[1] + new_vel[1]) * dt;
    plant->position.d3 += 0.5 * (old_vel[2] + new_vel[2]) * dt;
    plant->position.theta4 += 0.5 * (old_vel[3] + new_vel[3]) * dt;
}

void plant_update_position(PlantState* plant, const JointState* target, double dt) {
    if (dt <= 0.0) return;
    
    double tau[4] = {
        plant->time_constants[0],
        plant->time_constants[1],
        plant->time_constants[2],
        plant->time_constants[3]
    };
    
    double* pos[4] = {
        &plant->position.theta1,
        &plant->position.theta2,
        &plant->position.d3,
        &plant->position.theta4
    };
    
    double* vel[4] = {
        &plant->velocity.theta1,
        &plant->velocity.theta2,
        &plant->velocity.d3,
        &plant->velocity.theta4
    };
    
    double tgt[4] = {target->theta1, target->theta2, target->d3, target->theta4};
    
    for (int i = 0; i < 4; i++) {
        double alpha = dt / tau[i];
        if (alpha > 1.0) alpha = 1.0;
        
        double old_pos = *pos[i];
        *pos[i] = old_pos + (tgt[i] - old_pos) * alpha;
        *vel[i] = (*pos[i] - old_pos) / dt;
    }
}

void plant_reset(PlantState* plant, const JointState* position) {
    plant->position = *position;
    
    plant->velocity.theta1 = 0.0;
    plant->velocity.theta2 = 0.0;
    plant->velocity.d3 = 0.0;
    plant->velocity.theta4 = 0.0;
}

JointState plant_get_position(const PlantState* plant) {
    return plant->position;
}

JointState plant_get_velocity(const PlantState* plant) {
    return plant->velocity;
}

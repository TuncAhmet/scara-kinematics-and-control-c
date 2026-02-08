/* Debug test for ik_is_reachable */
#include <stdio.h>
#include <math.h>
#include "scara/inverse_kinematics.h"
#include "scara/forward_kinematics.h"
#include "scara/dh_transform.h"
#include "scara/math_utils.h"

int main(void) {
    SCARAConfig cfg = scara_config_default();
    printf("Config: L1=%.3f L2=%.3f d1=%.3f d3_min=%.3f d3_max=%.3f\n", 
           cfg.L1, cfg.L2, cfg.d1, cfg.d3_min, cfg.d3_max);
    printf("Max reach = %.3f, Min reach = %.3f\n", cfg.L1+cfg.L2, fabs(cfg.L1-cfg.L2));
    
    EndEffectorPose target = {0.30, 0.10, 0.35, 0.0};
    double reach_sq = target.x*target.x + target.y*target.y;
    double reach = sqrt(reach_sq);
    double d3_req = cfg.d1 - target.z;
    printf("Target: x=%.3f y=%.3f z=%.3f\n", target.x, target.y, target.z);
    printf("reach=%.6f, d3_required=%.6f\n", reach, d3_req);
    printf("max_reach=%.6f, min_reach=%.6f\n", cfg.L1+cfg.L2, fabs(cfg.L1-cfg.L2));
    
    /* Manual checks */
    double max_reach = cfg.L1 + cfg.L2;
    double min_reach = fabs(cfg.L1 - cfg.L2);
    printf("reach_sq=%.9f, max_sq=%.9f, min_sq=%.9f\n", reach_sq, max_reach*max_reach, min_reach*min_reach);
    
    bool xy_ok = (reach_sq <= (max_reach*max_reach + 1e-6)) && (reach_sq >= (min_reach*min_reach - 1e-6));
    bool z_ok = (d3_req >= cfg.d3_min - 1e-6) && (d3_req <= cfg.d3_max + 1e-6);
    printf("XY reachable: %d, Z reachable: %d\n", xy_ok, z_ok);
    
    bool result = ik_is_reachable(&cfg, &target);
    printf("ik_is_reachable result: %d\n", result);
    
    return 0;
}

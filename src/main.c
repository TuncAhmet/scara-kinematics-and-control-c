/**
 * @file main.c
 * @brief SCARA Robot Simulation Main Entry Point
 * 
 * Runs the simulation loop at 500 Hz with TCP communication for UI.
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <signal.h>

#ifdef _WIN32
#include <windows.h>
#define SLEEP_MS(ms) Sleep(ms)
#else
#include <time.h>
#include <unistd.h>
#define SLEEP_MS(ms) usleep((ms) * 1000)
#endif

#include "scara/types.h"
#include "scara/simulation.h"
#include "scara/comm_protocol.h"
#include "scara/math_utils.h"

/* Configuration */
#define LOOP_RATE_HZ        500         /* Control loop rate */
#define TELEMETRY_RATE_HZ   50          /* Telemetry broadcast rate */
#define COMM_PORT           5555        /* TCP port */

/* Global flag for clean shutdown */
static volatile bool g_running = true;

#ifdef _WIN32
BOOL WINAPI console_handler(DWORD signal) {
    if (signal == CTRL_C_EVENT) {
        printf("\nShutting down...\n");
        g_running = false;
        return TRUE;
    }
    return FALSE;
}
#else
void signal_handler(int sig) {
    (void)sig;
    printf("\nShutting down...\n");
    g_running = false;
}
#endif

/**
 * @brief Get current time in seconds (high resolution)
 */
static double get_time_seconds(void) {
#ifdef _WIN32
    static LARGE_INTEGER frequency = {0};
    if (frequency.QuadPart == 0) {
        QueryPerformanceFrequency(&frequency);
    }
    LARGE_INTEGER counter;
    QueryPerformanceCounter(&counter);
    return (double)counter.QuadPart / (double)frequency.QuadPart;
#else
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
#endif
}

/**
 * @brief Process received command
 */
static void process_command(SimulationState* sim, const Command* cmd) {
    switch (cmd->type) {
        case CMD_GOTO_POSE:
            printf("CMD: Go to pose (%.3f, %.3f, %.3f, %.3f)\n",
                   cmd->data.target_pose.x, cmd->data.target_pose.y,
                   cmd->data.target_pose.z, cmd->data.target_pose.yaw);
            if (!simulation_set_target(sim, &cmd->data.target_pose)) {
                printf("  -> Target unreachable!\n");
            }
            break;
            
        case CMD_GOTO_JOINTS:
            printf("CMD: Go to joints (%.3f, %.3f, %.3f, %.3f)\n",
                   cmd->data.target_joints.theta1, cmd->data.target_joints.theta2,
                   cmd->data.target_joints.d3, cmd->data.target_joints.theta4);
            simulation_set_target_joints(sim, &cmd->data.target_joints);
            break;
            
        case CMD_STOP:
            printf("CMD: Stop\n");
            simulation_stop(sim);
            break;
            
        case CMD_RESET:
            printf("CMD: Reset\n");
            simulation_reset(sim);
            break;
            
        case CMD_GET_STATUS:
            /* Status is sent automatically via telemetry */
            break;
            
        default:
            break;
    }
}

int main(int argc, char* argv[]) {
    (void)argc;
    (void)argv;
    
    printf("===========================================\n");
    printf("  SCARA Robot Simulation\n");
    printf("  Control Loop: %d Hz\n", LOOP_RATE_HZ);
    printf("  TCP Port: %d\n", COMM_PORT);
    printf("===========================================\n\n");
    
    /* Set up signal handler */
#ifdef _WIN32
    SetConsoleCtrlHandler(console_handler, TRUE);
#else
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
#endif
    
    /* Initialize simulation */
    SimulationState sim = simulation_create((double)LOOP_RATE_HZ);
    printf("Simulation initialized.\n");
    printf("Robot config: L1=%.2fm, L2=%.2fm, d1=%.2fm\n",
           sim.robot_config.L1, sim.robot_config.L2, sim.robot_config.d1);
    
    /* Initialize communication */
    CommServer server = comm_server_create(COMM_PORT);
    if (!comm_server_start(&server)) {
        printf("ERROR: Failed to start TCP server on port %d\n", COMM_PORT);
        return 1;
    }
    printf("TCP server started on port %d\n", COMM_PORT);
    printf("Waiting for client connection...\n\n");
    
    /* Timing variables */
    double dt = 1.0 / LOOP_RATE_HZ;
    int telemetry_divider = LOOP_RATE_HZ / TELEMETRY_RATE_HZ;
    int loop_counter = 0;
    
    double last_time = get_time_seconds();
    double lag = 0.0;
    
    /* Main loop */
    while (g_running) {
        double current_time = get_time_seconds();
        double elapsed = current_time - last_time;
        last_time = current_time;
        lag += elapsed;
        
        /* Accept new connections */
        if (!server.client_connected) {
            comm_server_accept(&server);
        }
        
        /* Receive commands */
        if (server.client_connected) {
            Command cmd;
            while (comm_server_receive(&server, &cmd)) {
                process_command(&sim, &cmd);
            }
        }
        
        /* Fixed timestep simulation update */
        while (lag >= dt) {
            simulation_step(&sim);
            lag -= dt;
            loop_counter++;
            
            /* Send telemetry at reduced rate */
            if (server.client_connected && (loop_counter % telemetry_divider) == 0) {
                Telemetry telemetry = simulation_get_telemetry(&sim);
                comm_server_send_telemetry(&server, &telemetry);
            }
        }
        
        /* Sleep to avoid busy-waiting */
        double remaining = dt - (get_time_seconds() - current_time);
        if (remaining > 0.001) {
            SLEEP_MS((int)(remaining * 500));  /* Sleep for half remaining time */
        }
    }
    
    /* Cleanup */
    comm_server_shutdown(&server);
    printf("Server shutdown complete.\n");
    
    return 0;
}

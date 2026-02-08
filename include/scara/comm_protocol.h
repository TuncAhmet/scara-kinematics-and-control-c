/**
 * @file comm_protocol.h
 * @brief TCP communication protocol for UI interaction
 */

#ifndef SCARA_COMM_PROTOCOL_H
#define SCARA_COMM_PROTOCOL_H

#include "types.h"
#include "simulation.h"

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
typedef SOCKET socket_t;
#define INVALID_SOCK INVALID_SOCKET
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
typedef int socket_t;
#define INVALID_SOCK (-1)
#endif

#define COMM_DEFAULT_PORT       5555
#define COMM_MAX_BUFFER_SIZE    4096
#define COMM_MAX_CLIENTS        1       /* Single client for simplicity */

/**
 * @brief Command types from UI
 */
typedef enum {
    CMD_NONE = 0,
    CMD_GOTO_POSE,          /**< Go to specified pose */
    CMD_GOTO_JOINTS,        /**< Go to specified joint angles */
    CMD_STOP,               /**< Stop current motion */
    CMD_RESET,              /**< Reset to home position */
    CMD_GET_STATUS          /**< Request current status */
} CommandType;

/**
 * @brief Parsed command from UI
 */
typedef struct {
    CommandType type;
    uint32_t sequence;
    double timestamp;
    
    union {
        EndEffectorPose target_pose;
        JointState target_joints;
    } data;
} Command;

/**
 * @brief Communication server state
 */
typedef struct {
    socket_t server_socket;
    socket_t client_socket;
    bool is_initialized;
    bool client_connected;
    
    uint16_t port;
    
    char recv_buffer[COMM_MAX_BUFFER_SIZE];
    char send_buffer[COMM_MAX_BUFFER_SIZE];
    
    int recv_len;
} CommServer;

/**
 * @brief Initialize communication server
 * @param port TCP port number
 * @return Initialized server state
 */
CommServer comm_server_create(uint16_t port);

/**
 * @brief Start listening for connections
 * @param server Communication server
 * @return true on success
 */
bool comm_server_start(CommServer* server);

/**
 * @brief Check for and accept new client connections (non-blocking)
 * @param server Communication server
 * @return true if new client connected
 */
bool comm_server_accept(CommServer* server);

/**
 * @brief Receive and parse command from client (non-blocking)
 * @param server Communication server
 * @param cmd Output command structure
 * @return true if command was received
 */
bool comm_server_receive(CommServer* server, Command* cmd);

/**
 * @brief Send telemetry to connected client
 * @param server Communication server
 * @param telemetry Telemetry data to send
 * @return true on success
 */
bool comm_server_send_telemetry(CommServer* server, const Telemetry* telemetry);

/**
 * @brief Close client connection
 * @param server Communication server
 */
void comm_server_disconnect_client(CommServer* server);

/**
 * @brief Shutdown server
 * @param server Communication server
 */
void comm_server_shutdown(CommServer* server);

/**
 * @brief Parse JSON command string
 * @param json JSON string
 * @param cmd Output command structure
 * @return true on successful parse
 */
bool comm_parse_command(const char* json, Command* cmd);

/**
 * @brief Format telemetry as JSON string
 * @param telemetry Telemetry data
 * @param buffer Output buffer
 * @param buffer_size Buffer size
 * @return Number of characters written
 */
int comm_format_telemetry(const Telemetry* telemetry, char* buffer, int buffer_size);

#endif /* SCARA_COMM_PROTOCOL_H */

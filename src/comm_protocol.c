/**
 * @file comm_protocol.c
 * @brief TCP communication protocol implementation
 */

#include "scara/comm_protocol.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef _WIN32
#pragma comment(lib, "ws2_32.lib")
#endif

/* Simple JSON parsing helpers (minimal implementation) */
static bool parse_double(const char* json, const char* key, double* value) {
    char search[64];
    snprintf(search, sizeof(search), "\"%s\":", key);
    
    const char* pos = strstr(json, search);
    if (!pos) return false;
    
    pos += strlen(search);
    while (*pos == ' ' || *pos == '\t') pos++;
    
    *value = atof(pos);
    return true;
}

static bool parse_int(const char* json, const char* key, int* value) {
    char search[64];
    snprintf(search, sizeof(search), "\"%s\":", key);
    
    const char* pos = strstr(json, search);
    if (!pos) return false;
    
    pos += strlen(search);
    while (*pos == ' ' || *pos == '\t') pos++;
    
    *value = atoi(pos);
    return true;
}

static bool parse_string(const char* json, const char* key, char* value, int max_len) {
    char search[64];
    snprintf(search, sizeof(search), "\"%s\":", key);
    
    const char* pos = strstr(json, search);
    if (!pos) return false;
    
    pos += strlen(search);
    /* Skip whitespace after colon */
    while (*pos == ' ' || *pos == '\t') pos++;
    
    /* Expect opening quote */
    if (*pos != '"') return false;
    pos++;
    
    int i = 0;
    while (*pos && *pos != '"' && i < max_len - 1) {
        value[i++] = *pos++;
    }
    value[i] = '\0';
    return true;
}

CommServer comm_server_create(uint16_t port) {
    CommServer server;
    memset(&server, 0, sizeof(server));
    
    server.port = port;
    server.server_socket = INVALID_SOCK;
    server.client_socket = INVALID_SOCK;
    server.is_initialized = false;
    server.client_connected = false;
    
    return server;
}

bool comm_server_start(CommServer* server) {
#ifdef _WIN32
    /* Initialize Winsock */
    WSADATA wsa_data;
    if (WSAStartup(MAKEWORD(2, 2), &wsa_data) != 0) {
        return false;
    }
#endif
    
    /* Create socket */
    server->server_socket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (server->server_socket == INVALID_SOCK) {
        return false;
    }
    
    /* Set socket options */
    int opt = 1;
#ifdef _WIN32
    setsockopt(server->server_socket, SOL_SOCKET, SO_REUSEADDR, 
               (const char*)&opt, sizeof(opt));
#else
    setsockopt(server->server_socket, SOL_SOCKET, SO_REUSEADDR, 
               &opt, sizeof(opt));
#endif
    
    /* Bind */
    struct sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(server->port);
    
    if (bind(server->server_socket, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
#ifdef _WIN32
        closesocket(server->server_socket);
#else
        close(server->server_socket);
#endif
        return false;
    }
    
    /* Listen */
    if (listen(server->server_socket, COMM_MAX_CLIENTS) < 0) {
#ifdef _WIN32
        closesocket(server->server_socket);
#else
        close(server->server_socket);
#endif
        return false;
    }
    
    /* Set non-blocking mode */
#ifdef _WIN32
    u_long mode = 1;
    ioctlsocket(server->server_socket, FIONBIO, &mode);
#else
    int flags = fcntl(server->server_socket, F_GETFL, 0);
    fcntl(server->server_socket, F_SETFL, flags | O_NONBLOCK);
#endif
    
    server->is_initialized = true;
    return true;
}

bool comm_server_accept(CommServer* server) {
    if (!server->is_initialized || server->client_connected) {
        return false;
    }
    
    struct sockaddr_in client_addr;
    int addr_len = sizeof(client_addr);
    
#ifdef _WIN32
    server->client_socket = accept(server->server_socket, 
                                   (struct sockaddr*)&client_addr, &addr_len);
#else
    server->client_socket = accept(server->server_socket,
                                   (struct sockaddr*)&client_addr, 
                                   (socklen_t*)&addr_len);
#endif
    
    if (server->client_socket == INVALID_SOCK) {
        return false;
    }
    
    /* Set client socket to non-blocking */
#ifdef _WIN32
    u_long mode = 1;
    ioctlsocket(server->client_socket, FIONBIO, &mode);
#else
    int flags = fcntl(server->client_socket, F_GETFL, 0);
    fcntl(server->client_socket, F_SETFL, flags | O_NONBLOCK);
#endif
    
    server->client_connected = true;
    printf("Client connected\n");
    return true;
}

bool comm_server_receive(CommServer* server, Command* cmd) {
    if (!server->client_connected) {
        return false;
    }
    
    int bytes_received;
#ifdef _WIN32
    bytes_received = recv(server->client_socket, server->recv_buffer, 
                          COMM_MAX_BUFFER_SIZE - 1, 0);
    
    if (bytes_received == SOCKET_ERROR) {
        int err = WSAGetLastError();
        if (err != WSAEWOULDBLOCK) {
            printf("Recv socket error: %d (disconnecting)\n", err);
            comm_server_disconnect_client(server);
        }
        /* WSAEWOULDBLOCK just means no data available - not an error */
        return false;
    }
    
    if (bytes_received == 0) {
        /* Graceful close by client */
        printf("Client closed connection gracefully\n");
        comm_server_disconnect_client(server);
        return false;
    }
#else
    bytes_received = recv(server->client_socket, server->recv_buffer,
                          COMM_MAX_BUFFER_SIZE - 1, MSG_DONTWAIT);
    
    if (bytes_received < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK) {
            comm_server_disconnect_client(server);
        }
        return false;
    }
    
    if (bytes_received == 0) {
        comm_server_disconnect_client(server);
        return false;
    }
#endif
    
    server->recv_buffer[bytes_received] = '\0';
    printf("DEBUG: Received %d bytes: %s\n", bytes_received, server->recv_buffer);
    
    /* Parse command */
    bool result = comm_parse_command(server->recv_buffer, cmd);
    printf("DEBUG: Parse result=%d, cmd_type=%d\n", result, cmd->type);
    return result;
}

bool comm_parse_command(const char* json, Command* cmd) {
    memset(cmd, 0, sizeof(Command));
    cmd->type = CMD_NONE;
    
    /* Parse sequence number */
    int seq = 0;
    parse_int(json, "seq", &seq);
    cmd->sequence = (uint32_t)seq;
    
    /* Parse timestamp */
    parse_double(json, "timestamp", &cmd->timestamp);
    
    /* Parse command type */
    char cmd_str[32] = {0};
    if (!parse_string(json, "command", cmd_str, sizeof(cmd_str))) {
        return false;
    }
    
    if (strcmp(cmd_str, "goto") == 0 || strcmp(cmd_str, "goto_pose") == 0) {
        cmd->type = CMD_GOTO_POSE;
        
        /* Parse target pose */
        const char* target = strstr(json, "\"target\"");
        if (target) {
            parse_double(target, "x", &cmd->data.target_pose.x);
            parse_double(target, "y", &cmd->data.target_pose.y);
            parse_double(target, "z", &cmd->data.target_pose.z);
            parse_double(target, "yaw", &cmd->data.target_pose.yaw);
        }
    } else if (strcmp(cmd_str, "goto_joints") == 0) {
        cmd->type = CMD_GOTO_JOINTS;
        
        const char* target = strstr(json, "\"target\"");
        if (target) {
            parse_double(target, "theta1", &cmd->data.target_joints.theta1);
            parse_double(target, "theta2", &cmd->data.target_joints.theta2);
            parse_double(target, "d3", &cmd->data.target_joints.d3);
            parse_double(target, "theta4", &cmd->data.target_joints.theta4);
        }
    } else if (strcmp(cmd_str, "stop") == 0) {
        cmd->type = CMD_STOP;
    } else if (strcmp(cmd_str, "reset") == 0) {
        cmd->type = CMD_RESET;
    } else if (strcmp(cmd_str, "status") == 0) {
        cmd->type = CMD_GET_STATUS;
    }
    
    return cmd->type != CMD_NONE;
}

bool comm_server_send_telemetry(CommServer* server, const Telemetry* telemetry) {
    if (!server->client_connected) {
        return false;
    }
    
    int len = comm_format_telemetry(telemetry, server->send_buffer, COMM_MAX_BUFFER_SIZE);
    if (len <= 0) {
        return false;
    }
    
    /* Add newline delimiter */
    server->send_buffer[len++] = '\n';
    
    int sent = send(server->client_socket, server->send_buffer, len, 0);
    if (sent < 0) {
#ifdef _WIN32
        int err = WSAGetLastError();
        if (err != WSAEWOULDBLOCK) {
            comm_server_disconnect_client(server);
        }
#endif
        return false;
    }
    
    return true;
}

int comm_format_telemetry(const Telemetry* telemetry, char* buffer, int buffer_size) {
    const char* mode_str;
    switch (telemetry->mode) {
        case MODE_IDLE: mode_str = "idle"; break;
        case MODE_MOVING: mode_str = "moving"; break;
        case MODE_REACHED: mode_str = "reached"; break;
        case MODE_ERROR: mode_str = "error"; break;
        default: mode_str = "unknown"; break;
    }
    
    return snprintf(buffer, buffer_size,
        "{"
        "\"seq\":%u,"
        "\"timestamp\":%.6f,"
        "\"joints\":{\"theta1\":%.6f,\"theta2\":%.6f,\"d3\":%.6f,\"theta4\":%.6f},"
        "\"pose\":{\"x\":%.6f,\"y\":%.6f,\"z\":%.6f,\"yaw\":%.6f},"
        "\"errors\":{\"theta1\":%.6f,\"theta2\":%.6f,\"d3\":%.6f,\"theta4\":%.6f},"
        "\"mode\":\"%s\","
        "\"ik_valid\":%s,"
        "\"at_target\":%s,"
        "\"progress\":%.3f"
        "}",
        telemetry->sequence,
        telemetry->timestamp,
        telemetry->joints.theta1, telemetry->joints.theta2,
        telemetry->joints.d3, telemetry->joints.theta4,
        telemetry->pose.x, telemetry->pose.y,
        telemetry->pose.z, telemetry->pose.yaw,
        telemetry->errors.theta1, telemetry->errors.theta2,
        telemetry->errors.d3, telemetry->errors.theta4,
        mode_str,
        telemetry->ik_valid ? "true" : "false",
        telemetry->at_target ? "true" : "false",
        telemetry->traj_progress
    );
}

void comm_server_disconnect_client(CommServer* server) {
    if (server->client_socket != INVALID_SOCK) {
#ifdef _WIN32
        closesocket(server->client_socket);
#else
        close(server->client_socket);
#endif
        server->client_socket = INVALID_SOCK;
        server->client_connected = false;
        printf("Client disconnected\n");
    }
}

void comm_server_shutdown(CommServer* server) {
    comm_server_disconnect_client(server);
    
    if (server->server_socket != INVALID_SOCK) {
#ifdef _WIN32
        closesocket(server->server_socket);
        WSACleanup();
#else
        close(server->server_socket);
#endif
        server->server_socket = INVALID_SOCK;
    }
    
    server->is_initialized = false;
}

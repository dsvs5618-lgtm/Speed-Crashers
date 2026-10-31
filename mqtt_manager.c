/**
 * @file mqtt_manager.c
 * @brief Lightweight Native MQTT 3.1.1 Client for Wireless Dashboard Communication
 *
 * Implements direct MQTT 3.1.1 packet serialization/deserialization over TCP sockets
 * with ZERO external third-party library dependencies for high portability on QNX SDP 8.0.
 */

#include "mqtt_manager.h"
#include "acc_config.h"
#include "qnx_compat.h"
#include <stdio.h>
#include <string.h>

#ifdef _WIN32
    #include <winsock2.h>
    #include <ws2tcpip.h>
#else
    #include <sys/socket.h>
    #include <netinet/in.h>
    #include <arpa/inet.h>
    #include <unistd.h>
    #include <fcntl.h>
#endif

/* MQTT 3.1.1 Packet Types */
#define MQTT_PKT_CONNECT     0x10
#define MQTT_PKT_CONNACK     0x20
#define MQTT_PKT_PUBLISH     0x30
#define MQTT_PKT_SUBSCRIBE   0x82
#define MQTT_PKT_SUBACK      0x90
#define MQTT_PKT_PINGREQ     0xC0
#define MQTT_PKT_PINGRESP    0xD0

static void set_socket_nonblocking(int fd) {
#ifdef _WIN32
    u_long mode = 1;
    ioctlsocket(fd, FIONBIO, &mode);
#else
    int flags = fcntl(fd, F_GETFL, 0);
    if (flags != -1) {
        fcntl(fd, F_SETFL, flags | O_NONBLOCK);
    }
#endif
}

bool mqtt_manager_init(mqtt_manager_t *mqtt, const char *broker_ip, int port) {
    if (!mqtt) return false;
    memset(mqtt, 0, sizeof(mqtt_manager_t));

    strncpy(mqtt->broker_host, (broker_ip && strlen(broker_ip) > 0) ? broker_ip : MQTT_DEFAULT_BROKER_HOST, sizeof(mqtt->broker_host) - 1);
    mqtt->broker_port = (port > 0) ? port : MQTT_DEFAULT_BROKER_PORT;
    mqtt->sock_fd = -1;
    mqtt->is_connected = false;

    mqtt->cached_sim_cmd.target_speed_kmh = 55.0f;
    mqtt->requested_set_speed_kmh         = VEHICLE_DEFAULT_SET_SPEED_KMH;
    mqtt->requested_mode                  = MODE_TABLETOP_SIM;

#ifdef _WIN32
    WSADATA wsa;
    WSAStartup(MAKEWORD(2, 2), &wsa);
#endif

    /* Create TCP Socket */
    int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sock < 0) {
        return false;
    }

    struct sockaddr_in serv_addr;
    memset(&serv_addr, 0, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons((uint16_t)mqtt->broker_port);
    serv_addr.sin_addr.s_addr = inet_addr(mqtt->broker_host);

    /* Attempt connect */
    if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) == 0) {
        mqtt->sock_fd = sock;

        /* Send MQTT 3.1.1 CONNECT packet */
        uint8_t conn_pkt[] = {
            MQTT_PKT_CONNECT, 0x1A,            /* Remaining Length: 26 bytes */
            0x00, 0x04, 'M', 'Q', 'T', 'T',    /* Protocol Name */
            0x04,                              /* Protocol Level 4 (MQTT 3.1.1) */
            0x02,                              /* Clean Session */
            0x00, 0x3C,                        /* Keepalive 60s */
            0x00, 0x0E, 'Q', 'N', 'X', '_', 'A', 'C', 'C', '_', 'C', 'A', 'R', '_', '0', '1'
        };
        send(sock, (const char *)conn_pkt, sizeof(conn_pkt), 0);

        /* Subscribe to command topics: qnx_acc/command and qnx_acc/sim_lead */
        uint8_t sub_pkt[] = {
            MQTT_PKT_SUBSCRIBE, 0x22,          /* Packet Type & Len */
            0x00, 0x01,                        /* Packet Identifier = 1 */
            0x00, 0x0F, 'q', 'n', 'x', '_', 'a', 'c', 'c', '/', 'c', 'o', 'm', 'm', 'a', 'n', 'd', 0x00,
            0x00, 0x10, 'q', 'n', 'x', '_', 'a', 'c', 'c', '/', 's', 'i', 'm', '_', 'l', 'e', 'a', 'd', 0x00
        };
        send(sock, (const char *)sub_pkt, sizeof(sub_pkt), 0);

        set_socket_nonblocking(sock);
        mqtt->is_connected = true;
    } else {
        /* Connection failed or broker not running yet (graceful non-blocking degradation) */
#ifdef _WIN32
        closesocket(sock);
#else
        close(sock);
#endif
        mqtt->sock_fd = -1;
    }

    return true;
}

bool mqtt_manager_publish_telemetry(mqtt_manager_t *mqtt, const acc_telemetry_t *telem) {
    if (!mqtt || !telem) return false;

    /* Build JSON Payload */
    char json_buf[896];
    int len = snprintf(json_buf, sizeof(json_buf),
        "{"
        "\"seq\":%llu,\"ts\":%llu,\"mode\":%d,\"state\":\"%s\","
        "\"ego_kph\":%.1f,\"lead_kph\":%.1f,\"max_kph\":%.1f,"
        "\"dist_m\":%.2f,\"safe_m\":%.2f,\"rel_v\":%.2f,\"ttc\":%.2f,"
        "\"accel\":%.2f,\"pwm\":%.1f,"
        "\"led_norm\":%d,\"led_crs\":%d,\"brk\":%d,\"aeb\":%d,"
        "\"front_m\":%.2f,\"left_m\":%.2f,\"right_m\":%.2f,"
        "\"front_ok\":%d,\"left_ok\":%d,\"right_ok\":%d,\"degraded\":%d,"
        "\"cutin_l\":%d,\"cutin_r\":%d,\"lat_us\":%u,\"jit_us\":%u,\"wd_ok\":%d,\"alert\":\"%s\""
        "}",
        (unsigned long long)telem->seq_num,
        (unsigned long long)telem->timestamp_ms,
        (int)telem->current_mode,
        telem->state_name,
        telem->ego_speed_kmh,
        telem->lead_speed_kmh,
        telem->configured_max_speed_kmh,
        telem->relative_distance_m,
        telem->safe_distance_m,
        telem->relative_velocity_mps,
        telem->time_to_collision_sec,
        telem->commanded_accel_mps2,
        telem->motor_pwm_percent,
        telem->led_normal_active ? 1 : 0,
        telem->led_cruise_active ? 1 : 0,
        telem->brake_active ? 1 : 0,
        telem->aeb_active ? 1 : 0,
        telem->front_sensor_dist_m,
        telem->left_sensor_dist_m,
        telem->right_sensor_dist_m,
        telem->front_sensor_ok ? 1 : 0,
        telem->left_sensor_ok ? 1 : 0,
        telem->right_sensor_ok ? 1 : 0,
        telem->degraded_mode_active ? 1 : 0,
        telem->left_cutin_detected ? 1 : 0,
        telem->right_cutin_detected ? 1 : 0,
        telem->rtos_latency_us,
        telem->max_jitter_us,
        telem->watchdog_ok ? 1 : 0,
        telem->alert_message
    );

    if (mqtt->sock_fd >= 0 && mqtt->is_connected) {
        /* Encode MQTT PUBLISH packet */
        const char *topic = MQTT_TOPIC_TELEMETRY;
        uint16_t topic_len = (uint16_t)strlen(topic);
        uint32_t rem_len = 2 + topic_len + len;

        uint8_t header[8];
        int h_idx = 0;
        header[h_idx++] = MQTT_PKT_PUBLISH;

        /* Variable-length encoding */
        uint32_t x = rem_len;
        do {
            uint8_t byte = x % 128;
            x /= 128;
            if (x > 0) byte |= 128;
            header[h_idx++] = byte;
        } while (x > 0);

        uint8_t top_len_bytes[2] = { (uint8_t)(topic_len >> 8), (uint8_t)(topic_len & 0xFF) };

        send(mqtt->sock_fd, (const char *)header, h_idx, 0);
        send(mqtt->sock_fd, (const char *)top_len_bytes, 2, 0);
        send(mqtt->sock_fd, topic, topic_len, 0);
        send(mqtt->sock_fd, json_buf, len, 0);

        mqtt->last_tx_timestamp_ms = qnx_get_time_ms();
        return true;
    }

    return false;
}

void mqtt_manager_poll(mqtt_manager_t *mqtt) {
    if (!mqtt) return;

    if (mqtt->sock_fd < 0 || !mqtt->is_connected) {
        /* Try reconnecting every 3 seconds if disconnected */
        static uint64_t last_reconnect_ms = 0;
        uint64_t now_ms = qnx_get_time_ms();
        if (now_ms - last_reconnect_ms > 3000) {
            last_reconnect_ms = now_ms;
            mqtt_manager_init(mqtt, mqtt->broker_host, mqtt->broker_port);
        }
        return;
    }

    char rx_buf[1024];
    int n = recv(mqtt->sock_fd, rx_buf, sizeof(rx_buf) - 1, 0);
    if (n > 0) {
        rx_buf[n] = '\0';
        mqtt->last_rx_timestamp_ms = qnx_get_time_ms();

        /* Quick check for keywords in incoming payload */
        if (strstr(rx_buf, "\"mode\":\"SIM\"") || strstr(rx_buf, "\"mode\":\"SIMULATION\"") ||
            strstr(rx_buf, "\"mode\":\"TABLETOP\"") || strstr(rx_buf, "\"mode\":0")) {
            mqtt->requested_mode = MODE_SIMULATED_PHYSICS;
            mqtt->mode_switch_requested = true;
        } else if (strstr(rx_buf, "\"mode\":\"SENSORS\"") || strstr(rx_buf, "\"mode\":\"LIVE\"") ||
                   strstr(rx_buf, "\"mode\":\"GROUND\"") || strstr(rx_buf, "\"mode\":1")) {
            mqtt->requested_mode = MODE_LIVE_SENSORS;
            mqtt->mode_switch_requested = true;
        }

        /* Check for sudden brake trigger */
        if (strstr(rx_buf, "\"sudden_brake\":true")) {
            mqtt->cached_sim_cmd.sudden_brake_active = true;
        } else if (strstr(rx_buf, "\"sudden_brake\":false")) {
            mqtt->cached_sim_cmd.sudden_brake_active = false;
        }

        /* Check for cut-in trigger */
        if (strstr(rx_buf, "\"cut_in\":true")) {
            mqtt->cached_sim_cmd.cut_in_event_active = true;
        } else if (strstr(rx_buf, "\"cut_in\":false")) {
            mqtt->cached_sim_cmd.cut_in_event_active = false;
        }

        /* Check for lead reverse */
        if (strstr(rx_buf, "\"lead_reverse\":true")) {
            mqtt->cached_sim_cmd.lead_reverse_active = true;
        } else if (strstr(rx_buf, "\"lead_reverse\":false")) {
            mqtt->cached_sim_cmd.lead_reverse_active = false;
        }

        /* Check for sensor fault injection toggles */
        if (strstr(rx_buf, "\"toggle_fault_front\":true") || strstr(rx_buf, "\"fault_front\":true")) {
            mqtt->fault_toggle_front_requested = true;
        }
        if (strstr(rx_buf, "\"toggle_fault_left\":true") || strstr(rx_buf, "\"fault_left\":true")) {
            mqtt->fault_toggle_left_requested = true;
        }
        if (strstr(rx_buf, "\"toggle_fault_right\":true") || strstr(rx_buf, "\"fault_right\":true")) {
            mqtt->fault_toggle_right_requested = true;
        }

        /* Check for set speed adjustments */
        char *speed_ptr = strstr(rx_buf, "\"set_speed\":");
        if (speed_ptr) {
            float spd = 0.0f;
            if (sscanf(speed_ptr + 12, "%f", &spd) == 1 && spd >= 0.0f) {
                mqtt->requested_set_speed_kmh = spd;
                mqtt->speed_change_requested = true;
            }
        }

        /* Check for lead vehicle speed command from judge slider */
        char *lead_spd_ptr = strstr(rx_buf, "\"lead_speed\":");
        if (lead_spd_ptr) {
            float lspd = 0.0f;
            if (sscanf(lead_spd_ptr + 13, "%f", &lspd) == 1 && lspd >= 0.0f) {
                mqtt->cached_sim_cmd.target_speed_kmh = lspd;
            }
        }

        /* E-Stop */
        if (strstr(rx_buf, "\"estop\":true")) {
            mqtt->estop_requested = true;
        }
    }
}

void mqtt_manager_close(mqtt_manager_t *mqtt) {
    if (!mqtt) return;
    if (mqtt->sock_fd >= 0) {
#ifdef _WIN32
        closesocket(mqtt->sock_fd);
        WSACleanup();
#else
        close(mqtt->sock_fd);
#endif
        mqtt->sock_fd = -1;
    }
    mqtt->is_connected = false;
}


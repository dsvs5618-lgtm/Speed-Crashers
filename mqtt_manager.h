/**
 * @file mqtt_manager.h
 * @brief Wireless Mosquitto MQTT Client for Laptop Dashboard Communication
 */

#ifndef MQTT_MANAGER_H
#define MQTT_MANAGER_H

#include "acc_types.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    char broker_host[64];
    int  broker_port;
    bool is_connected;
    int  sock_fd;

    /* Cached Incoming Commands */
    sim_lead_command_t       cached_sim_cmd;
    vehicle_operating_mode_t requested_mode;
    float                    requested_set_speed_kmh;
    bool                     mode_switch_requested;
    bool                     speed_change_requested;
    bool                     estop_requested;
    bool                     fault_toggle_front_requested;
    bool                     fault_toggle_left_requested;
    bool                     fault_toggle_right_requested;

    uint64_t last_rx_timestamp_ms;
    uint64_t last_tx_timestamp_ms;
} mqtt_manager_t;

/**
 * @brief Initialize MQTT client and connect to broker on laptop hotspot
 */
bool mqtt_manager_init(mqtt_manager_t *mqtt, const char *broker_ip, int port);

/**
 * @brief Publish telemetry frame as JSON string to MQTT_TOPIC_TELEMETRY
 */
bool mqtt_manager_publish_telemetry(mqtt_manager_t *mqtt, const acc_telemetry_t *telem);

/**
 * @brief Poll network socket non-blocking for incoming dashboard commands
 */
void mqtt_manager_poll(mqtt_manager_t *mqtt);

/**
 * @brief Disconnect and free resources
 */
void mqtt_manager_close(mqtt_manager_t *mqtt);

#ifdef __cplusplus
}
#endif

#endif /* MQTT_MANAGER_H */


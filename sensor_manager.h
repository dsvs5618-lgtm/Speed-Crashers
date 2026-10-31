/**
 * @file sensor_manager.h
 * @brief Physical Sensor Ingestion: Front LiDAR (UART) & Dual Ultrasonics (Left/Right)
 */

#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include "acc_types.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    vehicle_operating_mode_t current_mode;
    int  lidar_fd;
    bool lidar_connected;
    bool gpio_initialized;

    /* Interactive Sensor Fault Injection (QNX Isolation Proof) */
    bool fault_inject_front;
    bool fault_inject_left;
    bool fault_inject_right;

    /* Filtered Sensor Caches */
    float last_valid_front_dist_m;
    float last_valid_left_dist_m;
    float last_valid_right_dist_m;

    uint64_t last_lidar_rx_time_ns;
    uint64_t last_left_rx_time_ns;
    uint64_t last_right_rx_time_ns;

    /* Timeout counters */
    uint32_t lidar_timeout_count;
    uint32_t ultrasonic_timeout_count;
} sensor_manager_t;

/**
 * @brief Initialize sensor manager, configure UART and GPIO lines
 */
bool sensor_manager_init(sensor_manager_t *mgr, vehicle_operating_mode_t initial_mode);

/**
 * @brief Safe Mode Switching (flushes buffers and cleanly isolates data paths)
 */
void sensor_manager_set_mode(sensor_manager_t *mgr, vehicle_operating_mode_t new_mode);

/**
 * @brief Read all active sensors according to operating mode
 * In MODE_SIMULATED_PHYSICS: physical sensors are idle (virtual lead scenario).
 * In MODE_LIVE_SENSORS: physical LiDAR (UART) and Dual Ultrasonics (Shared Trig) are actively sampled.
 */
bool sensor_manager_read(sensor_manager_t *mgr, sensor_raw_data_t *raw_out);

/**
 * @brief Interactive Fault Injection Toggle
 * @param sensor_id 1 = Front LiDAR, 2 = Left Ultrasonic, 3 = Right Ultrasonic
 */
void sensor_manager_toggle_fault(sensor_manager_t *mgr, int sensor_id);

/**
 * @brief Cleanup and release hardware handles
 */
void sensor_manager_close(sensor_manager_t *mgr);

#ifdef __cplusplus
}
#endif

#endif /* SENSOR_MANAGER_H */

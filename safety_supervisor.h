/**
 * @file safety_supervisor.h
 * @brief ASIL-D Safety Supervisor, TTC Evaluator & Watchdog Interlock (Priority 255)
 */

#ifndef SAFETY_SUPERVISOR_H
#define SAFETY_SUPERVISOR_H

#include "acc_types.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    /* Watchdog Heartbeat Tracking */
    uint64_t last_heartbeat_sensor_ms;
    uint64_t last_heartbeat_estimator_ms;
    uint64_t last_heartbeat_controller_ms;
    uint64_t last_heartbeat_actuator_ms;

    uint32_t consecutive_deadline_misses;
    bool     watchdog_tripped;

    /* Safety States */
    acc_fsm_state_t current_state;
    safety_status_t status;
    bool            degraded_mode; /* Sensor fault isolated, system continues operating */

    /* Plausibility Tracking */
    float    prev_valid_distance_m;
    uint64_t prev_valid_timestamp_ns;
} safety_supervisor_t;

/**
 * @brief Initialize the safety supervisor and watchdog timers
 */
void safety_supervisor_init(safety_supervisor_t *sup);

/**
 * @brief Calculate Time-To-Collision (TTC) with safety clamping
 * @param distance_m Relative distance to lead target
 * @param rel_velocity_mps Relative velocity (v_lead - v_ego). Negative means closing.
 * @return TTC in seconds (or TTC_INFINITE_VALUE_SEC if gap opening)
 */
float safety_supervisor_calc_ttc(float distance_m, float rel_velocity_mps);

/**
 * @brief Register thread heartbeat (watchdog kick)
 */
void safety_supervisor_kick_watchdog(safety_supervisor_t *sup, uint32_t task_id);

/**
 * @brief Verify sensor plausibility (detect impossible spatial jumps)
 */
bool safety_supervisor_check_plausibility(safety_supervisor_t *sup, 
                                          float new_dist_m, 
                                          float ego_speed_mps, 
                                          float dt_sec);

/**
 * @brief Execute the ASIL-D Supervisory Check (200 Hz / 5ms loop)
 * Arbitrates FSM transitions, triggers AEB override if collision threat detected,
 * verifies rear sensor clearance before reverse, and enforces Safe Torque Off.
 */
void safety_supervisor_evaluate(safety_supervisor_t *sup,
                                const estimated_state_t *state,
                                const sensor_raw_data_t *raw_sensors,
                                control_command_t *cmd,
                                vehicle_operating_mode_t mode);

/**
 * @brief Get human-readable state name string
 */
const char* safety_supervisor_get_state_str(acc_fsm_state_t state);

#ifdef __cplusplus
}
#endif

#endif /* SAFETY_SUPERVISOR_H */


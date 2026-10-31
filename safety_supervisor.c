/**
 * @file safety_supervisor.c
 * @brief ISO 26262 ASIL-D Safety Supervisor, TTC Engine & Multi-Sensor Safety Monitor
 *
 * Priority Hierarchy:
 * - Priority 1: Front PONO LiDAR Tracking (Closing speed, Distance, AEB Trigger)
 * - Priority 2: Left & Right Ultrasonic Cut-in Detection (Side clearance < 30cm)
 * - Priority 3: Visual Signaling (3 LEDs: Cruise, Normal, Dual-Function Brake/AEB)
 */

#include "safety_supervisor.h"
#include "acc_config.h"
#include "qnx_compat.h"
#include <string.h>
#include <math.h>

void safety_supervisor_init(safety_supervisor_t *sup) {
    if (!sup) return;
    memset(sup, 0, sizeof(safety_supervisor_t));

    uint64_t now_ms = qnx_get_time_ms();
    sup->last_heartbeat_sensor_ms     = now_ms;
    sup->last_heartbeat_estimator_ms  = now_ms;
    sup->last_heartbeat_controller_ms = now_ms;
    sup->last_heartbeat_actuator_ms   = now_ms;

    sup->current_state = ACC_STATE_IDLE;
    sup->status.time_to_collision_sec = TTC_INFINITE_VALUE_SEC;
    sup->status.safe_headway_dist_m   = STANDSTILL_DISTANCE_M;
    sup->status.sensor_plausibility_ok = true;
    sup->status.left_cutin_detected   = false;
    sup->status.right_cutin_detected  = false;
}

float safety_supervisor_calc_ttc(float distance_m, float rel_velocity_mps) {
    if (distance_m <= 0.05f) {
        return 0.0f; /* Imminent impact */
    }

    /* rel_velocity_mps = v_lead - v_ego. Closing gap is negative */
    if (rel_velocity_mps >= -0.05f) {
        return TTC_INFINITE_VALUE_SEC; /* Gap opening or matching speed */
    }

    float closing_speed_mps = -rel_velocity_mps;
    float ttc = distance_m / closing_speed_mps;

    return (ttc > 0.0f) ? ttc : 0.0f;
}

void safety_supervisor_kick_watchdog(safety_supervisor_t *sup, uint32_t task_id) {
    if (!sup) return;
    uint64_t now_ms = qnx_get_time_ms();

    switch (task_id) {
        case TASK_PRIO_SENSOR_INGEST:
            sup->last_heartbeat_sensor_ms = now_ms;
            break;
        case TASK_PRIO_STATE_ESTIMATOR:
            sup->last_heartbeat_estimator_ms = now_ms;
            break;
        case TASK_PRIO_ACC_CONTROLLER:
            sup->last_heartbeat_controller_ms = now_ms;
            break;
        case TASK_PRIO_MOTOR_ACTUATOR:
            sup->last_heartbeat_actuator_ms = now_ms;
            break;
        default:
            break;
    }
}

bool safety_supervisor_check_plausibility(safety_supervisor_t *sup, 
                                          float new_dist_m, 
                                          float ego_speed_mps, 
                                          float dt_sec) {
    if (!sup) return false;

    if (sup->prev_valid_distance_m <= 0.0f) {
        sup->prev_valid_distance_m = new_dist_m;
        sup->prev_valid_timestamp_ns = qnx_get_time_ns();
        return true;
    }

    /* Physical limits: maximum relative closing speed ~ 35 m/s */
    float max_delta = (ego_speed_mps + 35.0f) * dt_sec + 1.0f;
    float actual_delta = fabsf(new_dist_m - sup->prev_valid_distance_m);

    if (actual_delta > max_delta && actual_delta > SENSOR_PLAUSIBILITY_MAX_JUMP_M) {
        return false; /* Unphysical sensor jump rejected */
    }

    sup->prev_valid_distance_m = new_dist_m;
    sup->prev_valid_timestamp_ns = qnx_get_time_ns();
    return true;
}

void safety_supervisor_evaluate(safety_supervisor_t *sup,
                                const estimated_state_t *state,
                                const sensor_raw_data_t *raw_sensors,
                                control_command_t *cmd,
                                vehicle_operating_mode_t mode) {
    if (!sup || !state || !cmd) return;

    uint64_t now_ms = qnx_get_time_ms();

    /* ---------------------------------------------------------------------- */
    /* 1. Real-Time Task Watchdog Heartbeat Evaluation                        */
    /* ---------------------------------------------------------------------- */
    uint64_t max_silent_ms = 0;
    if (now_ms > sup->last_heartbeat_sensor_ms) {
        uint64_t dt = now_ms - sup->last_heartbeat_sensor_ms;
        if (dt > max_silent_ms) max_silent_ms = dt;
    }
    if (now_ms > sup->last_heartbeat_estimator_ms) {
        uint64_t dt = now_ms - sup->last_heartbeat_estimator_ms;
        if (dt > max_silent_ms) max_silent_ms = dt;
    }
    if (now_ms > sup->last_heartbeat_controller_ms) {
        uint64_t dt = now_ms - sup->last_heartbeat_controller_ms;
        if (dt > max_silent_ms) max_silent_ms = dt;
    }

    if (max_silent_ms > WATCHDOG_TIMEOUT_MS) {
        sup->consecutive_deadline_misses++;
        if (sup->consecutive_deadline_misses >= MAX_CONSECUTIVE_DEADLINE_MISS) {
            sup->watchdog_tripped = true;
            sup->current_state = ACC_STATE_FAULT_SAFE_STATE;
            sup->status.system_fault_code = 0xE001; /* Watchdog Trip */
        }
    } else {
        if (sup->consecutive_deadline_misses > 0) {
            sup->consecutive_deadline_misses--;
        }
    }

    /* ---------------------------------------------------------------------- */
    /* 2. Priority 1: Front LiDAR Distance & TTC Evaluation                   */
    /* ---------------------------------------------------------------------- */
    float ttc = safety_supervisor_calc_ttc(state->relative_distance_m, state->relative_velocity_mps);
    sup->status.time_to_collision_sec = ttc;
    sup->status.ttc_warning_active   = (ttc <= TTC_WARNING_THRESHOLD_SEC);

    bool aeb_trigger = false;
    if (mode == MODE_TABLETOP_SIM) {
        /* Bench demo: trigger AEB on critical TTC or ground close proximity */
        if (ttc <= TTC_CRITICAL_AEB_SEC || state->relative_distance_m <= BENCH_FRONT_AEB_DIST_M) {
            aeb_trigger = true;
        }
    } else {
        /* Live ground testing */
        if (raw_sensors && raw_sensors->front_valid) {
            if (raw_sensors->front_distance_m <= BENCH_FRONT_AEB_DIST_M || ttc <= TTC_CRITICAL_AEB_SEC) {
                aeb_trigger = true;
            }
        }
    }
    sup->status.aeb_preempt_active = aeb_trigger;

    /* ---------------------------------------------------------------------- */
    /* 3. Priority 2: Left & Right Ultrasonic Cut-in Detection (< 30cm)       */
    /* ---------------------------------------------------------------------- */
    bool left_cutin = false;
    bool right_cutin = false;

    if (raw_sensors) {
        if (raw_sensors->left_valid && raw_sensors->left_distance_m > 0.02f &&
            raw_sensors->left_distance_m <= BENCH_SIDE_CUTIN_THRESH_M) {
            left_cutin = true;
        }
        if (raw_sensors->right_valid && raw_sensors->right_distance_m > 0.02f &&
            raw_sensors->right_distance_m <= BENCH_SIDE_CUTIN_THRESH_M) {
            right_cutin = true;
        }
    }

    sup->status.left_cutin_detected  = left_cutin;
    sup->status.right_cutin_detected = right_cutin;

    /* QNX Process Isolation Check: Detect if any sensor channel is offline */
    sup->degraded_mode = false;
    if (raw_sensors) {
        if (!raw_sensors->front_valid || !raw_sensors->left_valid || !raw_sensors->right_valid) {
            sup->degraded_mode = true;
        }
    }

    /* ---------------------------------------------------------------------- */
    /* 4. Finite State Machine (FSM) State Transitions                        */
    /* ---------------------------------------------------------------------- */
    if (sup->watchdog_tripped) {
        sup->current_state = ACC_STATE_FAULT_SAFE_STATE;
    } else if (aeb_trigger) {
        sup->current_state = ACC_STATE_EMERGENCY_BRAKING;
    } else if (left_cutin || right_cutin) {
        /* Side cut-in introduces immediate lateral hazard: apply safety braking */
        sup->current_state = ACC_STATE_NORMAL_BRAKING;
    } else {
        switch (sup->current_state) {
            case ACC_STATE_IDLE:
                if (state->ego_speed_mps > 0.1f || cmd->demanded_accel_mps2 > 0.0f) {
                    sup->current_state = state->target_detected ? ACC_STATE_FOLLOWING : ACC_STATE_CRUISING;
                }
                break;

            case ACC_STATE_CRUISING:
                if (state->target_detected) {
                    sup->current_state = ACC_STATE_FOLLOWING;
                }
                break;

            case ACC_STATE_FOLLOWING:
                if (!state->target_detected) {
                    sup->current_state = ACC_STATE_CRUISING;
                } else if (cmd->demanded_accel_mps2 < -0.6f) {
                    sup->current_state = ACC_STATE_NORMAL_BRAKING;
                } else if (state->ego_speed_mps < 0.05f && state->relative_distance_m < 3.5f) {
                    sup->current_state = ACC_STATE_STOPPED;
                }
                break;

            case ACC_STATE_NORMAL_BRAKING:
                if (state->ego_speed_mps < 0.05f) {
                    sup->current_state = ACC_STATE_STOPPED;
                } else if (cmd->demanded_accel_mps2 >= -0.2f && !left_cutin && !right_cutin) {
                    sup->current_state = state->target_detected ? ACC_STATE_FOLLOWING : ACC_STATE_CRUISING;
                }
                break;

            case ACC_STATE_EMERGENCY_BRAKING:
                /* Latch AEB until vehicle comes to a complete halt */
                if (state->ego_speed_mps < 0.05f) {
                    sup->current_state = ACC_STATE_STOPPED;
                }
                break;

            case ACC_STATE_STOPPED:
                if (cmd->demanded_accel_mps2 > 0.1f && state->relative_distance_m > 3.0f && !aeb_trigger) {
                    sup->current_state = ACC_STATE_FOLLOWING;
                }
                break;

            case ACC_STATE_FAULT_SAFE_STATE:
            default:
                break;
        }
    }

    /* ---------------------------------------------------------------------- */
    /* 5. Actuation & 3-LED Command Enforcement                               */
    /*    - Normal LED (GPIO 21): Normal/Standstill mode                      */
    /*    - Cruise LED (GPIO 20): Cruise/Following mode                       */
    /*    - Brake & AEB LED (GPIO 17): Continuous for Brake, Blinking for AEB */
    /* ---------------------------------------------------------------------- */
    if (sup->current_state == ACC_STATE_FAULT_SAFE_STATE) {
        /* Safe Stop: 0% PWM, short brake */
        cmd->demanded_accel_mps2 = DECEL_AEB_EMERGENCY_MPS2;
        cmd->motor_pwm_left_pct  = 0.0f;
        cmd->motor_pwm_right_pct = 0.0f;
        cmd->led_normal_mode     = false;
        cmd->led_cruise_mode     = false;
        cmd->led_brake_solid     = false;
        cmd->led_aeb_blink       = true; /* Rapid strobe hazard */
        cmd->brake_light_active  = true;
        cmd->led_strobe_active   = true;
        cmd->safe_torque_off     = true;
    } else if (sup->current_state == ACC_STATE_EMERGENCY_BRAKING) {
        /* AEB Preemption: Zero throttle, maximum deceleration, Blinking LED */
        cmd->demanded_accel_mps2 = DECEL_AEB_EMERGENCY_MPS2;
        cmd->motor_pwm_left_pct  = 0.0f;
        cmd->motor_pwm_right_pct = 0.0f;
        cmd->led_normal_mode     = false;
        cmd->led_cruise_mode     = false;
        cmd->led_brake_solid     = false;
        cmd->led_aeb_blink       = true; /* 10Hz Blinking Strobe */
        cmd->brake_light_active  = true;
        cmd->led_strobe_active   = true;
        cmd->safe_torque_off     = false;
    } else if (sup->current_state == ACC_STATE_NORMAL_BRAKING) {
        /* Normal Service Deceleration: Solid Brake LED */
        if (left_cutin || right_cutin) {
            /* Cap acceleration to negative deceleration buffer */
            if (cmd->demanded_accel_mps2 > -1.5f) {
                cmd->demanded_accel_mps2 = -1.5f;
            }
        }
        cmd->led_normal_mode    = false;
        cmd->led_cruise_mode    = true;  /* Active ACC deceleration */
        cmd->led_brake_solid    = true;  /* Continuous Solid Brake LED */
        cmd->led_aeb_blink      = false;
        cmd->brake_light_active = true;
        cmd->led_strobe_active  = false;
        cmd->safe_torque_off    = false;
    } else if (sup->current_state == ACC_STATE_CRUISING || sup->current_state == ACC_STATE_FOLLOWING) {
        /* Cruise / Following: Cruise Green LED ON */
        bool decel_active = (cmd->demanded_accel_mps2 < -0.3f);
        cmd->led_normal_mode    = false;
        cmd->led_cruise_mode    = true;  /* Green Cruise LED ON */
        cmd->led_brake_solid    = decel_active; /* Solid if slowing down */
        cmd->led_aeb_blink      = false;
        cmd->brake_light_active = decel_active;
        cmd->led_strobe_active  = false;
        cmd->safe_torque_off    = false;
    } else {
        /* ACC_STATE_IDLE or ACC_STATE_STOPPED: Normal LED ON */
        bool is_stopped = (sup->current_state == ACC_STATE_STOPPED);
        if (is_stopped) {
            cmd->motor_pwm_left_pct  = 0.0f;
            cmd->motor_pwm_right_pct = 0.0f;
        }
        cmd->led_normal_mode    = true;  /* Normal / Standby Blue/Yellow LED */
        cmd->led_cruise_mode    = false;
        cmd->led_brake_solid    = is_stopped; /* Hold brake at standstill */
        cmd->led_aeb_blink      = false;
        cmd->brake_light_active = is_stopped;
        cmd->led_strobe_active  = false;
        cmd->safe_torque_off    = false;
    }
}

const char* safety_supervisor_get_state_str(acc_fsm_state_t state) {
    switch (state) {
        case ACC_STATE_IDLE:                return "IDLE";
        case ACC_STATE_CRUISING:            return "CRUISING";
        case ACC_STATE_FOLLOWING:           return "FOLLOWING";
        case ACC_STATE_NORMAL_BRAKING:      return "NORMAL_BRAKING";
        case ACC_STATE_EMERGENCY_BRAKING:   return "EMERGENCY_BRAKING (AEB)";
        case ACC_STATE_STOPPED:             return "STOPPED";
        case ACC_STATE_FAULT_SAFE_STATE:    return "FAULT_SAFE_STATE";
        default:                            return "UNKNOWN";
    }
}

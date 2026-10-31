/**
 * @file acc_types.h
 * @brief Common Types, Enums, and Telemetry Structures for QNX ACC System
 */

#ifndef ACC_TYPES_H
#define ACC_TYPES_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/* 1. Operating Modes & FSM States                                            */
/* ========================================================================== */

typedef enum {
    MODE_SIMULATED_PHYSICS = 0, /* Mode 0: Simulated lead vehicle, bench wheels spin   */
    MODE_LIVE_SENSORS      = 1  /* Mode 1: Live physical LiDAR & Dual Ultrasonics       */
} vehicle_operating_mode_t;

/* Backward compatibility & clarity aliases */
#define MODE_TABLETOP_SIM      MODE_SIMULATED_PHYSICS
#define MODE_GROUND_LIVE       MODE_LIVE_SENSORS

/**
 * @brief ISO 26262 ASIL-D Compliant FSM States
 * Namespaced with ACC_STATE_ to prevent macro collisions with native QNX Neutrino states
 */
typedef enum {
    ACC_STATE_IDLE                = 0, /* Standby, waiting for engagement      */
    ACC_STATE_CRUISING            = 1, /* Open road ahead, tracking set speed  */
    ACC_STATE_FOLLOWING           = 2, /* Lead car detected, maintaining gap   */
    ACC_STATE_NORMAL_BRAKING      = 3, /* Normal service deceleration          */
    ACC_STATE_EMERGENCY_BRAKING   = 4, /* Critical TTC < 1.4s, AEB engaged     */
    ACC_STATE_STOPPED             = 5, /* Controlled standstill                */
    ACC_STATE_FAULT_SAFE_STATE    = 6  /* Safe stop / motor short-brake        */
} acc_fsm_state_t;

/* Backward compatibility aliases */
#ifndef STATE_IDLE
#define STATE_IDLE                ACC_STATE_IDLE
#endif
#ifndef STATE_CRUISING
#define STATE_CRUISING            ACC_STATE_CRUISING
#endif
#ifndef STATE_FOLLOWING
#define STATE_FOLLOWING           ACC_STATE_FOLLOWING
#endif
#ifndef STATE_NORMAL_BRAKING
#define STATE_NORMAL_BRAKING      ACC_STATE_NORMAL_BRAKING
#endif
#ifndef STATE_EMERGENCY_BRAKING
#define STATE_EMERGENCY_BRAKING   ACC_STATE_EMERGENCY_BRAKING
#endif
#ifndef STATE_FAULT_SAFE_STATE
#define STATE_FAULT_SAFE_STATE    ACC_STATE_FAULT_SAFE_STATE
#endif

typedef enum {
    DATA_SOURCE_SIMULATION = 0,
    DATA_SOURCE_PHYSICAL   = 1
} data_source_t;

/* ========================================================================== */
/* 2. Sensor Ingestion & State Estimation Frames                             */
/* ========================================================================== */

/**
 * @brief Raw Sensor Readings (Front LiDAR + Left & Right Ultrasonics)
 */
typedef struct {
    uint64_t timestamp_ns;
    data_source_t source;

    /* Front Tracking: PONO TSD10 ToF LiDAR (Priority 1) */
    float    front_distance_m;
    float    front_doppler_vel_mps;
    bool     front_valid;
    uint8_t  front_signal_quality;

    /* Side Ultrasonic Sensors (Priority 2 Cut-in Detection, Shared Trigger) */
    float    left_distance_m;
    bool     left_valid;

    float    right_distance_m;
    bool     right_valid;

    /* Sensor Health & Fault Isolation Flags */
    bool     front_fault_injected;
    bool     left_fault_injected;
    bool     right_fault_injected;
} sensor_raw_data_t;

/**
 * @brief Filtered System State (Output from 3-State Kalman Filter)
 */
typedef struct {
    uint64_t timestamp_ns;

    /* Ego Vehicle Dynamics */
    float    ego_speed_mps;
    float    ego_speed_kmh;
    float    ego_accel_mps2;

    /* Lead Vehicle Target (from Front LiDAR) */
    bool     target_detected;
    float    relative_distance_m;
    float    relative_velocity_mps;
    float    lead_speed_mps;
    float    lead_speed_kmh;
    float    lead_accel_mps2;

    /* Side Obstacles (from Dual Ultrasonics) */
    float    left_distance_m;
    bool     left_cutin_hazard;
    float    right_distance_m;
    bool     right_cutin_hazard;

    /* Filter Confidence & Health */
    float    estimation_variance;
    bool     tracking_healthy;
    uint32_t consecutive_dropped_frames;
} estimated_state_t;

/* ========================================================================== */
/* 3. Actuation Command Frame                                                */
/* ========================================================================== */

/**
 * @brief Output Commands to DFRobot DRI0044 & 3 Dedicated LEDs
 */
typedef struct {
    uint64_t timestamp_ns;

    float    demanded_accel_mps2;
    float    speed_error_mps;
    float    distance_error_m;

    /* DFRobot DRI0044 (TB6612FNG) Motor PWM (-100.0% to +100.0%) */
    float    motor_pwm_left_pct;
    float    motor_pwm_right_pct;

    /* 3 Visual Indicator LEDs:
     * - LED 1: Brake & AEB (GPIO 17) -> Solid for Service Brake, Blinking for AEB
     * - LED 2: Normal Mode (GPIO 21)
     * - LED 3: Cruise Mode (GPIO 20)
     */
    bool     led_normal_mode;      /* Blue/Yellow LED: Normal/Standstill Mode */
    bool     led_cruise_mode;      /* Green LED: Adaptive Cruise Active       */
    bool     led_brake_solid;      /* Red LED: Continuous Solid Service Brake */
    bool     led_aeb_blink;        /* Red LED: Rapid 10Hz Blinking Strobe     */

    /* Compatibility aliases */
    bool     brake_light_active;
    bool     led_strobe_active;
    bool     safe_torque_off;
    bool     reversing_engaged;
} control_command_t;

/* ========================================================================== */
/* 4. Safety Supervisor & Collision Risk Frame                                */
/* ========================================================================== */

typedef struct {
    float    time_to_collision_sec;
    float    safe_headway_dist_m;
    bool     ttc_warning_active;   /* TTC < 2.4s                             */
    bool     aeb_preempt_active;   /* TTC < 1.4s                             */
    bool     left_cutin_detected;  /* Left obstacle < 30cm                   */
    bool     right_cutin_detected; /* Right obstacle < 30cm                  */
    bool     sensor_plausibility_ok;
    bool     watchdog_tripped;
    uint32_t system_fault_code;
} safety_status_t;

/* ========================================================================== */
/* 5. Simulator Controls                                                      */
/* ========================================================================== */

typedef struct {
    float    target_speed_kmh;
    float    target_accel_mps2;
    bool     sudden_brake_active;
    bool     cut_in_left_active;
    bool     cut_in_right_active;
    bool     cut_in_event_active;  /* General cut-in alias                   */
    bool     lead_reverse_active;  /* Lead stops/reverses test               */
    bool     sim_paused;

    /* Interactive Sensor Fault Injection */
    bool     fault_inject_front;
    bool     fault_inject_left;
    bool     fault_inject_right;
} sim_lead_command_t;

/* ========================================================================== */
/* 6. Unified Telemetry Frame (MQTT / Shared Memory / OLED)                  */
/* ========================================================================== */

typedef struct {
    uint64_t                 seq_num;
    uint64_t                 timestamp_ms;
    vehicle_operating_mode_t current_mode;
    acc_fsm_state_t          fsm_state;

    /* Speeds & Limits */
    float                    ego_speed_kmh;
    float                    lead_speed_kmh;
    float                    configured_max_speed_kmh;

    /* Distances & Relative Motion */
    float                    relative_distance_m;
    float                    safe_distance_m;
    float                    relative_velocity_mps;
    float                    time_to_collision_sec;

    /* Acceleration & Actuation */
    float                    commanded_accel_mps2;
    float                    motor_pwm_percent;
    bool                     led_normal_active;    /* Normal mode LED (GPIO 21) */
    bool                     led_cruise_active;    /* Cruise mode LED (GPIO 20) */
    bool                     brake_active;         /* Service brake active      */
    bool                     aeb_active;           /* AEB blinking active       */
    bool                     reverse_active;
    bool                     safe_torque_off_active;

    /* Proximity Sensors */
    float                    front_sensor_dist_m;  /* Front PONO LiDAR          */
    float                    left_sensor_dist_m;   /* Left Ultrasonic           */
    float                    right_sensor_dist_m;  /* Right Ultrasonic          */
    bool                     left_cutin_detected;  /* Left obstacle < 30cm      */
    bool                     right_cutin_detected; /* Right obstacle < 30cm     */

    /* QNX Process Isolation & Fault Tolerance Evidence */
    bool                     front_sensor_ok;
    bool                     left_sensor_ok;
    bool                     right_sensor_ok;
    bool                     degraded_mode_active; /* Operating safely with partial sensors */

    /* QNX RTOS Real-Time Performance Evidence */
    uint32_t                 rtos_latency_us;      /* Microsecond loop latency  */
    uint32_t                 max_jitter_us;        /* Max scheduling jitter     */
    bool                     watchdog_ok;
    uint32_t                 deadline_miss_count;  /* Hard real-time audit      */

    char                     state_name[32];
    char                     alert_message[64];
} acc_telemetry_t;

#ifdef __cplusplus
}
#endif

#endif /* ACC_TYPES_H */

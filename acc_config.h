/**
 * @file acc_config.h
 * @brief Global Configuration & Hardware Tuning Parameters for QNX ACC & AEB System
 *
 * Designed specifically for Raspberry Pi 4B on BlackBerry QNX Neutrino RTOS SDP 8.0 / 7.1.
 *
 * Hardware Architecture:
 * - Front: 1x PONO TSD10 ToF LiDAR (UART /dev/ser2)
 * - Sides: 2x Ultrasonic Sensors (Left & Right) with SINGLE SHARED TRIG Pin
 * - Motors: 2x N20 Geared Motors + Encoders via DFRobot DRI0044 (TB6612FNG, No STBY pin)
 * - Visual Actuators: 3 LEDs:
 *     1. Brake & AEB LED (GPIO 17) -> Continuous Solid for Brake, Rapid Strobe for AEB
 *     2. Normal Mode LED (GPIO 21) -> Blue/Yellow Indicator
 *     3. Cruise Mode LED (GPIO 20) -> Green Indicator
 * - Display: SSD1306 I2C OLED (/dev/i2c-1) displaying QNX low-latency RTOS evidence
 */

#ifndef ACC_CONFIG_H
#define ACC_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/* 1. RTOS Scheduling Priorities & Task Periods (Rate Monotonic Scheduling)   */
/* ========================================================================== */
#define TASK_PRIO_SAFETY_SUPERVISOR   255  /* Priority 255: ASIL-D AEB & Interlocks  */
#define TASK_PRIO_SENSOR_INGEST       240  /* Priority 240: LiDAR & Dual Ultrasonic  */
#define TASK_PRIO_STATE_ESTIMATOR     220  /* Priority 220: 3-State Kalman Filter    */
#define TASK_PRIO_ACC_CONTROLLER      200  /* Priority 200: Longitudinal PID & CTG  */
#define TASK_PRIO_MOTOR_ACTUATOR      180  /* Priority 180: Motor PWM & 3 LEDs HAL   */
#define TASK_PRIO_OLED_DISPLAY        100  /* Priority 100: OLED RTOS Diagnostics    */
#define TASK_PRIO_MQTT_CLIENT          80  /* Priority 80:  Wireless Telemetry       */

/* Execution Periods in Milliseconds */
#define PERIOD_SAFETY_MS              5    /* 200 Hz                                 */
#define PERIOD_SENSOR_INGEST_MS       10   /* 100 Hz                                 */
#define PERIOD_ESTIMATOR_MS           10   /* 100 Hz                                 */
#define PERIOD_CONTROLLER_MS          20   /* 50 Hz                                  */
#define PERIOD_ACTUATOR_MS            20   /* 50 Hz                                  */
#define PERIOD_OLED_MS                100  /* 10 Hz (OLED update)                    */
#define PERIOD_MQTT_MS                50   /* 20 Hz                                  */

/* ========================================================================== */
/* 2. Longitudinal Vehicle Limits & Regulations                              */
/* ========================================================================== */
#define VEHICLE_DEFAULT_SET_SPEED_KMH 60.0f /* Configured cruising set speed         */
#define VEHICLE_MAX_SPEED_LIMIT_KMH   80.0f /* Hard speed limit: never exceed        */
#define VEHICLE_MIN_ACTIVE_SPEED_KMH  0.0f  /* Minimum operational speed             */

#define ACCEL_MAX_COMFORT_MPS2        2.0f  /* Max positive acceleration             */
#define DECEL_MAX_SERVICE_MPS2       -3.5f  /* Max normal braking deceleration       */
#define DECEL_AEB_EMERGENCY_MPS2     -7.5f  /* Autonomous Emergency Braking (AEB)    */
#define JERK_LIMIT_MAX_MPS3           2.5f  /* ISO 15622 ride comfort jerk limit     */

/* Conversion constants */
#define KMH_TO_MPS(v)                 ((v) * 0.27777778f)
#define MPS_TO_KMH(v)                 ((v) * 3.6f)

/* Physical scale for bench car */
#define SPEED_SCALE_KMH_TO_PWM_PCT    (100.0f / VEHICLE_MAX_SPEED_LIMIT_KMH)

/* ========================================================================== */
/* 3. Safe Spacing & Time Headway Policy (ISO 15622)                         */
/* ========================================================================== */
#define HEADWAY_TIME_GAP_SEC          1.5f  /* Desired time headway gap (h_gap)      */
#define STANDSTILL_DISTANCE_M         3.0f  /* Standstill separation distance (d_0)  */
#define CRITICAL_STOP_DISTANCE_M      1.2f  /* Hard minimum crash avoidance gap      */
#define RADAR_DETECTION_RANGE_M       120.0f/* Maximum tracking range                */

/* Tabletop / Physical Obstacle Thresholds */
#define BENCH_FRONT_AEB_DIST_M        0.25f /* 25 cm emergency stop threshold        */
#define BENCH_FRONT_WARN_DIST_M       0.50f /* 50 cm alert following range           */
#define BENCH_SIDE_CUTIN_THRESH_M     0.30f /* 30 cm side cut-in warning threshold   */

/* ========================================================================== */
/* 4. Time-To-Collision (TTC) Safety Thresholds                               */
/* ========================================================================== */
#define TTC_WARNING_THRESHOLD_SEC     2.4f  /* Advisory warning threshold            */
#define TTC_CRITICAL_AEB_SEC          1.4f  /* Autonomous Emergency Braking trigger  */
#define TTC_INFINITE_VALUE_SEC        999.0f/* Safe gap / opening distance           */

/* ========================================================================== */
/* 5. Dual-Loop PID Controller Gains                                         */
/* ========================================================================== */
#define PID_KP_SPEED                  0.80f
#define PID_KI_SPEED                  0.05f
#define PID_KD_SPEED                  0.02f

#define PID_KP_DIST                   0.45f
#define PID_KI_DIST                   0.02f
#define PID_KD_DIST                   0.12f
#define PID_FEEDFORWARD_LEAD_ACCEL    0.80f

#define PID_INTEGRAL_MAX              3.0f
#define PID_INTEGRAL_MIN             -3.0f

/* ========================================================================== */
/* 6. Extended Kalman Filter (EKF) Noise Covariances                         */
/* ========================================================================== */
#define EKF_SIGMA_PROCESS_ACCEL       0.50f
#define EKF_SIGMA_MEAS_LIDAR_DIST     0.05f
#define EKF_SIGMA_MEAS_RADAR_VEL      0.15f
#define EKF_STALE_DATA_TIMEOUT_MS     150

/* ========================================================================== */
/* 7. ASIL-D Watchdog & Plausibility Tolerances                               */
/* ========================================================================== */
#define WATCHDOG_TIMEOUT_MS           60    /* Heartbeat loss threshold (ms)         */
#define MAX_CONSECUTIVE_DEADLINE_MISS 3     /* Fail-safe trip count                  */
#define SENSOR_PLAUSIBILITY_MAX_JUMP_M 5.0f /* Impossible single-frame jump (m)      */

/* ========================================================================== */
/* 8. Hardware Pinout Definition (Raspberry Pi 4 40-Pin Header)              */
/* ========================================================================== */
/* DFRobot DRI0044 Dual Motor Driver (TB6612FNG) - No STBY pin wired          */
#define PIN_MOTOR_PWMA                18    /* GPIO 18 (Pin 12, PWM0) - Left Speed   */
#define PIN_MOTOR_PWMB                13    /* GPIO 13 (Pin 33, PWM1) - Right Speed  */
#define PIN_MOTOR_AIN1                23    /* GPIO 23 (Pin 16) - Left Motor Dir 1   */
#define PIN_MOTOR_AIN2                24    /* GPIO 24 (Pin 18) - Left Motor Dir 2   */
#define PIN_MOTOR_BIN1                27    /* GPIO 27 (Pin 13) - Right Motor Dir 1  */
#define PIN_MOTOR_BIN2                22    /* GPIO 22 (Pin 15) - Right Motor Dir 2  */
#define PIN_MOTOR_STBY                (-1)  /* No STBY pin: Controlled via PWM=0/Brk */

/* N20 Metal Geared Motor Encoders (Quadrature Hall) */
#define PIN_ENCODER_LEFT_A            5     /* GPIO 5  (Pin 29)                      */
#define PIN_ENCODER_LEFT_B            6     /* GPIO 6  (Pin 31)                      */
#define PIN_ENCODER_RIGHT_A           19    /* GPIO 19 (Pin 35)                      */
#define PIN_ENCODER_RIGHT_B           26    /* GPIO 26 (Pin 37)                      */

/* Front Distance Sensor: PONO TSD10 ToF LiDAR (UART) */
#define PIN_PONO_LIDAR_TXD            14    /* GPIO 14 (Pin 8, TXD)                  */
#define PIN_PONO_LIDAR_RXD            15    /* GPIO 15 (Pin 10, RXD)                 */
#define PONO_LIDAR_UART_DEV           "/dev/ser2" /* QNX PL011 UART driver           */

/* Dual Ultrasonic Sensors: Left & Right with SINGLE SHARED TRIG PIN */
#define PIN_US_SHARED_TRIG            4     /* GPIO 4  (Pin 7)  - Shared Trigger Pin */
#define PIN_US_LEFT_ECHO              16    /* GPIO 16 (Pin 36) - Left Echo (via 1k/2k)*/
#define PIN_US_RIGHT_ECHO             12    /* GPIO 12 (Pin 32) - Right Echo(via 1k/2k)*/

/* Dedicated 3 Visual Indicator LEDs:
 * 1. Brake & AEB LED (Dual-Function: Solid ON = Service Brake; Blinking = AEB)
 * 2. Normal Mode LED
 * 3. Cruise Mode LED
 */
#define PIN_LED_BRAKE_AEB             17    /* GPIO 17 (Pin 11) - Red Brake & AEB Strobe */
#define PIN_LED_NORMAL_MODE           21    /* GPIO 21 (Pin 40) - Blue/Yellow Normal Mode*/
#define PIN_LED_CRUISE_MODE           20    /* GPIO 20 (Pin 38) - Green Cruise Mode      */
#define LED_AEB_STROBE_FREQ_HZ        10    /* AEB rapid strobe frequency (10 Hz)        */

/* Backward compatibility aliases */
#define PIN_LED_BRAKE                 PIN_LED_BRAKE_AEB
#define PIN_LED_AEB                   PIN_LED_BRAKE_AEB
#define PIN_LED_BRAKE_WARN            PIN_LED_BRAKE_AEB

/* SSD1306 I2C OLED Display */
#define OLED_I2C_DEV                  "/dev/i2c-1"
#define OLED_I2C_ADDR                 0x3C

/* ========================================================================== */
/* 8. Wireless MQTT Broker & Network Topics                                  */
/* ========================================================================== */
#define MQTT_DEFAULT_BROKER_HOST      "192.168.137.1"
#define MQTT_DEFAULT_BROKER_PORT      1883
#define MQTT_CLIENT_ID                "QNX_ACC_EGO_CAR"
#define MQTT_KEEPALIVE_SEC            10

#define MQTT_TOPIC_TELEMETRY          "qnx_acc/telemetry"
#define MQTT_TOPIC_COMMAND            "qnx_acc/command"
#define MQTT_TOPIC_SIM_LEAD           "qnx_acc/sim_lead"
#define MQTT_TOPIC_HEARTBEAT          "qnx_acc/heartbeat"
#define MQTT_TOPIC_ALERT              "qnx_acc/alert"

#ifdef __cplusplus
}
#endif

#endif /* ACC_CONFIG_H */

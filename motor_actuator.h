/**
 * @file motor_actuator.h
 * @brief DFRobot DRI0044 Dual Motor Driver (No STBY Pin) & 3-LED Actuation HAL
 *
 * Actuator Mapping:
 * - DFRobot DRI0044 (TB6612FNG Dual 1.2A DC Driver, No STBY pin wired)
 *     Left Motor:  PWMA (GPIO 18), AIN1 (GPIO 23), AIN2 (GPIO 24)
 *     Right Motor: PWMB (GPIO 13), BIN1 (GPIO 27), BIN2 (GPIO 22)
 *     Safe stop: Short-brake via AIN1/2 & BIN1/2 + 0% PWM
 *
 * - 3 Dedicated Visual Indicator LEDs:
 *     1. Brake & AEB LED (GPIO 17, Pin 11) -> Solid for Service Brake; 10Hz Blinking for AEB
 *     2. Normal Mode LED (GPIO 21, Pin 40) -> Blue/Yellow
 *     3. Cruise Mode LED (GPIO 20, Pin 38) -> Green
 */

#ifndef MOTOR_ACTUATOR_H
#define MOTOR_ACTUATOR_H

#include "acc_types.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool     is_initialized;
    float    current_left_pwm;    /* -100.0 to +100.0%                  */
    float    current_right_pwm;

    /* Encoders */
    int64_t  left_encoder_ticks;
    int64_t  right_encoder_ticks;
    float    measured_ground_speed_mps;

    /* 3 Visual Indicator LED States */
    bool     led_normal_on;       /* GPIO 21: Normal/Standstill Mode    */
    bool     led_cruise_on;       /* GPIO 20: Cruise Mode Active        */
    bool     led_brake_aeb_on;    /* GPIO 17: Brake/AEB output state    */
    uint32_t aeb_strobe_counter;  /* 10Hz Strobe cycle counter          */

    /* Compatibility aliases */
    bool     brake_led_on;
    bool     stby_engaged;
} motor_actuator_t;

/**
 * @brief Initialize GPIO and PWM channels for DFRobot DRI0044 and 3 LEDs
 */
bool motor_actuator_init(motor_actuator_t *actuator);

/**
 * @brief Apply longitudinal control commands to DFRobot DRI0044 and 3 LEDs
 */
void motor_actuator_apply_command(motor_actuator_t *actuator, const control_command_t *cmd);

/**
 * @brief Safe Stop: Sets PWM to 0% and engages short-circuit dynamic motor braking
 */
void motor_actuator_safe_stop(motor_actuator_t *actuator);

/**
 * @brief Read measured vehicle speed from Hall wheel encoders
 */
float motor_actuator_get_speed_mps(motor_actuator_t *actuator);

void motor_actuator_close(motor_actuator_t *actuator);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_ACTUATOR_H */

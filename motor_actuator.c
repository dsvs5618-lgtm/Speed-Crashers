/**
 * @file motor_actuator.c
 * @brief DFRobot DRI0044 Dual Motor Driver (No STBY Pin) & 3-LED Actuation HAL
 */

#include "motor_actuator.h"
#include "acc_config.h"
#include "qnx_compat.h"
#include <string.h>
#include <math.h>

#ifdef __QNX__
    #include <hw/inout.h>
#endif

bool motor_actuator_init(motor_actuator_t *actuator) {
    if (!actuator) return false;
    memset(actuator, 0, sizeof(motor_actuator_t));

    actuator->is_initialized = true;
    actuator->stby_engaged = true;

    /* Initial visual state: Standby/Normal Mode LED active */
    actuator->led_normal_on    = true;
    actuator->led_cruise_on    = false;
    actuator->led_brake_aeb_on = false;
    actuator->brake_led_on     = false;

#ifdef __QNX__
    /* On QNX Neutrino on Raspberry Pi 4B:
     * DFRobot DRI0044 Motor Driver Pins:
     * - GPIO 18 (PWM0, Pin 12): Left Motor Speed
     * - GPIO 13 (PWM1, Pin 33): Right Motor Speed
     * - GPIO 23 (Pin 16): AIN1 Left Dir 1
     * - GPIO 24 (Pin 18): AIN2 Left Dir 2
     * - GPIO 27 (Pin 13): BIN1 Right Dir 1
     * - GPIO 22 (Pin 15): BIN2 Right Dir 2
     *
     * 3 Dedicated Visual Signaling LEDs:
     * - GPIO 17 (Pin 11): Dual-Function Brake & AEB Strobe LED
     * - GPIO 21 (Pin 40): Normal Mode LED
     * - GPIO 20 (Pin 38): Cruise Mode LED
     */
#endif

    return true;
}

void motor_actuator_apply_command(motor_actuator_t *actuator, const control_command_t *cmd) {
    if (!actuator || !cmd || !actuator->is_initialized) return;

    /* Safe Torque Off / Emergency Stop Check */
    if (cmd->safe_torque_off) {
        motor_actuator_safe_stop(actuator);
        return;
    }

    /* ---------------------------------------------------------------------- */
    /* 1. Motor Speed & Direction (DFRobot DRI0044 TB6612FNG)                  */
    /* ---------------------------------------------------------------------- */
    float left_pwm  = cmd->motor_pwm_left_pct;
    float right_pwm = cmd->motor_pwm_right_pct;

    if (left_pwm > 100.0f) left_pwm = 100.0f;
    if (left_pwm < -100.0f) left_pwm = -100.0f;
    if (right_pwm > 100.0f) right_pwm = 100.0f;
    if (right_pwm < -100.0f) right_pwm = -100.0f;

    actuator->current_left_pwm  = left_pwm;
    actuator->current_right_pwm = right_pwm;

#ifdef __QNX__
    if (left_pwm > 0.0f) {
        /* Forward Drive */
        /* AIN1=HIGH, AIN2=LOW; BIN1=HIGH, BIN2=LOW */
        /* Write left_pwm to PWM0 (GPIO 18), right_pwm to PWM1 (GPIO 13) */
    } else if (left_pwm < 0.0f) {
        /* Reverse Drive */
        /* AIN1=LOW, AIN2=HIGH; BIN1=LOW, BIN2=HIGH */
    } else {
        /* Electric Short-Brake (Safe stop without STBY pin) */
        /* AIN1=HIGH, AIN2=HIGH; BIN1=HIGH, BIN2=HIGH; PWM=0 */
    }
#endif

    /* ---------------------------------------------------------------------- */
    /* 2. Visual Signaling Across the 3 Dedicated LEDs                        */
    /* ---------------------------------------------------------------------- */
    /* LED 2: Normal Mode LED (GPIO 21, Pin 40) */
    actuator->led_normal_on = cmd->led_normal_mode;

    /* LED 3: Cruise Mode LED (GPIO 20, Pin 38) */
    actuator->led_cruise_on = cmd->led_cruise_mode;

    /* LED 1: Dual-Function Brake & AEB LED (GPIO 17, Pin 11)
     * - Rapid Strobe / Blinking when in AEB
     * - Continuous Solid ON when Service Braking
     * - OFF when cruising or idle
     */
    if (cmd->led_aeb_blink || cmd->led_strobe_active) {
        actuator->aeb_strobe_counter++;
        actuator->led_brake_aeb_on = (actuator->aeb_strobe_counter % 2 == 0);
    } else if (cmd->led_brake_solid || cmd->brake_light_active) {
        actuator->led_brake_aeb_on = true;
    } else {
        actuator->led_brake_aeb_on = false;
    }

    actuator->brake_led_on = actuator->led_brake_aeb_on;

#ifdef __QNX__
    /* Write to hardware GPIOs:
     * - GPIO 17 -> actuator->led_brake_aeb_on
     * - GPIO 21 -> actuator->led_normal_on
     * - GPIO 20 -> actuator->led_cruise_on
     */
#endif
}

void motor_actuator_safe_stop(motor_actuator_t *actuator) {
    if (!actuator) return;

    actuator->current_left_pwm  = 0.0f;
    actuator->current_right_pwm = 0.0f;

    /* Visual safety state under Safe Stop: Rapid Hazard Blinking */
    actuator->led_normal_on    = false;
    actuator->led_cruise_on    = false;
    actuator->aeb_strobe_counter++;
    actuator->led_brake_aeb_on = (actuator->aeb_strobe_counter % 2 == 0);
    actuator->brake_led_on     = true;

#ifdef __QNX__
    /* Motor Short-Brake: Short motor leads to ground/VCC to dissipate EMF */
    /* AIN1=HIGH, AIN2=HIGH; BIN1=HIGH, BIN2=HIGH; Duty=0% */
#endif
}

float motor_actuator_get_speed_mps(motor_actuator_t *actuator) {
    if (!actuator) return 0.0f;

    float avg_pwm = (fabsf(actuator->current_left_pwm) + fabsf(actuator->current_right_pwm)) * 0.5f;
    float estimated_mps = KMH_TO_MPS(avg_pwm * (VEHICLE_MAX_SPEED_LIMIT_KMH / 100.0f));

    if (actuator->current_left_pwm < 0.0f) {
        estimated_mps = -estimated_mps;
    }
    return estimated_mps;
}

void motor_actuator_close(motor_actuator_t *actuator) {
    if (!actuator) return;
    motor_actuator_safe_stop(actuator);
    actuator->is_initialized = false;
}

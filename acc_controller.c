/**
 * @file acc_controller.c
 * @brief Longitudinal ACC Controller Implementation with Dual-Loop PID and Jerk Limiter
 */

#include "acc_controller.h"
#include "acc_config.h"
#include <math.h>

void acc_controller_init(acc_controller_t *ctrl, float default_set_kmh, float max_limit_kmh) {
    if (!ctrl) return;

    ctrl->hard_max_speed_kmh = (max_limit_kmh > 0.0f) ? max_limit_kmh : VEHICLE_MAX_SPEED_LIMIT_KMH;
    
    if (default_set_kmh > ctrl->hard_max_speed_kmh) {
        ctrl->user_set_speed_kmh = ctrl->hard_max_speed_kmh;
    } else if (default_set_kmh >= 0.0f) {
        ctrl->user_set_speed_kmh = default_set_kmh;
    } else {
        ctrl->user_set_speed_kmh = VEHICLE_DEFAULT_SET_SPEED_KMH;
    }

    ctrl->time_gap_sec       = HEADWAY_TIME_GAP_SEC;
    ctrl->standstill_gap_m   = STANDSTILL_DISTANCE_M;

    ctrl->speed_integral     = 0.0f;
    ctrl->prev_speed_error   = 0.0f;
    ctrl->dist_integral      = 0.0f;
    ctrl->prev_dist_error    = 0.0f;
    ctrl->prev_commanded_accel_mps2 = 0.0f;
    ctrl->fsm_state          = ACC_STATE_IDLE;
}

void acc_controller_set_target_speed(acc_controller_t *ctrl, float set_speed_kmh) {
    if (!ctrl) return;
    if (set_speed_kmh > ctrl->hard_max_speed_kmh) {
        ctrl->user_set_speed_kmh = ctrl->hard_max_speed_kmh;
    } else if (set_speed_kmh < 0.0f) {
        ctrl->user_set_speed_kmh = 0.0f;
    } else {
        ctrl->user_set_speed_kmh = set_speed_kmh;
    }
}

void acc_controller_set_max_speed(acc_controller_t *ctrl, float max_speed_kmh) {
    if (!ctrl || max_speed_kmh <= 0.0f) return;
    ctrl->hard_max_speed_kmh = max_speed_kmh;
    if (ctrl->user_set_speed_kmh > ctrl->hard_max_speed_kmh) {
        ctrl->user_set_speed_kmh = ctrl->hard_max_speed_kmh;
    }
}

float acc_controller_calc_safe_distance(const acc_controller_t *ctrl, float ego_speed_mps) {
    if (!ctrl) return STANDSTILL_DISTANCE_M;
    if (ego_speed_mps < 0.0f) ego_speed_mps = 0.0f;

    /* Constant Time Gap Spacing Law: d_desired = d0 + h_gap * v_ego */
    float d_safe = ctrl->standstill_gap_m + (ctrl->time_gap_sec * ego_speed_mps);
    return d_safe;
}

void acc_controller_compute(acc_controller_t *ctrl, 
                            const estimated_state_t *state, 
                            float dt_sec, 
                            control_command_t *out_cmd) {
    if (!ctrl || !state || !out_cmd || dt_sec <= 0.0f) return;

    float v_ego = state->ego_speed_mps;
    if (v_ego < 0.0f) v_ego = 0.0f;

    /* Strict Maximum Speed Enforcement: v_target never exceeds hard_max_speed */
    float v_target_kmh = ctrl->user_set_speed_kmh;
    if (v_target_kmh > ctrl->hard_max_speed_kmh) {
        v_target_kmh = ctrl->hard_max_speed_kmh;
    }
    float v_target_mps = KMH_TO_MPS(v_target_kmh);

    /* ---------------------------------------------------------------------- */
    /* 1. Speed Control Loop (Empty road cruising)                            */
    /* ---------------------------------------------------------------------- */
    float speed_err = v_target_mps - v_ego;
    ctrl->speed_integral += speed_err * dt_sec;

    /* Anti-windup clamping */
    if (ctrl->speed_integral > PID_INTEGRAL_MAX) ctrl->speed_integral = PID_INTEGRAL_MAX;
    if (ctrl->speed_integral < PID_INTEGRAL_MIN) ctrl->speed_integral = PID_INTEGRAL_MIN;

    float speed_deriv = (speed_err - ctrl->prev_speed_error) / dt_sec;
    ctrl->prev_speed_error = speed_err;

    float a_speed = (PID_KP_SPEED * speed_err) + 
                    (PID_KI_SPEED * ctrl->speed_integral) + 
                    (PID_KD_SPEED * speed_deriv);

    /* ---------------------------------------------------------------------- */
    /* 2. Distance Control Loop (Vehicle following)                           */
    /* ---------------------------------------------------------------------- */
    float a_dist = a_speed; /* Default if no target */
    float d_safe = acc_controller_calc_safe_distance(ctrl, v_ego);
    float dist_err = 0.0f;

    if (state->target_detected) {
        dist_err = state->relative_distance_m - d_safe;
        ctrl->dist_integral += dist_err * dt_sec;

        if (ctrl->dist_integral > PID_INTEGRAL_MAX) ctrl->dist_integral = PID_INTEGRAL_MAX;
        if (ctrl->dist_integral < PID_INTEGRAL_MIN) ctrl->dist_integral = PID_INTEGRAL_MIN;

        float dist_deriv = state->relative_velocity_mps; /* d(dist_err)/dt = v_rel */
        ctrl->prev_dist_error = dist_err;

        /* Distance control with Lead Vehicle Acceleration Feedforward */
        a_dist = (PID_KP_DIST * dist_err) + 
                 (PID_KI_DIST * ctrl->dist_integral) + 
                 (PID_KD_DIST * dist_deriv) + 
                 (PID_FEEDFORWARD_LEAD_ACCEL * state->lead_accel_mps2);
    } else {
        /* No target: gently decay distance integral */
        ctrl->dist_integral *= 0.95f;
    }

    /* ---------------------------------------------------------------------- */
    /* 3. Dynamic Acceleration Arbitration: min(a_speed, a_dist)              */
    /* ---------------------------------------------------------------------- */
    float a_arbitrated;
    if (state->target_detected) {
        /* Always pick the most conservative (lowest) acceleration */
        a_arbitrated = (a_dist < a_speed) ? a_dist : a_speed;
    } else {
        a_arbitrated = a_speed;
    }

    /* Bound commanded acceleration within comfort and service braking limits */
    if (a_arbitrated > ACCEL_MAX_COMFORT_MPS2) a_arbitrated = ACCEL_MAX_COMFORT_MPS2;
    if (a_arbitrated < DECEL_MAX_SERVICE_MPS2) a_arbitrated = DECEL_MAX_SERVICE_MPS2;

    /* ---------------------------------------------------------------------- */
    /* 4. ISO 15622 Slew-Rate Jerk Limiting (|da/dt| <= 2.5 m/s^3)             */
    /* ---------------------------------------------------------------------- */
    float max_delta_a = JERK_LIMIT_MAX_MPS3 * dt_sec;
    float delta_a = a_arbitrated - ctrl->prev_commanded_accel_mps2;

    if (delta_a > max_delta_a) {
        a_arbitrated = ctrl->prev_commanded_accel_mps2 + max_delta_a;
    } else if (delta_a < -max_delta_a) {
        a_arbitrated = ctrl->prev_commanded_accel_mps2 - max_delta_a;
    }
    ctrl->prev_commanded_accel_mps2 = a_arbitrated;

    /* ---------------------------------------------------------------------- */
    /* 5. Map Acceleration to TB6612 Motor PWM and Brake Actuation            */
    /* ---------------------------------------------------------------------- */
    out_cmd->demanded_accel_mps2 = a_arbitrated;
    out_cmd->speed_error_mps     = speed_err;
    out_cmd->distance_error_m    = dist_err;
    out_cmd->timestamp_ns        = state->timestamp_ns;

    if (a_arbitrated >= 0.0f) {
        /* Positive Acceleration / Cruising Throttle */
        float throttle_ratio = a_arbitrated / ACCEL_MAX_COMFORT_MPS2;
        /* Base throttle proportional to target speed */
        float base_pwm = (v_ego / KMH_TO_MPS(ctrl->hard_max_speed_kmh)) * 70.0f;
        float pwm = base_pwm + (throttle_ratio * 30.0f);
        if (pwm > 100.0f) pwm = 100.0f;
        if (pwm < 0.0f)   pwm = 0.0f;

        out_cmd->motor_pwm_left_pct  = pwm;
        out_cmd->motor_pwm_right_pct = pwm;
        out_cmd->brake_light_active  = false;
        out_cmd->led_strobe_active   = false;
        out_cmd->safe_torque_off     = false;
        out_cmd->reversing_engaged   = false;
    } else {
        /* Service Braking */
        float brake_intensity = fabsf(a_arbitrated) / fabsf(DECEL_MAX_SERVICE_MPS2);
        float pwm = (1.0f - brake_intensity) * (v_ego / KMH_TO_MPS(ctrl->hard_max_speed_kmh)) * 40.0f;
        if (pwm < 0.0f) pwm = 0.0f;

        out_cmd->motor_pwm_left_pct  = pwm;
        out_cmd->motor_pwm_right_pct = pwm;
        out_cmd->brake_light_active  = true;  /* Engage brake light */
        out_cmd->led_strobe_active   = false;
        out_cmd->safe_torque_off     = false;
        out_cmd->reversing_engaged   = false;
    }
}

void acc_controller_reset_integrators(acc_controller_t *ctrl) {
    if (!ctrl) return;
    ctrl->speed_integral = 0.0f;
    ctrl->dist_integral  = 0.0f;
    ctrl->prev_speed_error = 0.0f;
    ctrl->prev_dist_error  = 0.0f;
    ctrl->prev_commanded_accel_mps2 = 0.0f;
}


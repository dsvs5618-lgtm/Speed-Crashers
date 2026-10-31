/**
 * @file acc_controller.h
 * @brief Longitudinal ACC Controller, CTG Spacing Law & ISO 15622 Jerk Limiter
 */

#ifndef ACC_CONTROLLER_H
#define ACC_CONTROLLER_H

#include "acc_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    /* Setpoints & Constraints */
    float user_set_speed_kmh;
    float hard_max_speed_kmh;
    float time_gap_sec;
    float standstill_gap_m;

    /* PID States for Speed Control Loop */
    float speed_integral;
    float prev_speed_error;

    /* PID States for Distance Control Loop */
    float dist_integral;
    float prev_dist_error;

    /* Jerk Limiter State */
    float prev_commanded_accel_mps2;

    /* Current FSM State */
    acc_fsm_state_t fsm_state;
} acc_controller_t;

/**
 * @brief Initialize the ACC controller with configuration limits
 */
void acc_controller_init(acc_controller_t *ctrl, float default_set_kmh, float max_limit_kmh);

/**
 * @brief Update the user's cruising target speed with strict clamping
 */
void acc_controller_set_target_speed(acc_controller_t *ctrl, float set_speed_kmh);

/**
 * @brief Update the hard maximum allowed speed
 */
void acc_controller_set_max_speed(acc_controller_t *ctrl, float max_speed_kmh);

/**
 * @brief Compute dynamic safe following distance based on speed and relative motion
 */
float acc_controller_calc_safe_distance(const acc_controller_t *ctrl, float ego_speed_mps);

/**
 * @brief Execute the dual-loop control algorithm and arbitrate acceleration command
 * @param ctrl Controller instance
 * @param state Filtered vehicle state
 * @param dt_sec Delta execution time (typically 0.020s for 50Hz loop)
 * @param out_cmd Resulting control command
 */
void acc_controller_compute(acc_controller_t *ctrl, 
                            const estimated_state_t *state, 
                            float dt_sec, 
                            control_command_t *out_cmd);

/**
 * @brief Reset PID error integrators (e.g., after braking or mode change)
 */
void acc_controller_reset_integrators(acc_controller_t *ctrl);

#ifdef __cplusplus
}
#endif

#endif /* ACC_CONTROLLER_H */


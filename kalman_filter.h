/**
 * @file kalman_filter.h
 * @brief 3-State Extended Kalman Filter for Sensor Fusion and Tracking
 *
 * State vector: x = [ d_rel, v_rel, a_lead ]^T
 * Filters high-frequency measurement noise, jitter, and dropped frames from LiDAR/Radar.
 */

#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include "acc_types.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    /* State Vector: [0]=distance (m), [1]=rel_velocity (m/s), [2]=lead_accel (m/s^2) */
    float x[3];

    /* Error Covariance Matrix (3x3) */
    float P[3][3];

    /* Process Noise Covariance Q (3x3) */
    float Q[3][3];

    /* Measurement Noise Covariance R */
    float R_dist;
    float R_vel;

    /* Diagnostics */
    uint64_t last_update_time_ns;
    uint32_t consecutive_dropped_frames;
    bool     is_initialized;
    float    innovation_dist;
} kalman_filter_t;

/**
 * @brief Initialize the Kalman Filter state and covariances
 */
void kalman_filter_init(kalman_filter_t *kf, float initial_dist, float initial_v_rel);

/**
 * @brief Prediction Step (Physics state extrapolation based on vehicle kinematics)
 * @param kf Pointer to filter instance
 * @param dt_sec Delta time in seconds
 * @param ego_accel_mps2 Current ego vehicle measured/commanded acceleration
 */
void kalman_filter_predict(kalman_filter_t *kf, float dt_sec, float ego_accel_mps2);

/**
 * @brief Measurement Update Step (Correction with new sensor frame)
 * @param kf Pointer to filter instance
 * @param measured_dist Distance reading from sensor (m)
 * @param measured_vel Relative velocity reading from sensor (m/s)
 * @param has_vel_measurement True if direct Doppler/velocity reading is available
 * @param signal_quality Sensor confidence/quality (0 to 100%)
 * @return true if measurement accepted, false if rejected as outlier (plausibility check)
 */
bool kalman_filter_update(kalman_filter_t *kf, 
                          float measured_dist, 
                          float measured_vel, 
                          bool has_vel_measurement, 
                          uint8_t signal_quality);

/**
 * @brief Extract filtered state into estimated_state_t
 */
void kalman_filter_get_state(const kalman_filter_t *kf, 
                             float current_ego_speed_mps, 
                             estimated_state_t *out_state);

/**
 * @brief Reset filter to clean state (e.g. during target loss or mode switch)
 */
void kalman_filter_reset(kalman_filter_t *kf);

#ifdef __cplusplus
}
#endif

#endif /* KALMAN_FILTER_H */


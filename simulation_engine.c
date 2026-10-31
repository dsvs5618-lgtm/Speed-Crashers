/**
 * @file simulation_engine.c
 * @brief Tabletop Demonstration Mode Physics Engine & Synthetic Sensor Generation
 */

#include "simulation_engine.h"
#include "acc_config.h"
#include <string.h>
#include <math.h>

static float generate_gaussian_noise(uint32_t *seed, float std_dev) {
    *seed = (*seed * 1103515245 + 12345) & 0x7fffffff;
    float u1 = (float)(*seed) / 2147483648.0f;
    *seed = (*seed * 1103515245 + 12345) & 0x7fffffff;
    float u2 = (float)(*seed) / 2147483648.0f;

    if (u1 < 0.0001f) u1 = 0.0001f;
    float z0 = sqrtf(-2.0f * logf(u1)) * cosf(2.0f * 3.14159265f * u2);
    return z0 * std_dev;
}

void simulation_engine_init(simulation_engine_t *sim) {
    if (!sim) return;
    memset(sim, 0, sizeof(simulation_engine_t));

    sim->lead_speed_mps        = KMH_TO_MPS(55.0f);
    sim->lead_accel_mps2       = 0.0f;
    sim->relative_distance_m   = 35.0f;
    sim->target_speed_mps      = KMH_TO_MPS(55.0f);
    sim->sudden_braking_active = false;
    sim->cut_in_left_active    = false;
    sim->cut_in_right_active   = false;
    sim->is_paused             = false;
    sim->left_sim_dist_m       = 2.5f; /* 2.5m (clear) */
    sim->right_sim_dist_m      = 2.5f; /* 2.5m (clear) */
    sim->rng_seed              = 1234567;
}

void simulation_engine_apply_command(simulation_engine_t *sim, const sim_lead_command_t *cmd) {
    if (!sim || !cmd) return;

    if (cmd->target_speed_kmh > 0.0f) {
        sim->target_speed_mps = KMH_TO_MPS(cmd->target_speed_kmh);
    }
    sim->sudden_braking_active = cmd->sudden_brake_active;
    sim->cut_in_left_active    = cmd->cut_in_left_active || cmd->cut_in_event_active;
    sim->cut_in_right_active   = cmd->cut_in_right_active;
    sim->is_paused             = cmd->sim_paused;

    if (cmd->target_accel_mps2 != 0.0f) {
        sim->lead_accel_mps2 = cmd->target_accel_mps2;
    }
}

void simulation_engine_step(simulation_engine_t *sim, 
                            float dt_sec, 
                            float ego_speed_mps, 
                            sensor_raw_data_t *out_sensors) {
    if (!sim || !out_sensors || dt_sec <= 0.0f) return;

    if (sim->is_paused) return;

    /* 1. Integrate Lead Vehicle Dynamics */
    if (sim->sudden_braking_active) {
        sim->lead_accel_mps2 = -7.0f; /* Emergency braking scenario */
        sim->lead_speed_mps += sim->lead_accel_mps2 * dt_sec;
        if (sim->lead_speed_mps < 0.0f) {
            sim->lead_speed_mps = 0.0f;
            sim->lead_accel_mps2 = 0.0f;
        }
    } else {
        /* Smooth convergence to target speed */
        float speed_err = sim->target_speed_mps - sim->lead_speed_mps;
        sim->lead_accel_mps2 = speed_err * 0.8f;
        if (sim->lead_accel_mps2 > ACCEL_MAX_COMFORT_MPS2)  sim->lead_accel_mps2 = ACCEL_MAX_COMFORT_MPS2;
        if (sim->lead_accel_mps2 < DECEL_MAX_SERVICE_MPS2)  sim->lead_accel_mps2 = DECEL_MAX_SERVICE_MPS2;

        sim->lead_speed_mps += sim->lead_accel_mps2 * dt_sec;
        if (sim->lead_speed_mps < 0.0f) sim->lead_speed_mps = 0.0f;
    }

    /* 2. Integrate Relative Distance */
    float rel_velocity = sim->lead_speed_mps - ego_speed_mps;
    sim->relative_distance_m += rel_velocity * dt_sec;
    if (sim->relative_distance_m < 0.15f) sim->relative_distance_m = 0.15f;
    if (sim->relative_distance_m > RADAR_DETECTION_RANGE_M) sim->relative_distance_m = RADAR_DETECTION_RANGE_M;

    /* 3. Side Cut-in Obstacle Simulation (< 30cm) */
    if (sim->cut_in_left_active) {
        sim->left_sim_dist_m = 0.20f; /* 20 cm cut-in */
    } else {
        sim->left_sim_dist_m = 2.5f;
    }

    if (sim->cut_in_right_active) {
        sim->right_sim_dist_m = 0.20f; /* 20 cm cut-in */
    } else {
        sim->right_sim_dist_m = 2.5f;
    }

    /* 4. Synthesize Sensor Measurements with Gaussian Noise */
    float lidar_noise = generate_gaussian_noise(&sim->rng_seed, EKF_SIGMA_MEAS_LIDAR_DIST);
    float measured_dist = sim->relative_distance_m + lidar_noise;
    if (measured_dist < 0.1f) measured_dist = 0.1f;

    out_sensors->front_distance_m       = measured_dist;
    out_sensors->front_doppler_vel_mps  = rel_velocity;
    out_sensors->front_valid            = true;
    out_sensors->front_signal_quality   = 95;

    out_sensors->left_distance_m        = sim->left_sim_dist_m;
    out_sensors->left_valid             = true;

    out_sensors->right_distance_m       = sim->right_sim_dist_m;
    out_sensors->right_valid            = true;
}

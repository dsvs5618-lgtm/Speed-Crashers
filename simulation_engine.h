/**
 * @file simulation_engine.h
 * @brief Tabletop Demonstration Mode Physics Engine & Synthetic Sensor Generator
 */

#ifndef SIMULATION_ENGINE_H
#define SIMULATION_ENGINE_H

#include "acc_types.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    /* Lead Vehicle State */
    float lead_speed_mps;
    float lead_accel_mps2;
    float relative_distance_m;

    /* Operator Scenario Directives */
    float target_speed_mps;
    bool  sudden_braking_active;
    bool  cut_in_left_active;
    bool  cut_in_right_active;
    bool  is_paused;

    /* Side Obstacles (Left & Right Ultrasonics) */
    float left_sim_dist_m;
    float right_sim_dist_m;

    uint32_t rng_seed;
} simulation_engine_t;

void simulation_engine_init(simulation_engine_t *sim);
void simulation_engine_apply_command(simulation_engine_t *sim, const sim_lead_command_t *cmd);
void simulation_engine_step(simulation_engine_t *sim, 
                            float dt_sec, 
                            float ego_speed_mps, 
                            sensor_raw_data_t *out_sensors);

#ifdef __cplusplus
}
#endif

#endif /* SIMULATION_ENGINE_H */

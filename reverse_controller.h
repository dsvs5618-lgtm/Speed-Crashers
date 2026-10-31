/**
 * @file reverse_controller.h
 * @brief Reverse Controller Stub (No rear ultrasonic sensor installed)
 */

#ifndef REVERSE_CONTROLLER_H
#define REVERSE_CONTROLLER_H

#include "acc_types.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool  reverse_requested;
    bool  is_active;
} reverse_controller_t;

void reverse_controller_init(reverse_controller_t *rev);
bool reverse_controller_is_permitted(const reverse_controller_t *rev, float ego_speed_mps);
bool reverse_controller_compute(reverse_controller_t *rev, float *out_pwm);

#ifdef __cplusplus
}
#endif

#endif /* REVERSE_CONTROLLER_H */

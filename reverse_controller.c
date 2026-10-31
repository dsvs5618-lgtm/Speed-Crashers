/**
 * @file reverse_controller.c
 * @brief Reverse Controller Stub
 */

#include "reverse_controller.h"
#include <string.h>

void reverse_controller_init(reverse_controller_t *rev) {
    if (!rev) return;
    memset(rev, 0, sizeof(reverse_controller_t));
}

bool reverse_controller_is_permitted(const reverse_controller_t *rev, float ego_speed_mps) {
    (void)rev;
    (void)ego_speed_mps;
    return false; /* Reversing disabled without rear obstacle sensor */
}

bool reverse_controller_compute(reverse_controller_t *rev, float *out_pwm) {
    (void)rev;
    if (out_pwm) *out_pwm = 0.0f;
    return false;
}

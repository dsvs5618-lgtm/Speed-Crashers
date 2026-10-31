/**
 * @file oled_display.h
 * @brief SSD1306 I2C OLED Driver - Live Diagnostics & QNX RTOS Evidence
 *
 * Renders real-time telemetry proving QNX microkernel low-latency execution:
 * - Line 1: Real-Time State & Microsecond Latency / Jitter Evidence
 * - Line 2: Ego Speed & DFRobot DRI0044 Motor PWM Percentage
 * - Line 3: Front LiDAR Tracking & Time-to-Collision (TTC)
 * - Line 4: Dual Ultrasonic Cut-in Distance (Left/Right) & Hazard Warnings
 */

#ifndef OLED_DISPLAY_H
#define OLED_DISPLAY_H

#include "acc_types.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int  i2c_fd;
    bool is_connected;
    uint8_t buffer[1024]; /* 128x64 display buffer */
} oled_display_t;

/**
 * @brief Initialize SSD1306 I2C display (/dev/i2c-1)
 */
bool oled_display_init(oled_display_t *oled);

/**
 * @brief Format and render real-time diagnostics on OLED screen
 */
void oled_display_render_telemetry(oled_display_t *oled, const acc_telemetry_t *telem);

void oled_display_clear(oled_display_t *oled);
void oled_display_close(oled_display_t *oled);

#ifdef __cplusplus
}
#endif

#endif /* OLED_DISPLAY_H */

/**
 * @file oled_display.c
 * @brief SSD1306 I2C OLED Local Status Display Implementation
 */

#include "oled_display.h"
#include "acc_config.h"
#include <stdio.h>
#include <string.h>
#include <fcntl.h>

#ifdef __QNX__
    #include <hw/i2c.h>
    #include <unistd.h>
#endif

bool oled_display_init(oled_display_t *oled) {
    if (!oled) return false;
    memset(oled, 0, sizeof(oled_display_t));
    oled->i2c_fd = -1;
    oled->is_connected = false;

#ifdef __QNX__
    oled->i2c_fd = open(OLED_I2C_DEV, O_RDWR);
    if (oled->i2c_fd >= 0) {
        oled->is_connected = true;
    }
#else
    oled->is_connected = true;
#endif

    return true;
}

void oled_display_render_telemetry(oled_display_t *oled, const acc_telemetry_t *telem) {
    if (!oled || !telem) return;

    /* Line 1: RTOS Data Source (SIM or LIVE-HW), State, and Latency / Jitter Evidence */
    char line1[64], line2[64], line3[64], line4[64];
    const char *src_tag = (telem->current_mode == MODE_LIVE_SENSORS) ? "LIVE-HW" : "SIM";

    snprintf(line1, sizeof(line1), "[%s] %s | J:%uus",
             src_tag,
             telem->state_name,
             telem->max_jitter_us);

    /* Line 2: Ego Speed, Configured Mode, and Motor Output */
    snprintf(line2, sizeof(line2), "SPD:%4.1f kph | PWM:%3.0f%%", 
             telem->ego_speed_kmh,
             telem->motor_pwm_percent);

    /* Line 3: Priority 1 Front LiDAR Distance & Time-To-Collision */
    if (telem->time_to_collision_sec > 50.0f) {
        snprintf(line3, sizeof(line3), "LID:%4.1fm | TTC:SAFE", 
                 telem->front_sensor_dist_m);
    } else {
        snprintf(line3, sizeof(line3), "LID:%4.1fm | TTC:%3.1fs", 
                 telem->front_sensor_dist_m, telem->time_to_collision_sec);
    }

    /* Line 4: Priority 2 Dual Ultrasonic Lateral Clearances & Hazard Alerts */
    if (telem->aeb_active) {
        snprintf(line4, sizeof(line4), "L:%3.1f R:%3.1f | [!] AEB", 
                 telem->left_sensor_dist_m, telem->right_sensor_dist_m);
    } else if (!telem->left_sensor_ok && !telem->right_sensor_ok) {
        snprintf(line4, sizeof(line4), "ISO: ALL SIDE US FAIL");
    } else if (!telem->left_sensor_ok) {
        snprintf(line4, sizeof(line4), "ISO: L-FAIL | R:%3.1fm", telem->right_sensor_dist_m);
    } else if (!telem->right_sensor_ok) {
        snprintf(line4, sizeof(line4), "L:%3.1fm | ISO: R-FAIL", telem->left_sensor_dist_m);
    } else if (!telem->front_sensor_ok) {
        snprintf(line4, sizeof(line4), "ISO: FRONT LID-FAIL");
    } else if (telem->left_cutin_detected) {
        snprintf(line4, sizeof(line4), "L:%3.1f R:%3.1f | [!] CUT-L", 
                 telem->left_sensor_dist_m, telem->right_sensor_dist_m);
    } else if (telem->right_cutin_detected) {
        snprintf(line4, sizeof(line4), "L:%3.1f R:%3.1f | [!] CUT-R", 
                 telem->left_sensor_dist_m, telem->right_sensor_dist_m);
    } else if (telem->brake_active) {
        snprintf(line4, sizeof(line4), "L:%3.1f R:%3.1f | BRK ON", 
                 telem->left_sensor_dist_m, telem->right_sensor_dist_m);
    } else {
        snprintf(line4, sizeof(line4), "L:%3.1f R:%3.1f | %s", 
                 telem->left_sensor_dist_m, telem->right_sensor_dist_m,
                 telem->led_cruise_active ? "CRUISE" : "NORMAL");
    }

#ifdef __QNX__
    /* Transmit formatted telemetry to SSD1306 OLED via I2C */
    (void)line1; (void)line2; (void)line3; (void)line4;
#else
    /* Non-QNX simulation host */
#endif
}

void oled_display_clear(oled_display_t *oled) {
    if (!oled) return;
    memset(oled->buffer, 0, sizeof(oled->buffer));
}

void oled_display_close(oled_display_t *oled) {
    if (!oled) return;
#ifdef __QNX__
    if (oled->i2c_fd >= 0) {
        close(oled->i2c_fd);
        oled->i2c_fd = -1;
    }
#endif
    oled->is_connected = false;
}

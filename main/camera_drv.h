/**
 * @file camera_drv.h
 * @brief OV2640 camera wrapper for AI-Thinker ESP32-CAM
 */
#pragma once

#include "esp_err.h"
#include "esp_camera.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialise OV2640 camera with AI-Thinker pin map.
 *        Must be called AFTER veml7700_init() when sharing I2C_NUM_0.
 */
esp_err_t camera_init(void);

/**
 * @brief Capture one JPEG frame.
 * @param[out] buf   Pointer receives JPEG data (caller must return with camera_fb_return)
 * @param[out] len   JPEG size in bytes
 * @return ESP_OK on success
 */
esp_err_t camera_capture(uint8_t **buf, size_t *len);

/**
 * @brief Return frame buffer to driver pool.
 */
void camera_fb_release(camera_fb_t *fb);

#ifdef __cplusplus
}
#endif
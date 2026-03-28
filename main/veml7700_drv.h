/**
 * @file veml7700_drv.h
 * @brief VEML7700 ambient light sensor driver with Auto-Gain algorithm
 *        Uses legacy driver/i2c.h API (avoids conflict with esp32-camera).
 */
#pragma once

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialise VEML7700 on I2C_NUM_0.
 *        MUST be called BEFORE camera_init().
 * @param i2c_mutex  Mutex shared with other I2C users (camera task etc.)
 */
esp_err_t veml7700_init(SemaphoreHandle_t i2c_mutex);

/**
 * @brief Read ambient light with auto-gain / auto-integration-time.
 *        Applies datasheet non-linear correction when lux > 1000.
 * @param[out] lux  Corrected illuminance in lux
 * @return ESP_OK on success
 */
esp_err_t veml7700_read_lux(float *lux);

#ifdef __cplusplus
}
#endif
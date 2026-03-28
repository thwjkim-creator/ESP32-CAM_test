/**
 * @file veml7700_drv.c
 * @brief VEML7700 (SEN0228) driver – new driver/i2c_master.h API
 *
 * ESP-IDF 5.5.2의 esp32-camera가 내부적으로 driver_ng(i2c_master)를
 * 등록하므로, legacy driver/i2c.h 사용 시 충돌(abort) 발생.
 * → 신규 i2c_master.h API 사용 + I2C_NUM_1 포트로 분리.
 *
 * 배선: SDA → GPIO 14, SCL → GPIO 15  (I2C_NUM_1)
 * 카메라 SCCB: GPIO 26/27             (I2C_NUM_0, esp32-camera 내부 관리)
 *
 * Auto-Gain algorithm:
 *   raw < 100   → increase gain / integration time  (dark)
 *   raw > 10000 → decrease gain / integration time  (bright)
 *   lux > 1000  → apply datasheet polynomial correction
 */
#include "veml7700_drv.h"

#include <math.h>
#include "driver/i2c_master.h"
#include "esp_log.h"

static const char *TAG = "VEML7700";

/* ── Hardware constants ── */
#define VEML7700_ADDR          0x10

#define VEML7700_REG_ALS_CONF  0x00
#define VEML7700_REG_ALS       0x04
#define VEML7700_REG_WHITE     0x05

/* SDA/SCL for SEN0228 — uses I2C_NUM_1 to avoid camera conflict */
#define I2C_SDA_PIN            14
#define I2C_SCL_PIN            15
#define I2C_FREQ_HZ            100000   /* 100 kHz – safe for VEML7700 */

/* ── Gain / Integration-time tables ── */
typedef struct {
    uint8_t  bits;      /* register value */
    float    scale;     /* multiplier (for reference) */
} gain_entry_t;

typedef struct {
    uint8_t  bits;
    uint16_t ms;        /* integration time in ms */
} it_entry_t;

/*  Gain options  (ALS_GAIN field [12:11])  */
static const gain_entry_t GAINS[] = {
    { 0x00, 1.0f  },   /* x1   */
    { 0x01, 2.0f  },   /* x2   */
    { 0x02, 0.125f},   /* x1/8 */
    { 0x03, 0.25f },   /* x1/4 */
};
#define GAIN_COUNT  (sizeof(GAINS) / sizeof(GAINS[0]))

/*  Integration-time options  (ALS_IT field [9:6])  */
static const it_entry_t ITS[] = {
    { 0x0C, 25  },
    { 0x08, 50  },
    { 0x00, 100 },
    { 0x01, 200 },
    { 0x02, 400 },
    { 0x03, 800 },
};
#define IT_COUNT    (sizeof(ITS) / sizeof(ITS[0]))

/* ── Resolution (lux per count) look-up  –  from datasheet ── */
/*  Rows: gain index 0-3  |  Columns: IT index 0-5              */
static const float RESOLUTION[4][6] = {
    /* Gx1   */ { 0.2304f, 0.1152f, 0.0576f, 0.0288f, 0.0144f, 0.0072f },
    /* Gx2   */ { 0.1152f, 0.0576f, 0.0288f, 0.0144f, 0.0072f, 0.0036f },
    /* Gx1/8 */ { 1.8432f, 0.9216f, 0.4608f, 0.2304f, 0.1152f, 0.0576f },
    /* Gx1/4 */ { 0.9216f, 0.4608f, 0.2304f, 0.1152f, 0.0576f, 0.0288f },
};

/* ── Module state ── */
static SemaphoreHandle_t         s_mutex    = NULL;
static i2c_master_bus_handle_t   s_bus      = NULL;
static i2c_master_dev_handle_t   s_dev      = NULL;
static uint8_t                   s_gain_idx = 0;   /* start at x1   */
static uint8_t                   s_it_idx   = 2;   /* start at 100ms */

/* ────────────────── Low-level I2C helpers (new API) ──────── */

static esp_err_t veml_write_reg(uint8_t reg, uint16_t val)
{
    uint8_t data[3] = { reg, (uint8_t)(val & 0xFF), (uint8_t)(val >> 8) };
    return i2c_master_transmit(s_dev, data, sizeof(data), pdMS_TO_TICKS(100));
}

static esp_err_t veml_read_reg(uint8_t reg, uint16_t *val)
{
    uint8_t cmd = reg;
    uint8_t buf[2] = {0};
    esp_err_t ret = i2c_master_transmit_receive(s_dev, &cmd, 1,
                                                 buf, 2, pdMS_TO_TICKS(100));
    if (ret == ESP_OK) {
        *val = (uint16_t)(buf[1] << 8 | buf[0]);
    }
    return ret;
}

/* ────────────────── Configuration helpers ────────────────── */

static uint16_t build_conf(void)
{
    /*  [12:11] ALS_GAIN | [9:6] ALS_IT | [1:0] ALS_SD=0 (power on) */
    uint16_t conf = 0;
    conf |= ((uint16_t)GAINS[s_gain_idx].bits) << 11;
    conf |= ((uint16_t)ITS[s_it_idx].bits) << 6;
    return conf;
}

static esp_err_t apply_conf(void)
{
    return veml_write_reg(VEML7700_REG_ALS_CONF, build_conf());
}

/* ────────────────── Datasheet non-linear correction ──────── */

static float correct_lux(float raw_lux)
{
    /* Polynomial: 6.0135e-13 x L^4 - 9.3924e-9 x L^3
                 + 8.1488e-5 x L^2 + 1.0023 x L                */
    double L = (double)raw_lux;
    double corrected = 6.0135e-13 * L * L * L * L
                     - 9.3924e-9  * L * L * L
                     + 8.1488e-5  * L * L
                     + 1.0023     * L;
    return (float)corrected;
}

/* ────────────────── Public API ────────────────────────────── */

esp_err_t veml7700_init(SemaphoreHandle_t i2c_mutex)
{
    s_mutex = i2c_mutex;

    /* Create I2C master bus on port 1 (port 0 is used by esp32-camera) */
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port              = I2C_NUM_1,
        .sda_io_num            = I2C_SDA_PIN,
        .scl_io_num            = I2C_SCL_PIN,
        .clk_source            = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt     = 7,
        .flags.enable_internal_pullup = true,
    };
    esp_err_t ret = i2c_new_master_bus(&bus_cfg, &s_bus);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C bus creation failed: 0x%x", ret);
        return ret;
    }

    /* Add VEML7700 device */
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = VEML7700_ADDR,
        .scl_speed_hz    = I2C_FREQ_HZ,
    };
    ret = i2c_master_bus_add_device(s_bus, &dev_cfg, &s_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C device add failed: 0x%x", ret);
        return ret;
    }

    /* Power on with default gain/IT */
    ret = apply_conf();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "VEML7700 not responding (0x%x)", ret);
        return ret;
    }

    /* Wait for first integration cycle */
    vTaskDelay(pdMS_TO_TICKS(ITS[s_it_idx].ms + 10));

    ESP_LOGI(TAG, "VEML7700 initialised on I2C_NUM_1 (SDA=%d, SCL=%d)",
             I2C_SDA_PIN, I2C_SCL_PIN);
    return ESP_OK;
}

esp_err_t veml7700_read_lux(float *lux)
{
    if (!s_mutex || !lux) return ESP_ERR_INVALID_ARG;

    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(500)) != pdTRUE) {
        ESP_LOGW(TAG, "I2C mutex timeout");
        return ESP_ERR_TIMEOUT;
    }

    uint16_t raw = 0;
    esp_err_t ret = veml_read_reg(VEML7700_REG_ALS, &raw);
    if (ret != ESP_OK) {
        xSemaphoreGive(s_mutex);
        return ret;
    }

    /* ── Auto-Gain logic ── */
    bool reconfigured = false;

    if (raw < 100) {
        /* Too dark -> increase sensitivity */
        if (s_it_idx < IT_COUNT - 1) {
            s_it_idx++;
            reconfigured = true;
        } else if (s_gain_idx < 1) {
            s_gain_idx = 1;
            reconfigured = true;
        }
    } else if (raw > 10000) {
        /* Too bright -> decrease sensitivity */
        if (s_it_idx > 0) {
            s_it_idx--;
            reconfigured = true;
        } else if (s_gain_idx != 2 && s_gain_idx != 3) {
            s_gain_idx = 2;
            reconfigured = true;
        }
    }

    if (reconfigured) {
        apply_conf();
        vTaskDelay(pdMS_TO_TICKS(ITS[s_it_idx].ms + 10));
        ret = veml_read_reg(VEML7700_REG_ALS, &raw);
        if (ret != ESP_OK) {
            xSemaphoreGive(s_mutex);
            return ret;
        }
        ESP_LOGD(TAG, "Auto-gain adjusted: gain_idx=%u, it_idx=%u",
                 s_gain_idx, s_it_idx);
    }

    xSemaphoreGive(s_mutex);

    /* Calculate lux */
    float resolution = RESOLUTION[s_gain_idx][s_it_idx];
    float raw_lux = (float)raw * resolution;

    /* Apply non-linear correction above 1000 lux */
    if (raw_lux > 1000.0f) {
        *lux = correct_lux(raw_lux);
    } else {
        *lux = raw_lux;
    }

    ESP_LOGD(TAG, "raw=%u  gain_idx=%u  it_idx=%u  lux=%.2f",
             raw, s_gain_idx, s_it_idx, *lux);
    return ESP_OK;
}
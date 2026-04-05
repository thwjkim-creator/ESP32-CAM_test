/**
 * @file main.c
 * @brief ESP32-CAM + VEML7700 — capture JPEG & lux, send via HTTP/MQTT
 *
 * Tasks:
 *   camera_task   — 10 s period, JPEG → HTTP POST
 *   sensor_task   —  5 s period, lux  → MQTT JSON (QoS 1)
 *
 * Init order (important):
 *   1. VEML7700  (installs legacy I2C driver on I2C_NUM_0)
 *   2. Camera    (uses its own SCCB on GPIO 26/27)
 */
#include <stdio.h>
#include <string.h>
#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_sntp.h"
#include "nvs_flash.h"

#include "esp_http_client.h"
#include "mqtt_client.h"
#include "esp_camera.h"

#include "camera_drv.h"
#include "veml7700_drv.h"

/* ═══════════════════════════════════════════════════════════
 *  Configuration — edit these for your environment
 * ═══════════════════════════════════════════════════════════ */
#define WIFI_SSID              "YS_Start_Up_03"
#define WIFI_PASS              "ys000003"

#define HTTP_POST_URL          "http://172.21.101.29:8080/upload"
#define MQTT_BROKER_URI        "mqtt://172.21.101.29:1883"
#define MQTT_TOPIC_LUX         "sensor/veml7700/lux"
#define MQTT_TOPIC_STATUS      "sensor/veml7700/status"
#define MQTT_LWT_MSG           "{\"status\":\"offline\"}"
#define MQTT_ONLINE_MSG        "{\"status\":\"online\"}"

#define TASK_PERIOD_MS         10000   /* 카메라 + 센서 공통 주기 (10초) */

/* ═══════════════════════════════════════════════════════════ */

static const char *TAG = "MAIN";

/* Event group bits */
#define WIFI_CONNECTED_BIT     BIT0
static EventGroupHandle_t s_wifi_events;

/* Shared I2C mutex */
static SemaphoreHandle_t s_i2c_mutex;

/* MQTT client handle */
static esp_mqtt_client_handle_t s_mqtt_client = NULL;
static volatile bool s_mqtt_connected = false;

/* ────────────────── Wi-Fi ────────────────────────────────── */

static void wifi_event_handler(void *arg, esp_event_base_t base,
                               int32_t id, void *data)
{
    if (base == WIFI_EVENT) {
        switch (id) {
        case WIFI_EVENT_STA_START:
            esp_wifi_connect();
            break;
        case WIFI_EVENT_STA_DISCONNECTED:
            ESP_LOGW(TAG, "Wi-Fi disconnected, reconnecting…");
            xEventGroupClearBits(s_wifi_events, WIFI_CONNECTED_BIT);
            esp_wifi_connect();
            break;
        default:
            break;
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *evt = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&evt->ip_info.ip));
        xEventGroupSetBits(s_wifi_events, WIFI_CONNECTED_BIT);
    }
}

static void wifi_init(void)
{
    s_wifi_events = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL, NULL));

    wifi_config_t wifi_cfg = {
        .sta = {
            .ssid     = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Wi-Fi STA started, connecting to %s …", WIFI_SSID);
}

static void wait_for_wifi(void)
{
    xEventGroupWaitBits(s_wifi_events, WIFI_CONNECTED_BIT,
                        pdFALSE, pdTRUE, portMAX_DELAY);
}

/* ────────────────── SNTP (for timestamps) ────────────────── */

static void sntp_init_time(void)
{
    /* KST 타임존 설정 (localtime_r에서 자동 적용) */
    setenv("TZ", "KST-9", 1);
    tzset();

    ESP_LOGI(TAG, "Initialising SNTP …");
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_init();

    /* Wait up to 15 s for time sync */
    int retry = 0;
    while (esp_sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET && retry < 15) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        retry++;
    }
    if (retry >= 15) {
        ESP_LOGW(TAG, "SNTP sync timeout — timestamps may be epoch");
    } else {
        ESP_LOGI(TAG, "SNTP synchronised");
    }
}

/* ────────────────── MQTT ─────────────────────────────────── */

static void mqtt_event_handler(void *arg, esp_event_base_t base,
                               int32_t id, void *data)
{
    esp_mqtt_event_handle_t evt = (esp_mqtt_event_handle_t)data;

    switch ((esp_mqtt_event_id_t)id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT connected");
        s_mqtt_connected = true;
        /* Publish online status */
        esp_mqtt_client_publish(s_mqtt_client, MQTT_TOPIC_STATUS,
                                MQTT_ONLINE_MSG, 0, 1, 1);
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "MQTT disconnected");
        s_mqtt_connected = false;
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGE(TAG, "MQTT error type=%d", evt->error_handle->error_type);
        break;
    default:
        break;
    }
}

static void mqtt_init(void)
{
    const esp_mqtt_client_config_t cfg = {
        .broker.address.uri = MQTT_BROKER_URI,
        .session = {
            .last_will = {
                .topic   = MQTT_TOPIC_STATUS,
                .msg     = MQTT_LWT_MSG,
                .msg_len = strlen(MQTT_LWT_MSG),
                .qos     = 1,
                .retain  = 1,
            },
            .keepalive = 30,
        },
        .network.reconnect_timeout_ms = 5000,
    };

    s_mqtt_client = esp_mqtt_client_init(&cfg);
    esp_mqtt_client_register_event(s_mqtt_client, ESP_EVENT_ANY_ID,
                                   mqtt_event_handler, NULL);
    esp_mqtt_client_start(s_mqtt_client);
    ESP_LOGI(TAG, "MQTT client started → %s", MQTT_BROKER_URI);
}

/* ────────────────── Timestamp helper ─────────────────────── */

static void get_kst_timestamp(char *buf, size_t len)
{
    time_t now;
    struct tm ti;
    time(&now);
    localtime_r(&now, &ti);
    strftime(buf, len, "%Y-%m-%dT%H:%M:%S+09:00", &ti);
}

/* ────────────────── Unified sensor + camera task ────────── */

static void sensor_camera_task(void *arg)
{
    char ts[40];
    char payload[128];

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(TASK_PERIOD_MS));

        /* ── 1. 타임스탬프 한 번만 생성 (카메라 + 조도 공유) ── */
        get_kst_timestamp(ts, sizeof(ts));

        /* ── 2. 조도 측정 ── */
        float lux = 0.0f;
        bool lux_ok = false;
        esp_err_t ret = veml7700_read_lux(&lux);
        if (ret == ESP_OK) {
            lux_ok = true;
            snprintf(payload, sizeof(payload),
                     "{\"lux\":%.2f,\"timestamp\":\"%s\"}", lux, ts);

            ESP_LOGI(TAG, "Lux=%.2f  ts=%s", lux, ts);

            if (s_mqtt_connected) {
                int msg_id = esp_mqtt_client_publish(
                    s_mqtt_client, MQTT_TOPIC_LUX,
                    payload, 0, /*qos*/1, /*retain*/0);
                if (msg_id < 0) {
                    ESP_LOGW(TAG, "MQTT publish failed");
                }
            } else {
                ESP_LOGW(TAG, "MQTT not connected, skipping publish");
            }
        } else {
            ESP_LOGW(TAG, "Lux read failed: %s", esp_err_to_name(ret));
        }

        /* ── 3. 카메라 촬영 ── */
        if (xSemaphoreTake(s_i2c_mutex, pdMS_TO_TICKS(2000)) != pdTRUE) {
            ESP_LOGW(TAG, "camera: mutex timeout");
            continue;
        }

        camera_fb_t *fb = esp_camera_fb_get();
        xSemaphoreGive(s_i2c_mutex);

        if (!fb) {
            ESP_LOGE(TAG, "Camera capture failed");
            continue;
        }

        ESP_LOGI(TAG, "JPEG captured: %u bytes  ts=%s", (unsigned)fb->len, ts);

        /* HTTP POST */
        esp_http_client_config_t http_cfg = {
            .url    = HTTP_POST_URL,
            .method = HTTP_METHOD_POST,
            .timeout_ms = 10000,
        };
        esp_http_client_handle_t client = esp_http_client_init(&http_cfg);
        esp_http_client_set_header(client, "Content-Type", "image/jpeg");
        esp_http_client_set_header(client, "X-Timestamp", ts);
        esp_http_client_set_post_field(client, (const char *)fb->buf, (int)fb->len);

        esp_err_t err = esp_http_client_perform(client);
        if (err == ESP_OK) {
            int status = esp_http_client_get_status_code(client);
            ESP_LOGI(TAG, "HTTP POST → %d", status);
        } else {
            ESP_LOGE(TAG, "HTTP POST failed: %s", esp_err_to_name(err));
        }

        esp_http_client_cleanup(client);
        esp_camera_fb_return(fb);
    }
}

/* ────────────────── app_main ─────────────────────────────── */

void app_main(void)
{
    /* NVS (required by Wi-Fi) */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* Create shared I2C mutex */
    s_i2c_mutex = xSemaphoreCreateMutex();
    assert(s_i2c_mutex);

    /* ─── 1. VEML7700 first (installs I2C driver on NUM_0) ─── */
    ESP_ERROR_CHECK(veml7700_init(s_i2c_mutex));

    /* ─── 2. Camera second ─── */
    ESP_ERROR_CHECK(camera_init());

    /* ─── 3. Wi-Fi ─── */
    wifi_init();
    wait_for_wifi();

    /* ─── 4. SNTP ─── */
    sntp_init_time();

    /* ─── 5. MQTT ─── */
    mqtt_init();

    /* Small delay for MQTT connect before tasks start publishing */
    vTaskDelay(pdMS_TO_TICKS(2000));

    /* ─── 6. Launch unified task ─── */
    xTaskCreatePinnedToCore(sensor_camera_task, "sensor_camera_task",
                            8192, NULL, 5, NULL, 0);

    ESP_LOGI(TAG, "Task launched ✓");
}
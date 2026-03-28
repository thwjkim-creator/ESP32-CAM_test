idf_component_register(
    SRCS "main.c" "camera_drv.c" "veml7700_drv.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_wifi esp_event esp_http_client mqtt nvs_flash esp_timer esp_psram
)
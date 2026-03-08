cmake_minimum_required(VERSION 3.16)

idf_component_register(
    SRCS
        "main.c"
        "es8311.c"
    INCLUDE_DIRS
        "."
    REQUIRES
        nvs_flash
        esp_timer
        esp_http_server
        esp_wifi
        esp_event
        driver
        freertos
        spi_flash
        spiffs
        vfs
)
// main.c for ESP32-S3 wireless IMU + audio logger via HTTP download

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <unistd.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "driver/gpio.h"

#include "driver/i2c.h"
#include "driver/i2s.h"
#include "esp_spiffs.h"

#include "es8311.h"
#include "es8311_reg.h"
#include "config.h"

#define TAG "DATA_COLLECT"
#define IMU_ADDR 0x6A
#define I2C_PORT I2C_NUM_0
#define I2S_PORT I2S_NUM_0
#define CSV_PATH "/spiffs/dog.csv"
#define SAMPLE_RATE_HZ 2                     // Collect data every 500ms
#define MONITOR_BUTTON_GPIO BOOT_BUTTON_GPIO // Use boot button as monitor trigger

// Flash storage configuration for 30 minutes of data
#define MAX_STORAGE_TIME_MINUTES 30 // Store up to 30 minutes of data
#define RECORDS_PER_MINUTE (SAMPLE_RATE_HZ * 60)
#define MAX_RECORDS (MAX_STORAGE_TIME_MINUTES * RECORDS_PER_MINUTE)
#define ES8311_CODEC_DEFAULT_ADDR 0x18 // Replace with actual address if different

// NVS storage namespace and keys
#define STORAGE_NAMESPACE "sensor_data"
#define BATCH_COUNT_KEY "batch_count"
#define BATCH_DATA_KEY_PREFIX "batch_"
#define BATCH_MAX_SIZE 512 // Increased batch size to store more data per batch
#define MAX_BATCHES 300    // Reduced from 600 to 300 to use less NVS space
static es8311_handle_t s_es8311 = NULL;
static FILE *log_file = NULL;
static nvs_handle_t storage_handle = 0;
static char batch_buffer[BATCH_MAX_SIZE];
static size_t batch_index = 0;
static uint32_t batch_counter = 0;
static bool logging_active = false; // Flag to control logging state
static TaskHandle_t log_task_handle = NULL;

// Forward declarations
static esp_err_t init_nvs_storage(void);
static esp_err_t add_sensor_data_to_batch(uint32_t timestamp, float ax, float ay, float az,
                                          float gx, float gy, float gz, int16_t audio_min,
                                          int16_t audio_max, float audio_rms);
static esp_err_t flush_sensor_data_batch(void);
static esp_err_t clear_nvs_sensor_data(void);
static esp_err_t export_and_clear_handler(httpd_req_t *req); // Add forward declaration
static void log_task(void *arg);
static void start_logging(void);
static void stop_logging(void);
static void dump_nvs_data_to_console(void);
static void dump_spiffs_data_to_console(void);
static void clear_all_data(void);
static void parse_serial_commands(void);
static void serial_command_task(void *arg);
//--------------------------------------
// IMU (LSM6DS3 assumed)
//--------------------------------------
static void imu_init(void)
{
    i2c_master_write_to_device(I2C_PORT, IMU_ADDR, (uint8_t[]){0x10, 0x40}, 2, 100 / portTICK_PERIOD_MS); // CTRL1_XL
    i2c_master_write_to_device(I2C_PORT, IMU_ADDR, (uint8_t[]){0x11, 0x40}, 2, 100 / portTICK_PERIOD_MS); // CTRL2_G
    i2c_master_write_to_device(I2C_PORT, IMU_ADDR, (uint8_t[]){0x12, 0x44}, 2, 100 / portTICK_PERIOD_MS); // CTRL3_C
    vTaskDelay(pdMS_TO_TICKS(50));
}

static void parse_xyz(uint8_t *raw, float scale, float *x, float *y, float *z)
{
    int16_t rx = (int16_t)((raw[1] << 8) | raw[0]);
    int16_t ry = (int16_t)((raw[3] << 8) | raw[2]);
    int16_t rz = (int16_t)((raw[5] << 8) | raw[4]);
    *x = rx * scale;
    *y = ry * scale;
    *z = rz * scale;
}

static void read_imu(float *ax, float *ay, float *az, float *gx, float *gy, float *gz)
{
    uint8_t accel_raw[6], gyro_raw[6];
    i2c_master_write_read_device(I2C_PORT, IMU_ADDR, (uint8_t[]){0x28}, 1, accel_raw, 6, 100 / portTICK_PERIOD_MS);
    i2c_master_write_read_device(I2C_PORT, IMU_ADDR, (uint8_t[]){0x22}, 1, gyro_raw, 6, 100 / portTICK_PERIOD_MS);
    parse_xyz(accel_raw, 0.061f / 1000, ax, ay, az);
    parse_xyz(gyro_raw, 4.375f / 1000, gx, gy, gz);
}

//--------------------------------------
// Audio: ES8311 + I2S
//--------------------------------------
static void audio_codec_init(void)
{
    s_es8311 = es8311_create(I2C_PORT, AUDIO_CODEC_ES8311_ADDR);
    es8311_clock_config_t clk_cfg = {
        .mclk_inverted = false,
        .sclk_inverted = false,
        .mclk_from_mclk_pin = true,
        .mclk_frequency = 4096000,
        .sample_frequency = AUDIO_INPUT_SAMPLE_RATE,
    };
    ESP_ERROR_CHECK(es8311_init(s_es8311, &clk_cfg, ES8311_RESOLUTION_16, ES8311_RESOLUTION_16));
    es8311_microphone_config(s_es8311, false);
    es8311_microphone_gain_set(s_es8311, ES8311_MIC_GAIN_36DB);
    es8311_voice_mute(s_es8311, false);
}

static void i2s_init(void)
{
    i2s_config_t i2s_cfg = {
        .mode = I2S_MODE_MASTER | I2S_MODE_RX,
        .sample_rate = AUDIO_INPUT_SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S, // Updated from deprecated format
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 4,
        .dma_buf_len = 320,
        .use_apll = false,
    };
    i2s_pin_config_t pin_cfg = {
        .mck_io_num = AUDIO_I2S_GPIO_MCLK,
        .bck_io_num = AUDIO_I2S_GPIO_BCLK,
        .ws_io_num = AUDIO_I2S_GPIO_WS,
        .data_out_num = -1,
        .data_in_num = AUDIO_I2S_GPIO_DIN,
    };
    i2s_driver_install(I2S_PORT, &i2s_cfg, 0, NULL);
    i2s_set_pin(I2S_PORT, &pin_cfg);
}

//--------------------------------------
// NVS Flush Task
//--------------------------------------
static void nvs_flush_task(void *arg)
{
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(60000)); // Flush every 60 seconds (less frequent to conserve space)
        if (logging_active)
        {
            flush_sensor_data_batch();
            esp_err_t err = nvs_commit(storage_handle);
            if (err != ESP_OK)
            {
                ESP_LOGE(TAG, "Failed to commit NVS changes: %s", esp_err_to_name(err));
            }
            else
            {
                ESP_LOGI(TAG, "Periodic NVS data committed to flash");
            }
        }
    }
} //--------------------------------------
// CSV Logger
//--------------------------------------
static void log_task(void *arg)
{
    int16_t audio_buf[320];
    size_t bytes_read;
    uint32_t counter = 0;

    ESP_LOGI(TAG, "Log task started");

    while (logging_active) // Only run while logging is active
    {
        float ax, ay, az, gx, gy, gz;
        read_imu(&ax, &ay, &az, &gx, &gy, &gz);

        i2s_read(I2S_PORT, audio_buf, sizeof(audio_buf), &bytes_read, pdMS_TO_TICKS(100));
        int samples = bytes_read / 2;
        int16_t min = 32767, max = -32768;
        double sum = 0.0;
        for (int i = 0; i < samples; i++)
        {
            int16_t s = audio_buf[i];
            if (s < min)
                min = s;
            if (s > max)
                max = s;
            sum += s * s;
        }
        double rms = sqrt(sum / samples) / 32768.0;
        uint32_t ts = esp_timer_get_time() / 1000;

        // Save to NVS batch - reduce frequency to conserve space
        counter++;
        if (counter % 20 == 0)
        { // Save every 20th reading (every 10 seconds at 2Hz)
            add_sensor_data_to_batch(ts, ax, ay, az, gx, gy, gz, min, max, rms);
        }
        if (log_file)
        {
            fprintf(log_file,
                    "%lu,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%d,%d,%.4f\n",
                    ts, ax, ay, az, gx, gy, gz, min, max, rms);
            fflush(log_file); // Ensure file data is written
        }

        // Flush to NVS less frequently to conserve space
        if (counter % 40 == 0)
        { // Flush every 20 seconds
            flush_sensor_data_batch();

            // Commit to ensure data is written to flash
            esp_err_t err = nvs_commit(storage_handle);
            if (err != ESP_OK)
            {
                ESP_LOGE(TAG, "Failed to commit NVS changes: %s", esp_err_to_name(err));
            }
            else
            {
                ESP_LOGI(TAG, "NVS data committed to flash");
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000 / SAMPLE_RATE_HZ));
    }

    // Task cleanup
    ESP_LOGI(TAG, "Log task stopping");
    log_task_handle = NULL;

    // Final flush before exiting
    flush_sensor_data_batch();
    esp_err_t err = nvs_commit(storage_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to commit final NVS changes: %s", esp_err_to_name(err));
    }
    else
    {
        ESP_LOGI(TAG, "Final NVS data committed to flash");
    }

    vTaskDelete(NULL); // Self-delete
}

//--------------------------------------
// Flash Storage (NVS) Functions
//--------------------------------------
static esp_err_t init_nvs_storage(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        // NVS partition was truncated and needs to be erased
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    // Open NVS handle
    err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &storage_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Error (%s) opening NVS handle!", esp_err_to_name(err));
        return err;
    }

    // Initialize batch buffer with CSV header
    batch_index = 0;
    strncpy(batch_buffer, "timestamp_ms,ax,ay,az,gx,gy,gz,audio_min,audio_max,audio_rms\n", BATCH_MAX_SIZE);
    batch_index = strlen(batch_buffer);

    // Read the batch counter from NVS
    err = nvs_get_u32(storage_handle, BATCH_COUNT_KEY, &batch_counter);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND)
    {
        ESP_LOGE(TAG, "Error (%s) reading batch counter from NVS!", esp_err_to_name(err));
    }

    // If there are too many batches, clear some to make space
    if (batch_counter > MAX_BATCHES - 10)
    {
        ESP_LOGW(TAG, "Too many batches (%lu) at startup, clearing oldest data", batch_counter);
        clear_nvs_sensor_data();
        batch_counter = 0;
    }
    ESP_LOGI(TAG, "NVS storage initialized successfully with %lu batches", batch_counter);
    return ESP_OK;
}

static esp_err_t add_sensor_data_to_batch(uint32_t timestamp, float ax, float ay, float az,
                                          float gx, float gy, float gz, int16_t audio_min,
                                          int16_t audio_max, float audio_rms)
{
    // Check if we have enough space in the buffer
    char temp_str[128];
    int len = snprintf(temp_str, sizeof(temp_str), "%lu,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%d,%d,%.4f\n",
                       timestamp, ax, ay, az, gx, gy, gz, audio_min, audio_max, audio_rms);

    // If adding this entry would exceed buffer size, flush first
    if (batch_index + len >= BATCH_MAX_SIZE - 1)
    {
        esp_err_t err = flush_sensor_data_batch();
        if (err != ESP_OK)
        {
            return err;
        }
    }

    // Check again if we have space after flushing
    if (batch_index + len < BATCH_MAX_SIZE - 1)
    {
        // Add the new data to the batch
        batch_index += snprintf(batch_buffer + batch_index, BATCH_MAX_SIZE - batch_index,
                                "%lu,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%d,%d,%.4f\n",
                                timestamp, ax, ay, az, gx, gy, gz, audio_min, audio_max, audio_rms);
    }

    return ESP_OK;
}

static esp_err_t flush_sensor_data_batch(void)
{
    if (batch_index <= strlen("timestamp_ms,ax,ay,az,gx,gy,gz,audio_min,audio_max,audio_rms\n"))
    {
        // Nothing to flush
        return ESP_OK;
    }

    // Debug information about current state
    ESP_LOGD(TAG, "Flushing batch: index=%d, counter=%lu", batch_index, batch_counter);

    // Check if we're approaching the maximum number of batches
    if (batch_counter >= MAX_BATCHES - 5)
    {
        ESP_LOGW(TAG, "Approaching maximum batches limit (%lu/%d), auto-clearing oldest data",
                 batch_counter, MAX_BATCHES); // Auto-clear oldest data to make space
        // Delete the oldest 50 batches to make room (more aggressive clearing)
        for (int i = 0; i < 50 && i < batch_counter; i++)
        {
            char batch_key[32];
            snprintf(batch_key, sizeof(batch_key), "%s%lu", BATCH_DATA_KEY_PREFIX, (unsigned long)i);
            nvs_erase_key(storage_handle, batch_key);
        }
        // Shift remaining batches down
        for (int i = 50; i < batch_counter; i++)
        {
            char old_key[32], new_key[32];
            snprintf(old_key, sizeof(old_key), "%s%lu", BATCH_DATA_KEY_PREFIX, (unsigned long)i);
            snprintf(new_key, sizeof(new_key), "%s%lu", BATCH_DATA_KEY_PREFIX, (unsigned long)(i - 50));

            // Get the data
            size_t required_size;
            esp_err_t err = nvs_get_str(storage_handle, old_key, NULL, &required_size);
            if (err == ESP_OK)
            {
                char *data_str = malloc(required_size);
                if (data_str)
                {
                    err = nvs_get_str(storage_handle, old_key, data_str, &required_size);
                    if (err == ESP_OK)
                    {
                        nvs_set_str(storage_handle, new_key, data_str);
                    }
                    free(data_str);
                }
            }
            // Erase the old key
            nvs_erase_key(storage_handle, old_key);
        }
        // Update batch counter
        if (batch_counter > 50)
        {
            batch_counter -= 50;
        }
        else
        {
            batch_counter = 0;
        }
        nvs_set_u32(storage_handle, BATCH_COUNT_KEY, batch_counter);
    }

    // Check if we've reached the maximum number of batches
    if (batch_counter >= MAX_BATCHES)
    {
        ESP_LOGW(TAG, "Maximum number of batches reached, clearing oldest data");
        // More aggressive clearing - remove 100 oldest batches
        for (int i = 0; i < 100 && i < batch_counter; i++)
        {
            char batch_key[32];
            snprintf(batch_key, sizeof(batch_key), "%s%lu", BATCH_DATA_KEY_PREFIX, (unsigned long)i);
            nvs_erase_key(storage_handle, batch_key);
        }
        // Shift remaining batches down
        for (int i = 100; i < batch_counter; i++)
        {
            char old_key[32], new_key[32];
            snprintf(old_key, sizeof(old_key), "%s%lu", BATCH_DATA_KEY_PREFIX, (unsigned long)i);
            snprintf(new_key, sizeof(new_key), "%s%lu", BATCH_DATA_KEY_PREFIX, (unsigned long)(i - 100));

            // Get the data
            size_t required_size;
            esp_err_t err = nvs_get_str(storage_handle, old_key, NULL, &required_size);
            if (err == ESP_OK)
            {
                char *data_str = malloc(required_size);
                if (data_str)
                {
                    err = nvs_get_str(storage_handle, old_key, data_str, &required_size);
                    if (err == ESP_OK)
                    {
                        nvs_set_str(storage_handle, new_key, data_str);
                    }
                    free(data_str);
                }
            }
            // Erase the old key
            nvs_erase_key(storage_handle, old_key);
        }
        // Update batch counter
        if (batch_counter > 100)
        {
            batch_counter -= 100;
        }
        else
        {
            batch_counter = 0;
        }
        nvs_set_u32(storage_handle, BATCH_COUNT_KEY, batch_counter);
    }
    // Create a unique key for this batch
    char batch_key[32];
    snprintf(batch_key, sizeof(batch_key), "%s%lu", BATCH_DATA_KEY_PREFIX, batch_counter);

    esp_err_t err = nvs_set_str(storage_handle, batch_key, batch_buffer);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to save sensor data batch to NVS: %s", esp_err_to_name(err));
        // Try to clear the batch to prevent continuous errors
        batch_index = 0;
        strncpy(batch_buffer, "timestamp_ms,ax,ay,az,gx,gy,gz,audio_min,audio_max,audio_rms\n", BATCH_MAX_SIZE);
        batch_index = strlen(batch_buffer);
        return err;
    }

    // Update and save the batch counter
    batch_counter++;
    err = nvs_set_u32(storage_handle, BATCH_COUNT_KEY, batch_counter);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to update batch counter in NVS: %s", esp_err_to_name(err));
        return err;
    }

    err = nvs_commit(storage_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to commit changes to NVS: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "Flushed sensor data batch #%lu to NVS (%d bytes)", batch_counter - 1, batch_index);

    // Reset buffer with header
    batch_index = 0;
    strncpy(batch_buffer, "timestamp_ms,ax,ay,az,gx,gy,gz,audio_min,audio_max,audio_rms\n", BATCH_MAX_SIZE);
    batch_index = strlen(batch_buffer);

    return ESP_OK;
}

static esp_err_t clear_nvs_sensor_data(void)
{
    esp_err_t err;
    uint32_t count;

    // Read the current batch counter
    err = nvs_get_u32(storage_handle, BATCH_COUNT_KEY, &count);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND)
    {
        ESP_LOGE(TAG, "Failed to read batch counter from NVS: %s", esp_err_to_name(err));
        return err;
    }

    // Delete all batch data
    for (uint32_t i = 0; i < count && i < MAX_BATCHES; i++)
    {
        char batch_key[32];
        snprintf(batch_key, sizeof(batch_key), "%s%lu", BATCH_DATA_KEY_PREFIX, i);

        err = nvs_erase_key(storage_handle, batch_key);
        if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND)
        {
            ESP_LOGE(TAG, "Failed to erase batch %lu from NVS: %s", i, esp_err_to_name(err));
            // Continue with other batches
        }
    }

    // Reset the counter to 0
    batch_counter = 0;
    err = nvs_set_u32(storage_handle, BATCH_COUNT_KEY, batch_counter);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to reset batch counter in NVS: %s", esp_err_to_name(err));
        return err;
    }

    // Commit changes
    err = nvs_commit(storage_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to commit NVS changes: %s", esp_err_to_name(err));
        return err;
    }

    // Reset batch buffer
    batch_index = 0;
    strncpy(batch_buffer, "timestamp_ms,ax,ay,az,gx,gy,gz,audio_min,audio_max,audio_rms\n", BATCH_MAX_SIZE);
    batch_index = strlen(batch_buffer);

    ESP_LOGI(TAG, "Cleared all sensor data batches from NVS");
    return ESP_OK;
}

esp_err_t clear_nvs_data_handler(httpd_req_t *req)
{
    esp_err_t err = clear_nvs_sensor_data();
    if (err != ESP_OK)
    {
        httpd_resp_sendstr(req, "Failed to clear NVS data");
        return ESP_FAIL;
    }

    httpd_resp_sendstr(req, "NVS data cleared successfully");
    return ESP_OK;
}

// Add a new handler for exporting and clearing data
esp_err_t export_and_clear_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "Export and clear request received");

    // Send HTTP header
    httpd_resp_set_type(req, "text/csv");
    httpd_resp_set_hdr(req, "Content-Disposition", "attachment; filename=\"sensor_data.csv\"");
    httpd_resp_send_chunk(req, "timestamp_ms,ax,ay,az,gx,gy,gz,audio_min,audio_max,audio_rms\n",
                          strlen("timestamp_ms,ax,ay,az,gx,gy,gz,audio_min,audio_max,audio_rms\n"));

    esp_err_t err;
    uint32_t count;

    // Read the batch counter from NVS
    err = nvs_get_u32(storage_handle, BATCH_COUNT_KEY, &count);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read batch counter from NVS: %s", esp_err_to_name(err));
        httpd_resp_send_chunk(req, NULL, 0);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Exporting %lu sensor data batches", count);

    // Load all sensor data batches and send them
    for (uint32_t i = 0; i < count && i < MAX_BATCHES; i++)
    {
        char batch_key[32];
        snprintf(batch_key, sizeof(batch_key), "%s%lu", BATCH_DATA_KEY_PREFIX, i);

        // Get the required buffer size
        size_t required_size;
        err = nvs_get_str(storage_handle, batch_key, NULL, &required_size);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to get required size for batch %lu: %s", i, esp_err_to_name(err));
            continue;
        }

        // Allocate buffer and read the data
        char *data_str = malloc(required_size);
        if (data_str == NULL)
        {
            ESP_LOGE(TAG, "Failed to allocate memory for batch %lu", i);
            continue;
        }

        err = nvs_get_str(storage_handle, batch_key, data_str, &required_size);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to read batch %lu from NVS: %s", i, esp_err_to_name(err));
            free(data_str);
            continue;
        }

        // Find the header line and skip it (we already sent the header)
        char *data_start = strchr(data_str, '\n');
        if (data_start != NULL)
        {
            data_start++; // Skip the newline
            // Send the data (without the header)
            httpd_resp_send_chunk(req, data_start, strlen(data_start));
        }

        free(data_str);
    }

    // Clear all data after export
    clear_nvs_sensor_data();

    // End response
    httpd_resp_send_chunk(req, NULL, 0);

    ESP_LOGI(TAG, "Data exported and cleared successfully");
    return ESP_OK;
}

//--------------------------------------
// Web Server Management
//--------------------------------------
static httpd_handle_t global_server = NULL;

// Forward declarations for URI handlers
extern httpd_uri_t download_uri;
extern httpd_uri_t nvs_data_uri;
extern httpd_uri_t clear_nvs_uri;
extern httpd_uri_t export_clear_uri; // Add forward declaration

httpd_handle_t start_webserver(void)
{
    // Stop existing server if running
    if (global_server != NULL)
    {
        ESP_LOGI(TAG, "Stopping existing web server");
        httpd_stop(global_server);
        global_server = NULL;
    }

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 8192; // Increase stack size for better stability

    ESP_LOGI(TAG, "Starting web server on port %d", config.server_port);

    if (httpd_start(&global_server, &config) == ESP_OK)
    {
        ESP_LOGI(TAG, "Web server started successfully");
        httpd_register_uri_handler(global_server, &download_uri);
        httpd_register_uri_handler(global_server, &nvs_data_uri);
        httpd_register_uri_handler(global_server, &clear_nvs_uri);
        httpd_register_uri_handler(global_server, &export_clear_uri); // Add the new endpoint
        ESP_LOGI(TAG, "Registered URI handlers: /download, /nvs-data, /clear-nvs, /export-clear");
    }
    else
    {
        ESP_LOGE(TAG, "Failed to start web server");
        global_server = NULL;
    }

    return global_server;
}

// Function to restart web server
static void restart_webserver(void)
{
    ESP_LOGI(TAG, "Restarting web server");
    start_webserver();
}

// Web monitoring task function
static void web_monitor_task(void *arg)
{
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(60000)); // Check every minute

        // If web server is not running, restart it
        if (global_server == NULL)
        {
            ESP_LOGW(TAG, "Web server not running, restarting...");
            restart_webserver();
        }
    }
}

//--------------------------------------
// HTTP server to serve /spiffs/dog.csv
//--------------------------------------
esp_err_t download_get_handler(httpd_req_t *req)
{
    FILE *f = fopen(CSV_PATH, "r");
    if (!f)
        return ESP_FAIL;
    char line[256];
    while (fgets(line, sizeof(line), f))
        httpd_resp_send_chunk(req, line, strlen(line));
    httpd_resp_send_chunk(req, NULL, 0);
    fclose(f);
    return ESP_OK;
}

esp_err_t nvs_data_get_handler(httpd_req_t *req)
{
    esp_err_t err;
    uint32_t count;

    // Read the batch counter from NVS
    err = nvs_get_u32(storage_handle, BATCH_COUNT_KEY, &count);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read batch counter from NVS: %s", esp_err_to_name(err));
        return ESP_FAIL;
    }

    // Send HTTP header
    httpd_resp_send_chunk(req, "timestamp_ms,ax,ay,az,gx,gy,gz,audio_min,audio_max,audio_rms\n",
                          strlen("timestamp_ms,ax,ay,az,gx,gy,gz,audio_min,audio_max,audio_rms\n"));

    // Load all sensor data batches and send them
    for (uint32_t i = 0; i < count && i < MAX_BATCHES; i++)
    {
        char batch_key[32];
        snprintf(batch_key, sizeof(batch_key), "%s%lu", BATCH_DATA_KEY_PREFIX, i);

        // Get the required buffer size
        size_t required_size;
        err = nvs_get_str(storage_handle, batch_key, NULL, &required_size);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to get required size for batch %lu: %s", i, esp_err_to_name(err));
            continue;
        }

        // Allocate buffer and read the data
        char *data_str = malloc(required_size);
        if (data_str == NULL)
        {
            ESP_LOGE(TAG, "Failed to allocate memory for batch %lu", i);
            continue;
        }

        err = nvs_get_str(storage_handle, batch_key, data_str, &required_size);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to read batch %lu from NVS: %s", i, esp_err_to_name(err));
            free(data_str);
            continue;
        }

        // Find the header line and skip it (we already sent the header)
        char *data_start = strchr(data_str, '\n');
        if (data_start != NULL)
        {
            data_start++; // Skip the newline
            // Send the data (without the header)
            httpd_resp_send_chunk(req, data_start, strlen(data_start));
        }

        free(data_str);
    }

    // End response
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

httpd_uri_t download_uri = {
    .uri = "/download",
    .method = HTTP_GET,
    .handler = download_get_handler};

httpd_uri_t nvs_data_uri = {
    .uri = "/nvs-data",
    .method = HTTP_GET,
    .handler = nvs_data_get_handler};

httpd_uri_t clear_nvs_uri = {
    .uri = "/clear-nvs",
    .method = HTTP_GET,
    .handler = clear_nvs_data_handler};

// Add the new export and clear URI handler
httpd_uri_t export_clear_uri = {
    .uri = "/export-clear",
    .method = HTTP_GET,
    .handler = export_and_clear_handler};

// Event handler for WiFi events
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        ESP_LOGI(TAG, "Disconnected from WiFi, reconnecting...");
        // Add a small delay before reconnecting to avoid rapid reconnection attempts
        vTaskDelay(pdMS_TO_TICKS(1000));
        esp_wifi_connect();
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Got IP address: " IPSTR, IP2STR(&event->ip_info.ip));
        ESP_LOGI(TAG, "Device is now accessible at http://" IPSTR, IP2STR(&event->ip_info.ip));
    }
}
//--------------------------------------
// WiFi connect
//--------------------------------------
void wifi_init_sta(void)
{
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    // Register event handlers
    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    esp_wifi_start();
    ESP_LOGI(TAG, "WiFi connecting to %s...", WIFI_SSID);
}
//--------------------------------------
// Button Handler for Monitor Trigger
//--------------------------------------
static void button_task(void *arg)
{
    // Configure button GPIO as input with pull-up
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << MONITOR_BUTTON_GPIO);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    uint32_t press_start_time = 0;
    uint32_t last_press_time = 0;
    bool is_button_pressed = false;
    uint8_t press_count = 0;
    uint32_t last_release_time = 0;

    while (1)
    {
        uint32_t current_time = esp_timer_get_time() / 1000; // ms

        // Check button state
        if (gpio_get_level(MONITOR_BUTTON_GPIO) == 0)
        { // Button is pressed (active low)
            if (!is_button_pressed)
            {
                // New button press detected
                is_button_pressed = true;
                press_start_time = current_time;

                // Debounce - only register press if 200ms has passed since last press
                if (current_time - last_press_time > 200)
                {
                    last_press_time = current_time;

                    // Check if this press is within 1 second of the last release for counting purposes
                    if (current_time - last_release_time < 1000)
                    {
                        press_count++;
                        ESP_LOGD(TAG, "Button press count: %d", press_count);
                        
                        // If this is the third press, clear all data
                        if (press_count >= 3)
                        {
                            ESP_LOGI(TAG, "Triple press detected, clearing all data...");
                            clear_all_data();
                            press_count = 0; // Reset counter
                        }
                    }
                    else
                    {
                        // Reset counter if too much time has passed
                        press_count = 1;
                    }

                    // Toggle logging state (only on first press of a sequence)
                    if (press_count <= 1)
                    {
                        if (logging_active)
                        {
                            ESP_LOGI(TAG, "Stopping logging via button press");
                            stop_logging();
                        }
                        else
                        {
                            ESP_LOGI(TAG, "Starting logging via button press");
                            start_logging();
                        }
                    }
                }
            }
            else
            {
                // Button still pressed - check for long press (5 seconds) only
                if (current_time - press_start_time > 5000)
                {
                    // Long press detected (5 seconds) - dump data to console
                    ESP_LOGI(TAG, "Long press detected, dumping data to console...");
                    dump_nvs_data_to_console();
                    dump_spiffs_data_to_console();

                    // Prevent repeated triggering
                    press_start_time = current_time + 10000; // Set far in future
                }
            }
        }
        else
        {
            // Button released
            if (is_button_pressed)
            {
                // Record the time when button was released
                last_release_time = current_time;
            }
            is_button_pressed = false;
        }
        vTaskDelay(pdMS_TO_TICKS(50)); // Check button 20 times per second
    }
}
//--------------------------------------
// Logging Control Functions
//--------------------------------------
static void start_logging(void)
{
    if (logging_active)
    {
        ESP_LOGW(TAG, "Logging already active");
        return;
    }

    logging_active = true;
    ESP_LOGI(TAG, "Logging started");

    // Create logging task if it doesn't exist
    if (log_task_handle == NULL)
    {
        BaseType_t result = xTaskCreate(log_task, "log_task", 8192, NULL, 5, &log_task_handle);
        if (result != pdPASS)
        {
            ESP_LOGE(TAG, "Failed to create log_task");
            logging_active = false;
            log_task_handle = NULL;
        }
        else
        {
            ESP_LOGI(TAG, "Log task created successfully");
        }
    }
}

static void stop_logging(void)
{
    if (!logging_active)
    {
        ESP_LOGW(TAG, "Logging already inactive");
        return;
    }

    logging_active = false;
    ESP_LOGI(TAG, "Logging stopped signal sent");

    // Wait for the logging task to finish and clean up
    int timeout = 0;
    while (log_task_handle != NULL && timeout < 100)
    { // Wait up to 10 seconds
        vTaskDelay(pdMS_TO_TICKS(100));
        timeout++;
    }

    if (timeout >= 100)
    {
        ESP_LOGW(TAG, "Timeout waiting for log task to stop");
    }

    // Flush any remaining data
    flush_sensor_data_batch();
    esp_err_t err = nvs_commit(storage_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to commit NVS changes: %s", esp_err_to_name(err));
    }
    else
    {
        ESP_LOGI(TAG, "Final NVS data committed to flash");
    }
}

//--------------------------------------
// MAIN
//--------------------------------------
void app_main(void)
{
    ESP_LOGI(TAG, "Booting...");

    // Initialize NVS storage
    esp_err_t err = init_nvs_storage();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize NVS storage: %s", esp_err_to_name(err));
    }

    nvs_flash_init();
    i2c_config_t i2c_cfg = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = AUDIO_CODEC_I2C_SDA_PIN,
        .scl_io_num = AUDIO_CODEC_I2C_SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000};
    i2c_param_config(I2C_PORT, &i2c_cfg);
    i2c_driver_install(I2C_PORT, i2c_cfg.mode, 0, 0, 0);

    imu_init();
    audio_codec_init();
    i2s_init();

    // Small delay to ensure system is ready
    vTaskDelay(pdMS_TO_TICKS(100));

    // Initialize SPIFFS for data storage    ESP_LOGI(TAG, "Initializing SPIFFS for data storage...");
    esp_vfs_spiffs_conf_t spiffs_cfg = {
        .base_path = "/spiffs",
        .partition_label = "spiffs", // Explicitly specify partition label
        .max_files = 5,
        .format_if_mount_failed = true};

    err = esp_vfs_spiffs_register(&spiffs_cfg);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to register SPIFFS: %s", esp_err_to_name(err));
        if (err == ESP_FAIL)
        {
            ESP_LOGE(TAG, "Failed to mount or format filesystem");
        }
        else if (err == ESP_ERR_NOT_FOUND)
        {
            ESP_LOGE(TAG, "Failed to find SPIFFS partition");
            // Try with default partition label
            spiffs_cfg.partition_label = NULL;
            err = esp_vfs_spiffs_register(&spiffs_cfg);
            if (err != ESP_OK)
            {
                ESP_LOGE(TAG, "Failed to register SPIFFS with default partition: %s", esp_err_to_name(err));
                // Since SPIFFS is critical for our flash storage approach, we should halt if it fails
                return;
            }
            else
            {
                ESP_LOGI(TAG, "Successfully registered SPIFFS with default partition");
            }
        }
        else
        {
            ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(err));
            // Since SPIFFS is critical for our flash storage approach, we should halt if it fails
            return;
        }
    }
    // Get SPIFFS partition information
    size_t total = 0, used = 0;
    err = esp_spiffs_info(NULL, &total, &used);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(err));
    }
    else
    {
        ESP_LOGI(TAG, "SPIFFS Partition - Total: %d bytes, Used: %d bytes, Free: %d bytes", total, used, total - used);
    }

    // Open/create the log file for appending data
    log_file = fopen(CSV_PATH, "a"); // Use append mode to preserve existing data
    if (log_file)
    {
        // Check if file is empty and write header if needed
        fseek(log_file, 0, SEEK_END);
        long file_size = ftell(log_file);
        if (file_size == 0)
        {
            fprintf(log_file, "timestamp_ms,ax,ay,az,gx,gy,gz,audio_min,audio_max,audio_rms\n");
            ESP_LOGI(TAG, "Created new CSV file with header at %s", CSV_PATH);
        }
        else
        {
            ESP_LOGI(TAG, "Opened existing CSV file at %s (size: %ld bytes)", CSV_PATH, file_size);
        }
        fflush(log_file);
    }
    else
    {
        ESP_LOGE(TAG, "Failed to open %s for writing/appending", CSV_PATH);
    }

    // Initialize WiFi
    wifi_init_sta();
    // Create button monitoring task with larger stack size to prevent overflow
    xTaskCreate(button_task, "button_task", 4096, NULL, 4, NULL);

    // Create a task to periodically check and restart web server if needed
    xTaskCreate(web_monitor_task, "web_monitor_task", 2048, NULL, 2, NULL);

    // Create periodic NVS flush task with larger stack size
    xTaskCreate(nvs_flush_task, "nvs_flush_task", 4096, NULL, 3, NULL);

    // Create serial command parsing task
    xTaskCreate(serial_command_task, "serial_command_task", 2048, NULL, 2, NULL);

    // Start logging automatically for battery operation    ESP_LOGI(TAG, "Starting automatic logging for battery operation");
    start_logging();

    // Give WiFi some time to connect automatically through event handlers
    vTaskDelay(pdMS_TO_TICKS(5000));

    // Check if we got an IP address, if not, try to connect manually
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (netif)
    {
        esp_netif_ip_info_t ip_info;
        if (esp_netif_get_ip_info(netif, &ip_info) == ESP_OK && ip_info.ip.addr != 0)
        {
            ESP_LOGI(TAG, "Already connected to WiFi with IP: " IPSTR, IP2STR(&ip_info.ip));
        }
        else
        {
            ESP_LOGI(TAG, "Attempting manual WiFi connection...");
            // Try manual connection if automatic connection failed
            int wifi_retry_count = 0;
            while (wifi_retry_count < 5)
            {
                esp_err_t wifi_err = esp_wifi_connect();
                if (wifi_err == ESP_OK)
                {
                    ESP_LOGI(TAG, "WiFi connection command sent successfully");
                    // Wait for IP assignment
                    vTaskDelay(pdMS_TO_TICKS(3000));

                    // Check if we got an IP
                    if (esp_netif_get_ip_info(netif, &ip_info) == ESP_OK && ip_info.ip.addr != 0)
                    {
                        ESP_LOGI(TAG, "WiFi connected successfully with IP: " IPSTR, IP2STR(&ip_info.ip));
                        break;
                    }
                }
                else
                {
                    ESP_LOGE(TAG, "Failed to connect to WiFi: %s", esp_err_to_name(wifi_err));
                }

                wifi_retry_count++;
                vTaskDelay(pdMS_TO_TICKS(3000));
            }

            if (wifi_retry_count >= 5)
            {
                ESP_LOGE(TAG, "Failed to connect to WiFi after 5 attempts");
            }
        }
    }

    // Start web server
    httpd_handle_t server = start_webserver();
    if (server != NULL)
    {
        ESP_LOGI(TAG, "Web server started successfully");
    }
    else
    {
        ESP_LOGE(TAG, "Failed to start web server");
    }

    ESP_LOGI(TAG, "System ready. Device IP should be displayed above.");
}

// Function to dump NVS data to serial console for debugging
static void dump_nvs_data_to_console(void)
{
    esp_err_t err;
    uint32_t count;

    // Read the batch counter from NVS
    err = nvs_get_u32(storage_handle, BATCH_COUNT_KEY, &count);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read batch counter from NVS: %s", esp_err_to_name(err));
        return;
    }

    ESP_LOGI(TAG, "=== NVS Data Dump ===");
    ESP_LOGI(TAG, "Total batches: %lu", count);

    // Open file for writing the dump
    FILE *dump_file = fopen("/spiffs/nvs_dump.csv", "w");
    if (dump_file)
    {
        ESP_LOGI(TAG, "Writing NVS data to /spiffs/nvs_dump.csv");
    }
    else
    {
        ESP_LOGE(TAG, "Failed to open /spiffs/nvs_dump.csv for writing");
    }

    // Print header
    printf("timestamp_ms,ax,ay,az,gx,gy,gz,audio_min,audio_max,audio_rms\n");
    if (dump_file)
    {
        fprintf(dump_file, "timestamp_ms,ax,ay,az,gx,gy,gz,audio_min,audio_max,audio_rms\n");
    }

    // Load all sensor data batches and print them
    for (uint32_t i = 0; i < count && i < MAX_BATCHES; i++)
    {
        char batch_key[32];
        snprintf(batch_key, sizeof(batch_key), "%s%lu", BATCH_DATA_KEY_PREFIX, i);

        // Get the required buffer size
        size_t required_size;
        err = nvs_get_str(storage_handle, batch_key, NULL, &required_size);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to get required size for batch %lu: %s", i, esp_err_to_name(err));
            continue;
        }

        // Allocate buffer and read the data
        char *data_str = malloc(required_size);
        if (data_str == NULL)
        {
            ESP_LOGE(TAG, "Failed to allocate memory for batch %lu", i);
            continue;
        }

        err = nvs_get_str(storage_handle, batch_key, data_str, &required_size);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to read batch %lu from NVS: %s", i, esp_err_to_name(err));
            free(data_str);
            continue;
        }

        // Find the header line and skip it (we already printed the header)
        char *data_start = strchr(data_str, '\n');
        if (data_start != NULL)
        {
            data_start++; // Skip the newline
            // Print the data (without the header)
            printf("%s", data_start);
            if (dump_file)
            {
                fprintf(dump_file, "%s", data_start);
            }
        }

        free(data_str);
    }

    if (dump_file)
    {
        fclose(dump_file);
        ESP_LOGI(TAG, "NVS data dump saved to /spiffs/nvs_dump.csv");
    }

    ESP_LOGI(TAG, "=== End of NVS Data Dump ===");
}

// Function to dump SPIFFS data to serial console for debugging
static void dump_spiffs_data_to_console(void)
{
    FILE *f = fopen(CSV_PATH, "r");
    if (!f)
    {
        ESP_LOGE(TAG, "Failed to open %s for reading", CSV_PATH);
        return;
    }

    ESP_LOGI(TAG, "=== SPIFFS Data Dump ===");

    // Open file for writing the dump
    FILE *dump_file = fopen("/spiffs/spiffs_dump.csv", "w");
    if (dump_file)
    {
        ESP_LOGI(TAG, "Writing SPIFFS data to /spiffs/spiffs_dump.csv");
    }
    else
    {
        ESP_LOGE(TAG, "Failed to open /spiffs/spiffs_dump.csv for writing");
    }

    char line[256];
    int line_count = 0;
    // Read and print each line
    while (fgets(line, sizeof(line), f))
    {
        // Skip empty lines
        if (strlen(line) <= 1)
        {
            continue;
        }

        // Print the line (remove trailing newline if present)
        size_t len = strlen(line);
        if (len > 0 && line[len - 1] == '\n')
        {
            line[len - 1] = '\0';
        }

        printf("%s\n", line);
        if (dump_file)
        {
            fprintf(dump_file, "%s\n", line);
        }
        line_count++;

        // Limit output to prevent overwhelming the console
        if (line_count > 1000)
        {
            ESP_LOGW(TAG, "Truncating output at 1000 lines to prevent console overflow");
            break;
        }
    }

    fclose(f);
    if (dump_file)
    {
        fclose(dump_file);
        ESP_LOGI(TAG, "SPIFFS data dump saved to /spiffs/spiffs_dump.csv");
    }
    ESP_LOGI(TAG, "=== End of SPIFFS Data Dump (Total lines: %d) ===", line_count);
}

// Function to clear all stored data (both NVS and SPIFFS)
static void clear_all_data(void)
{
    ESP_LOGI(TAG, "Clearing all stored data...");

    // Clear NVS data
    esp_err_t err = clear_nvs_sensor_data();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to clear NVS data: %s", esp_err_to_name(err));
    }
    else
    {
        ESP_LOGI(TAG, "NVS data cleared successfully");
    }

    // Close current log file if open
    if (log_file)
    {
        fclose(log_file);
        log_file = NULL;
    }

    // Delete and recreate the CSV file
    unlink(CSV_PATH); // Delete existing file

    // Reopen file for writing
    log_file = fopen(CSV_PATH, "w");
    if (log_file)
    {
        // Write CSV header
        fprintf(log_file, "timestamp_ms,ax,ay,az,gx,gy,gz,audio_min,audio_max,audio_rms\n");
        fflush(log_file);
        ESP_LOGI(TAG, "SPIFFS CSV file recreated successfully");
    }
    else
    {
        ESP_LOGE(TAG, "Failed to recreate CSV file at %s", CSV_PATH);
    }

    ESP_LOGI(TAG, "All data cleared successfully");
}

// Function to parse serial commands for data management
static void parse_serial_commands(void)
{
    static char command_buffer[64];
    static int buffer_index = 0;
    
    // Check if data is available on stdin (serial)
    int ch;
    while ((ch = fgetc(stdin)) != EOF)
    {
        if (ch == '\n' || ch == '\r')
        {
            // End of command received
            if (buffer_index > 0)
            {
                command_buffer[buffer_index] = '\0';
                
                // Process the command
                if (strcmp(command_buffer, "CLEAR_DATA") == 0)
                {
                    ESP_LOGI(TAG, "Serial command CLEAR_DATA received, clearing all data...");
                    clear_all_data();
                }
                else if (strcmp(command_buffer, "DUMP_DATA") == 0)
                {
                    ESP_LOGI(TAG, "Serial command DUMP_DATA received, dumping all data...");
                    dump_nvs_data_to_console();
                    dump_spiffs_data_to_console();
                }
                else if (strcmp(command_buffer, "HELP") == 0)
                {
                    ESP_LOGI(TAG, "Available serial commands:");
                    ESP_LOGI(TAG, "  CLEAR_DATA - Clear all stored data (NVS and SPIFFS)");
                    ESP_LOGI(TAG, "  DUMP_DATA  - Dump all data to console and files");
                    ESP_LOGI(TAG, "  HELP       - Show this help message");
                }
                else
                {
                    ESP_LOGW(TAG, "Unknown command: %s (type HELP for available commands)", command_buffer);
                }
                
                // Reset buffer
                buffer_index = 0;
            }
        }
        else if (buffer_index < sizeof(command_buffer) - 1)
        {
            // Add character to buffer
            command_buffer[buffer_index++] = ch;
        }
        else
        {
            // Buffer overflow, reset
            buffer_index = 0;
        }
    }
}

// Task to periodically check for serial commands
static void serial_command_task(void *arg)
{
    while (1)
    {
        // Check for serial commands
        parse_serial_commands();
        
        // Check every 100ms
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

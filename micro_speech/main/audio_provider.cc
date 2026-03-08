/* Copyright 2019 The TensorFlow Authors. All Rights Reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
==============================================================================*/

#include "audio_provider.h"

#include <cstdlib>
#include <cstring>
#include <cmath> 
// FreeRTOS.h must be included before some of the following dependencies.
// Solves b/150260343.
// clang-format off
#include "freertos/FreeRTOS.h"
// clang-format on
#include "es8311.h" 
#include "example_config.h"
#include "esp_log.h"
#include "esp_spi_flash.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/task.h"
#include "ringbuf.h"
#include "micro_model_settings.h"
#include "driver/i2s_std.h" 
#include "esp_check.h"

using namespace std;

// for c2 and c3, I2S support was added from IDF v4.4 onwards
#define NO_I2S_SUPPORT CONFIG_IDF_TARGET_ESP32C2 || \
                          (CONFIG_IDF_TARGET_ESP32C3 \
                          && (ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(4, 4, 0)))

static const char* TAG = "TF_LITE_AUDIO_PROVIDER";
/* ringbuffer to hold the incoming audio data */
ringbuf_t* g_audio_capture_buffer;
volatile int32_t g_latest_audio_timestamp = 0;
/* model requires 20ms new data from g_audio_capture_buffer and 10ms old data
 * each time , storing old data in the histrory buffer , {
 * history_samples_to_keep = 10 * 16 } */
constexpr int32_t history_samples_to_keep =
    ((kFeatureDurationMs - kFeatureStrideMs) *
     (kAudioSampleFrequency / 1000));
/* new samples to get each time from ringbuffer, { new_samples_to_get =  20 * 16
 * } */
constexpr int32_t new_samples_to_get =
    (kFeatureStrideMs * (kAudioSampleFrequency / 1000));

const int32_t kAudioCaptureBufferSize = 40000;
const int32_t i2s_bytes_to_read = 3200;

static i2s_chan_handle_t tx_handle = NULL;
static i2s_chan_handle_t rx_handle = NULL;

namespace {
int16_t g_audio_output_buffer[kMaxAudioSampleSize * 32];
bool g_is_audio_initialized = false;
int16_t g_history_buffer[history_samples_to_keep];

#if !NO_I2S_SUPPORT
uint8_t g_i2s_read_buffer[i2s_bytes_to_read] = {};
#if CONFIG_IDF_TARGET_ESP32
i2s_port_t i2s_port = I2S_NUM_1; // for esp32-eye
#else
i2s_port_t i2s_port = I2S_NUM_0; // for esp32-s3-eye
#endif
#endif
}  // namespace

static void i2s_init(void) {
  // 1. 初始化I2C总线（用于控制ES8311）
#if !defined(CONFIG_EXAMPLE_BSP)
  const i2c_config_t i2c_config = {
      .mode = I2C_MODE_MASTER,
      .sda_io_num = I2C_SDA_IO,
      .scl_io_num = I2C_SCL_IO,
      .sda_pullup_en = GPIO_PULLUP_ENABLE,
      .scl_pullup_en = GPIO_PULLUP_ENABLE,
      .master = {
        .clk_speed = 100000,
    },
  };
  ESP_ERROR_CHECK(i2c_param_config(I2C_NUM, &i2c_config));
  ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM, I2C_MODE_MASTER, 0, 0, 0));
#else
  ESP_ERROR_CHECK(bsp_i2c_init());
#endif

  // 2. 初始化ES8311编解码器
  es8311_handle_t es_handle = es8311_create(I2C_NUM, ES8311_ADDRRES_0);
    const es8311_clock_config_t clock_config = {
        .mclk_inverted = false,
        .sclk_inverted = false,
        .mclk_from_mclk_pin = true,
        .mclk_frequency = EXAMPLE_MCLK_FREQ_HZ,
        .sample_frequency = EXAMPLE_SAMPLE_RATE};

  ESP_ERROR_CHECK(es8311_init(es_handle, &clock_config, ES8311_RESOLUTION_16, ES8311_RESOLUTION_16));
  ESP_RETURN_VOID_ON_ERROR(es8311_sample_frequency_config(es_handle, EXAMPLE_MCLK_FREQ_HZ, EXAMPLE_SAMPLE_RATE), TAG, "set es8311 sample frequency failed");
  ESP_RETURN_VOID_ON_ERROR(es8311_microphone_config(es_handle, false), TAG, "set es8311 microphone failed");
  // 3. 配置I2S通道（与ES8311连接）
  i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(i2s_port, I2S_ROLE_MASTER);
  chan_cfg.auto_clear = true; // Auto clear the legacy data in the DMA buffer
  ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &tx_handle, &rx_handle));
  i2s_std_config_t std_cfg = {
      .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(EXAMPLE_SAMPLE_RATE),
      .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
      .gpio_cfg = {
          .mclk = I2S_MCK_IO,
          .bclk = I2S_BCK_IO,
          .ws = I2S_WS_IO,
          .dout = I2S_DO_IO,
          .din = I2S_DI_IO,
          .invert_flags = {
            .mclk_inv = false,
            .bclk_inv = false,
            .ws_inv = false,
        },
      },
  };
  ESP_LOGI(TAG, "GPIO配置验证：");
  ESP_LOGI(TAG, "MCLK: GPIO%d (输出)", std_cfg.gpio_cfg.mclk);
  ESP_LOGI(TAG, "BCLK: GPIO%d (输出)", std_cfg.gpio_cfg.bclk);
  ESP_LOGI(TAG, "WS: GPIO%d (输出)", std_cfg.gpio_cfg.ws);
  ESP_LOGI(TAG, "DIN: GPIO%d (输入)", std_cfg.gpio_cfg.din);
  ESP_LOGI(TAG, "I2S RX Channel Handle: %p", rx_handle);
  // 配置并启用通道
  ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_handle, &std_cfg)); //查了半天原来是rx启动了tx没启动！草
  ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_handle, &std_cfg)); // 先配置模式
  ESP_ERROR_CHECK(i2s_channel_enable(tx_handle));
  ESP_ERROR_CHECK(i2s_channel_enable(rx_handle)); // 再启用
  // 在i2s_channel_enable后添加：
  ESP_LOGI(TAG, "I2S通道配置：");
  ESP_LOGI(TAG, "数据位宽: %d bits", std_cfg.slot_cfg.data_bit_width);
  ESP_LOGI(TAG, "通道模式: %s", (std_cfg.slot_cfg.slot_mode == I2S_SLOT_MODE_MONO ? "单声道" : "立体声"));
  ESP_LOGI(TAG, "I2S RX Channel Enabled Successfully");

  // 在i2s_init()中添加：
  ESP_LOGI(TAG, "MCLK频率: %d Hz", clock_config.mclk_frequency);
  ESP_LOGI(TAG, "期望MCLK频率: %d Hz",EXAMPLE_MCLK_FREQ_HZ);
  ESP_LOGI(TAG, "当前采样率: %d Hz", EXAMPLE_SAMPLE_RATE);
}

static void CaptureSamples(void* arg) {
  int16_t *mic_data = (int16_t*)malloc(i2s_bytes_to_read);
  if (!mic_data)
  {
      ESP_LOGE(TAG, "[echo] No memory for read data buffer");
      abort();
  }
  esp_err_t ret = ESP_OK;
  size_t bytes_read = 0;
  size_t bytes_write = 0;
  bool audio_detected = false;
  float db_threshold = 40.0f;   // dB阈值（例如 20dB）
  ESP_LOGI(TAG, "CaptureSamples Task: rx_handle = %p", rx_handle); // 验证非空
  ESP_LOGI(TAG, "CaptureSamples Task: tx_handle = %p", tx_handle); // 验证非空
  if (rx_handle == NULL) {
    ESP_LOGE(TAG, "I2S Channel not initialized!");
    while(1); // 阻塞
  }
  ESP_LOGI(TAG, "[echo] Echo start");

  // const size_t buffer_size = i2s_bytes_to_read; // 使用预定义的3200字节
  // int16_t audio_buffer[buffer_size / sizeof(int16_t)]; // 1600个样本的静态数组
  // // int16_t* audio_buffer = (int16_t*)malloc(buffer_size); // 动态分配缓冲区
  // ESP_LOGI(TAG, "buffer_size: %d bytes", buffer_size);
  // ESP_LOGI(TAG, "audio_buffer address: %p", audio_buffer);
  // // 确保buffer_size与配置匹配：
  // static_assert(buffer_size == i2s_bytes_to_read, "Buffer size mismatch");

  // ESP_LOGI(TAG, "bytes_read before read: %d", bytes_read); // 应为0
  // ret = i2s_channel_read(rx_handle, audio_buffer, buffer_size, &bytes_read, pdMS_TO_TICKS(5000));
  // ESP_LOGI(TAG, "bytes_read after read: %d", bytes_read); // 检查是否变化

  while (1) {
    memset(mic_data, 0, i2s_bytes_to_read);
    //多次实验，和延时，缓存都没关系，rx_handle和tx_handle也都是有值的。为什么和i2s_es8311结果完全不同呢？答案：因为只enable了rx通道，没有enable tx通道！
    ret = i2s_channel_read(rx_handle, mic_data, i2s_bytes_to_read, &bytes_read, pdMS_TO_TICKS(1000));
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "I2S Read Failed! Error Code: 0x%x", ret);
      // 根据错误码处理
      switch (ret) {
          case ESP_ERR_TIMEOUT:
              ESP_LOGE(TAG, "Read Timed Out!");
              break;
          case ESP_ERR_INVALID_ARG:
              ESP_LOGE(TAG, "Invalid Argument!");
              break;
          default:
              ESP_LOGE(TAG, "Unknown Error");
              break;
      }
    }
    if (bytes_read <= 0) {
      ESP_LOGE(TAG, "Error in I2S read : %d", bytes_read);
    } else {
      memcpy(g_i2s_read_buffer, mic_data, bytes_read); 
      int16_t* audio_data = (int16_t*)g_i2s_read_buffer;
      int samples = bytes_read / sizeof(int16_t);
      int sum = 0;
      for (int i = 0; i < samples; i++) {
        sum += audio_data[i] * audio_data[i];
      }
      float rms = sqrt(static_cast<float>(sum) / samples);
      float db_value = 20 * std::log10(rms); 
      //过滤低于一定阈值的环境噪声
      if (db_value > db_threshold) {
        // ESP_LOGI(TAG, "Audio Detected: RMS=%.2f, dB=%.2f", rms, db_value);
      } else {
        // ESP_LOGI(TAG, "Noise Detected: RMS=%.2f, dB=%.2f", rms, db_value);
       }
      
      if (bytes_read < i2s_bytes_to_read) {
        ESP_LOGW(TAG, "Partial I2S read");
      }

// 如果 I2S 配置为 16位数据，则 bytes_read 的原始数据已经是 16 位，但代码将其视为 32 位处理，导致数据被错误截断（如 raw_sample >> 8 会丢失高 8 位有效数据）。
// #if CONFIG_IDF_TARGET_ESP32S3
//       // rescale the data
//       for (int i = 0; i < bytes_read / 4; ++i) {
//         // ((int16_t *) g_i2s_read_buffer)[i] = ((int32_t *) g_i2s_read_buffer)[i] >> 8;
//         int32_t raw_sample = ((int32_t*)g_i2s_read_buffer)[i];
//         ((int16_t*)g_i2s_read_buffer)[i] = (int16_t)(raw_sample >> 8); 
//       }
//       bytes_read = bytes_read / 2;
// #endif
      
      /* write bytes read by i2s into ring buffer */
      int bytes_written = rb_write(g_audio_capture_buffer,
                                   (uint8_t*)g_i2s_read_buffer, bytes_read, pdMS_TO_TICKS(500));
      if (bytes_written != bytes_read) {
        ESP_LOGI(TAG, "Could only write %d bytes out of %d", bytes_written, bytes_read);
      }
      else{
        // 写入环形缓冲区后
        // ESP_LOGI(TAG, "RingBuffer Write: %d bytes (Filled: %d/%ld)",
        // bytes_written, rb_filled(g_audio_capture_buffer), kAudioCaptureBufferSize);
      }
      /* update the timestamp (in ms) to let the model know that new data has
       * arrived */
      g_latest_audio_timestamp = g_latest_audio_timestamp +
          ((1000 * (bytes_written / 2)) / kAudioSampleFrequency);
      if (bytes_written <= 0) {
        ESP_LOGE(TAG, "Could Not Write in Ring Buffer: %d ", bytes_written);
      } else if (bytes_written < bytes_read) {
        ESP_LOGW(TAG, "Partial Write");
      }
      
    }
  }
  // 如果在这里是循环执行，是不是可以注释掉任务删除？
  vTaskDelete(NULL);
  // ESP_LOGI(TAG, "Bytes Read from I2S: %d", bytes_read);
  // ESP_LOGI(TAG, "First Sample Value: %d", ((int16_t*)g_i2s_read_buffer)[0]);
}

TfLiteStatus InitAudioRecording() {
  g_latest_audio_timestamp = 0;
  ESP_LOGI(TAG, "Initializing Audio Recording...");
  i2s_init();
  g_audio_capture_buffer = rb_init("tf_ringbuffer", kAudioCaptureBufferSize);
  if (!g_audio_capture_buffer) {
    ESP_LOGE(TAG, "Error creating ring buffer");
    return kTfLiteError;
  }
  /* create CaptureSamples Task which will get the i2s_data from mic and fill it
   * in the ring buffer */
  ESP_LOGI(TAG, "I2S finished Initializing...");
  xTaskCreate(CaptureSamples, "CaptureSamples", 8192, NULL, 5, NULL);
  while (!g_latest_audio_timestamp) {
    vTaskDelay(1); // one tick delay to avoid watchdog
  }
  ESP_LOGI(TAG, "Audio Recording started");
  return kTfLiteOk;
}

TfLiteStatus GetAudioSamples1(int* audio_samples_size, int16_t** audio_samples)
{
  if (!g_is_audio_initialized) {
    TfLiteStatus init_status = InitAudioRecording();
    if (init_status != kTfLiteOk) {
      return init_status;
    }
    g_is_audio_initialized = true;
  }
  int bytes_read =
    rb_read(g_audio_capture_buffer, (uint8_t*)(g_audio_output_buffer),new_samples_to_get * sizeof(int16_t), pdMS_TO_TICKS(200));
  if (bytes_read < 0) {
    ESP_LOGI(TAG, "Couldn't read data in time");
    bytes_read = 0;
  }
  *audio_samples_size = bytes_read;
  *audio_samples = g_audio_output_buffer;
  return kTfLiteOk;
}

TfLiteStatus GetAudioSamples(int start_ms, int duration_ms,
                             int* audio_samples_size, int16_t** audio_samples) {
  if (!g_is_audio_initialized) {
    TfLiteStatus init_status = InitAudioRecording();
    if (init_status != kTfLiteOk) {
      return init_status;
    }
    g_is_audio_initialized = true;
  }

  /* copy 160 samples (320 bytes) into output_buff from history */
  memcpy((void*)(g_audio_output_buffer), (void*)(g_history_buffer),
         history_samples_to_keep * sizeof(int16_t));

  /* copy 320 samples (640 bytes) from rb at ( int16_t*(g_audio_output_buffer) +
   * 160 ), first 160 samples (320 bytes) will be from history */
  int bytes_read =
      rb_read(g_audio_capture_buffer,
              ((uint8_t*)(g_audio_output_buffer + history_samples_to_keep)),
              new_samples_to_get * sizeof(int16_t), pdMS_TO_TICKS(200));
  if (bytes_read < 0) {
    ESP_LOGE(TAG, " Model Could not read data from Ring Buffer");
  } else if (bytes_read < new_samples_to_get * sizeof(int16_t)) {
    ESP_LOGD(TAG, "RB FILLED RIGHT NOW IS %d",
             rb_filled(g_audio_capture_buffer));
    ESP_LOGD(TAG, " Partial Read of Data by Model ");
    ESP_LOGV(TAG, " Could only read %d bytes when required %d bytes ",
             bytes_read, (int) (new_samples_to_get * sizeof(int16_t)));
  }

  /* copy 320 bytes from output_buff into history */
  memcpy((void*)(g_history_buffer),
         (void*)(g_audio_output_buffer + new_samples_to_get),
         history_samples_to_keep * sizeof(int16_t));

  *audio_samples_size = kMaxAudioSampleSize;
  *audio_samples = g_audio_output_buffer;
  // ESP_LOGI(TAG, "Bytes Read from RingBuf: %d", bytes_read);
  // ESP_LOGI(TAG, "First Sample in Output Buffer: %d", g_audio_output_buffer[0]);
  return kTfLiteOk;
}

int32_t LatestAudioTimestamp() { return g_latest_audio_timestamp; }

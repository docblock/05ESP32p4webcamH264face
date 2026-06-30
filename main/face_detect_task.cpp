/*
 * Image classification task — C++ implementation with extern "C" wrappers
 * to replace face detection with ImageNet classification using MobileNetV2.
 *
 * Pipeline per inference cycle:
 *   1. face_detect_task_post_frame() copies the RGB888 frame (non-blocking).
 *   2. The classification task wakes up, runs MobileNetV2.
 *   3. Result (top-1 category name and score) is stored in a mutex-protected struct and
 *      logged via ESP_LOGI. The HTTP endpoints read the struct.
 *
 * Task pinning:
 *   Core 0  – classification task (heavy inference; camera stream runs on Core 1)
 *   Core 1  – frame_processing_task (PPA + JPEG, see image_processor.c)
 */

#include "face_detect_task.h"
#include "imagenet_cls.hpp"
#include "example_config.h"

#include "esp_log.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include <atomic>
#include <cstring>
#include <vector>

static const char *TAG = "imagenet_cls";

// ── Frame dimensions (must match PPA output in image_processor.c) ───────────
#define CLS_IMG_WIDTH  FINAL_WIDTH
#define CLS_IMG_HEIGHT FINAL_HEIGHT
#define CLS_IMG_BYTES  (CLS_IMG_WIDTH * CLS_IMG_HEIGHT * 3)

// ── Model instance (created once inside the task) ───────────────────────────
static ImageNetCls *g_imagenet_cls = nullptr;

// ── Frame ping-pong ──────────────────────────────────────────────────────────
static uint8_t          *g_frame_buf  = nullptr;
static SemaphoreHandle_t g_frame_sem  = nullptr;  // binary: signals new frame
static std::atomic<bool> g_frame_busy{false};     // true while task is busy

// ── Result storage ───────────────────────────────────────────────────────────
static SemaphoreHandle_t g_result_mutex = nullptr;
static struct {
    char      name[64];
    float     score;
    TickType_t last_detection_tick;
} g_result = {"no_face", 0.0f, 0};

// ── Task function ─────────────────────────────────────────────────────────────
static void classification_task_fn(void * /*arg*/)
{
    ESP_LOGI(TAG, "Task started (free stack: %u)", uxTaskGetStackHighWaterMark(NULL));

    // Suppress verbose logging from underlying libraries
    esp_log_level_set("imagenet_cls", ESP_LOG_INFO);

    ESP_LOGI(TAG, "Creating ImageNetCls classifier...");
    g_imagenet_cls = new ImageNetCls(ImageNetCls::MOBILENETV2_S8_V1, false);
    ESP_LOGI(TAG, "ImageNetCls loaded and ready");

    TickType_t last_log_tick = 0;
    while (true) {
        // Block until a new frame has been posted
        xSemaphoreTake(g_frame_sem, portMAX_DELAY);

        // Build esp-dl image descriptor pointing into our frame buffer
        dl::image::img_t img;
        img.data     = g_frame_buf;
        img.width    = CLS_IMG_WIDTH;
        img.height   = CLS_IMG_HEIGHT;
        img.pix_type = dl::image::DL_IMAGE_PIX_TYPE_RGB888;

        // Run classification inference
        std::vector<dl::cls::result_t> &results = g_imagenet_cls->run(img);

        if (!results.empty()) {
            auto &top = results[0];

            if (xSemaphoreTake(g_result_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                strncpy(g_result.name, top.cat_name, sizeof(g_result.name) - 1);
                g_result.name[sizeof(g_result.name) - 1] = '\0';
                g_result.score = top.score;
                g_result.last_detection_tick = xTaskGetTickCount();
                xSemaphoreGive(g_result_mutex);
            }

            TickType_t now = xTaskGetTickCount();
            if ((now - last_log_tick) >= pdMS_TO_TICKS(1000)) {
                ESP_LOGI(TAG, "Top class: %s (Prob: %.1f%%)", top.cat_name, top.score * 100.0f);
                for (size_t i = 0; i < results.size(); ++i) {
                    ESP_LOGI(TAG, "  #%d: %s -> Prob: %.1f%%", (int)i + 1, results[i].cat_name, results[i].score * 100.0f);
                }
                last_log_tick = now;
            }
        } else {
            if (xSemaphoreTake(g_result_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                strncpy(g_result.name, "no_face", sizeof(g_result.name) - 1);
                g_result.name[sizeof(g_result.name) - 1] = '\0';
                g_result.score = 0.0f;
                xSemaphoreGive(g_result_mutex);
            }
        }

        // Allow next frame to be posted
        g_frame_busy.store(false);
    }
}

// ── Public C API wrappers ───────────────────────────────────────────────────
extern "C" {

void face_detect_task_init(void)
{
    ESP_LOGI(TAG, "Initializing classification task...");

    g_frame_buf = static_cast<uint8_t *>(
        heap_caps_malloc(CLS_IMG_BYTES, MALLOC_CAP_SPIRAM));
    if (!g_frame_buf) {
        ESP_LOGE(TAG, "Failed to allocate frame buffer in PSRAM");
        return;
    }
    ESP_LOGI(TAG, "Allocated frame buffer in PSRAM at %p (%d bytes)", g_frame_buf, CLS_IMG_BYTES);

    g_frame_sem    = xSemaphoreCreateBinary();
    g_result_mutex = xSemaphoreCreateMutex();
    if (!g_frame_sem || !g_result_mutex) {
        ESP_LOGE(TAG, "Failed to create FreeRTOS sync objects");
        return;
    }

    // Start classification task on Core 0 with 64KB stack
    BaseType_t rc = xTaskCreatePinnedToCore(
        classification_task_fn, "imagenet_cls", 65536, nullptr, 4, nullptr, 0);
    if (rc != pdPASS) {
        ESP_LOGE(TAG, "Failed to create classification task (%d)", rc);
        return;
      }
      ESP_LOGI(TAG, "Classification task started on Core 0");
  }

  void face_detect_task_post_frame(const uint8_t *rgb888, int width, int height)
  {
      if (!g_frame_buf || !g_frame_sem) {
          return;
      }
      // Drop the frame if the task is currently busy
      bool expected = false;
      if (!g_frame_busy.compare_exchange_strong(expected, true)) {
          return;
      }

      size_t copy_len = (size_t)(width * height * 3);
      if (copy_len > CLS_IMG_BYTES) {
          copy_len = CLS_IMG_BYTES;
      }
      memcpy(g_frame_buf, rgb888, copy_len);
      xSemaphoreGive(g_frame_sem);
  }

  void face_detect_task_get_result(char *buf, size_t buf_len, float *score, int *x1, int *y1, int *x2, int *y2)
  {
      if (!buf || buf_len == 0) {
          return;
      }

      // Initialize outputs
      if (x1) *x1 = 0;
      if (y1) *y1 = 0;
      if (x2) *x2 = 0;
      if (y2) *y2 = 0;

      if (g_result_mutex && xSemaphoreTake(g_result_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
          strncpy(buf, g_result.name, buf_len - 1);
          buf[buf_len - 1] = '\0';
          if (score) {
              *score = g_result.score;
          }
          xSemaphoreGive(g_result_mutex);
      } else {
          strncpy(buf, "unknown", buf_len - 1);
          buf[buf_len - 1] = '\0';
          if (score) {
              *score = 0.0f;
          }
      }
  }

  } // extern "C"

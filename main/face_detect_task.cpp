/*
 * Face detection task — C++ implementation with extern "C" wrappers
 * so that plain-C files can call face_detect_task_init / _post_frame / _get_result.
 *
 * Pipeline per inference cycle:
 *   1. face_detect_task_post_frame() copies the 224×224 RGB888 frame (non-blocking).
 *   2. The face detection task wakes up, runs HumanFaceDetect.
 *   3. Result ("face" / "no_face" + score) is stored in a mutex-protected struct and
 *      logged via ESP_LOGI. The HTTP /gesture endpoint reads the struct.
 *
 * Task pinning:
 *   Core 0  – face detection task (heavy inference; camera stream runs on Core 1)
 *   Core 1  – frame_processing_task (PPA + JPEG, see image_processor.c)
 */

#include "face_detect_task.h"
#include "human_face_detect.hpp"

#include "esp_log.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include <atomic>
#include <cstring>

static const char *TAG = "face_detect";

// ── Frame dimensions (must match PPA output in image_processor.c) ───────────
#define FACE_IMG_WIDTH  224
#define FACE_IMG_HEIGHT 224
#define FACE_IMG_BYTES  (FACE_IMG_WIDTH * FACE_IMG_HEIGHT * 3)

// ── Model instance (created once inside the task) ───────────────────────────
static HumanFaceDetect *g_face_detect = nullptr;

// ── Frame ping-pong ──────────────────────────────────────────────────────────
static uint8_t          *g_frame_buf  = nullptr;
static SemaphoreHandle_t g_frame_sem  = nullptr;  // binary: signals new frame
static std::atomic<bool> g_frame_busy{false};     // true while task is busy

// ── Result storage ───────────────────────────────────────────────────────────
static SemaphoreHandle_t g_result_mutex = nullptr;
// Keep the last detected face visible for this many ms before reverting to "no_face".
#define FACE_HOLD_MS 800
static struct {
    char      name[32];
    float     score;
    int       x1, y1, x2, y2;
    TickType_t last_detection_tick;  // tick of last non-"no_face" result
} g_result = {"no_face", 0.0f, 0, 0, 0, 0, 0};

// ── Task function ─────────────────────────────────────────────────────────────
static void face_detect_task_fn(void * /*arg*/)
{
    ESP_LOGI(TAG, "Task fn started (free stack: %u)", uxTaskGetStackHighWaterMark(NULL));

    // Suppress per-frame warnings from the esp-dl library.
    esp_log_level_set("human_face_detect", ESP_LOG_ERROR);

    // Load models here (inside the task) so that the caller's init sequence
    // is not blocked by flash reads.
    ESP_LOGI(TAG, "Creating HumanFaceDetect...");
    g_face_detect = new HumanFaceDetect();
    ESP_LOGI(TAG, "Models loaded — ready");

    TickType_t last_log_tick = 0;
    while (true) {
        // Block until a new frame has been posted.
        xSemaphoreTake(g_frame_sem, portMAX_DELAY);

        // Build esp-dl image descriptor (points into our private buffer).
        dl::image::img_t img;
        img.data     = g_frame_buf;
        img.width    = FACE_IMG_WIDTH;
        img.height   = FACE_IMG_HEIGHT;
        img.pix_type = dl::image::DL_IMAGE_PIX_TYPE_RGB888;

        // Run face detection
        const std::list<dl::detect::result_t> &detect_res = g_face_detect->run(img);

        float max_score = 0.0f;
        int max_x1 = 0, max_y1 = 0, max_x2 = 0, max_y2 = 0;
        for (const auto &res : detect_res) {
            if (res.score > max_score) {
                max_score = res.score;
                if (res.box.size() >= 4) {
                    max_x1 = res.box[0];
                    max_y1 = res.box[1];
                    max_x2 = res.box[2];
                    max_y2 = res.box[3];
                }
            }
        }

        // Store result under mutex and log throttled to once per 500ms.
        if (xSemaphoreTake(g_result_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            if (max_score > 0.01f) {
                strncpy(g_result.name, "face", sizeof(g_result.name) - 1);
                g_result.name[sizeof(g_result.name) - 1] = '\0';
                g_result.score = max_score;
                g_result.x1 = max_x1;
                g_result.y1 = max_y1;
                g_result.x2 = max_x2;
                g_result.y2 = max_y2;
                g_result.last_detection_tick = xTaskGetTickCount();
            } else {
                // Only revert to "no_face" after the hold time has expired
                TickType_t now = xTaskGetTickCount();
                if ((now - g_result.last_detection_tick) >= pdMS_TO_TICKS(FACE_HOLD_MS)) {
                    strncpy(g_result.name, "no_face", sizeof(g_result.name) - 1);
                    g_result.name[sizeof(g_result.name) - 1] = '\0';
                    g_result.score = 0.0f;
                    g_result.x1 = 0;
                    g_result.y1 = 0;
                    g_result.x2 = 0;
                    g_result.y2 = 0;
                }
            }

            TickType_t now = xTaskGetTickCount();
            if ((now - last_log_tick) >= pdMS_TO_TICKS(500)) {
                if (strcmp(g_result.name, "face") == 0) {
                    ESP_LOGI(TAG, "%s detected (%.1f%%)", g_result.name, g_result.score * 100.0f);
                } else {
                    ESP_LOGD(TAG, "No face detected");
                }
                last_log_tick = now;
            }
            xSemaphoreGive(g_result_mutex);
        }

        // Allow the next frame to be posted.
        g_frame_busy.store(false);
    }
}

// ── Public C-linkage API ──────────────────────────────────────────────────────
extern "C" {

void face_detect_task_init(void)
{
    ESP_LOGI(TAG, "face_detect_task_init entered");

    g_frame_buf = static_cast<uint8_t *>(
        heap_caps_malloc(FACE_IMG_BYTES, MALLOC_CAP_SPIRAM));
    ESP_LOGI(TAG, "frame buf alloc: %p (need %d bytes)", g_frame_buf, FACE_IMG_BYTES);
    if (!g_frame_buf) {
        ESP_LOGE(TAG, "Failed to allocate frame buffer");
        return;
    }

    g_frame_sem    = xSemaphoreCreateBinary();
    g_result_mutex = xSemaphoreCreateMutex();
    if (!g_frame_sem || !g_result_mutex) {
        ESP_LOGE(TAG, "Failed to create synchronization primitives");
        return;
    }

    // Pin to Core 0; frame_processing_task runs on Core 1.
    // 64 KB stack: esp-dl C++ objects need generous stack space.
    BaseType_t rc = xTaskCreatePinnedToCore(
        face_detect_task_fn, "face_detect", 65536, nullptr, 4, nullptr, 0);
    if (rc != pdPASS) {
        ESP_LOGE(TAG, "xTaskCreatePinnedToCore failed (%d)", rc);
        return;
    }
    ESP_LOGI(TAG, "Face detection task started on Core 0 (stack 64KB)");
}

void face_detect_task_post_frame(const uint8_t *rgb888, int width, int height)
{
    if (!g_frame_buf || !g_frame_sem) {
        return;
    }
    // Non-blocking: drop frame if task is still processing the previous one.
    bool expected = false;
    if (!g_frame_busy.compare_exchange_strong(expected, true)) {
        return;
    }
    size_t copy_len = (size_t)(width * height * 3);
    if (copy_len > FACE_IMG_BYTES) {
        copy_len = FACE_IMG_BYTES;
    }
    memcpy(g_frame_buf, rgb888, copy_len);
    xSemaphoreGive(g_frame_sem);
}

void face_detect_task_get_result(char *buf, size_t buf_len, float *score, int *x1, int *y1, int *x2, int *y2)
{
    if (!buf || buf_len == 0) {
        return;
    }
    if (g_result_mutex &&
        xSemaphoreTake(g_result_mutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        strncpy(buf, g_result.name, buf_len - 1);
        buf[buf_len - 1] = '\0';
        if (score) {
            *score = g_result.score;
        }
        if (x1) *x1 = g_result.x1;
        if (y1) *y1 = g_result.y1;
        if (x2) *x2 = g_result.x2;
        if (y2) *y2 = g_result.y2;
        xSemaphoreGive(g_result_mutex);
    } else {
        strncpy(buf, "unknown", buf_len - 1);
        buf[buf_len - 1] = '\0';
        if (score) {
            *score = 0.0f;
        }
        if (x1) *x1 = 0;
        if (y1) *y1 = 0;
        if (x2) *x2 = 0;
        if (y2) *y2 = 0;
    }
}

} // extern "C"

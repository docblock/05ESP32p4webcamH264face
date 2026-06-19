/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "image_processor.h"
#include "camera_controller.h"  // frame_event_t
#include "mjpeg_server.h"       // video_server_init, video_server_push_frame
#include "example_config.h"     // CAMERA_WIDTH, CAMERA_HEIGHT, FINAL_WIDTH, FINAL_HEIGHT
#include "face_detect_task.h"
#include <string.h>
#include "esp_log.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_cache.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "driver/ppa.h"
#include "esp_h264_enc_single_hw.h"
#include "esp_h264_alloc.h"

static const char *TAG = "img_proc";

// Output buffer size: YUV420 is 1.5 bytes per pixel, aligned to 64 bytes
#define FINAL_BUFFER_SIZE_ALIGNED ((((FINAL_WIDTH * FINAL_HEIGHT * 3) / 2) + 63) & ~63)

// Dynamic PPA SRM variables calculated at startup
static uint16_t s_ppa_crop_w   = FINAL_WIDTH;
static uint16_t s_ppa_crop_h   = FINAL_HEIGHT;
static uint16_t s_ppa_offset_x = 0;
static uint16_t s_ppa_offset_y = 0;
static float    s_ppa_scale_x  = 1.0f;
static float    s_ppa_scale_y  = 1.0f;

static ppa_client_handle_t   ppa_srm_handle     = NULL;
static uint8_t              *ppa_output_buffer  = NULL;
static esp_h264_enc_handle_t h264_enc           = NULL;
static uint8_t              *h264_buffer        = NULL;
static uint32_t              s_h264_buf_size    = 0;
static uint32_t              s_pts_counter      = 0;

static QueueHandle_t         s_frame_queue      = NULL;
static QueueHandle_t         s_buffer_pool      = NULL;

// Scaled RGB888 buffer for AI inference
static uint8_t              *s_ai_rgb888_buf    = NULL;
static uint32_t              s_ai_rgb888_size   = 0;
static SemaphoreHandle_t     s_ai_buf_mutex     = NULL;

/*
 * PPA (Pixel Processing Accelerator): Hardware-accelerated image transformation.
 * Performs centered crop, scaling, mirroring, and RGB565 -> YUV420 (O_UYY_E_VYY) color space conversion.
 */
static esp_err_t transform_image_with_ppa(const uint8_t *src, uint8_t *dst)
{
    ppa_srm_oper_config_t srm_config = {
        .in = {
            .buffer         = src,
            .pic_w          = CAMERA_WIDTH,
            .pic_h          = CAMERA_HEIGHT,
            .block_w        = s_ppa_crop_w,
            .block_h        = s_ppa_crop_h,
            .block_offset_x = s_ppa_offset_x,
            .block_offset_y = s_ppa_offset_y,
            .srm_cm         = PPA_SRM_COLOR_MODE_RGB565,
        },
        .out = {
            .buffer         = dst,
            .buffer_size    = FINAL_BUFFER_SIZE_ALIGNED,
            .pic_w          = FINAL_WIDTH,
            .pic_h          = FINAL_HEIGHT,
            .block_offset_x = 0,
            .block_offset_y = 0,
            .srm_cm         = PPA_SRM_COLOR_MODE_YUV420,
            .yuv_range      = PPA_COLOR_RANGE_FULL,
            .yuv_std        = PPA_COLOR_CONV_STD_RGB_YUV_BT601,
        },
        .rotation_angle = PPA_SRM_ROTATION_ANGLE_0,
        .scale_x        = s_ppa_scale_x,
        .scale_y        = s_ppa_scale_y,
        .rgb_swap       = 0,
        .byte_swap      = 0,
        .mode           = PPA_TRANS_MODE_BLOCKING,
        .mirror_x       = true,
        .mirror_y       = false,
    };

    esp_err_t ret = ppa_do_scale_rotate_mirror(ppa_srm_handle, &srm_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "PPA transform failed: %s", esp_err_to_name(ret));
    }
    return ret;
}

/*
 * Frame processing task (runs on Core 1).
 */
static void frame_processing_task(void *arg)
{
    frame_event_t evt;
    esp_h264_enc_in_frame_t in_frame = {0};
    esp_h264_enc_out_frame_t out_frame = {0};
    uint32_t frame_count = 0;

    ESP_LOGI(TAG, "Frame processing task started on Core %d", xPortGetCoreID());

    while (1) {
        if (xQueueReceive(s_frame_queue, &evt, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        frame_count++;
        // Skip every second frame to downsample 50fps sensor to 25fps target, matching JMuxer rate and avoiding lag
        if (frame_count % 2 == 0) {
            xQueueSend(s_buffer_pool, &evt.buffer, 0);
            continue;
        }

        // ESP_LOGI(TAG, "[Frame #%lu] Received raw frame: size=%d", (unsigned long)frame_count, (int)evt.len);

        // Invalidate input cache before reading the camera frame
        esp_cache_msync((void *)evt.buffer, evt.len, ESP_CACHE_MSYNC_FLAG_DIR_M2C);

        // Step 1: PPA transformation for H.264
        // ESP_LOGI(TAG, "[Frame #%lu] Starting PPA scale/convert...", (unsigned long)frame_count);
        esp_err_t ppa_ret = transform_image_with_ppa((const uint8_t *)evt.buffer, ppa_output_buffer);
        //ESP_LOGI(TAG, "[Frame #%lu] PPA finished, result=%d", (unsigned long)frame_count, ppa_ret);

        // Step 1.5: PPA transformation to RGB888 for AI inference
        if (s_ai_rgb888_buf && s_ai_buf_mutex) {
            if (xSemaphoreTake(s_ai_buf_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                ppa_srm_oper_config_t ai_srm_config = {
                    .in = {
                        .buffer         = evt.buffer,
                        .pic_w          = CAMERA_WIDTH,
                        .pic_h          = CAMERA_HEIGHT,
                        .block_w        = s_ppa_crop_w,
                        .block_h        = s_ppa_crop_h,
                        .block_offset_x = s_ppa_offset_x,
                        .block_offset_y = s_ppa_offset_y,
                        .srm_cm         = PPA_SRM_COLOR_MODE_RGB565,
                    },
                    .out = {
                        .buffer         = s_ai_rgb888_buf,
                        .buffer_size    = s_ai_rgb888_size,
                        .pic_w          = FINAL_WIDTH,
                        .pic_h          = FINAL_HEIGHT,
                        .block_offset_x = 0,
                        .block_offset_y = 0,
                        .srm_cm         = PPA_SRM_COLOR_MODE_RGB888,
                    },
                    .rotation_angle = PPA_SRM_ROTATION_ANGLE_0,
                    .scale_x        = s_ppa_scale_x,
                    .scale_y        = s_ppa_scale_y,
                    .rgb_swap       = 1,
                    .byte_swap      = 0,
                    .mode           = PPA_TRANS_MODE_BLOCKING,
                    .mirror_x       = true,
                    .mirror_y       = false,
                };
                esp_err_t ai_ret = ppa_do_scale_rotate_mirror(ppa_srm_handle, &ai_srm_config);
                if (ai_ret == ESP_OK) {
                    esp_cache_msync(s_ai_rgb888_buf, s_ai_rgb888_size, ESP_CACHE_MSYNC_FLAG_DIR_M2C);
                    face_detect_task_post_frame(s_ai_rgb888_buf, FINAL_WIDTH, FINAL_HEIGHT);
                } else {
                    ESP_LOGE(TAG, "PPA AI transform failed: %d", ai_ret);
                }
                xSemaphoreGive(s_ai_buf_mutex);
            } else {
                ESP_LOGW(TAG, "AI buffer locked, skipping AI conversion for frame #%lu", (unsigned long)frame_count);
            }
        }

        // Release camera raw frame buffer immediately
        xQueueSend(s_buffer_pool, &evt.buffer, 0);

        if (ppa_ret != ESP_OK) {
            ESP_LOGE(TAG, "[Frame #%lu] Skipping H264 due to PPA error", (unsigned long)frame_count);
            continue;
        }

        // Sync cache to main memory after PPA output before feeding it to H264 encoder
        esp_cache_msync(ppa_output_buffer, FINAL_BUFFER_SIZE_ALIGNED, ESP_CACHE_MSYNC_FLAG_DIR_C2M);

        // Reset in_frame and out_frame properties for every iteration
        in_frame.raw_data.buffer = ppa_output_buffer;
        in_frame.raw_data.len = FINAL_BUFFER_SIZE_ALIGNED;
        in_frame.pts = s_pts_counter++;

        out_frame.raw_data.buffer = h264_buffer;
        out_frame.raw_data.len = s_h264_buf_size;
        out_frame.length = 0;

        // Step 2: H264 hardware encoding
        //ESP_LOGI(TAG, "[Frame #%lu] Starting H264 encode process...", (unsigned long)frame_count);
        esp_err_t ret = esp_h264_enc_process(h264_enc, &in_frame, &out_frame);
        //ESP_LOGI(TAG, "[Frame #%lu] H264 encode process finished, result=%d, length=%d", 
        //         (unsigned long)frame_count, ret, (int)out_frame.length);

        if (ret == ESP_H264_ERR_OK) {
            video_server_push_frame(out_frame.raw_data.buffer, out_frame.length);
        } else {
            ESP_LOGE(TAG, "H264 encode failed on frame #%lu: %d", (unsigned long)frame_count, ret);
        }
    }
}

esp_err_t image_processor_init(QueueHandle_t frame_queue, QueueHandle_t buffer_pool)
{
    s_frame_queue = frame_queue;
    s_buffer_pool = buffer_pool;

    // Check resolution alignment guidelines for hardware H.264 encoder
    if (FINAL_WIDTH % 16 != 0 || FINAL_HEIGHT % 16 != 0) {
        ESP_LOGW(TAG, "Target resolution (%dx%d) is not a multiple of 16. Hardware H264 encoder may produce artifacts or fail.", FINAL_WIDTH, FINAL_HEIGHT);
    }

    // 1. Calculate PPA crop and scale parameters automatically
    // The hardware scale step in PPA is 1/16.
    // To prevent the hardware from locking up, the scaled input size must match the output size exactly:
    // (crop_w * n) / 16 == FINAL_WIDTH. Thus (FINAL_WIDTH * 16) must be divisible by n.
    float s_min_x = (float)FINAL_WIDTH / (float)CAMERA_WIDTH;
    float s_min_y = (float)FINAL_HEIGHT / (float)CAMERA_HEIGHT;
    float s_min = (s_min_x > s_min_y) ? s_min_x : s_min_y;

    int n = (int)(s_min * 16.0f + 0.999f);
    if (n < 1) n = 1;

    // Search for the smallest n >= min_n that allows exact scaling mapping and is even (required for YUV420/YUV422 in PPA)
    while (n <= 256) {
        if (n % 2 == 0 && (FINAL_WIDTH * 16) % n == 0 && (FINAL_HEIGHT * 16) % n == 0) {
            int w = (FINAL_WIDTH * 16) / n;
            int h = (FINAL_HEIGHT * 16) / n;
            // Both crop dimensions must be even for YUV420 in PPA
            if (w <= CAMERA_WIDTH && h <= CAMERA_HEIGHT && w % 2 == 0 && h % 2 == 0) {
                s_ppa_crop_w = w;
                s_ppa_crop_h = h;
                break;
            }
        }
        n++;
    }

    // Add small epsilon to float scale values to prevent truncation rounding errors (e.g. 0.4375 casting to 6/16 instead of 7/16), which leaves green bars
    s_ppa_scale_x = (float)n / 16.0f + 0.001f;
    s_ppa_scale_y = (float)n / 16.0f + 0.001f;

    s_ppa_offset_x = ((CAMERA_WIDTH - s_ppa_crop_w) / 2) & ~1;
    s_ppa_offset_y = ((CAMERA_HEIGHT - s_ppa_crop_h) / 2) & ~1;

    ESP_LOGI(TAG, "PPA Auto-Config: Crop %dx%d (Offset %d,%d) -> Scale %.4f (n=%d) -> Output %dx%d",
             s_ppa_crop_w, s_ppa_crop_h, s_ppa_offset_x, s_ppa_offset_y, s_ppa_scale_x, n, FINAL_WIDTH, FINAL_HEIGHT);

    // Register PPA client
    ppa_client_config_t ppa_client_config = { .oper_type = PPA_OPERATION_SRM };
    ESP_ERROR_CHECK(ppa_register_client(&ppa_client_config, &ppa_srm_handle));
    ESP_LOGI(TAG, "PPA engine initialized");

    // Output buffer for transformed image (YUV420)
    // Use esp_h264_aligned_calloc to guarantee cache alignment and handle initialization cache sync.
    uint32_t actual_out_size = 0;
    ppa_output_buffer = esp_h264_aligned_calloc(64, 1, FINAL_BUFFER_SIZE_ALIGNED, &actual_out_size, ESP_H264_MEM_SPIRAM);
    if (!ppa_output_buffer) {
        ESP_LOGE(TAG, "Failed to allocate PPA output buffer");
        return ESP_ERR_NO_MEM;
    }

    // 2. Initialize H264 hardware encoder
    esp_h264_enc_cfg_hw_t h264_cfg = { 0 };
    h264_cfg.gop = 30; // GOP size (keyframe interval)
    h264_cfg.fps = 25; // Frame rate
    h264_cfg.res.width = FINAL_WIDTH;
    h264_cfg.res.height = FINAL_HEIGHT;
    h264_cfg.rc.bitrate = 1500000; // 1.5 Mbps target bitrate for crisp quality under motion
    h264_cfg.rc.qp_min = 10;
    h264_cfg.rc.qp_max = 30; // Quantization cap to prevent severe compression blurring
    h264_cfg.pic_type = ESP_H264_RAW_FMT_O_UYY_E_VYY;

    esp_h264_err_t h_err = esp_h264_enc_hw_new(&h264_cfg, &h264_enc);
    if (h_err != ESP_H264_ERR_OK) {
        ESP_LOGE(TAG, "Failed to create H264 hardware encoder: %d", h_err);
        return ESP_FAIL;
    }

    h_err = esp_h264_enc_open(h264_enc);
    if (h_err != ESP_H264_ERR_OK) {
        ESP_LOGE(TAG, "Failed to open H264 encoder: %d", h_err);
        return ESP_FAIL;
    }

    // Allocate H264 output buffer
    s_h264_buf_size = FINAL_BUFFER_SIZE_ALIGNED;
    uint32_t actual_size = 0;
    h264_buffer = esp_h264_aligned_calloc(64, 1, s_h264_buf_size, &actual_size, ESP_H264_MEM_SPIRAM);
    if (!h264_buffer) {
        ESP_LOGE(TAG, "Failed to allocate H264 output buffer");
        return ESP_ERR_NO_MEM;
    }
    s_h264_buf_size = actual_size;

    ESP_LOGI(TAG, "H264 hardware encoder and buffers initialized successfully");

    // Allocate RGB888 AI Buffer
    s_ai_rgb888_size = ((FINAL_WIDTH * FINAL_HEIGHT * 3) + 63) & ~63;
    uint32_t actual_ai_size = 0;
    s_ai_rgb888_buf = esp_h264_aligned_calloc(64, 1, s_ai_rgb888_size, &actual_ai_size, ESP_H264_MEM_SPIRAM);
    if (!s_ai_rgb888_buf) {
        ESP_LOGE(TAG, "Failed to allocate AI RGB888 buffer");
        return ESP_ERR_NO_MEM;
    }
    s_ai_rgb888_size = actual_ai_size;

    s_ai_buf_mutex = xSemaphoreCreateMutex();
    if (!s_ai_buf_mutex) {
        ESP_LOGE(TAG, "Failed to create AI buffer mutex");
        return ESP_ERR_NO_MEM;
    }

    // Initialize streaming server with H.264 buffer size capacity
    ESP_ERROR_CHECK(video_server_init(s_h264_buf_size));

    return ESP_OK;
}

uint8_t *image_processor_lock_ai_buffer(uint32_t *out_size)
{
    if (!s_ai_buf_mutex || !s_ai_rgb888_buf) {
        return NULL;
    }
    if (xSemaphoreTake(s_ai_buf_mutex, portMAX_DELAY) == pdTRUE) {
        if (out_size) {
            *out_size = s_ai_rgb888_size;
        }
        return s_ai_rgb888_buf;
    }
    return NULL;
}

void image_processor_unlock_ai_buffer(void)
{
    if (s_ai_buf_mutex) {
        xSemaphoreGive(s_ai_buf_mutex);
    }
}

void image_processor_start_task(void)
{
    xTaskCreatePinnedToCore(frame_processing_task, "frame_proc", 8192, NULL, 5, NULL, 1);
}

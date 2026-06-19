/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "camera_controller.h"
#include <string.h>
#include "sdkconfig.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_cache.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "driver/i2c_master.h"
#include "driver/isp.h"
#include "esp_cam_ctlr_csi.h"
#include "esp_cam_ctlr.h"
#include "esp_ldo_regulator.h"
#include "example_sensor_init.h"
#include "example_config.h"

static const char *TAG = "cam_ctrl";

#define NUM_FRAME_BUFFERS 2

typedef struct {
    void  *frame_buffers[NUM_FRAME_BUFFERS];
    size_t buffer_size;
    int    current_buffer_idx;
} camera_context_t;

static camera_context_t      cam_ctx      = {0};
static QueueHandle_t         xFrameQueue  = NULL;
static QueueHandle_t         xBufferPool  = NULL;
static uint8_t              *discard_buffer = NULL;
static esp_cam_ctlr_handle_t cam_handle   = NULL;
static esp_ldo_channel_handle_t s_ldo_mipi_phy = NULL;
static i2c_master_bus_handle_t s_i2c_bus_handle = NULL;

/*
 * ISR-Callback: wird aufgerufen, sobald ein kompletter Frame empfangen wurde.
 * Liefert den fertigen Frame an den Verarbeitungs-Task oder gibt den Buffer
 * sofort in den Pool zurück, falls der Task noch beschäftigt ist.
 */
static bool IRAM_ATTR s_camera_get_finished_trans(esp_cam_ctlr_handle_t handle,
                                                   esp_cam_ctlr_trans_t *trans,
                                                   void *user_data)
{
    BaseType_t higherPriorityTaskWoken = pdFALSE;

    if (trans->buffer == discard_buffer) {
        return false;
    }

    frame_event_t evt = {
        .buffer = trans->buffer,
        .len    = trans->received_size,
    };

    if (xQueueSendFromISR(xFrameQueue, &evt, &higherPriorityTaskWoken) != pdTRUE) {
        // Queue voll: Frame verwerfen, Buffer sofort zurück in den Pool
        xQueueSendFromISR(xBufferPool, &trans->buffer, NULL);
    }

    return higherPriorityTaskWoken == pdTRUE;
}

/*
 * ISR-Callback: wird aufgerufen, bevor der DMA-Transfer für den nächsten Frame startet.
 * Muss sofort einen gültigen Schreibbereich liefern. Falls kein freier Buffer im Pool
 * ist, wird der Discard-Buffer verwendet.
 */
static bool IRAM_ATTR s_camera_get_new_vb(esp_cam_ctlr_handle_t handle,
                                           esp_cam_ctlr_trans_t *trans,
                                           void *user_data)
{
    camera_context_t *ctx = (camera_context_t *)user_data;
    BaseType_t woken = pdFALSE;
    void *buf = NULL;

    if (xQueueReceiveFromISR(xBufferPool, &buf, &woken) != pdTRUE) {
        buf = discard_buffer;
    }

    trans->buffer = buf;
    trans->buflen = ctx->buffer_size;
    return woken == pdTRUE;
}

esp_err_t camera_controller_init(void)
{
    esp_err_t ret;

    // MIPI LDO einschalten
    esp_ldo_channel_config_t ldo_mipi_phy_config = {
        .chan_id    = CONFIG_EXAMPLE_USED_LDO_CHAN_ID,
        .voltage_mv = CONFIG_EXAMPLE_USED_LDO_VOLTAGE_MV,
    };
    ESP_ERROR_CHECK(esp_ldo_acquire_channel(&ldo_mipi_phy_config, &s_ldo_mipi_phy));

    // Frame-Buffer allokieren (PSRAM, 64-Byte-aligned)
    cam_ctx.buffer_size        = CAMERA_WIDTH * CAMERA_HEIGHT * EXAMPLE_RGB565_BITS_PER_PIXEL / 8;
    cam_ctx.current_buffer_idx = 0;
    for (int i = 0; i < NUM_FRAME_BUFFERS; i++) {
        cam_ctx.frame_buffers[i] = heap_caps_aligned_calloc(64, 1, cam_ctx.buffer_size, MALLOC_CAP_SPIRAM);
        if (!cam_ctx.frame_buffers[i]) {
            ESP_LOGE(TAG, "Failed to allocate frame buffer %d", i);
            return ESP_ERR_NO_MEM;
        }
        memset(cam_ctx.frame_buffers[i], 0xFF, cam_ctx.buffer_size);
        esp_cache_msync((void *)cam_ctx.frame_buffers[i], cam_ctx.buffer_size, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
        ESP_LOGI(TAG, "Allocated buffer[%d]: %p", i, cam_ctx.frame_buffers[i]);
    }

    // Kamera-Sensor per I2C/SCCB initialisieren
    example_sensor_init(I2C_NUM_0, &s_i2c_bus_handle);

    // CSI-Controller konfigurieren
    esp_cam_ctlr_csi_config_t csi_config = {
        .ctlr_id                = 0,
        .h_res                  = CAMERA_WIDTH,
        .v_res                  = CAMERA_HEIGHT,
        .lane_bit_rate_mbps     = EXAMPLE_MIPI_CSI_LANE_BITRATE_MBPS,
        .input_data_color_type  = CAM_CTLR_COLOR_RAW8,
        .output_data_color_type = CAM_CTLR_COLOR_RGB565,
        .data_lane_num          = 2,
        .byte_swap_en           = false,
        .queue_items            = 2,
    };
    ret = esp_cam_new_csi_ctlr(&csi_config, &cam_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "csi init fail[%d]", ret);
        return ret;
    }

    esp_cam_ctlr_evt_cbs_t cbs = {
        .on_get_new_trans  = s_camera_get_new_vb,
        .on_trans_finished = s_camera_get_finished_trans,
    };
    if (esp_cam_ctlr_register_event_callbacks(cam_handle, &cbs, &cam_ctx) != ESP_OK) {
        ESP_LOGE(TAG, "ops register fail");
        return ESP_FAIL;
    }
    ESP_ERROR_CHECK(esp_cam_ctlr_enable(cam_handle));

    // ISP-Prozessor konfigurieren (RAW8 → RGB565)
    isp_proc_handle_t isp_proc = NULL;
    esp_isp_processor_cfg_t isp_config = {
        .clk_hz                 = 80 * 1000 * 1000,
        .input_data_source      = ISP_INPUT_DATA_SOURCE_CSI,
        .input_data_color_type  = ISP_COLOR_RAW8,
        .output_data_color_type = ISP_COLOR_RGB565,
        .has_line_start_packet  = false,
        .has_line_end_packet    = false,
        .h_res                  = CAMERA_WIDTH,
        .v_res                  = CAMERA_HEIGHT,
    };
    ESP_ERROR_CHECK(esp_isp_new_processor(&isp_config, &isp_proc));
    ESP_ERROR_CHECK(esp_isp_enable(isp_proc));

    // Discard-Buffer: Kamera schreibt hier hin, wenn der Pool leer ist
    discard_buffer = heap_caps_aligned_calloc(64, 1, cam_ctx.buffer_size, MALLOC_CAP_SPIRAM);
    if (!discard_buffer) {
        ESP_LOGE(TAG, "Failed to alloc discard buffer");
        return ESP_ERR_NO_MEM;
    }

    // Buffer-Pool befüllen
    xBufferPool = xQueueCreate(NUM_FRAME_BUFFERS, sizeof(void *));
    for (int i = 0; i < NUM_FRAME_BUFFERS; i++) {
        void *buf = cam_ctx.frame_buffers[i];
        xQueueSend(xBufferPool, &buf, 0);
    }

    // Frame-Queue (Tiefe 1): ISR legt fertigen Frame rein, Task holt ihn ab
    xFrameQueue = xQueueCreate(1, sizeof(frame_event_t));

    return ESP_OK;
}

esp_err_t camera_controller_start(void)
{
    ESP_LOGI(TAG, "Starting camera...");
    return esp_cam_ctlr_start(cam_handle);
}

QueueHandle_t camera_controller_get_frame_queue(void) { return xFrameQueue; }
QueueHandle_t camera_controller_get_buffer_pool(void)  { return xBufferPool; }

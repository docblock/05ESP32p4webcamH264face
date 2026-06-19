/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#include "wifi_manager.h"
#include "camera_controller.h"
#include "image_processor.h"
#include "face_detect_task.h"

static const char *TAG = "cam_main";

void app_main(void)
{
    // Initialize NVS (required for WiFi)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    if (CONFIG_LOG_MAXIMUM_LEVEL > CONFIG_LOG_DEFAULT_LEVEL) {
        esp_log_level_set("wifi", CONFIG_LOG_MAXIMUM_LEVEL);
    }

    // Connect to WiFi
    ESP_LOGI(TAG, "Initializing WiFi Station Mode...");
    wifi_manager_init_sta();

    // Initialize Camera Hardware (LDO, Sensor, CSI, ISP, Buffer-Pool, Queues)
    ESP_LOGI(TAG, "Initializing Camera Controller...");
    ESP_ERROR_CHECK(camera_controller_init());

    // Initialize Image Processing Pipeline (PPA, JPEG, MJPEG-Server)
    ESP_LOGI(TAG, "Initializing Image Processor...");
    ESP_ERROR_CHECK(image_processor_init(
        camera_controller_get_frame_queue(),
        camera_controller_get_buffer_pool()
    ));

    // Initialize and start Face Detection task on Core 0
    face_detect_task_init();

    // Start processing task, then start camera acquisition
    image_processor_start_task();
    ESP_ERROR_CHECK(camera_controller_start());

    ESP_LOGI(TAG, "Webcam application started successfully.");

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        ESP_LOGD(TAG, "Keep-alive tick");
    }
}

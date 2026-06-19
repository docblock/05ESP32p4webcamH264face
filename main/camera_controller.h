/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_err.h"
#include <stddef.h>

/**
 * @brief Beschreibt einen fertigen Kamera-Frame, der vom ISR an den Verarbeitungs-Task
 *        weitergereicht wird.
 */
typedef struct {
    void   *buffer;
    size_t  len;
    int     buffer_idx;
} frame_event_t;

/**
 * @brief Kamera-Hardware initialisieren (LDO, Frame-Buffer, Sensor, CSI, ISP, Queues).
 *        Startet die Kamera noch nicht.
 */
esp_err_t     camera_controller_init(void);

/**
 * @brief Kamera-Stream starten. Muss nach camera_controller_init() und dem Start des
 *        Verarbeitungs-Tasks aufgerufen werden.
 */
esp_err_t     camera_controller_start(void);

/** @brief Queue-Handle für fertige Frames (von ISR → Verarbeitungs-Task). */
QueueHandle_t camera_controller_get_frame_queue(void);

/** @brief Queue-Handle für den Pool freier Frame-Buffer (zwischen ISR und Task). */
QueueHandle_t camera_controller_get_buffer_pool(void);

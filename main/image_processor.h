/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_err.h"

/**
 * @brief Bildverarbeitungs-Pipeline initialisieren:
 *        PPA-Client (Crop/Skalierung/RGB-Konvertierung), JPEG-Encoder, MJPEG-Streaming-Server.
 *
 * @param frame_queue  Queue-Handle für eingehende Kamera-Frames (von camera_controller).
 * @param buffer_pool  Queue-Handle für den Pool freier Frame-Buffer (von camera_controller).
 * @return ESP_OK bei Erfolg.
 */
esp_err_t image_processor_init(QueueHandle_t frame_queue, QueueHandle_t buffer_pool);

/**
 * @brief FreeRTOS-Task starten, der Frames dequeued, per PPA+JPEG verarbeitet
 *        und an den MJPEG-Server übergibt.
 */
void image_processor_start_task(void);

/**
 * @brief Sperrt den Puffer mit den skalierten RGB565-Bildern (224x224) für die KI-Inferenz.
 *        Muss nach der Verwendung mit image_processor_unlock_ai_buffer freigegeben werden.
 *
 * @param out_size Optionale Rückgabe der Puffergröße.
 * @return Zeiger auf den RGB565-Bildpuffer oder NULL bei Fehlern.
 */
uint8_t *image_processor_lock_ai_buffer(uint32_t *out_size);

/**
 * @brief Gibt den Inferenz-Puffer wieder frei, damit der PPA neue Frames schreiben kann.
 */
void image_processor_unlock_ai_buffer(void);

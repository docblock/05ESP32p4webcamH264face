#pragma once

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

/**
 * @brief Initialize Video server: allocate shared frame buffer,
 *        create synchronization primitives, and start HTTP server on port 80.
 *        Serves the viewer page at "/" and the H264 stream at "/stream".
 *
 * @param buf_size  Maximum size of a single frame in bytes.
 * @return ESP_OK on success, ESP_ERR_NO_MEM or ESP_FAIL on error.
 */
esp_err_t video_server_init(size_t buf_size);

/**
 * @brief Push a new H264 frame to the stream.
 *        Thread-safe, can be called from any task context.
 *        Drops the frame silently if the mutex cannot be acquired within 10 ms.
 *
 * @param data  Pointer to encoded frame data.
 * @param size  Size of the data in bytes.
 */
void video_server_push_frame(const uint8_t *data, size_t size);

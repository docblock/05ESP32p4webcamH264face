#pragma once

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize human face detection: allocate frame buffer and
 *        synchronization primitives, then start the face detection FreeRTOS task
 *        on Core 0. Models are loaded lazily inside the task at startup.
 */
void face_detect_task_init(void);

/**
 * @brief Post an RGB888 frame for face detection inference (non-blocking).
 *        If the task is still processing the previous frame, this call returns
 *        immediately without blocking the caller.
 *
 * @param rgb888  Pointer to RGB888 pixel data.
 * @param width   Frame width in pixels.
 * @param height  Frame height in pixels.
 */
void face_detect_task_post_frame(const uint8_t *rgb888, int width, int height);

/**
 * @brief Thread-safe read of the latest face detection result.
 *        Returns "no_face" until a face is detected.
 *
 * @param buf      Output buffer for the result label string.
 * @param buf_len  Size of output buffer (including null terminator).
 * @param score    Optional: receives the confidence score [0.0 .. 1.0].
 * @param x1       Optional: receives the left_up_x coordinate.
 * @param y1       Optional: receives the left_up_y coordinate.
 * @param x2       Optional: receives the right_down_x coordinate.
 * @param y2       Optional: receives the right_down_y coordinate.
 */
void face_detect_task_get_result(char *buf, size_t buf_len, float *score, int *x1, int *y1, int *x2, int *y2);

#ifdef __cplusplus
}
#endif

/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#define EXAMPLE_RGB565_BITS_PER_PIXEL           16
#define EXAMPLE_MIPI_CSI_LANE_BITRATE_MBPS      200 //line_rate = pclk * 4

#define CAMERA_WIDTH   800
#define CAMERA_HEIGHT  800

// Der Jpeg Encoder unterstützt nur bestimmte Auflösungen, daher müssen wir die Zielauflösung
// auf ein vielfaches von 16 setzen, damit die Jpeg Komprimierung funktioniert. 
// 16, 32, 48, 64, 80, 96, 112, 128, 144, 160, 176, 192, 208, 224, 240, 256, 272, 288, 304,
// 320, 336, 352, 368, 384, 400, 416, 432, 448, 464, 480, 496, 512, 528, 544, 560, 576, 592, 608,
// 624, 640, 656, 672, 688, 704, 720, 736, 752, 768, 784, 800

#define FINAL_WIDTH    224 //224
#define FINAL_HEIGHT   224 //224

#ifdef __cplusplus
}
#endif

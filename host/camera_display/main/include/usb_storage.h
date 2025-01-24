/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdint.h>
#include <string.h>

/**
 * @brief Initialize USB MSC device
 *
 * @param[in] new_device_address Address of a newly connected USB MSC device
 */
void msc_init_device(uint8_t new_device_address);

/**
 * @brief De-initialize USB MSC device
 */
void msc_deinit_device(void);

/**
 * @brief Save frame from USB Camera to the USB MSC device
 *
 * @param[in] frame_i Number of frame (will be used for frame file name frame_1.jpeg)
 * @param[in] frame_data Frame data buffer
 * @param[in] frame_len Frame buffer size in bytes
 */
void msc_save_jpeg_frame(int frame_i, uint8_t *frame_data, size_t frame_len);


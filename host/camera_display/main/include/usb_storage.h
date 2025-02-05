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
 *
 * @return
 *    - ESP_OK: USB MSC Device initialize successfully
 *    - ESP_ERR_NOT_FOUND: Failed to opend a file for writing
 */
esp_err_t msc_init_device(uint8_t new_device_address);

/**
 * @brief De-initialize USB MSC device
 *
 * @return
 *    - ESP_OK: USB MSC Device de-initialized successfully
 */
esp_err_t msc_deinit_device(void);

/**
 * @brief Save frame from USB Camera to the USB MSC device
 *
 * @param[in] frame_i Number of frame (will be used for frame file name frame_1.jpeg)
 * @param[in] frame_data Frame data buffer
 * @param[in] frame_len Frame buffer size in bytes
 *
 * @return
 *    - ESP_OK: Frame successfully saved to msc flash drive
 *    - ESP_ERR_NOT_FOUND; Failed to open a file for writing
 *    - ESP_ERR_INVALID_SIZE: Failed to write entire frame to the file
 */
esp_err_t msc_save_jpeg_frame(int frame_i, uint8_t *frame_data, size_t frame_len);

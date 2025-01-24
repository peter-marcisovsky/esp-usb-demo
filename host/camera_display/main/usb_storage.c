/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <sys/stat.h>
#include <dirent.h>
#include <inttypes.h>
#include <errno.h>
#include <ffconf.h>
#include "sdkconfig.h"
#include "esp_err.h"
#include "esp_log.h"
#include "usb/msc_host.h"
#include "usb/msc_host_vfs.h"
#include "usb_storage.h"

// Path in the Virtual File System, where the USB flash drive is going to be mounted
#define MNT_PATH                 "/usb"

const char *directory_base =     "/usb/esp";                 // Base esp directory
const char *directory_frames =   "/usb/esp/frames";          // Directory fro saving frames
const char *info_file_path =     "/usb/esp/info.txt";        // Info file path

static const char *TAG =    "example: storage";

static msc_host_device_handle_t msc_device = NULL;
static msc_host_vfs_handle_t vfs_handle = NULL;

/**
 * @brief Print info about connected MSC USB Device
 *
 * @param[in] info Structure holding the device info
 */
static void print_msc_device_info(msc_host_device_info_t *info)
{
    const size_t megabyte = 1024 * 1024;
    uint64_t capacity = ((uint64_t)info->sector_size * info->sector_count) / megabyte;

    printf("Device info:\n");
    printf("\t Capacity: %llu MB\n", capacity);
    printf("\t Sector size: %"PRIu32"\n", info->sector_size);
    printf("\t Sector count: %"PRIu32"\n", info->sector_count);
    printf("\t PID: 0x%04X \n", info->idProduct);
    printf("\t VID: 0x%04X \n", info->idVendor);
#ifndef CONFIG_NEWLIB_NANO_FORMAT
    wprintf(L"\t iProduct: %S \n", info->iProduct);
    wprintf(L"\t iManufacturer: %S \n", info->iManufacturer);
    wprintf(L"\t iSerialNumber: %S \n", info->iSerialNumber);
#endif
}

void msc_init_device(uint8_t new_device_address)
{
    // 1. MSC flash drive connected. Open it and map it to Virtual File System
    ESP_ERROR_CHECK(msc_host_install_device(new_device_address, &msc_device));
    const esp_vfs_fat_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 3,
        .allocation_unit_size = 8192,
    };
    ESP_ERROR_CHECK(msc_host_vfs_register(msc_device, MNT_PATH, &mount_config, &vfs_handle));

    // 2. Print information about the connected disk
    msc_host_device_info_t info;
    ESP_ERROR_CHECK(msc_host_get_device_info(msc_device, &info));
    print_msc_device_info(&info);

    // 3. List all the files in root directory
    ESP_LOGI(TAG, "ls command output:");
    struct dirent *d;
    DIR *dh = opendir(MNT_PATH);
    assert(dh);
    while ((d = readdir(dh)) != NULL) {
        printf("%s\n", d->d_name);
    }
    closedir(dh);

    // Create /usb/esp directory
    struct stat s = {0};
    const bool base_directory_exists = stat(directory_base, &s) == 0;
    if (!base_directory_exists) {
        if (mkdir(directory_base, 0775) != 0) {
            ESP_LOGE(TAG, "mkdir failed with errno: %s", strerror(errno));
        }
    }

    // Create /usb/esp/frames directory
    const bool frames_directory_exists = stat(directory_frames, &s) == 0;
    if (!frames_directory_exists) {
        if (mkdir(directory_frames, 0775) != 0) {
            ESP_LOGE(TAG, "mkdir failed with errno: %s", strerror(errno));
        }
    }

    // Create /usb/esp/info.txt file, if it doesn't exist
    if (stat(info_file_path, &s) != 0) {
        ESP_LOGI(TAG, "Creating file");
        FILE *f = fopen(info_file_path, "w");
        if (f == NULL) {
            ESP_LOGE(TAG, "Failed to open file for writing");
            return;
        }
        fprintf(f, "This is an ESP-USB Demo\n");
        fclose(f);
    }
}

void msc_deinit_device(void)
{
    if (vfs_handle) {
        ESP_ERROR_CHECK(msc_host_vfs_unregister(vfs_handle));
        vfs_handle = NULL;
    }
    if (msc_device) {
        ESP_ERROR_CHECK(msc_host_uninstall_device(msc_device));
        msc_device = NULL;
    }
}

void msc_save_jpeg_frame(int frame_i, uint8_t *frame_data, size_t frame_len)
{
    char file_path[64];
    sprintf(file_path, "%s/frame_%d.jpg", directory_frames, frame_i);

    FILE *f = fopen(file_path, "wb");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file %s for writing", file_path);
    } else {
        size_t written = fwrite(frame_data, 1, frame_len, f);
        if (written != frame_len) {
            ESP_LOGE(TAG, "Failed to write entire frame to file");
        } else {
            ESP_LOGI(TAG, "Saved frame to %s", file_path);
        }
        fclose(f);
    }
}
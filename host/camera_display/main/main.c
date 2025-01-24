/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_timer.h"
#include "esp_err.h"
#include "esp_log.h"
#include "usb/usb_host.h"
#include "usb/msc_host.h"
#include "usb/uvc_host.h"
#include "jpeg_decoder.h"
#include "usb_storage.h"

// Configs
#define EXAMPLE_USB_DEVICE_VID      CONFIG_DEMO_USB_UVC_DEVICE_VID              // Camera VID
#define EXAMPLE_USB_DEVICE_PID      CONFIG_DEMO_USB_UVC_DEVICE_PID              // Camera PID
#define FRAME_H_RES                 CONFIG_DEMO_USB_UVC_DEVICE_FRAME_H_RES      // Camera frame horizontal resolution
#define FRAME_V_RES                 CONFIG_DEMO_USB_UVC_DEVICE_FRAME_V_RES      // Camera frame vertical resolution
#define FPS                         CONFIG_DEMO_USB_UVC_DEVICE_FPS              // Camera FPS
#define DECODE_EVERY_XTH_FRAME      CONFIG_DEMO_DECODE_EVERY_XTH_FRAME          // Every xth frame will be decoded and send to display. This save CPU time

#define DECODE_WORKING_BUFFER_SIZE 4000 // We must increase JPEG decoder working buffer size

#if CONFIG_SPIRAM
#define NUMBER_OF_FRAME_BUFFERS     (3) // Number of frames from the camera
#else
#define NUMBER_OF_FRAME_BUFFERS     (2) // Number of frames from the camera
#endif


static const char *TAG = "example";
static uint16_t *fb = NULL;                 // Framebuffer for decoded data (to LCD)
static QueueHandle_t frame_queue = NULL;    // Queue of received frames that are passed to processing task
static QueueHandle_t app_queue = NULL;      // Application Queue
static uvc_host_stream_hdl_t stream;

/**
 * @brief Application Queue and its messages ID
 */
typedef struct {
    enum {
        APP_MSC_DEVICE_CONNECTED,               // USB MSC device connect event
        APP_MSC_DEVICE_DISCONNECTED,            // USB MSC device disconnect event
        APP_UVC_DEVICE_DISCONNECTED,            // USB UVC device disconnect event
    } id;
    union {
        uint8_t new_msc_dev_address;            // Address of new USB MSC device for APP_MSC_DEVICE_CONNECTED event
        uvc_host_stream_hdl_t uvc_stream_hdl;   // UVC Stream handle
    } data;
} app_message_t;


static void msc_event_cb(const msc_host_event_t *event, void *arg)
{
    if (event->event == MSC_DEVICE_CONNECTED) {
        ESP_LOGI(TAG, "MSC device connected (usb_addr=%d)", event->device.address);
        app_message_t message = {
            .id = APP_MSC_DEVICE_CONNECTED,
            .data.new_msc_dev_address = event->device.address,
        };
        xQueueSend(app_queue, &message, portMAX_DELAY);
    } else if (event->event == MSC_DEVICE_DISCONNECTED) {
        ESP_LOGI(TAG, "MSC device disconnected");
        app_message_t message = {
            .id = APP_MSC_DEVICE_DISCONNECTED,
        };
        xQueueSend(app_queue, &message, portMAX_DELAY);
    }
}


bool frame_callback(const uvc_host_frame_t *frame, void *user_ctx)
{
    bool frame_processed = false; // If we return false from this callback, we must return the frame with uvc_host_frame_return(stream, frame);

    ESP_LOGD(TAG, "Frame callback");
    switch (frame->vs_format.format) {
    case UVC_VS_FORMAT_YUY2: {
        ESP_LOGD(TAG, "YUY2 frame %dx%d", frame->vs_format.h_res, frame->vs_format.v_res);
        //if (fb) {
        //    yuy2_to_rgb565(frame->data, fb, frame->vs_format.h_res, frame->vs_format.v_res);
        //    esp_lcd_panel_draw_bitmap(display_panel, 0, 0, frame->vs_format.h_res, frame->vs_format.v_res, (const void *)fb);
        //}
        //frame_processed = true;
        break;
    }
    case UVC_VS_FORMAT_H264:
    case UVC_VS_FORMAT_H265:
    case UVC_VS_FORMAT_MJPEG: {
        // Attempt to put the new frame into the queue without checking if it is full.
        // If successful, we save processing time since we avoid the overhead of a full check for every frame.
        BaseType_t frame_put_to_queue = xQueueSendToBack(frame_queue, &frame, 0);

        // Adding the frame to our queue should never fail, because our frame_queue has the same length as the UVC driver queue (NUMBER_OF_FRAME_BUFFERS)
        assert(frame_put_to_queue == pdPASS);
        break;
    }
    default:
        ESP_LOGI(TAG, "Unsupported format!");
        break;
    }
    return frame_processed;
}


static void stream_callback(const uvc_host_stream_event_data_t *event, void *user_ctx)
{
    switch (event->type) {
    case UVC_HOST_TRANSFER_ERROR:
        ESP_LOGE(TAG, "USB error has occurred, err_no = %i", event->transfer_error.error);
        break;
    case UVC_HOST_DEVICE_DISCONNECTED:
        ESP_LOGI(TAG, "Device suddenly disconnected");
        app_message_t message = {
            .id = APP_UVC_DEVICE_DISCONNECTED,
            .data.uvc_stream_hdl = event->device_disconnected.stream_hdl,
        };
        xQueueSend(app_queue, &message, portMAX_DELAY);
        break;
    case UVC_HOST_FRAME_BUFFER_OVERFLOW:
        ESP_LOGW(TAG, "Frame buffer overflow");
        break;
    case UVC_HOST_FRAME_BUFFER_UNDERFLOW:
        ESP_LOGW(TAG, "Frame buffer underflow");
        break;
    default:
        abort();
        break;
    }
}


static void frame_processing_task(void *pvParameters)
{
    uvc_host_frame_t *frame;
    uint8_t *jpeg_working_buffer = malloc(DECODE_WORKING_BUFFER_SIZE);
    assert(jpeg_working_buffer);

    while (1) {
        xQueueReceive(frame_queue, &frame, portMAX_DELAY);
        ESP_LOGI(TAG, "MJPEG frame %dx%d %d bytes", frame->vs_format.h_res, frame->vs_format.v_res, frame->data_len);

        static int frame_i = 0;

        msc_save_jpeg_frame(frame_i, frame->data, frame->data_len);

        if (fb && ((frame_i % DECODE_EVERY_XTH_FRAME) == 0)) {
            frame_i = 0;
            esp_jpeg_image_cfg_t jpeg_cfg = {
                .indata = (uint8_t *)frame->data,
                .indata_size = frame->data_len,
                .outbuf = (uint8_t *)fb,
                .outbuf_size = FRAME_H_RES * FRAME_V_RES * 2,
                .out_format = JPEG_IMAGE_FORMAT_RGB565,
                .out_scale = JPEG_IMAGE_SCALE_0,
                .flags = {
                    .swap_color_bytes = 0,
                },
                .advanced = {
                    .working_buffer = jpeg_working_buffer,
                    .working_buffer_size = DECODE_WORKING_BUFFER_SIZE,
                },
            };
            esp_jpeg_image_output_t outimg;

            if (ESP_OK == esp_jpeg_decode(&jpeg_cfg, &outimg)) {
                ESP_LOGI(TAG, "Decoding OK");
                //esp_lcd_panel_draw_bitmap(display_panel, 0, 0, outimg.width, outimg.height, (const void *)fb);
            } else {
                ESP_LOGW(TAG, "Decoding failed");
            }
        } else {
            ESP_LOGI(TAG, "Skipping decoding of received MJPEG frame");
        }
        frame_i++;

        uvc_host_frame_return(stream, frame);
    }

    // This code should never be reached. Leaving it here for completeness
    free(jpeg_working_buffer);
    vTaskDelete(NULL);
}

static const uvc_host_stream_config_t stream_config = {
    .event_cb = stream_callback,
    .frame_cb = frame_callback,
    .user_ctx = NULL,
    .usb = {
        .vid = EXAMPLE_USB_DEVICE_VID,
        .pid = EXAMPLE_USB_DEVICE_PID,
        .uvc_stream_index = 0,
    },
    .vs_format = {
        .h_res = FRAME_H_RES,
        .v_res = FRAME_V_RES,
        .fps = FPS,
        .format = UVC_VS_FORMAT_MJPEG,
    },
    .advanced = {
        .number_of_frame_buffers = NUMBER_OF_FRAME_BUFFERS,
        .frame_size = 40 * 1024,
#if CONFIG_SPIRAM
        .frame_heap_caps = MALLOC_CAP_SPIRAM,
#else
        .frame_heap_caps = 0,
#endif
        .number_of_urbs = 3,
        .urb_size = 4 * 1024,
    },
};


static void usb_task(void *args)
{
    ESP_LOGI(TAG, "Installing USB Host");
    const usb_host_config_t host_config = { 
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
        .skip_phy_setup = false,
        .enum_filter_cb = NULL,
        .root_port_unpowered = false,
    };
    ESP_ERROR_CHECK(usb_host_install(&host_config));


    ESP_LOGI(TAG, "Installing MSC driver");
    const msc_host_driver_config_t msc_config = {
        .create_backround_task = true,
        .task_priority = 5,
        .stack_size = 4096,
        .callback = msc_event_cb,
    };
    ESP_ERROR_CHECK(msc_host_install(&msc_config));


    ESP_LOGI(TAG, "Installing UVC driver");
    const uvc_host_driver_config_t uvc_driver_config = {
        .driver_task_stack_size = 6 * 1024,
        .driver_task_priority = 6,
        .xCoreID = tskNO_AFFINITY,
        .create_background_task = true,
    };
    ESP_ERROR_CHECK(uvc_host_install(&uvc_driver_config));

    // Notify the app_main task, that the usb_host, msc_host and uvc_host have been installed
    xTaskNotifyGive(args);

    bool has_clients = true;
    while (true) {
        uint32_t event_flags;
        usb_host_lib_handle_events(portMAX_DELAY, &event_flags);

        // Release devices once all clients has deregistered
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            has_clients = false;
            if (usb_host_device_free_all() == ESP_OK) {
                break;
            };
        }
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE && !has_clients) {
            break;
        }
    }

    vTaskDelay(10); // Give clients some time to uninstall
    ESP_LOGI(TAG, "De-Initializing USB");
    ESP_ERROR_CHECK(usb_host_uninstall());
    vTaskDelete(NULL);
}


void app_main(void)
{
    // Create application message queue and frame queue
    app_queue = xQueueCreate(5, sizeof(app_message_t));
    frame_queue = xQueueCreate(NUMBER_OF_FRAME_BUFFERS, sizeof (uvc_host_frame_t *));
    assert(app_queue || frame_queue);

    // Create USB Host Lib handling task, install drivers
    BaseType_t task_created = xTaskCreate(usb_task, "usb_task", 4 * 1024, xTaskGetCurrentTaskHandle(), 10, NULL);
    assert(task_created == pdTRUE);

    // Wait until the drivers are installed
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // Create frame processing task
    task_created = xTaskCreate(frame_processing_task, "frame_processing", 4 * 1024, NULL, 2, NULL);
    assert(task_created == pdTRUE);

    fb = heap_caps_aligned_alloc(64, FRAME_H_RES * FRAME_V_RES * 2, MALLOC_CAP_INTERNAL);
    if (fb == NULL) {
        ESP_LOGW(TAG, "Insufficient memory for LCD frame buffer. LCD output disabled.");
    }

    bool uvc_dev_connected = false;
    TickType_t app_queue_ticks = pdMS_TO_TICKS(500);
    while(1) {
        app_message_t msg;

        if (!uvc_dev_connected) {
            ESP_LOGI(TAG, "Opening UVC device 0x%04X:0x%04X\t%dx%d@%2.1fFPS...",
                     stream_config.usb.vid, stream_config.usb.pid, stream_config.vs_format.h_res, stream_config.vs_format.v_res, stream_config.vs_format.fps);
            esp_err_t err = uvc_host_stream_open(&stream_config, pdMS_TO_TICKS(1000), &stream);
            if (ESP_OK != err) {
                //ESP_LOGI(TAG, "Failed to open device");
                app_queue_ticks = pdMS_TO_TICKS(500);
            } else {
                ESP_LOGI(TAG, "Opened device");
                ESP_ERROR_CHECK(uvc_host_stream_start(stream));
                app_queue_ticks = portMAX_DELAY;
                uvc_dev_connected = true;
            }
        }

        if (xQueueReceive(app_queue, &msg, app_queue_ticks)){
            if (APP_MSC_DEVICE_CONNECTED == msg.id) {
                ESP_LOGI(TAG, "MSC Device connected -> Init MSC Device");
                msc_init_device(msg.data.new_msc_dev_address);
            }
            if (APP_MSC_DEVICE_DISCONNECTED == msg.id) {
                ESP_LOGI(TAG, "MSC Device disconnected -> De-Init MSC Device");
                msc_deinit_device();
            }
            if (APP_UVC_DEVICE_DISCONNECTED == msg.id) {
                ESP_LOGI(TAG, "UVC Device disconnected -> Close the UVC stream");
                ESP_ERROR_CHECK(uvc_host_stream_close(msg.data.uvc_stream_hdl));
                uvc_dev_connected = false;
            }
        }
    }
}
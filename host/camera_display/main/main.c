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
#include "esp_console.h"
#include "usb/usb_host.h"
#include "usb/msc_host.h"
#include "usb/uvc_host.h"
#include "usb_storage.h"
#include "argtable3/argtable3.h"
#include "display.h"
#include "esp_lcd_panel_ops.h"

#include "esp_private/periph_ctrl.h"
#include "driver/jpeg_decode.h"


// Configs
//#define EXAMPLE_USB_DEVICE_VID      CONFIG_DEMO_USB_UVC_DEVICE_VID              // Camera VID
//#define EXAMPLE_USB_DEVICE_PID      CONFIG_DEMO_USB_UVC_DEVICE_PID              // Camera PID
//#define FRAME_H_RES                 CONFIG_DEMO_USB_UVC_DEVICE_FRAME_H_RES      // Camera frame horizontal resolution
//#define FRAME_V_RES                 CONFIG_DEMO_USB_UVC_DEVICE_FRAME_V_RES      // Camera frame vertical resolution

#define EXAMPLE_USB_DEVICE_VID      0x32e4              // Camera VID
#define EXAMPLE_USB_DEVICE_PID      0x9415              // Camera PID
//#define FRAME_H_RES                 CONFIG_DEMO_USB_UVC_DEVICE_FRAME_H_RES      // Camera frame horizontal resolution
#define FRAME_H_RES                 960      // Camera frame horizontal resolution
//#define FRAME_V_RES                 CONFIG_DEMO_USB_UVC_DEVICE_FRAME_V_RES      // Camera frame vertical resolution
#define FRAME_V_RES                 540      // Camera frame vertical resolution

//#define FPS                         CONFIG_DEMO_USB_UVC_DEVICE_FPS              // Camera FPS
//#define DECODE_EVERY_XTH_FRAME      CONFIG_DEMO_DECODE_EVERY_XTH_FRAME          // Every xth frame will be decoded and send to display. This save CPU time
//#define NUMBER_OF_FRAME_BUFFERS     CONFIG_DEMO_FRAME_BUFFS_COUNT               // Number of frames from the camera
#define FPS                         30              // Camera FPS
#define DECODE_EVERY_XTH_FRAME      1          // Every xth frame will be decoded and send to display. This save CPU time
#define NUMBER_OF_FRAME_BUFFERS     5               // Number of frames from the camera

#define CONSOLE_PROMPT              CONFIG_IDF_TARGET

static const char *TAG = "esp-usb-demo";

#define APP_QUEUE_CHECK(condition, warning_message) \
    do {                                           \
        if (condition) {                           \
            ESP_LOGW(TAG, warning_message);        \
            break;                                 \
        }                                          \
    } while (0)


/**
 * @brief Application Queue and its messages ID
 */
typedef struct {
    enum {
        APP_MSC_DEVICE_CONNECTED,               // USB MSC device connect event
        APP_MSC_DEVICE_DISCONNECTED,            // USB MSC device disconnect event
        APP_MSC_STORAGE_START_SAVING_FRAMES,    // USB MSC device start saving frames from camera
        APP_MSC_STORAGE_STOP_SAVING_FRAMES,     // USB MSC device stop saving frames from camera
        APP_UVC_DEVICE_DISCONNECTED,            // USB UVC device disconnect event
        APP_UVC_STREAM_START,                   // Start UVC stream
        APP_UVC_STREAM_STOP,                    // Stop UVC stream
    } id;
    union {
        uint8_t new_msc_dev_address;            // Address of new USB MSC device for APP_MSC_DEVICE_CONNECTED event
        uvc_host_stream_hdl_t uvc_stream_hdl;   // UVC Stream handle
    } data;
} app_message_t;

/**
 * @brief Status of USB Devices
 */

typedef struct {
    bool msc_opened;                            // USB MSC Device opened status
    bool msc_saving_frames;                     // USB MSC Device working status
    bool uvc_opened;                            // USB UVC Device opened status
    bool uvc_streaming;                         // USB UVC Device working status
} usb_devs_status_t;
static usb_devs_status_t usb_devs_status;


/**
 * @brief repl console arguments
 */
typedef struct {
    struct arg_str *action;
    struct arg_end *end;
} control_args_t;
static control_args_t stream_control_args, storage_control_args;

static QueueHandle_t frame_queue = NULL;        // Queue of received frames that are passed to processing task
static QueueHandle_t app_queue = NULL;          // Application Queue
static esp_lcd_panel_handle_t display_panel;    // Display panel handle
static jpeg_decoder_handle_t jpgd_handle = NULL;// JPEG Decoder handle

static jpeg_decode_cfg_t decode_cfg = {
    .output_format = JPEG_DECODE_OUT_FORMAT_RGB565,
    .conv_std = JPEG_YUV_RGB_CONV_STD_BT709,
    .rgb_order = JPEG_DEC_RGB_ELEMENT_ORDER_BGR,
};

static jpeg_decode_memory_alloc_cfg_t rx_mem_cfg = {
    .buffer_direction = JPEG_DEC_ALLOC_OUTPUT_BUFFER,
};

/**
 * @brief Console function for video stream control
 */
static int console_stream_control(int argc, char **argv)
{
    // Check console errors
    if (arg_parse(argc, argv, (void **) &stream_control_args) != 0) {
        arg_print_errors(stderr, stream_control_args.end, argv[0]);
        return 1;
    }

    // Check if an argument was given
    if (!stream_control_args.action->count) {
        ESP_LOGW(TAG, "Console: No argument given");
        return 1;
    }

    // Start argument
    if (strcmp("start", stream_control_args.action->sval[0]) == 0) {
        ESP_LOGI(TAG, "Console: video start");

        app_message_t stream_start_message = {
                .id = APP_UVC_STREAM_START,
        };
        xQueueSend(app_queue, &stream_start_message, portMAX_DELAY);

    // Stop argument
    } else if (strcmp("stop", stream_control_args.action->sval[0]) == 0) {
        ESP_LOGI(TAG, "Console: video stop");

        app_message_t stream_stop_message = {
            .id = APP_UVC_STREAM_STOP,
        };
        xQueueSend(app_queue, &stream_stop_message, portMAX_DELAY);

    // Invalid argument
    } else {
        ESP_LOGW(TAG, "Console: invalid argument: video %s", stream_control_args.action->sval[0]);
    }

    return 0;
}


/**
 * @brief Console function for flash storage control
 */
static int console_storage_control(int argc, char **argv)
{
    // Check console errors
    if (arg_parse(argc, argv, (void **) &storage_control_args) != 0) {
        arg_print_errors(stderr, storage_control_args.end, argv[0]);
        return 1;
    }

    // Check if an argument was given
    if (!storage_control_args.action->count) {
        ESP_LOGW(TAG, "Console: No argument given");
        return 1;
    }

    // Start argument
    if (strcmp("start", storage_control_args.action->sval[0]) == 0) {
        ESP_LOGI(TAG, "Console: storage save start");

        app_message_t storage_start_message = {
                .id = APP_MSC_STORAGE_START_SAVING_FRAMES,
        };
        xQueueSend(app_queue, &storage_start_message, portMAX_DELAY);

    // Stop argument
    } else if (strcmp("stop", storage_control_args.action->sval[0]) == 0) {
        ESP_LOGI(TAG, "Console: storage save stop");

        app_message_t storage_stop_message = {
                .id = APP_MSC_STORAGE_STOP_SAVING_FRAMES,
        };
        xQueueSend(app_queue, &storage_stop_message, portMAX_DELAY);

    // Invalid argument
    } else {
        ESP_LOGW(TAG, "Console: invalid argument: video %s", storage_control_args.action->sval[0]);
    }
    return 0;
}


/**
 * @brief Initialize repl console
 */
static void init_repl_console(void)
{
    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    repl_config.prompt = CONSOLE_PROMPT ">";
    repl_config.max_cmdline_length = 16;

    // Init console based on menuconfig settings
#if defined(CONFIG_ESP_CONSOLE_UART_DEFAULT) || defined(CONFIG_ESP_CONSOLE_UART_CUSTOM)
    esp_console_dev_uart_config_t hw_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_uart(&hw_config, &repl_config, &repl));

    // USJ console can be set only on esp32p4, having separate USB PHYs for USB_OTG and USJ
#elif defined(CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG) && defined(CONFIG_IDF_TARGET_ESP32P4)
    esp_console_dev_usb_serial_jtag_config_t hw_config = ESP_CONSOLE_DEV_USB_SERIAL_JTAG_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_usb_serial_jtag(&hw_config, &repl_config, &repl));

#else
#error Unsupported console type
#endif

    stream_control_args.action = arg_str0(NULL, NULL, "<start|stop>", "Start/Stop video stream");
    stream_control_args.end = arg_end(1);

    const esp_console_cmd_t stream_cmd = {
        .command = "stream",
        .help = "Video stream control",
        .hint = NULL,
        .func = &console_stream_control,
        .argtable = &stream_control_args,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&stream_cmd));         // Register stream control command

    storage_control_args.action = arg_str0(NULL, NULL, "<start|stop>", "Start/Stop saving frames to flash drive");
    storage_control_args.end = arg_end(1);

    const esp_console_cmd_t save_cmd = {
        .command = "storage",
        .help = "Flash storage control",
        .hint = NULL,
        .func = &console_storage_control,
        .argtable = &storage_control_args,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&save_cmd));           // Register storage control command
    ESP_ERROR_CHECK(esp_console_start_repl(repl));                  // Start console task
}


/**
 * @brief Events callback from MSC Class driver
 */
static void msc_event_cb(const msc_host_event_t *event, void *arg)
{
    switch(event->event) {
    case MSC_DEVICE_CONNECTED:
        ESP_LOGI(TAG, "MSC device connected (usb_addr=%d)", event->device.address);

        app_message_t msc_dconn_message = {
            .id = APP_MSC_DEVICE_CONNECTED,
            .data.new_msc_dev_address = event->device.address,
        };
        xQueueSend(app_queue, &msc_dconn_message, portMAX_DELAY);
        break;
    case MSC_DEVICE_DISCONNECTED:
        ESP_LOGI(TAG, "MSC device disconnected");

        app_message_t msc_conn_message = {
            .id = APP_MSC_DEVICE_DISCONNECTED,
        };
        xQueueSend(app_queue, &msc_conn_message, portMAX_DELAY);
        break;
    default:
        ESP_LOGW(TAG, "Unsupported MSC event");
        break;
    }
}


/**
 * @brief Frame callback
 */
bool frame_callback(const uvc_host_frame_t *frame, void *user_ctx)
{
    bool frame_processed = false; // If we return false from this callback, we must return the frame with uvc_host_frame_return(stream_hdl, frame);

    ESP_LOGD(TAG, "Frame callback");
    switch (frame->vs_format.format) {
    case UVC_VS_FORMAT_YUY2: {
        ESP_LOGD(TAG, "YUY2 frame %dx%d", frame->vs_format.h_res, frame->vs_format.v_res);
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


/**
 * @brief Stream callback
 */
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


/**
 * @brief Frame processing task
 */
static void frame_processing_task(void *pvParameters)
{
    uvc_host_frame_t *frame;
    size_t rx_buffer_size;
    uint8_t *rx_buf = (uint8_t*)jpeg_alloc_decoder_mem(FRAME_H_RES * FRAME_V_RES * 3, &rx_mem_cfg, &rx_buffer_size);
    assert(rx_buf);
    uint32_t decoded_size = 0;

    uvc_host_stream_hdl_t *stream_hdl = (uvc_host_stream_hdl_t*)pvParameters;
    assert(stream_hdl);
    assert(jpgd_handle);

    while (1) {
        xQueueReceive(frame_queue, &frame, portMAX_DELAY);
        ESP_LOGD(TAG, "MJPEG frame %dx%d %d bytes", frame->vs_format.h_res, frame->vs_format.v_res, frame->data_len);

        static int frame_i = 0;

        if ((frame_i % DECODE_EVERY_XTH_FRAME) == 0) {

            // Save frame to USB flash drive
            if (usb_devs_status.msc_opened && usb_devs_status.msc_saving_frames) {
                static unsigned int frame_save_count = 0;
                msc_save_jpeg_frame(frame_save_count++, frame->data, frame->data_len);
            }

            frame_i = 0;
            if (ESP_OK == jpeg_decoder_process(jpgd_handle, &decode_cfg, frame->data, frame->data_len, rx_buf, rx_buffer_size, &decoded_size)) {
                ESP_LOGD(TAG, "Decoding OK");
                esp_lcd_panel_draw_bitmap(display_panel, 0, 0, FRAME_H_RES, FRAME_V_RES, (const void *)rx_buf);
            } else {
                ESP_LOGW(TAG, "Decoding failed");
            }
        } else {
            ESP_LOGD(TAG, "Skipping decoding of received MJPEG frame");
        }
        frame_i++;

        uvc_host_frame_return(*stream_hdl, frame);
    }

    // This code should never be reached. Leaving it here for completeness
    free(rx_buf);
    vTaskDelete(NULL);
}


/**
 * @brief Initialize HW JPEG Decoder
 */
static void init_jpeg_decode_engine(void)
{
    jpeg_decode_engine_cfg_t decode_eng_cfg = {
        .intr_priority = 3,
        .timeout_ms = 20,
    };

    ESP_ERROR_CHECK(jpeg_new_decoder_engine(&decode_eng_cfg, &jpgd_handle));
}


/**
 * @brief Initialize Class drivers and USB Host lib tasks
 */
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
        .core_id = tskNO_AFFINITY,
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
    esp_lcd_panel_io_handle_t display_io;
    const bsp_display_config_t config = {};

    bsp_display_new(&config, &display_panel, &display_io);
    bsp_display_backlight_on();

    // Create application message queue and frame queue
    app_queue = xQueueCreate(5, sizeof(app_message_t));
    frame_queue = xQueueCreate(NUMBER_OF_FRAME_BUFFERS, sizeof (uvc_host_frame_t *));
    assert(app_queue || frame_queue);

    // Create USB Host Lib handling task, install drivers
    BaseType_t task_created = xTaskCreate(usb_task, "usb_task", 4 * 1024, xTaskGetCurrentTaskHandle(), 10, NULL);
    assert(task_created == pdTRUE);

    init_repl_console();                        // Init console
    init_jpeg_decode_engine();                  // Init JPEG HW Decoder
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);    // Wait until the drivers are installed

    // Create frame processing task
    uvc_host_stream_hdl_t stream_hdl = NULL;
    task_created = xTaskCreate(frame_processing_task, "frame_task", 4 * 1024, (void*)(&stream_hdl), 2, NULL);
    assert(task_created == pdTRUE);

    // Stream config
    const uvc_host_stream_config_t stream_config = {
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
            .frame_size = 120 * 1024,
    #if CONFIG_SPIRAM
            .frame_heap_caps = MALLOC_CAP_SPIRAM,
    #else
            .frame_heap_caps = 0,
    #endif
            .number_of_urbs = 3,
            .urb_size = 4 * 1024,
        },
    };

    // Init devices status
    usb_devs_status.msc_opened = false;
    usb_devs_status.msc_saving_frames = false;
    usb_devs_status.uvc_opened = false;
    usb_devs_status.uvc_streaming = false;

    TickType_t app_queue_ticks = pdMS_TO_TICKS(500);
    while(1) {
        app_message_t msg;

        if (!usb_devs_status.uvc_opened) {
            ESP_LOGI(TAG, "Opening UVC device 0x%04X:0x%04X\t%dx%d@%2.1fFPS...",
                     stream_config.usb.vid, stream_config.usb.pid, stream_config.vs_format.h_res, stream_config.vs_format.v_res, stream_config.vs_format.fps);
            esp_err_t err = uvc_host_stream_open(&stream_config, pdMS_TO_TICKS(500), &stream_hdl);
            if (ESP_OK != err) {
                //ESP_LOGI(TAG, "Failed to open device");
                app_queue_ticks = pdMS_TO_TICKS(500);
            } else {
                ESP_LOGI(TAG, "UVC device opened, opening stream");
                ESP_ERROR_CHECK(uvc_host_stream_start(stream_hdl));
                ESP_LOGI(TAG, "Stream opened, streaming...");
                app_queue_ticks = portMAX_DELAY;
                usb_devs_status.uvc_opened = true;
                usb_devs_status.uvc_streaming = true;
            }
        }

        if (xQueueReceive(app_queue, &msg, app_queue_ticks)){
            switch (msg.id) {
            case APP_MSC_DEVICE_CONNECTED:
                ESP_LOGI(TAG, "MSC Device connected -> Init MSC Device");

                // Check if MSC device already connected
                APP_QUEUE_CHECK(usb_devs_status.msc_opened, "Another MSC device already connected, the demo can handle only one MSC device at a time");
                msc_init_device(msg.data.new_msc_dev_address);

                // Record MSC device opened
                usb_devs_status.msc_opened = true;
                usb_devs_status.msc_saving_frames = true;
                break;
            case APP_MSC_DEVICE_DISCONNECTED:
                ESP_LOGI(TAG, "MSC Device disconnected -> De-Init MSC Device");
                msc_deinit_device();

                // Record MSC device closed
                usb_devs_status.msc_opened = false;
                usb_devs_status.msc_saving_frames = false;
                break;
            case APP_MSC_STORAGE_START_SAVING_FRAMES:
                ESP_LOGI(TAG, "MSC Device start saving frames to flash storage");

                // MSC device must be opened first
                APP_QUEUE_CHECK(!usb_devs_status.msc_opened, "Can't start saving frames to storage, MSC device not opened");
                // Check if already saving frames
                APP_QUEUE_CHECK(usb_devs_status.msc_saving_frames, "MSC device already saving frames to storage");

                // Record MS device saving frames to storage
                usb_devs_status.msc_saving_frames = true;
                break;

            case APP_MSC_STORAGE_STOP_SAVING_FRAMES:
                ESP_LOGI(TAG, "MSC Device stop saving frames to flash storage");

                // MSC device must be opened first
                APP_QUEUE_CHECK(!usb_devs_status.msc_opened, "Can't stop saving frames to storage, MSC device not opened");
                // Check if already stopped saving frames
                APP_QUEUE_CHECK(!usb_devs_status.msc_saving_frames, "MSC device already stopped saving frames to storage");

                // Record MS device not saving frames to storage
                usb_devs_status.msc_saving_frames = false;
                break;
            case APP_UVC_DEVICE_DISCONNECTED:
                ESP_LOGI(TAG, "UVC Device disconnected -> Close the UVC stream");
                ESP_ERROR_CHECK(uvc_host_stream_close(msg.data.uvc_stream_hdl));

                // Record UVC device closed
                usb_devs_status.uvc_opened = false;
                usb_devs_status.uvc_streaming = false;
                break;
            case APP_UVC_STREAM_START:
                ESP_LOGI(TAG, "UVC Starting stream");

                // UVC device must be opened first
                APP_QUEUE_CHECK(!usb_devs_status.uvc_opened, "Can't start stream, UVC device not opened");
                // Check if UVC Device is already streaming
                APP_QUEUE_CHECK(usb_devs_status.uvc_streaming, "UVC Device already streaming");
                ESP_ERROR_CHECK(uvc_host_stream_start(stream_hdl));

                // Record UVC device streaming
                usb_devs_status.uvc_streaming = true;
                break;
            case APP_UVC_STREAM_STOP:
                ESP_LOGI(TAG, "UVC Stopping stream");

                // UVC device must be opened first
                APP_QUEUE_CHECK(!usb_devs_status.uvc_opened, "Can't stop stream, UVC device not opened");
                // Check if UVC Device is already stopped
                APP_QUEUE_CHECK(!usb_devs_status.uvc_streaming, "UVC Device stream already stopped");
                ESP_ERROR_CHECK(uvc_host_stream_stop(stream_hdl));

                // Record UVC device not streaming
                usb_devs_status.uvc_streaming = false;
                break;
            default:
                break;
            }
        }
    }
}
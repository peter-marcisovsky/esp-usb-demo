/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
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
#include "usb/uvc_host.h"
#include "argtable3/argtable3.h"
#include "display.h"
#include "esp_lcd_panel_ops.h"
#include "driver/ppa.h"
#include "driver/jpeg_decode.h"
#include "esp_private/periph_ctrl.h"


// Configs
#define NUMBER_OF_FRAME_BUFFERS     CONFIG_DEMO_FRAME_BUFFS_COUNT
// VID PID
#define USB_DEVICE_0_VID            CONFIG_DEMO_USB_UVC_DEVICE_0_VID
#define USB_DEVICE_0_PID            CONFIG_DEMO_USB_UVC_DEVICE_0_PID
#define USB_DEVICE_1_VID            CONFIG_DEMO_USB_UVC_DEVICE_1_VID
#define USB_DEVICE_1_PID            CONFIG_DEMO_USB_UVC_DEVICE_1_PID

// Full screen resolution device 0
#define USB_DEVICE_0_H_RES_FULL     CONFIG_DEMO_USB_UVC_DEVICE_0_FRAME_H_RES_FULL
#define USB_DEVICE_0_V_RES_FULL     CONFIG_DEMO_USB_UVC_DEVICE_0_FRAME_V_RES_FULL
#define USB_DEVICE_0_FPS_FULL       CONFIG_DEMO_USB_UVC_DEVICE_0_FPS_FULL
// Full screen resolution device 1
#define USB_DEVICE_1_H_RES_FULL     CONFIG_DEMO_USB_UVC_DEVICE_1_FRAME_H_RES_FULL
#define USB_DEVICE_1_V_RES_FULL     CONFIG_DEMO_USB_UVC_DEVICE_1_FRAME_V_RES_FULL
#define USB_DEVICE_1_FPS_FULL       CONFIG_DEMO_USB_UVC_DEVICE_1_FPS_FULL

// Half screen resolution device 0
#define USB_DEVICE_0_H_RES_HALF     CONFIG_DEMO_USB_UVC_DEVICE_0_FRAME_H_RES_HALF
#define USB_DEVICE_0_V_RES_HALF     CONFIG_DEMO_USB_UVC_DEVICE_0_FRAME_V_RES_HALF
#define USB_DEVICE_0_FPS_HALF       CONFIG_DEMO_USB_UVC_DEVICE_0_FPS_HALF
// Hals screen resolution device 1
#define USB_DEVICE_1_H_RES_HALF     CONFIG_DEMO_USB_UVC_DEVICE_1_FRAME_H_RES_HALF
#define USB_DEVICE_1_V_RES_HALF     CONFIG_DEMO_USB_UVC_DEVICE_1_FRAME_V_RES_HALF
#define USB_DEVICE_1_FPS_HALF       CONFIG_DEMO_USB_UVC_DEVICE_1_FPS_HALF

#define CONSOLE_PROMPT              CONFIG_IDF_TARGET

static const char *TAG = "esp-usb-demo";

#define APP_QUEUE_CHECK(condition, warning_msg, stream_id)      \
        if (!(condition)) {                                     \
            ESP_LOGW(TAG, "Stream %d: %s", stream_id, warning_msg); \
            break;                                              \
        }                                                       \

#define PROCESS_TASK_CHECK(ret_error, operation, core_id)       \
do {                                                            \
    if (ret_error == ESP_OK) {                                  \
        ESP_LOGD(TAG, "%s on core %d OK", operation, core_id);  \
    } else {                                                    \
        ESP_LOGE(TAG, "%s on core %d failed: %s",               \
        operation, core_id, esp_err_to_name(ret_error));        \
        continue;                                               \
    }                                                           \
} while (0)

#define APP_ASSERT_CHECK(condition, warning_msg)                \
    do {                                                        \
        if (!condition) {                                       \
            ESP_LOGE(TAG, warning_msg);                         \
            assert(true);                                       \
        }                                                       \
    } while (0)

// Check if a frame starts with the SOI (start of image) marker 0xd8 and ends with the EOI (end of image) marker 0xd9
#define CHECK_FRAME_COMPLETENESS(frame, complete)                       \
    do {                                                                \
        (complete) = ((frame)->data[0] == 0xff &&                       \
                      (frame)->data[1] == 0xd8 &&                       \
                      (frame)->data[(frame)->data_len - 2] == 0xff &&   \
                      (frame)->data[(frame)->data_len - 1] == 0xd9);    \
    } while (0)

#define ALIGN_UP(num, align)    (((num) + ((align) - 1)) & ~((align) - 1))

/**
 * @brief Video streams
 */
typedef enum {
    STREAM_ID_0 = 0,                            // Video stream being processed on core 0
    STREAM_ID_1,                                // Video stream being processed on core 1
} stream_id_t;

/**
 * @brief Camera resolutions
 */
typedef enum {
    RESOLUTION_FULL = 0,                        // Full resolution: One video stream from one camera drawn to the whole display
    RESOLUTION_HALF,                            // Half resolution: Two video streams drawn to the display together
} camera_resolution_t;

/**
 * @brief Application Queue and its messages ID
 */
typedef struct {
    enum {
        APP_UVC_DEVICE_CONNECTED,               // USB UVC device connect event
        APP_UVC_DEVICE_DISCONNECTED,            // USB UVC device disconnect event
        APP_UVC_STREAM_START,                   // Start UVC stream
        APP_UVC_STREAM_STOP,                    // Stop UVC stream
    } id;
    union {
        uvc_host_stream_hdl_t uvc_stream_hdl;   // UVC Stream handle
        stream_id_t stream_id;                  // Stream ID (Which Core is a stream running on)
    } data;
} app_message_t;

typedef struct {
    struct {
        QueueHandle_t frames_queue;                 // Frames Queue
        SemaphoreHandle_t mux_lock;                 // Mutex lock
    } constant;                                     // Constant variables
    struct {
        uvc_host_stream_config_t *stream_config;    // UVC Stream configuration
        bool dev_opened;                            // Device opened status
        bool dev_streaming;                         // Device streaming status
        bool dev_res_full;                          // Device resolution status
    } single_thread;
    struct {
        uvc_host_stream_hdl_t stream_hdl;           // Video stream handle
        uint8_t *jpeg_decoder_out_buf;              // Output frame buffer for JPEG Converter
        size_t jpeg_decoder_out_buf_size;           // JPEG Converter frame buffer size
        uint8_t *ppa_srm_out_buf;                   // Output frame buffer for PPA SRM
        ppa_srm_oper_config_t ppa_srm_oper_config;  // PPA SRM configuration struct
        camera_resolution_t camera_resolution;      // Current camera resolution
    } mux_protected;                                // Mutex protected variables
} uvc_dev_obj_t;

/**
 * @brief repl console arguments for video control
 */
typedef struct {
    struct arg_int *stream_start;
    struct arg_int *stream_stop;
    struct arg_end *end;
} control_stream_args_t;
static control_stream_args_t stream_control_args;

static QueueHandle_t app_queue = NULL;                  // Application Queue
static esp_lcd_panel_handle_t display_panel = NULL;     // Display panel handle
static jpeg_decoder_handle_t jpgd_handle = NULL;        // JPEG Decoder handle
static ppa_client_handle_t ppa_client_handle;           // PPA engine handle

// Config structure for JPEG decoder
static jpeg_decode_cfg_t decode_cfg = {
    .output_format = JPEG_DECODE_OUT_FORMAT_RGB565,
    .conv_std = JPEG_YUV_RGB_CONV_STD_BT709,
    .rgb_order = JPEG_DEC_RGB_ELEMENT_ORDER_BGR,
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
    if (stream_control_args.stream_start->count == 0 && stream_control_args.stream_stop->count == 0) {
        ESP_LOGW(TAG, "Console: No argument given");
        return 1;
    }

    // Start argument
    if (stream_control_args.stream_start->count) {
        ESP_LOGI(TAG, "Console: stream start");

        const int stream_id = stream_control_args.stream_start->ival[0];
        switch (stream_id) {
        case 0:
        case 1:
            const app_message_t stream_start_message = {
                .id = APP_UVC_STREAM_START,
                .data.stream_id = (stream_id ? STREAM_ID_1 : STREAM_ID_0),
            };
            xQueueSend(app_queue, &stream_start_message, portMAX_DELAY);
            break;
        default:
            ESP_LOGE(TAG, "Console: argument %d out of range of <0-1>", stream_id);
            break;
        }
    }

    // Stop argument
    if (stream_control_args.stream_stop->count) {
        ESP_LOGI(TAG, "Console: stream stop");

        const int stream_id = stream_control_args.stream_stop->ival[0];
        switch (stream_id) {
        case 0:
        case 1:
            const app_message_t stream_stop_message = {
                .id = APP_UVC_STREAM_STOP,
                .data.stream_id = (stream_id ? STREAM_ID_1 : STREAM_ID_0),
            };
            xQueueSend(app_queue, &stream_stop_message, portMAX_DELAY);
            break;
        default:
            ESP_LOGE(TAG, "Console: argument %d out of range of <0-1>", stream_id);
            break;
        }
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

    stream_control_args.stream_start = arg_int0("S", "start", "<stream_id>", "Start video stream");
    stream_control_args.stream_stop = arg_int0("T", "stop", "<stream_id>", "Stop video stream");
    stream_control_args.end = arg_end(1);

    const esp_console_cmd_t stream_cmd = {
        .command = "stream",
        .help = "Video stream control",
        .hint = NULL,
        .func = &console_stream_control,
        .argtable = &stream_control_args,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&stream_cmd));         // Register stream control command
    ESP_ERROR_CHECK(esp_console_start_repl(repl));                  // Start console task
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

        // Check if current frame is complete - if yes send it to queue
        bool frame_complete = false;
        CHECK_FRAME_COMPLETENESS(frame, frame_complete);
        if (!frame_complete) {
            ESP_LOGW(TAG, "Frame not complete, skipping");
            frame_processed = true;
            break;
        }

        ESP_LOGD(TAG, "frame_cb %dx%d %d bytes", frame->vs_format.h_res, frame->vs_format.v_res, frame->data_len);
        // Attempt to put the new frame into the queue without checking if it is full.
        // If successful, we save processing time since we avoid the overhead of a full check for every frame.
        QueueHandle_t frame_q = *((QueueHandle_t *)user_ctx);
        BaseType_t frame_put_to_queue = xQueueSendToBack(frame_q, &frame, 0);

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
    // Freertos task parameters
    uvc_dev_obj_t *task_ctx = (uvc_dev_obj_t *)pvParameters;

    // Get core ID to differentiate a display location to draw a bitmap based on Core ID
    // Separate task for each USB Camera running on a separate core
    const bool core_id = (bool)xTaskGetCoreID(NULL);
    const int x_display_offset = ((core_id == 0) ? 0 : 512);

    APP_ASSERT_CHECK(jpgd_handle, "JPEG Decode handle is NULL");
    APP_ASSERT_CHECK(ppa_client_handle, "PPA Handle is NULL");

    // Constant context parameters
    SemaphoreHandle_t mux_lock = task_ctx->constant.mux_lock;
    QueueHandle_t frame_q = task_ctx->constant.frames_queue;
    APP_ASSERT_CHECK(mux_lock, "Resolution change semaphore NULL");
    APP_ASSERT_CHECK(frame_q, "Frame queue NULL");

    uvc_host_frame_t *frame;                                                    // Frame taken from the frame queue
    uint32_t jpeg_decoded_size = 0;                                             // Size of JPEG-decoded image
    int frame_i = 0;                                                            // Frame count
    esp_err_t ret;

    while (1) {
        xQueueReceive(frame_q, &frame, portMAX_DELAY);
        frame_i++;

        ESP_LOGD(TAG, "MJPEG frame %d, %dx%d %d bytes", frame_i, frame->vs_format.h_res, frame->vs_format.v_res, frame->data_len);
        ret = jpeg_decoder_process(jpgd_handle, &decode_cfg, frame->data, frame->data_len, task_ctx->mux_protected.jpeg_decoder_out_buf, task_ctx->mux_protected.jpeg_decoder_out_buf_size, &jpeg_decoded_size);

        // UVC Frame is no longer needed, we can return it
        uvc_host_frame_return(task_ctx->mux_protected.stream_hdl, frame);
        PROCESS_TASK_CHECK(ret, "JPEG Decoding", core_id);

        if (task_ctx->mux_protected.camera_resolution == RESOLUTION_HALF) {
            // Rotate the bitmap in 90 degrees, to be able to display 2 video streams on display on top of each other
            ret = ppa_do_scale_rotate_mirror(ppa_client_handle, &task_ctx->mux_protected.ppa_srm_oper_config);
            PROCESS_TASK_CHECK(ret, "PPA rotate", core_id);

            // Draw bitmap to the display
            ret = esp_lcd_panel_draw_bitmap(display_panel, x_display_offset, 0, x_display_offset + frame->vs_format.v_res, frame->vs_format.h_res, (const void *)task_ctx->mux_protected.ppa_srm_out_buf);
            PROCESS_TASK_CHECK(ret, "Draw bitmap", core_id);
        } else {
            // Skip rotation

            // Draw bitmap to the display
            ret = esp_lcd_panel_draw_bitmap(display_panel, 0, 0, frame->vs_format.h_res, frame->vs_format.v_res, (const void *)task_ctx->mux_protected.jpeg_decoder_out_buf);
            PROCESS_TASK_CHECK(ret, "Draw bitmap", core_id);
        }
    }
}


/**
 * @brief Initialize HW JPEG Decoder
 */
static void init_jpeg_decode_engine(void)
{
    const jpeg_decode_engine_cfg_t decode_eng_cfg = {
        .intr_priority = 0,
        .timeout_ms = 30,
    };

    ESP_ERROR_CHECK(jpeg_new_decoder_engine(&decode_eng_cfg, &jpgd_handle));
}


/**
 * @brief Initialize HW PPA engine
 */
static void init_ppa_engine(void)
{
    const ppa_client_config_t ppa_client_config = {
        .oper_type = PPA_OPERATION_SRM,         // SRM operation (Scale Rotation Mirror)
        .max_pending_trans_num = 5,             // Processing queue
    };

    ESP_ERROR_CHECK(ppa_register_client(&ppa_client_config, &ppa_client_handle));
}


/**
 * @brief Events callback from UVC Class driver
 */
static void uvc_driver_event_cb(const uvc_host_driver_event_data_t *event, void *user_ctx)
{
    switch (event->type) {
    case UVC_HOST_DRIVER_EVENT_DEVICE_CONNECTED: {
        app_message_t uvc_conn_message = {
            .id = APP_UVC_DEVICE_CONNECTED,
        };
        xQueueSend(app_queue, &uvc_conn_message, portMAX_DELAY);
        break;
    }
    default:
        break;
    }
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

    ESP_LOGI(TAG, "Installing UVC driver");
    const uvc_host_driver_config_t uvc_driver_config = {
        .driver_task_stack_size = 6 * 1024,
        .driver_task_priority = 6,
        .xCoreID = tskNO_AFFINITY,
        .create_background_task = true,
        .event_cb = uvc_driver_event_cb,
    };
    ESP_ERROR_CHECK(uvc_host_install(&uvc_driver_config));

    // Notify the app_main task, that the usb_host and uvc_host have been installed
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


/**
 * @brief Initialize PPA SMR (Scale Rotate Mirror) HW peripheral
 * @note PPA is used for image rotation
 *
 * @param ppa_srm_oper_config: pointer to PPA SRM configuration
 * @param stream_config pointer to the current stream configuration
 * @param in_buf pointer to PPA SRM input buffer (the JPEG Decoder output buffer)
 * @param out_buf pointer to PPA SRM output buffer (the LCD panel draw buffer)
 */
static void ppa_srm_config(ppa_srm_oper_config_t *ppa_srm_oper_config, const uvc_host_stream_config_t *stream_config, uint8_t *in_buf, uint8_t *out_buf)
{
    const unsigned int frame_h_res = stream_config->vs_format.h_res;
    const unsigned int frame_v_res = stream_config->vs_format.v_res;

    const ppa_srm_color_mode_t in_cm = PPA_SRM_COLOR_MODE_RGB565;
    const ppa_srm_color_mode_t out_cm = PPA_SRM_COLOR_MODE_RGB565;
    const ppa_srm_rotation_angle_t rotation = PPA_SRM_ROTATION_ANGLE_90;
    const float scale = 1.0;

    const color_space_pixel_format_t out_pixel_format = {
        .color_type_id = out_cm,
    };
    const uint32_t out_buf_size = ALIGN_UP(frame_h_res * frame_v_res * color_hal_pixel_format_get_bit_depth(out_pixel_format) / 8, 64);

    const ppa_srm_oper_config_t oper_config = {
        .in.buffer = in_buf,
        .in.pic_w = frame_h_res,
        .in.pic_h = frame_v_res,
        .in.block_w = frame_h_res,
        .in.block_h = frame_v_res,
        .in.block_offset_x = 0,
        .in.block_offset_y = 0,
        .in.srm_cm = in_cm,

        .out.buffer = out_buf,
        .out.buffer_size = out_buf_size,
        .out.pic_w = frame_v_res,
        .out.pic_h = frame_h_res,
        .out.block_offset_x = 0,
        .out.block_offset_y = 0,
        .out.srm_cm = in_cm,

        .rotation_angle = rotation,
        .scale_x = scale,
        .scale_y = scale,

        .mirror_x = false,
        .mirror_y = true,

        .rgb_swap = 0,
        .byte_swap = 0,

        .mode = PPA_TRANS_MODE_BLOCKING,
    };

    *ppa_srm_oper_config = oper_config;
}


/**
 * @brief Initialize HW peripherals for image processing (JPEG decoder and PPA SRM)
 *
 * @param uvc_dev_obj: pointer to current processing task context
 * @param stream_config pointer to the current stream configuration
 * @param requested_resolution camera resolution to be used
 */
static void image_processing_config(uvc_dev_obj_t *uvc_dev_obj, camera_resolution_t requested_resolution)
{
    jpeg_decode_memory_alloc_cfg_t rx_mem_cfg = {
        .buffer_direction = JPEG_DEC_ALLOC_OUTPUT_BUFFER,
    };

    // Free previously allocated memories
    if (uvc_dev_obj->mux_protected.jpeg_decoder_out_buf) {
        free(uvc_dev_obj->mux_protected.jpeg_decoder_out_buf);
    }

    if (uvc_dev_obj->mux_protected.ppa_srm_out_buf) {
        free(uvc_dev_obj->mux_protected.ppa_srm_out_buf);
    }

    // JPEG HW Decoder image buffer
    size_t jpeg_decoder_out_buf_size;
    uint8_t *jpeg_decoder_out_buf = (uint8_t *)jpeg_alloc_decoder_mem(uvc_dev_obj->single_thread.stream_config->vs_format.h_res * uvc_dev_obj->single_thread.stream_config->vs_format.v_res * 3, &rx_mem_cfg, &jpeg_decoder_out_buf_size);
    APP_ASSERT_CHECK(jpeg_decoder_out_buf, "Failed to create JPEG Decoder image buffer");

    // PPA image buffer
    uint8_t *ppa_srm_out_buf = NULL;

    // We only need PPA for Half resolution
    if (requested_resolution == RESOLUTION_HALF) {
        ppa_srm_out_buf = (uint8_t *)heap_caps_aligned_alloc(4, uvc_dev_obj->single_thread.stream_config->vs_format.h_res * uvc_dev_obj->single_thread.stream_config->vs_format.v_res * 2, MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA);
        APP_ASSERT_CHECK(ppa_srm_out_buf, "Failed to create PPA Image buffer");
        ppa_srm_config(&uvc_dev_obj->mux_protected.ppa_srm_oper_config, uvc_dev_obj->single_thread.stream_config, jpeg_decoder_out_buf, ppa_srm_out_buf);
    } else {
        ppa_srm_oper_config_t oper_config = {};
        uvc_dev_obj->mux_protected.ppa_srm_oper_config = oper_config;
    }

    uvc_dev_obj->mux_protected.jpeg_decoder_out_buf = jpeg_decoder_out_buf;
    uvc_dev_obj->mux_protected.jpeg_decoder_out_buf_size = jpeg_decoder_out_buf_size;
    uvc_dev_obj->mux_protected.ppa_srm_out_buf = ppa_srm_out_buf;
    uvc_dev_obj->mux_protected.camera_resolution = requested_resolution;
    ESP_LOGI(TAG, "Setting resolution to %s", ((requested_resolution == RESOLUTION_FULL) ? ("full") : ("half")));
}


/**
 * @brief Update resolution of the current stream config from Full screen to Half, or other way around
 *
 * @param stream_config: pointer to the current stream configuration
 * @param requested_resolution camera resolution to be used
 * @param stream_id stream ID
 */
static void update_resolution(uvc_host_stream_config_t *stream_config, camera_resolution_t requested_resolution, stream_id_t stream_id)
{
    // Update frame resolution
    if (requested_resolution == RESOLUTION_HALF) {
        if (stream_id == STREAM_ID_0) {
            // Device: 0, Resolution: Half screen
            const uvc_host_stream_format_t new_format = {
                .h_res = USB_DEVICE_0_H_RES_HALF,
                .v_res = USB_DEVICE_0_V_RES_HALF,
                .fps = USB_DEVICE_0_FPS_HALF,
                .format = UVC_VS_FORMAT_MJPEG,
            };
            stream_config->vs_format = new_format;
        } else {
            // Device: 1, Resolution: Half screen
            const uvc_host_stream_format_t new_format = {
                .h_res = USB_DEVICE_1_H_RES_HALF,
                .v_res = USB_DEVICE_1_V_RES_HALF,
                .fps = USB_DEVICE_1_FPS_HALF,
                .format = UVC_VS_FORMAT_MJPEG,
            };
            stream_config->vs_format = new_format;
        }
    } else {
        if (stream_id == STREAM_ID_0) {
            // Device: 0, Resolution: Full screen
            const uvc_host_stream_format_t new_format = {
                .h_res = USB_DEVICE_0_H_RES_FULL,
                .v_res = USB_DEVICE_0_V_RES_FULL,
                .fps = USB_DEVICE_0_FPS_FULL,
                .format = UVC_VS_FORMAT_MJPEG,
            };
            stream_config->vs_format = new_format;
        } else {
            // Device: 1, Resolution: Full screen
            const uvc_host_stream_format_t new_format = {
                .h_res = USB_DEVICE_1_H_RES_FULL,
                .v_res = USB_DEVICE_1_V_RES_FULL,
                .fps = USB_DEVICE_1_FPS_FULL,
                .format = UVC_VS_FORMAT_MJPEG,
            };
            stream_config->vs_format = new_format;
        }
    }
}


/**
 * @brief Reopen video stream, when changing from Full screen resolution to half, or other way around
 *
 * @param uvc_dev_obj: pointer to current processing task context
 * @param requested_resolution camera resolution to be used
 * @param stream_id stream ID
 */
static void reopen_stream(uvc_dev_obj_t *uvc_dev_obj, camera_resolution_t requested_resolution, stream_id_t stream_id)
{
    // Stop and close the stream of the opened device
    ESP_ERROR_CHECK(uvc_host_stream_stop(uvc_dev_obj[stream_id].mux_protected.stream_hdl));

    // Wait for the frame queue to empty
    vTaskDelay(pdMS_TO_TICKS(50));
    ESP_ERROR_CHECK(uvc_host_stream_close(uvc_dev_obj[stream_id].mux_protected.stream_hdl));

    update_resolution(uvc_dev_obj[stream_id].single_thread.stream_config, requested_resolution, stream_id);

    // Reopen the device with new resolution
    ESP_LOGI(TAG, "Opening UVC device 0x%04X:0x%04X\t%dx%d@%2.1fFPS...",
             uvc_dev_obj[stream_id].single_thread.stream_config->usb.vid,
             uvc_dev_obj[stream_id].single_thread.stream_config->usb.pid,
             uvc_dev_obj[stream_id].single_thread.stream_config->vs_format.h_res,
             uvc_dev_obj[stream_id].single_thread.stream_config->vs_format.v_res,
             uvc_dev_obj[stream_id].single_thread.stream_config->vs_format.fps);
    ESP_ERROR_CHECK(uvc_host_stream_open(uvc_dev_obj[stream_id].single_thread.stream_config, pdMS_TO_TICKS(200), &uvc_dev_obj[stream_id].mux_protected.stream_hdl));

    // Clear the display (fill it with black pixels)
    bsp_display_clear(display_panel);

    // Setup JPEG and PPA
    image_processing_config(&uvc_dev_obj[stream_id], requested_resolution);

    ESP_ERROR_CHECK(uvc_host_stream_start(uvc_dev_obj[stream_id].mux_protected.stream_hdl));
    ESP_LOGI(TAG, "Stream opened, streaming...");
}


/**
 * @brief Open UVC device based on UVC driver callback
 *
 * @param uvc_dev_obj: pointer to current processing task context
 * @param stream_id stream ID
 *
 * @return
 *    - ESP_FAIL: Requested UVC device is not found
 *    - ESP_OK: UVC Device successfully connected
 */
static esp_err_t handle_device_connection(uvc_dev_obj_t *uvc_dev_obj, stream_id_t stream_id)
{
    const stream_id_t this_stream = stream_id;
    const stream_id_t other_stream = ((this_stream == STREAM_ID_0) ? STREAM_ID_1 : STREAM_ID_0);
    camera_resolution_t this_stream_target_resolution = RESOLUTION_HALF;
    bool this_stream_resolution_full = false;

    // Check if the other stream is opened, before opening this stream
    if (!uvc_dev_obj[other_stream].single_thread.dev_opened) {
        // The other stream is not opened, open this stream in the Full resolution
        update_resolution(uvc_dev_obj[this_stream].single_thread.stream_config, RESOLUTION_FULL, this_stream);
        this_stream_target_resolution = RESOLUTION_FULL;
        this_stream_resolution_full = true;
    }

    ESP_LOGI(TAG, "Opening UVC device 0x%04X:0x%04X\t%dx%d@%2.1fFPS...",
             uvc_dev_obj[this_stream].single_thread.stream_config->usb.vid,
             uvc_dev_obj[this_stream].single_thread.stream_config->usb.pid,
             uvc_dev_obj[this_stream].single_thread.stream_config->vs_format.h_res,
             uvc_dev_obj[this_stream].single_thread.stream_config->vs_format.v_res,
             uvc_dev_obj[this_stream].single_thread.stream_config->vs_format.fps);

    esp_err_t err = uvc_host_stream_open(uvc_dev_obj[this_stream].single_thread.stream_config, pdMS_TO_TICKS(200), &uvc_dev_obj[this_stream].mux_protected.stream_hdl);
    if (ESP_ERR_NOT_FOUND == err) {
        // Change the resolution back to Half
        update_resolution(uvc_dev_obj[this_stream].single_thread.stream_config, RESOLUTION_HALF, this_stream);

        ESP_LOGD(TAG, "Device not found");
        return ESP_FAIL;
    } else if (ESP_OK != err) {
        // Change the resolution back to Half
        update_resolution(uvc_dev_obj[this_stream].single_thread.stream_config, RESOLUTION_HALF, this_stream);

        ESP_LOGE(TAG, "Failed to open device");
        return ESP_FAIL;
    } else {
        // This device stream is opened, check if the other device stream is opened as well and set to full screen
        if (uvc_dev_obj[other_stream].single_thread.dev_opened && uvc_dev_obj[other_stream].single_thread.dev_res_full) {
            // The other device stream is streaming in full screen, reopen the stream to half screen
            reopen_stream(uvc_dev_obj, RESOLUTION_HALF, other_stream);
            // Record that the other device has resolution set to half screen
            uvc_dev_obj[other_stream].single_thread.dev_res_full = false;
        }

        ESP_LOGI(TAG, "UVC device opened, opening stream");
        image_processing_config(&uvc_dev_obj[this_stream], this_stream_target_resolution);
        ESP_ERROR_CHECK(uvc_host_stream_start(uvc_dev_obj[this_stream].mux_protected.stream_hdl));
        ESP_LOGI(TAG, "Stream opened, streaming...");

        // Record that this video stream is opened, video is streaming and resolution is set
        uvc_dev_obj[this_stream].single_thread.dev_opened = true;
        uvc_dev_obj[this_stream].single_thread.dev_streaming = true;
        uvc_dev_obj[this_stream].single_thread.dev_res_full = this_stream_resolution_full;

        return ESP_OK;
    }
}


void app_main(void)
{
    // Init BSP
    esp_lcd_panel_io_handle_t display_io;
    const bsp_display_config_t config = {};

    ESP_ERROR_CHECK(bsp_display_new(&config, &display_panel, &display_io));
    APP_ASSERT_CHECK(display_panel, "Display panel handle NULL");
    ESP_ERROR_CHECK(bsp_display_backlight_on());

    // Create application queue
    app_queue = xQueueCreate(5, sizeof(app_message_t));
    APP_ASSERT_CHECK(app_queue, "Failed to create App queue");

    // Create USB Host Lib handling task, install drivers
    BaseType_t task_created;
    task_created = xTaskCreate(usb_task, "usb_task", 4 * 1024, xTaskGetCurrentTaskHandle(), 10, NULL);
    APP_ASSERT_CHECK(task_created, "Failed to create usb_task");

    init_repl_console();                        // Init console
    init_jpeg_decode_engine();                  // Init JPEG HW Decoder
    init_ppa_engine();                          // Init PPA engine
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);    // Wait until the USB Host drivers are installed

    uvc_dev_obj_t uvc_dev_obj[2];
    for (int i = 0; i < 2; i++) {
        SemaphoreHandle_t mux_lock = xSemaphoreCreateMutex();
        QueueHandle_t frames_queue = xQueueCreate(NUMBER_OF_FRAME_BUFFERS, sizeof(uvc_host_frame_t *));
        APP_ASSERT_CHECK(mux_lock && frames_queue, "Failed to create Freertos primitives");

        const uvc_dev_obj_t dev_obj = {
            .constant.frames_queue = frames_queue,
            .constant.mux_lock = mux_lock,
            .single_thread.dev_opened = false,
            .single_thread.dev_streaming = false,
            .single_thread.dev_res_full = false,
        };

        uvc_dev_obj[i] = dev_obj;
    }

    // Create stream config for both devices
    uvc_host_stream_config_t stream_config[2] = {
        {
            .event_cb = stream_callback,
            .frame_cb = frame_callback,
            .user_ctx = &(uvc_dev_obj[0].constant.frames_queue),
            //.user_ctx = NULL,
            .usb = {
                .vid = USB_DEVICE_0_VID,
                .pid = USB_DEVICE_0_PID,
                .uvc_stream_index = 0,
            },
            .vs_format = {
                .h_res = USB_DEVICE_0_H_RES_HALF,
                .v_res = USB_DEVICE_0_V_RES_HALF,
                .fps = USB_DEVICE_0_FPS_HALF,
                .format = UVC_VS_FORMAT_MJPEG,
            },
            .advanced = {
                .number_of_frame_buffers = NUMBER_OF_FRAME_BUFFERS,
                .frame_size = 250 * 1024,
                .frame_heap_caps = MALLOC_CAP_SPIRAM,
                .number_of_urbs = 3,
                .urb_size = 4 * 1024,
            },
        },
        {
            .event_cb = stream_callback,
            .frame_cb = frame_callback,
            .user_ctx = &(uvc_dev_obj[1].constant.frames_queue),
            //.user_ctx = 0,
            .usb = {
                .vid = USB_DEVICE_1_VID,
                .pid = USB_DEVICE_1_PID,
                .uvc_stream_index = 0,
            },
            .vs_format = {
                .h_res = USB_DEVICE_1_H_RES_HALF,
                .v_res = USB_DEVICE_1_V_RES_HALF,
                .fps = USB_DEVICE_1_FPS_HALF,
                .format = UVC_VS_FORMAT_MJPEG,
            },
            .advanced = {
                .number_of_frame_buffers = NUMBER_OF_FRAME_BUFFERS,
                .frame_size = 250 * 1024,
                .frame_heap_caps = MALLOC_CAP_SPIRAM,
                .number_of_urbs = 3,
                .urb_size = 4 * 1024,
            },
        },
    };

    uvc_dev_obj[0].single_thread.stream_config = &stream_config[0];
    uvc_dev_obj[1].single_thread.stream_config = &stream_config[1];

    task_created = xTaskCreatePinnedToCore(frame_processing_task, "frame_task_core0", 4 * 1024, (void *)(&uvc_dev_obj[0]), 3, NULL, 0);
    APP_ASSERT_CHECK(task_created, "Failed to create frame_task on core 0");
    task_created = xTaskCreatePinnedToCore(frame_processing_task, "frame_task_core1", 4 * 1024, (void *)(&uvc_dev_obj[1]), 2, NULL, 1);
    APP_ASSERT_CHECK(task_created, "Failed to create frame_task on core 1");

    while (1) {
        app_message_t msg;

        if (xQueueReceive(app_queue, &msg, portMAX_DELAY)) {
            switch (msg.id) {
            case APP_UVC_DEVICE_DISCONNECTED:
                ESP_LOGI(TAG, "UVC Device disconnected -> Close the UVC stream");
                ESP_ERROR_CHECK(uvc_host_stream_close(msg.data.uvc_stream_hdl));

                const stream_id_t stream_to_be_closed = (msg.data.uvc_stream_hdl == uvc_dev_obj[STREAM_ID_0].mux_protected.stream_hdl) ? (STREAM_ID_0) : (STREAM_ID_1);
                const stream_id_t other_stream = (stream_to_be_closed == STREAM_ID_0 ? STREAM_ID_1 : STREAM_ID_0);

                uvc_dev_obj[stream_to_be_closed].single_thread.dev_opened = false;
                uvc_dev_obj[stream_to_be_closed].single_thread.dev_streaming = false;
                uvc_dev_obj[stream_to_be_closed].single_thread.dev_res_full = false;

                // Check if only one UVC Device is opened and if both devices have the same resolution set
                if (uvc_dev_obj[STREAM_ID_0].single_thread.dev_opened != uvc_dev_obj[STREAM_ID_1].single_thread.dev_opened &&
                        uvc_dev_obj[STREAM_ID_0].single_thread.dev_res_full == uvc_dev_obj[STREAM_ID_1].single_thread.dev_res_full) {

                    // Check if queue is empty
                    if (pdTRUE == xQueuePeek(app_queue, &msg, 0)) {
                        if (msg.id == APP_UVC_DEVICE_DISCONNECTED) {
                            // There is one more UVC Device Disconnected message in the queue
                            // The external hub was disconnected, we don't want to reopen stream with full resolution
                            ESP_LOGD(TAG, "App queue not empty");
                            break;
                        }
                    }
                    // Switch from Half screen resolution to Full screen resolution, while only one camera is connected
                    ESP_LOGI(TAG, "Reopening video stream of device %d from HALF to FULL screen", other_stream);
                    reopen_stream(uvc_dev_obj, RESOLUTION_FULL, other_stream);
                    // Record that the resolution will be updated to full screen
                    uvc_dev_obj[other_stream].single_thread.dev_res_full = true;
                }

                break;
            case APP_UVC_DEVICE_CONNECTED:
                ESP_LOGI(TAG, "UVC Device connected -> Open the UVC stream");
                if (!uvc_dev_obj[STREAM_ID_0].single_thread.dev_opened) {
                    if (ESP_OK == handle_device_connection(uvc_dev_obj, STREAM_ID_0)) {
                        // Device 0 connected
                        break;
                    }
                }

                if (!uvc_dev_obj[STREAM_ID_1].single_thread.dev_opened) {
                    if (ESP_OK == handle_device_connection(uvc_dev_obj, STREAM_ID_1)) {
                        // Device 1 connected
                        break;
                    }
                }

                ESP_LOGE(TAG, "No UVC Device was connected");
                break;
            case APP_UVC_STREAM_START: {
                const stream_id_t stream_id = msg.data.stream_id;
                ESP_LOGD(TAG, "Starting stream %d", stream_id);

                // UVC device must be opened first
                APP_QUEUE_CHECK(uvc_dev_obj[stream_id].single_thread.dev_opened, "can't start, UVC device not opened", stream_id);
                // Check if UVC Device is already streaming
                APP_QUEUE_CHECK(!uvc_dev_obj[stream_id].single_thread.dev_streaming, "already streaming", stream_id);
                ESP_ERROR_CHECK(uvc_host_stream_start(uvc_dev_obj[stream_id].mux_protected.stream_hdl));

                // Record UVC device streaming
                uvc_dev_obj[stream_id].single_thread.dev_streaming = true;
                ESP_LOGI(TAG, "Stream %d started", stream_id);
            }
            break;
            case APP_UVC_STREAM_STOP:
                const stream_id_t stream_id = msg.data.stream_id;
                ESP_LOGD(TAG, "Stopping stream %d", stream_id);

                // UVC device must be opened first
                APP_QUEUE_CHECK(uvc_dev_obj[stream_id].single_thread.dev_opened, "can't stop, UVC device not opened", stream_id);
                // Check if UVC Device is already stopped
                APP_QUEUE_CHECK(uvc_dev_obj[stream_id].single_thread.dev_streaming, "already stopped", stream_id);
                ESP_ERROR_CHECK(uvc_host_stream_stop(uvc_dev_obj[stream_id].mux_protected.stream_hdl));

                // Record UVC device not streaming
                uvc_dev_obj[stream_id].single_thread.dev_streaming = false;
                ESP_LOGI(TAG, "Stream %d stopped", stream_id);
                break;
            default:
                break;
            }
        }
    }
}

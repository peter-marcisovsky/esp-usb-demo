idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS ""
                    PRIV_REQUIRES usb fatfs esp_driver_gpio esp_timer esp_psram
                    )
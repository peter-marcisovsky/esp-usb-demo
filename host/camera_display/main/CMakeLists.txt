idf_component_register(SRCS "main.c" "usb_storage.c"
                       INCLUDE_DIRS "include"
                       PRIV_REQUIRES usb fatfs esp_driver_gpio esp_timer esp_psram
                       )
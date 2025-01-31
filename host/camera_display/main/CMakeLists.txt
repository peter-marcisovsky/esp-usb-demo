idf_component_register(SRCS "main.c" "usb_storage.c" "ek97009_init.c"
                       INCLUDE_DIRS "include"
                       PRIV_REQUIRES usb fatfs esp_driver_gpio esp_timer esp_psram console esp_lcd esp_driver_jpeg
                       )
idf_component_register(
    SRCS "probe_main.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES driver esp_driver_usb_serial_jtag esp_driver_tsens esp_driver_spi esp_driver_uart esp_driver_ledc
                  esp_psram esp_wifi nvs_flash esp_netif esp_event esp_timer efuse spi_flash esp_app_format esp_eth lwip
)

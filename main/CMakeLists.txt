set(srcs "avb_endpoint.c")

# The P4 boards' flash needs its own chip driver for auto-suspend.
if(CONFIG_SPI_FLASH_OVERRIDE_CHIP_DRIVER_LIST)
    list(APPEND srcs "flash_chips.c")
endif()

# Status OLED is optional hardware; compile it only when selected.
if(CONFIG_EXAMPLE_AVB_OLED_DISPLAY)
    list(APPEND srcs "avb_endpoint_oled.c")
endif()

# Perf-test mode adds a separate translation unit that runs in place of
# the production bring-up. Gated on the WiFi-capable target since the
# RX hook only makes sense on the STA build.
if(CONFIG_AVB_ENDPOINT_PERF_TEST_MODE AND IDF_TARGET STREQUAL "esp32c6")
    list(APPEND srcs "avb_endpoint_perf_test.c")
endif()

# Per-target dep gating. The per-medium blocks inside avb_endpoint.c
# self-gate on CONFIG_ESP_PTP_PORT*_MEDIUM_* via #ifdef, but the
# component graph still needs to know which managed/built-in components
# to pull in. esp_wifi only exists on Wi-Fi-capable targets.
#
# For the p4 wired build we deliberately don't pass PRIV_REQUIRES so
# IDF's legacy #include-scanning auto-resolves esp_eth, esp_netif,
# esp_avb, etc. (the historical behavior of this example).
if(IDF_TARGET STREQUAL "esp32c6")
    idf_component_register(SRCS ${srcs}
                           PRIV_REQUIRES esp_wifi esp_netif esp_event
                                         nvs_flash esp_timer esp_avb esp_ptp
                                         esp_driver_usb_serial_jtag
                                         esp_driver_uart)
else()
    idf_component_register(SRCS ${srcs} LDFRAGMENTS linker.lf)
endif()

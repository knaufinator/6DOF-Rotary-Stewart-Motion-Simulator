# Main component for Stewart Platform ESP-IDF project

idf_component_register(
    SRCS 
        "main.cpp"
        "helpers.cpp"
        "InverseKinematics.cpp"
        "AxisScaling.cpp"
        "MotionCueing.cpp"
        "EthernetTransport.cpp"
        "WifiTransport.cpp"
        "BleTransport.cpp"
    INCLUDE_DIRS 
        "."
        "../include"
    PRIV_REQUIRES
        driver        # For GPIO, RMT, GPTimer, SPI
        esp_timer     # For high-resolution timing
        nvs_flash     # For non-volatile storage
        freertos      # For tasks, semaphores
        esp_eth       # For W5500 SPI Ethernet
        esp_netif     # For TCP/IP network interface
        esp_event     # For Ethernet/IP event handlers
        esp_wifi      # For WiFi STA transport
        bt            # For BLE transport
)

# Enable C++11 support
set_source_files_properties(
    main.cpp helpers.cpp InverseKinematics.cpp AxisScaling.cpp MotionCueing.cpp EthernetTransport.cpp WifiTransport.cpp BleTransport.cpp
    PROPERTIES COMPILE_FLAGS "-std=gnu++11"
)

# Performance optimization flags
target_compile_options(${COMPONENT_LIB} PRIVATE 
    -O2                    # Optimization level 2
    -ffast-math           # Fast floating point math
    -fno-exceptions       # Disable C++ exceptions (saves space)
    -fno-rtti             # Disable RTTI (saves space)
)

# Enable runtime-toggled debug output (DBG:1 / DBG:0 serial commands)
# Enable WiFi transport (ESP32 WiFi STA + UDP listener)
target_compile_definitions(${COMPONENT_LIB} PRIVATE ENABLE_DEBUG_UART=1 ENABLE_WIFI=1 ENABLE_BLE=1)

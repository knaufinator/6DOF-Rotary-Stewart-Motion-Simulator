# Main component for Mini-6DOF ESP-IDF project

idf_component_register(
    SRCS
        "main.cpp"
        "helpers.cpp"
        "BleTransport.cpp"
        "CobsTransport.cpp"
    INCLUDE_DIRS
        "."
        "../include"
    PRIV_REQUIRES
        stewart-core  # Shared IK / AxisScaling / MotionCueing (git submodule = single source of truth)
        driver        # For LEDC PWM, GPIO, UART
        esp_timer     # For high-resolution timing
        nvs_flash     # For non-volatile storage
        freertos      # For tasks, semaphores
        bt            # For BLE transport
        vfs           # For uart_vfs line-ending control
    EMBED_FILES
        "laps123_moderate.m6p"   # baked 50Hz motion-cued laps 1-3 for autonomous playback
)

# Enable C++11 support (firmware sources; stewart-core compiles under its own component)
set_source_files_properties(
    main.cpp helpers.cpp BleTransport.cpp CobsTransport.cpp
    PROPERTIES COMPILE_FLAGS "-std=gnu++11"
)

# Performance optimization flags
target_compile_options(${COMPONENT_LIB} PRIVATE
    -O2
    -ffast-math
    -fno-exceptions
    -fno-rtti
)

# Enable runtime-toggled debug output (DBG:1 / DBG:0 serial commands)
target_compile_definitions(${COMPONENT_LIB} PRIVATE ENABLE_DEBUG_UART=1 ENABLE_BLE=1)

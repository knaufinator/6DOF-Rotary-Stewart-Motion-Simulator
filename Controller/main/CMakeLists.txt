# Main component for Stewart Platform ESP-IDF project

idf_component_register(
    SRCS 
        "main.cpp"
        "helpers.cpp"
        "InverseKinematics.cpp"
    INCLUDE_DIRS 
        "."
        "../include"
    PRIV_REQUIRES
        driver        # For GPIO, RMT, GPTimer
        esp_timer     # For high-resolution timing
        nvs_flash     # For non-volatile storage
        freertos      # For tasks, semaphores
)

# Enable C++11 support
set_source_files_properties(
    main.cpp helpers.cpp InverseKinematics.cpp
    PROPERTIES COMPILE_FLAGS "-std=gnu++11"
)

# Performance optimization flags
target_compile_options(${COMPONENT_LIB} PRIVATE 
    -O2                    # Optimization level 2
    -ffast-math           # Fast floating point math
    -fno-exceptions       # Disable C++ exceptions (saves space)
    -fno-rtti             # Disable RTTI (saves space)
)

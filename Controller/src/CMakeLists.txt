# Main component CMakeLists.txt for ESP-IDF
idf_component_register(
    SRCS 
        "helpers.cpp"
        "InverseKinematics.cpp"
        "main.cpp"
    INCLUDE_DIRS 
        "../include"
    REQUIRES
        driver
        esp_timer
        nvs_flash
        freertos
)

# Add C++11 support
target_compile_options(${COMPONENT_LIB} PRIVATE -std=gnu++11)

# Add optimization flags for performance
target_compile_options(${COMPONENT_LIB} PRIVATE 
    -O2
    -ffast-math
    -fno-exceptions
)

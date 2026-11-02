# ESP-IDF CMakeLists.txt for Stewart Platform Controller
# For native ESP-IDF build (not Arduino)
#
# Unified firmware: choose FIRMWARE_PROFILE=mini|r14|r13-bench|legacy.
# Each profile fixes its chip, I/O and safety configuration.
cmake_minimum_required(VERSION 3.16)
include(cmake/FirmwareProfile.cmake)
set(SDKCONFIG "${CMAKE_BINARY_DIR}/sdkconfig" CACHE FILEPATH "Per-build hardware configuration")
if(PROFILE_MINI)
    set(SDKCONFIG_DEFAULTS "${CMAKE_CURRENT_LIST_DIR}/profiles/mini/sdkconfig.defaults")
endif()

# Both ESP32 and ESP32-S3 are supported targets
set(SUPPORTED_TARGETS esp32 esp32s3)

# Shared IK / AxisScaling / MotionCueing come from the stewart-core submodule at the
# controller root (single source of truth = atan2f), consumed as an extra component.
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/stewart-core")

# Include ESP-IDF build system
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
idf_build_set_property(DEPENDENCIES_LOCK "${CMAKE_CURRENT_LIST_DIR}/profiles/dependencies.${PROFILE_TARGET}.lock")

# Define the project
project(stewart_platform)

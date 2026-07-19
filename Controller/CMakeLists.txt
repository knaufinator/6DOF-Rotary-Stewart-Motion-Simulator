# ESP-IDF CMakeLists.txt for Stewart Platform Controller
# For native ESP-IDF build (not Arduino)
#
# Supports two PCB revisions via target chip selection:
#   idf.py set-target esp32    → PCBv1 (original DevKit + MCP23S17 SPI expander)
#   idf.py set-target esp32s3  → PCBv2 (new board, direct MCPWM GPIO)
cmake_minimum_required(VERSION 3.16)

# Both ESP32 and ESP32-S3 are supported targets
set(SUPPORTED_TARGETS esp32 esp32s3)

# Shared IK / AxisScaling / MotionCueing come from the stewart-core submodule at the
# repo root (single source of truth = atan2f), consumed as an out-of-tree component.
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../stewart-core")

# Include ESP-IDF build system
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Define the project
project(stewart_platform)

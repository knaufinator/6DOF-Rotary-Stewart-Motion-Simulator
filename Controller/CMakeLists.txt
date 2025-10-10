# ESP-IDF CMakeLists.txt for Stewart Platform Controller
# For native ESP-IDF build (not Arduino)
cmake_minimum_required(VERSION 3.16)

# Set project name before including project.cmake
set(SUPPORTED_TARGETS esp32s3)

# Include ESP-IDF build system
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Define the project
project(stewart_platform)

# ESP-IDF CMakeLists.txt for Mini-6DOF Controller
# PWM servo variant of the Stewart Platform controller
cmake_minimum_required(VERSION 3.16)

# Support both ESP32 and ESP32-S3
# set(SUPPORTED_TARGETS esp32 esp32s3)

# Shared IK / AxisScaling / MotionCueing come from the SINGLE stewart-core submodule
# at the repo root (single source of truth = atan2f), consumed as an out-of-tree
# component — mirrors Controller/CMakeLists.txt. The mini/ variant does NOT carry its
# own components/stewart-core; the root submodule provides the `stewart-core` component
# that main/CMakeLists.txt PRIV_REQUIRES.
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../stewart-core")

# Include ESP-IDF build system
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Define the project
project(mini_6dof)

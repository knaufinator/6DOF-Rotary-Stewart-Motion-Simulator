cmake_minimum_required(VERSION 3.20)
project(stewart-tests LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 11)

# ── Shared C modules from ESP32 firmware ────────────────────────────
set(CONTROLLER_DIR ${CMAKE_SOURCE_DIR}/../Controller)

add_library(stewart_math STATIC
    ${CONTROLLER_DIR}/main/InverseKinematics.cpp
    ${CONTROLLER_DIR}/main/AxisScaling.cpp
    ${CONTROLLER_DIR}/main/MotionCueing.cpp
)
target_include_directories(stewart_math PUBLIC ${CONTROLLER_DIR}/include)
target_compile_definitions(stewart_math PRIVATE
    SIL_BUILD=1
    _USE_MATH_DEFINES
)

# ── Test executable ─────────────────────────────────────────────────
add_executable(stewart-tests
    test_main.cpp
    test_inverse_kinematics.cpp
    test_axis_scaling.cpp
    test_motion_cueing.cpp
)
target_include_directories(stewart-tests PRIVATE
    ${CONTROLLER_DIR}/include
    ${CMAKE_SOURCE_DIR}
)
target_link_libraries(stewart-tests PRIVATE stewart_math)
target_compile_definitions(stewart-tests PRIVATE
    SIL_BUILD=1
    _USE_MATH_DEFINES
)

# Link math library on non-Windows
if(NOT WIN32)
    target_link_libraries(stewart-tests PRIVATE m)
endif()

# ── CTest integration ──────────────────────────────────────────────
enable_testing()
add_test(NAME unit_tests COMMAND stewart-tests)

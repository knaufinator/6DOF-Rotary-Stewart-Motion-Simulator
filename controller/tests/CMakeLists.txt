cmake_minimum_required(VERSION 3.20)
project(stewart-tests LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 11)

# ── Shared math from the stewart-core submodule (single source of truth = atan2f) ──
set(STEWART_CORE ${CMAKE_SOURCE_DIR}/../stewart-core)

add_library(stewart_math STATIC
    ${STEWART_CORE}/src/InverseKinematics.cpp
    ${STEWART_CORE}/src/AxisScaling.cpp
    ${STEWART_CORE}/src/MotionCueing.cpp
)
target_include_directories(stewart_math PUBLIC ${STEWART_CORE}/include)
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
    test_estop_safety.cpp
    test_r14_six_axis_trajectory.cpp
    test_r14_seek_interlock.cpp
    test_r14_drive_alarm_latch.cpp
    test_r14_motion_admission.cpp
    test_r13_bench_pulse.cpp
    test_step_quiescence.cpp
    test_r14_board_io.cpp
)
target_include_directories(stewart-tests PRIVATE
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/../include
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

# Production runtime sources, with SDK/board substitutes scoped to this executable.
# No changes to ESP-IDF build bindings or hardware firmware options.
find_package(Threads REQUIRED)
add_executable(r14-runtime-scenarios
    integration/runtime_scenarios.cpp
    ../main/R14Hardware.cpp
    ../main/CobsTransport.cpp
)
target_include_directories(r14-runtime-scenarios PRIVATE integration/sdk ../include)
target_compile_definitions(r14-runtime-scenarios PRIVATE CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=1)
target_link_libraries(r14-runtime-scenarios PRIVATE Threads::Threads)
if(MSVC)
    target_compile_options(r14-runtime-scenarios PRIVATE /FI${CMAKE_CURRENT_SOURCE_DIR}/integration/sdk/compat.h)
endif()
foreach(scenario cold_boot retained_boot retained_bank1 bus_failure startup_bus_failure
        missing_adc remote_rejection alarm_during_bus_failure stale_status malformed_frame)
    add_test(NAME runtime_${scenario}
        COMMAND r14-runtime-scenarios ${scenario} ${CMAKE_CURRENT_BINARY_DIR}/runtime-${scenario}.json)
    set_tests_properties(runtime_${scenario} PROPERTIES TIMEOUT 10)
endforeach()

# Isolated actual-engine integration against a declared ideal peripheral/plant.
# No test SDK include or macro is added to a firmware target.
add_executable(r14-plant-scenarios plant/plant_scenarios.cpp plant/plant_model.cpp)
target_include_directories(r14-plant-scenarios PRIVATE plant/sdk ../include)
foreach(scenario vectors wrap trajectory locked limits stop_resume stop_phases irq_gap stop_timeout
        seek stall missed_steps hall_closed hall_open_boot hall_early hall_late
        wrong_direction drive_direction physical_limit estop field_loss stale deadline
        alarm_0 alarm_1 alarm_2 alarm_3 alarm_4 alarm_5)
    add_test(NAME plant_${scenario} COMMAND r14-plant-scenarios ${scenario}
        ${CMAKE_CURRENT_BINARY_DIR}/plant-${scenario}.json)
    set_tests_properties(plant_${scenario} PROPERTIES TIMEOUT 15)
endforeach()

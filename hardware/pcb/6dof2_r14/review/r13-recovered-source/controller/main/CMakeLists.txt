# Main component for Stewart Platform ESP-IDF project
#
# PCB version is auto-detected from the IDF target chip:
#   ESP32    → PCB_VERSION=1 (original DevKit + MCP23S17 SPI expander)
#   ESP32-S3 → PCB_VERSION=2 (new board, direct MCPWM GPIO + W5500 Ethernet)

# ── Source files (common to both PCB versions) ──
set(COMMON_SRCS
    "main.cpp"
    "helpers.cpp"
    "WifiTransport.cpp"
    "BleTransport.cpp"
    "CobsTransport.cpp"
)

# ── PCB-version-specific sources and dependencies ──
# IK / AxisScaling / MotionCueing now come from the stewart-core component (submodule).
set(PRIV_REQS
    stewart-core
    driver esp_timer nvs_flash freertos
    esp_netif esp_event esp_wifi bt esp_driver_usb_serial_jtag
)

if(IDF_TARGET STREQUAL "esp32")
    # PCBv1: MCP23S17 SPI expander for step/dir, no W5500 Ethernet
    set(PCB_SRCS ${COMMON_SRCS} "MCP23S17.cpp")
    set(PCB_VERSION 1)
elseif(IDF_TARGET STREQUAL "esp32s3")
    # PCBv2: Direct GPIO MCPWM + W5500 SPI Ethernet
    set(PCB_SRCS ${COMMON_SRCS} "EthernetTransport.cpp" "LedStatus.cpp")
    list(APPEND PRIV_REQS esp_eth)
    set(PCB_VERSION 2)
    # R13_BENCH_ONE_AXIS: console-only axis-0 bench mode, M1_OUTPUT_TEST_PLAN.md step 12
    # (hardware/pcb/6dof2_r13/bringup/). Bench builds only: idf.py -DR13_BENCH_ONE_AXIS=ON build
    option(R13_BENCH_ONE_AXIS "R13 console-only one-axis bench mode (never a release image)" OFF)
    if(R13_BENCH_ONE_AXIS)
        if(DEFINED ENV{CI} OR "$ENV{GITHUB_REF}" MATCHES "^refs/tags/")
            message(FATAL_ERROR "R13_BENCH_ONE_AXIS is a bench-only option and must not be built for CI or a release")
        endif()
        list(APPEND PCB_SRCS "R13BenchAxis.cpp")
    endif()
else()
    message(FATAL_ERROR "Unsupported target '${IDF_TARGET}': only esp32 (PCBv1) and esp32s3 (PCBv2) are supported")
endif()

idf_component_register(
    SRCS ${PCB_SRCS}
    INCLUDE_DIRS 
        "."
        "../include"
    PRIV_REQUIRES ${PRIV_REQS}
)

# Enable C++11 support for all source files
set_source_files_properties(${PCB_SRCS}
    PROPERTIES COMPILE_FLAGS "-std=gnu++11"
)

# Performance optimization flags
target_compile_options(${COMPONENT_LIB} PRIVATE 
    -O2                    # Optimization level 2
    -ffast-math           # Fast floating point math
    -fno-exceptions       # Disable C++ exceptions (saves space)
    -fno-rtti             # Disable RTTI (saves space)
)

# ── Compile definitions ──
# PCB_VERSION: 1 = original ESP32 DevKit, 2 = ESP32-S3 new board
# ENABLE_ETHERNET: only on PCBv2 (W5500 SPI wired on that board)
#
# STEP_DRIVER backend (PCBv2 only) — change this line to switch:
#   STEP_DRIVER_SSE           = SimpleStepEngine        (1 GPTimer ISR, 125 kHz/motor, needs GPTIMER_ISR_IRAM_SAFE=y)
#   STEP_DRIVER_MCPWM_ISR     = McpwmStepEngine          (1 MCPWM TEZ ISR, 125 kHz/motor, IRAM auto)
#   STEP_DRIVER_SHARED_MCPWM  = SharedMcpwmStepEngine   (1 direct TEP ISR + hw generator pulses gated by gen_force, 250 kHz/motor — recommended, bench-verified 2026-10-20)
#   STEP_DRIVER_MCPWM         = MCPWMMotorControl        (per-motor MCPWM+PCNT+RMT, 250 kHz/motor, complex)
set(STEP_DRIVER_BACKEND "STEP_DRIVER_SHARED_MCPWM")

target_compile_definitions(${COMPONENT_LIB} PRIVATE
    PCB_VERSION=${PCB_VERSION}
    ENABLE_DEBUG_UART=1
    ENABLE_WIFI=1
    ENABLE_BLE=1
)
if(PCB_VERSION EQUAL 2)
    # R14 integration is intentionally locked until the installed Hall, local
    # enable and drive-alarm IO adapter is electrically qualified. Existing
    # PCBv2/R13 bench firmware can opt out explicitly at configure time.
    option(R14_FAIL_CLOSED "Keep ESP32-S3 STEP outputs disarmed for R14 bringup" ON)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE
        ENABLE_ETHERNET=1
        ${STEP_DRIVER_BACKEND}=1
    )
    if(R14_FAIL_CLOSED)
        target_compile_definitions(${COMPONENT_LIB} PRIVATE R14_FAIL_CLOSED=1)
    endif()
    if(R13_BENCH_ONE_AXIS)
        if(NOT R14_FAIL_CLOSED)
            message(FATAL_ERROR "R13_BENCH_ONE_AXIS requires R14_FAIL_CLOSED=ON (the six-axis gate must stay closed)")
        endif()
        execute_process(COMMAND git rev-parse --short=12 HEAD
                        WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
                        OUTPUT_VARIABLE R13_BENCH_SHA OUTPUT_STRIP_TRAILING_WHITESPACE
                        ERROR_QUIET)
        execute_process(COMMAND git status --porcelain --untracked-files=no
                        WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
                        OUTPUT_VARIABLE R13_BENCH_DIRTY OUTPUT_STRIP_TRAILING_WHITESPACE
                        ERROR_QUIET)
        if(NOT R13_BENCH_SHA)
            set(R13_BENCH_SHA "unknown")
        endif()
        if(R13_BENCH_DIRTY)
            set(R13_BENCH_SHA "${R13_BENCH_SHA}-dirty")
        endif()
        message(STATUS "R13_BENCH_ONE_AXIS=ON (R14_FAIL_CLOSED=ON), firmware ${R13_BENCH_SHA}")
        target_compile_definitions(${COMPONENT_LIB} PRIVATE
            R13_BENCH_ONE_AXIS=1
            FW_GIT_SHA="${R13_BENCH_SHA}"
        )
    endif()
endif()

# Optional overrides of the shared-engine timing (idf.py -DSMSE_...=...). The header defaults
# match the verified bench values: PULSE_US=2 PERIOD_US_DEFAULT=4 PULSE_START=1 FORCE_UPMETHOD=1
# TEP_BOUNDARY_CYC=955 TEP_GUARD_CYC=100 ISR_TAIL_CYC=250. CMake caches -D values per build dir.
foreach(v SMSE_PULSE_US SMSE_PERIOD_US_DEFAULT SMSE_PULSE_START SMSE_FORCE_UPMETHOD
          SMSE_TEP_BOUNDARY_CYC SMSE_TEP_GUARD_CYC SMSE_ISR_TAIL_CYC)
    if(DEFINED ${v})
        target_compile_definitions(${COMPONENT_LIB} PRIVATE ${v}=${${v}})
    endif()
endforeach()

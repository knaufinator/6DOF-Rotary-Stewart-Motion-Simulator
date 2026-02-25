# Main component for Stewart Platform ESP-IDF project
#
# PCB version is auto-detected from the IDF target chip:
#   ESP32    → PCB_VERSION=1 (original DevKit + MCP23S17 SPI expander)
#   ESP32-S3 → PCB_VERSION=2 (new board, direct MCPWM GPIO + W5500 Ethernet)

# ── Source files (common to both PCB versions) ──
set(COMMON_SRCS
    "main.cpp"
    "helpers.cpp"
    "InverseKinematics.cpp"
    "AxisScaling.cpp"
    "MotionCueing.cpp"
    "WifiTransport.cpp"
    "BleTransport.cpp"
    "CobsTransport.cpp"
)

# ── PCB-version-specific sources and dependencies ──
set(PRIV_REQS
    driver esp_timer nvs_flash freertos
    esp_netif esp_event esp_wifi bt
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
#   STEP_DRIVER_SHARED_MCPWM  = SharedMcpwmStepEngine   (1 MCPWM TEZ ISR + hw comparator, 250 kHz/motor — recommended)
#   STEP_DRIVER_MCPWM         = MCPWMMotorControl        (per-motor MCPWM+PCNT+RMT, 250 kHz/motor, complex)
set(STEP_DRIVER_BACKEND "STEP_DRIVER_SHARED_MCPWM")

target_compile_definitions(${COMPONENT_LIB} PRIVATE
    PCB_VERSION=${PCB_VERSION}
    ENABLE_DEBUG_UART=1
    ENABLE_WIFI=1
    ENABLE_BLE=1
)
if(PCB_VERSION EQUAL 2)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE
        ENABLE_ETHERNET=1
        ${STEP_DRIVER_BACKEND}=1
    )
endif()

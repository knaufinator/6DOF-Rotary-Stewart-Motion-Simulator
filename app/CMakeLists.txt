cmake_minimum_required(VERSION 3.20)
project(stewart-platform LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Some fetched deps (e.g. cJSON v1.7.18) still declare cmake_minimum_required
# < 3.5, which CMake >= 4.0 (the VS 2026 CI runner) rejects outright. Provide a
# compatible policy floor so those subprojects still configure. This variable is
# read by each subproject's cmake_minimum_required; it is silently ignored on
# CMake < 3.31, so it is safe for older local toolchains too.
set(CMAKE_POLICY_VERSION_MINIMUM 3.5)

# ── Fetch dependencies ──────────────────────────────────────────────
include(FetchContent)

# GLFW (windowing + OpenGL context)
FetchContent_Declare(glfw
    GIT_REPOSITORY https://github.com/glfw/glfw.git
    GIT_TAG        3.4
    GIT_SHALLOW    TRUE
)
set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(glfw)

# Dear ImGui (immediate-mode GUI)
FetchContent_Declare(imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG        v1.91.8-docking
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(imgui)

# ImPlot (plotting for ImGui)
FetchContent_Declare(implot
    GIT_REPOSITORY https://github.com/epezent/implot.git
    GIT_TAG        v0.16
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(implot)

# cJSON (lightweight JSON parser)
FetchContent_Declare(cjson
    GIT_REPOSITORY https://github.com/DaveGamble/cJSON.git
    GIT_TAG        v1.7.18
    GIT_SHALLOW    TRUE
)
set(ENABLE_CJSON_TEST OFF CACHE BOOL "" FORCE)
set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
set(BUILD_SHARED_AND_STATIC_LIBS OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(cjson)

# ── ImGui library (compiled from fetched sources) ───────────────────
add_library(imgui_lib STATIC
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
    ${imgui_SOURCE_DIR}/imgui_demo.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
)
target_include_directories(imgui_lib PUBLIC
    ${imgui_SOURCE_DIR}
    ${imgui_SOURCE_DIR}/backends
)
target_link_libraries(imgui_lib PUBLIC glfw)
# ImGui uses its own built-in GL loader (imgui_impl_opengl3_loader.h) by default

# ── ImPlot library ──────────────────────────────────────────────────
add_library(implot_lib STATIC
    ${implot_SOURCE_DIR}/implot.cpp
    ${implot_SOURCE_DIR}/implot_items.cpp
)
target_include_directories(implot_lib PUBLIC ${implot_SOURCE_DIR})
target_link_libraries(implot_lib PUBLIC imgui_lib)

# ── Shared math from the stewart-core submodule (single source of truth = atan2f) ──
set(CONTROLLER_DIR ${CMAKE_SOURCE_DIR}/../controller)     # still needed below for cobs.h
set(STEWART_CORE ${CONTROLLER_DIR}/stewart-core)

add_library(stewart_math STATIC
    ${STEWART_CORE}/src/InverseKinematics.cpp
    ${STEWART_CORE}/src/AxisScaling.cpp
    ${STEWART_CORE}/src/MotionCueing.cpp
)
target_include_directories(stewart_math PUBLIC ${STEWART_CORE}/include)
# Desktop build — disable ESP-IDF specific code
target_compile_definitions(stewart_math PRIVATE
    SIL_BUILD=1
    _USE_MATH_DEFINES
)

# ── Main application ────────────────────────────────────────────────
add_executable(stewart-platform WIN32
    src/main.cpp
    src/automation.cpp
    src/app.cpp
    src/ui_panels.cpp
    src/platform_viz.cpp
    src/serial_port.cpp
    src/udp_transport.cpp
    src/plugin_manager.cpp
    src/test_harness_panel.cpp
    src/control_server.cpp
)
target_include_directories(stewart-platform PRIVATE
    src
    ${CONTROLLER_DIR}/include
    ${cjson_SOURCE_DIR}
)
target_link_libraries(stewart-platform PRIVATE
    imgui_lib
    implot_lib
    stewart_math
    cjson
)
target_compile_definitions(stewart-platform PRIVATE
    SIL_BUILD=1
    _USE_MATH_DEFINES
)

# Windows: link winsock2 for UDP, setupapi for serial enumeration, suppress console
if(WIN32)
    target_link_libraries(stewart-platform PRIVATE ws2_32 setupapi opengl32)
    set_target_properties(stewart-platform PROPERTIES LINK_FLAGS "/ENTRY:mainCRTStartup")
endif()

# Headless loopback integration. SDK substitutes apply only to the firmware
# fixture library, never to desktop socket code or the ESP-IDF firmware build.
include(CTest)
if(BUILD_TESTING AND WIN32)
    set(R14_TEST_FIXTURE ${CONTROLLER_DIR}/tests/integration)
    add_library(r14-transport-runtime STATIC
        ${R14_TEST_FIXTURE}/runtime_scenarios.cpp
        ${CONTROLLER_DIR}/main/R14Hardware.cpp
        ${CONTROLLER_DIR}/main/CobsTransport.cpp
    )
    target_include_directories(r14-transport-runtime PRIVATE
        ${R14_TEST_FIXTURE}/sdk ${CONTROLLER_DIR}/include)
    target_compile_definitions(r14-transport-runtime PRIVATE
        CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=1 R14_RUNTIME_NO_MAIN=1)
    if(MSVC)
        target_compile_options(r14-transport-runtime PRIVATE /FI${R14_TEST_FIXTURE}/sdk/compat.h)
    endif()
    add_executable(stewart-runtime-transport-tests
        tests/test_runtime_transport.cpp src/automation.cpp src/udp_transport.cpp)
    target_include_directories(stewart-runtime-transport-tests PRIVATE
        src ${cjson_SOURCE_DIR} ${R14_TEST_FIXTURE})
    target_link_libraries(stewart-runtime-transport-tests PRIVATE
        r14-transport-runtime cjson ws2_32)
    foreach(mode direct bridge)
        add_test(NAME desktop_runtime_${mode} COMMAND stewart-runtime-transport-tests
            ${mode} ${CMAKE_CURRENT_BINARY_DIR}/runtime-desktop-${mode}.json)
        set_tests_properties(desktop_runtime_${mode} PROPERTIES TIMEOUT 30
            ENVIRONMENT "STEWART_DOCUMENTATION_MODE=0")
    endforeach()
endif()

# Read-only safety regression: hardware transports are disabled before any I/O.
if(BUILD_TESTING)
    add_executable(stewart-recording-tests tests/test_recording.cpp)
    add_test(NAME recording_precision COMMAND stewart-recording-tests)
endif()
if(BUILD_TESTING AND WIN32)
    add_executable(stewart-docs-safety-tests
        tests/test_documentation_mode.cpp
        src/automation.cpp
        src/serial_port.cpp
        src/udp_transport.cpp
    )
    target_include_directories(stewart-docs-safety-tests PRIVATE src ${cjson_SOURCE_DIR})
    target_link_libraries(stewart-docs-safety-tests PRIVATE cjson ws2_32 setupapi)
    add_test(NAME documentation_transport_guards COMMAND stewart-docs-safety-tests)
    set_tests_properties(documentation_transport_guards PROPERTIES
        ENVIRONMENT "STEWART_DOCUMENTATION_MODE=1")
endif()

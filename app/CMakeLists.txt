cmake_minimum_required(VERSION 3.20)
project(stewart-platform LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

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

# ── Shared C modules from ESP32 firmware ────────────────────────────
set(CONTROLLER_DIR ${CMAKE_SOURCE_DIR}/../Controller)

add_library(stewart_math STATIC
    ${CONTROLLER_DIR}/main/InverseKinematics.cpp
    ${CONTROLLER_DIR}/main/AxisScaling.cpp
    ${CONTROLLER_DIR}/main/MotionCueing.cpp
)
target_include_directories(stewart_math PUBLIC ${CONTROLLER_DIR}/include)
# Desktop build — disable ESP-IDF specific code
target_compile_definitions(stewart_math PRIVATE
    SIL_BUILD=1
    _USE_MATH_DEFINES
)

# ── Main application ────────────────────────────────────────────────
add_executable(stewart-platform WIN32
    src/main.cpp
    src/app.cpp
    src/ui_panels.cpp
    src/platform_viz.cpp
    src/serial_port.cpp
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

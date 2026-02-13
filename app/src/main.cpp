/*
 * Stewart Platform Controller — Native C++ Application
 * GLFW + Dear ImGui + OpenGL 3.3
 *
 * Single executable, single render loop, no server, no browser.
 */
#include <cstdio>
#include <cstdlib>
#include <cmath>

// ImGui + backends (uses ImGui's built-in GL loader)
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "implot.h"

// ImGui's built-in GL function loader (provides glViewport, glClear, etc.)
#include "imgui_impl_opengl3_loader.h"

// GLFW (include after imgui to avoid GL header conflicts)
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

// Application
#include "app.h"
#include "ui_panels.h"

static void glfw_error_callback(int error, const char* description) {
    fprintf(stderr, "GLFW Error %d: %s\n", error, description);
}

int main(int, char**) {
    // ── GLFW Init ───────────────────────────────────────────────────
    glfwSetErrorCallback(glfw_error_callback);
    if (!glfwInit()) {
        fprintf(stderr, "Failed to initialize GLFW\n");
        return 1;
    }

    // OpenGL 3.3 core profile
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);

    GLFWwindow* window = glfwCreateWindow(1600, 900, "Stewart Platform Controller", nullptr, nullptr);
    if (!window) {
        fprintf(stderr, "Failed to create GLFW window\n");
        glfwTerminate();
        return 1;
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);  // vsync

    // ── ImGui Init ──────────────────────────────────────────────────
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();

    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
    io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;
    io.IniFilename = "stewart_imgui.ini";  // persist layout

    // ── Modern Fluent-Inspired Dark Theme ────────────────────────────
    ImGui::StyleColorsDark();
    ImGuiStyle& style = ImGui::GetStyle();

    // Load system font (Segoe UI on Windows) for modern look
    {
        const char* font_paths[] = {
            "C:\\Windows\\Fonts\\segoeui.ttf",
            "C:\\Windows\\Fonts\\SegoeUI.ttf",
        };
        bool font_loaded = false;
        for (auto path : font_paths) {
            FILE* f = fopen(path, "rb");
            if (f) {
                fclose(f);
                io.Fonts->AddFontFromFileTTF(path, 19.0f);
                font_loaded = true;
                break;
            }
        }
        if (!font_loaded) {
            // Fallback: use default font at a reasonable size
            ImFontConfig cfg;
            cfg.SizePixels = 19.0f;
            io.Fonts->AddFontDefault(&cfg);
        }
    }

    // Geometry: generous rounding, modern spacing
    style.WindowRounding    = 8.0f;
    style.ChildRounding     = 6.0f;
    style.FrameRounding     = 6.0f;
    style.PopupRounding     = 6.0f;
    style.ScrollbarRounding = 8.0f;
    style.GrabRounding      = 4.0f;
    style.TabRounding       = 6.0f;

    style.WindowPadding     = ImVec2(12, 12);
    style.FramePadding      = ImVec2(8, 5);
    style.ItemSpacing       = ImVec2(8, 6);
    style.ItemInnerSpacing  = ImVec2(6, 4);
    style.IndentSpacing     = 20.0f;
    style.ScrollbarSize     = 12.0f;
    style.GrabMinSize       = 8.0f;

    style.WindowBorderSize  = 1.0f;
    style.ChildBorderSize   = 1.0f;
    style.FrameBorderSize   = 0.0f;
    style.PopupBorderSize   = 1.0f;
    style.TabBorderSize     = 0.0f;
    style.TabBarBorderSize  = 1.0f;

    style.WindowTitleAlign  = ImVec2(0.0f, 0.5f);
    style.SeparatorTextAlign = ImVec2(0.0f, 0.5f);

    // Color palette: Fluent dark with teal accent
    ImVec4* c = style.Colors;
    // Accent: teal (#0ea5e9 → 0.05, 0.65, 0.91)
    const ImVec4 accent     = ImVec4(0.05f, 0.65f, 0.91f, 1.00f);
    const ImVec4 accentDim  = ImVec4(0.04f, 0.45f, 0.68f, 1.00f);
    const ImVec4 accentHov  = ImVec4(0.10f, 0.72f, 0.96f, 1.00f);

    // Backgrounds
    c[ImGuiCol_WindowBg]            = ImVec4(0.07f, 0.07f, 0.09f, 1.00f);
    c[ImGuiCol_ChildBg]             = ImVec4(0.08f, 0.08f, 0.10f, 1.00f);
    c[ImGuiCol_PopupBg]             = ImVec4(0.10f, 0.10f, 0.13f, 0.98f);
    c[ImGuiCol_MenuBarBg]           = ImVec4(0.09f, 0.09f, 0.11f, 1.00f);

    // Borders
    c[ImGuiCol_Border]              = ImVec4(0.18f, 0.18f, 0.22f, 0.60f);
    c[ImGuiCol_BorderShadow]        = ImVec4(0.00f, 0.00f, 0.00f, 0.00f);

    // Title bar
    c[ImGuiCol_TitleBg]             = ImVec4(0.07f, 0.07f, 0.09f, 1.00f);
    c[ImGuiCol_TitleBgActive]       = ImVec4(0.10f, 0.10f, 0.13f, 1.00f);
    c[ImGuiCol_TitleBgCollapsed]    = ImVec4(0.07f, 0.07f, 0.09f, 0.80f);

    // Frames (input boxes, combo boxes)
    c[ImGuiCol_FrameBg]             = ImVec4(0.12f, 0.12f, 0.15f, 1.00f);
    c[ImGuiCol_FrameBgHovered]      = ImVec4(0.16f, 0.16f, 0.20f, 1.00f);
    c[ImGuiCol_FrameBgActive]       = ImVec4(0.20f, 0.20f, 0.25f, 1.00f);

    // Tabs
    c[ImGuiCol_Tab]                 = ImVec4(0.10f, 0.10f, 0.13f, 1.00f);
    c[ImGuiCol_TabHovered]          = ImVec4(accentDim.x, accentDim.y, accentDim.z, 0.40f);
    c[ImGuiCol_TabSelected]         = ImVec4(0.14f, 0.14f, 0.18f, 1.00f);
    c[ImGuiCol_TabSelectedOverline] = accent;
    c[ImGuiCol_TabDimmed]           = ImVec4(0.08f, 0.08f, 0.10f, 1.00f);
    c[ImGuiCol_TabDimmedSelected]   = ImVec4(0.12f, 0.12f, 0.15f, 1.00f);

    // Buttons
    c[ImGuiCol_Button]              = ImVec4(0.14f, 0.14f, 0.18f, 1.00f);
    c[ImGuiCol_ButtonHovered]       = ImVec4(accentDim.x, accentDim.y, accentDim.z, 0.60f);
    c[ImGuiCol_ButtonActive]        = accent;

    // Headers (collapsing headers, selectable)
    c[ImGuiCol_Header]              = ImVec4(0.14f, 0.14f, 0.18f, 1.00f);
    c[ImGuiCol_HeaderHovered]       = ImVec4(accentDim.x, accentDim.y, accentDim.z, 0.40f);
    c[ImGuiCol_HeaderActive]        = ImVec4(accentDim.x, accentDim.y, accentDim.z, 0.60f);

    // Separators
    c[ImGuiCol_Separator]           = ImVec4(0.18f, 0.18f, 0.22f, 0.50f);
    c[ImGuiCol_SeparatorHovered]    = accentDim;
    c[ImGuiCol_SeparatorActive]     = accent;

    // Resize grip
    c[ImGuiCol_ResizeGrip]          = ImVec4(0.18f, 0.18f, 0.22f, 0.25f);
    c[ImGuiCol_ResizeGripHovered]   = accentDim;
    c[ImGuiCol_ResizeGripActive]    = accent;

    // Scrollbar
    c[ImGuiCol_ScrollbarBg]         = ImVec4(0.07f, 0.07f, 0.09f, 0.50f);
    c[ImGuiCol_ScrollbarGrab]       = ImVec4(0.22f, 0.22f, 0.26f, 1.00f);
    c[ImGuiCol_ScrollbarGrabHovered]= ImVec4(0.30f, 0.30f, 0.35f, 1.00f);
    c[ImGuiCol_ScrollbarGrabActive] = ImVec4(0.38f, 0.38f, 0.44f, 1.00f);

    // Slider
    c[ImGuiCol_SliderGrab]          = accentDim;
    c[ImGuiCol_SliderGrabActive]    = accent;

    // Checkbox / radio
    c[ImGuiCol_CheckMark]           = accent;

    // Docking
    c[ImGuiCol_DockingPreview]      = ImVec4(accent.x, accent.y, accent.z, 0.30f);
    c[ImGuiCol_DockingEmptyBg]      = ImVec4(0.05f, 0.05f, 0.07f, 1.00f);

    // Text
    c[ImGuiCol_Text]                = ImVec4(0.92f, 0.93f, 0.95f, 1.00f);
    c[ImGuiCol_TextDisabled]        = ImVec4(0.45f, 0.45f, 0.50f, 1.00f);

    // Table
    c[ImGuiCol_TableHeaderBg]       = ImVec4(0.10f, 0.10f, 0.13f, 1.00f);
    c[ImGuiCol_TableBorderStrong]   = ImVec4(0.18f, 0.18f, 0.22f, 0.60f);
    c[ImGuiCol_TableBorderLight]    = ImVec4(0.14f, 0.14f, 0.18f, 0.40f);
    c[ImGuiCol_TableRowBg]          = ImVec4(0.00f, 0.00f, 0.00f, 0.00f);
    c[ImGuiCol_TableRowBgAlt]       = ImVec4(0.10f, 0.10f, 0.13f, 0.40f);

    // Nav
    c[ImGuiCol_NavHighlight]        = accent;

    // When viewports are enabled, tweak style for platform windows
    if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable) {
        style.WindowRounding = 0.0f;
        c[ImGuiCol_WindowBg].w = 1.0f;
    }

    // Setup backends
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 330 core");

    // ── App Init ────────────────────────────────────────────────────
    g_app.loadSettings();  // recreates entities from saved JSON
    if (g_app.entities.empty()) {
        // First launch — create a default SIL entity
        g_app.addEntity("SIL Default", EntityType::SIL);
    }
    g_app.loadRecordingsFromDisk();
    g_app.loadTestSignalPresetsFromDisk();
    g_app.log(-1, "system", "Stewart Platform Controller started");
    g_app.log(-1, "system", "Add entities via Entity menu or [+ Add SIL/HIL]");

    printf("Stewart Platform Controller started\n");
    printf("  OpenGL: %s\n", (const char*)glGetString(GL_VERSION));
    printf("  Renderer: %s\n", (const char*)glGetString(GL_RENDERER));

    double last_time = glfwGetTime();
    int frame_counter = 0;
    double fps_timer = glfwGetTime();

    // Keep updating during window drag/resize (Windows modal message loop)
    glfwSetWindowRefreshCallback(window, [](GLFWwindow*) {
        g_app.frame_time = glfwGetTime();
        g_app.update();
    });

    // ── Main Loop ───────────────────────────────────────────────────
    while (!glfwWindowShouldClose(window) && g_app.running) {
        glfwPollEvents();

        double now = glfwGetTime();
        g_app.frame_time = now;
        g_app.frame_count++;

        // FPS tracking
        frame_counter++;
        if (now - fps_timer >= 1.0) {
            g_app.fps = frame_counter / (now - fps_timer);
            frame_counter = 0;
            fps_timer = now;
        }

        // ── Pipeline Update (all entities) ──────────────────────────
        g_app.update();

        // ── Render Frame ────────────────────────────────────────────
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // Draw all UI panels
        DrawUI();

        // Render
        ImGui::Render();
        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClearColor(0.06f, 0.06f, 0.08f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        // Multi-viewport support
        if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable) {
            GLFWwindow* backup = glfwGetCurrentContext();
            ImGui::UpdatePlatformWindows();
            ImGui::RenderPlatformWindowsDefault();
            glfwMakeContextCurrent(backup);
        }

        glfwSwapBuffers(window);
    }

    // ── Cleanup ─────────────────────────────────────────────────────
    g_app.stopUdpListener();
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();

    printf("Stewart Platform Controller exited cleanly\n");
    return 0;
}

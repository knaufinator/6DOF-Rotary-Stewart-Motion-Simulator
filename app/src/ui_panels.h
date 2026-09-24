#pragma once

#include "app.h"
#include <string>

struct cJSON;

// Draw the full UI — called once per frame from main loop
void DrawUI();

// Called BEFORE ImGui::NewFrame() each frame — applies any pending workspace
// ini load so docking state is consistent from the first frame after loading.
void PreFrameUI();

// Render-thread presentation API. Returned JSON is owned by the caller.
// Configure validates the entire request before changing any state; selection
// and docking are applied by the next ordinary UI frames, not a separate UI.
cJSON* UIAutomationState();
cJSON* UIAutomationConfigure(const cJSON* args, std::string& error);

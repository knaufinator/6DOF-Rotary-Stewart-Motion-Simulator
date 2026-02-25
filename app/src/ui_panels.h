#pragma once

#include "app.h"

// Draw the full UI — called once per frame from main loop
void DrawUI();

// Called BEFORE ImGui::NewFrame() each frame — applies any pending workspace
// ini load so docking state is consistent from the first frame after loading.
void PreFrameUI();

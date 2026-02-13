#pragma once

#include "imgui.h"

struct Entity;
struct VizCamera;

// Draw the 3D Stewart platform visualization into the given screen rectangle.
// hovered/active come from an InvisibleButton over the viewport area.
void DrawPlatformViz(ImDrawList* dl, ImVec2 origin, ImVec2 size,
                     Entity& e, VizCamera& cam,
                     bool hovered, bool active);

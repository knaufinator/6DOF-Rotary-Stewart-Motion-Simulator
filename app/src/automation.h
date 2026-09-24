#pragma once

// Process-immutable documentation/simulation policy. Set
// STEWART_DOCUMENTATION_MODE=1 BEFORE launching the process: App has a global
// constructor that runs before main(). There is deliberately no runtime setter.
bool IsDocumentationMode();

// Render-thread window controls; dimensions are GLFW logical client pixels.
bool SetAppWindowSize(int width, int height);
void GetAppWindowSize(int* width, int* height);

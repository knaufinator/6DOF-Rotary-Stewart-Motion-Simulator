#include "app.h"

void PipelineConfig::initDefaults() {
    initDefaultStewartConfig(&geometry);
    computeStepsPerDegree(&geometry);
    rebuildPlatform();
    computeAxisScalesFromGeometry(&axis_scales, &geometry, 0.9f);
    memset(&mca, 0, sizeof(mca));
    mca.enabled = 0;  // MCA off by default for manual sliders
    initInputFilter(&input_filter, 60.0f);
    occupant[0] = 0.0f;
    occupant[1] = 0.0f;
    occupant[2] = 800.0f;
    bit_depth = 12;
    platform_type = PlatformType::Stepper;  // default to stepper (main platform)
    servo.initDefaults();
    // Phase D: default intensity and per-axis gains
    intensity = 100.0f;
    for (int i = 0; i < 6; i++) axis_gain[i] = 100.0f;
    for (int i = 0; i < 6; i++) axis_invert[i] = false;
}

void PipelineConfig::rebuildPlatform() {
    buildPlatformFromConfig(&geometry, &platform);
}

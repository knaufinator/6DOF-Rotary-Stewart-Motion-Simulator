#include "platform_viz.h"
#include "app.h"
#include <cmath>
#include <cstdio>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ── Simple 3D math ──────────────────────────────────────────────────

struct Vec3 {
    float x, y, z;
};

static Vec3 v3(float x, float y, float z) { return {x, y, z}; }
static Vec3 v3sub(Vec3 a, Vec3 b) { return {a.x-b.x, a.y-b.y, a.z-b.z}; }
static Vec3 v3add(Vec3 a, Vec3 b) { return {a.x+b.x, a.y+b.y, a.z+b.z}; }
static Vec3 v3cross(Vec3 a, Vec3 b) {
    return {a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x};
}
static float v3dot(Vec3 a, Vec3 b) { return a.x*b.x + a.y*b.y + a.z*b.z; }
static float v3len(Vec3 v) { return sqrtf(v3dot(v, v)); }
static Vec3 v3norm(Vec3 v) {
    float l = v3len(v);
    return l > 1e-6f ? Vec3{v.x/l, v.y/l, v.z/l} : Vec3{0, 0, 1};
}

// ── Camera ──────────────────────────────────────────────────────────

struct Camera3D {
    Vec3  pos, fwd, right, up;
    float fov_factor;   // 1 / tan(fov/2)
};

static Camera3D buildCamera(Vec3 target, float azimuth, float elevation, float distance) {
    Camera3D c;
    float ce = cosf(elevation), se = sinf(elevation);
    float ca = cosf(azimuth),   sa = sinf(azimuth);

    c.pos = v3add(target, v3(distance*ce*ca, distance*ce*sa, distance*se));
    c.fwd = v3norm(v3sub(target, c.pos));

    Vec3 world_up = {0, 0, 1};
    c.right = v3norm(v3cross(c.fwd, world_up));
    c.up    = v3cross(c.right, c.fwd);
    c.fov_factor = 1.0f / tanf(35.0f * (float)M_PI / 180.0f);

    return c;
}

static ImVec2 project(Vec3 world, const Camera3D& cam, ImVec2 center, float half_h) {
    Vec3 d = v3sub(world, cam.pos);
    float x = v3dot(d, cam.right);
    float y = v3dot(d, cam.up);
    float z = v3dot(d, cam.fwd);
    if (z < 0.1f) z = 0.1f;

    float sx = center.x + (x / z) * cam.fov_factor * half_h;
    float sy = center.y - (y / z) * cam.fov_factor * half_h;
    return ImVec2(sx, sy);
}

// ── Rotation Matrix (ZYX Euler) ─────────────────────────────────────

static Vec3 rotateZYX(Vec3 p, float roll, float pitch, float yaw) {
    float cr = cosf(roll),  sr = sinf(roll);
    float cp = cosf(pitch), sp = sinf(pitch);
    float cy = cosf(yaw),   sy = sinf(yaw);

    Vec3 r;
    r.x = cy*cp*p.x + (cy*sp*sr - sy*cr)*p.y + (cy*sp*cr + sy*sr)*p.z;
    r.y = sy*cp*p.x + (sy*sp*sr + cy*cr)*p.y + (sy*sp*cr - cy*sr)*p.z;
    r.z =   -sp*p.x +            cp*sr   *p.y +            cp*cr   *p.z;
    return r;
}

// ── Depth-sorted line helper ────────────────────────────────────────

struct DepthLine {
    ImVec2 a, b;
    ImU32  col;
    float  thickness;
    float  depth;       // camera-space Z of midpoint (larger = farther)
};

static float camDepth(Vec3 world, const Camera3D& cam) {
    Vec3 d = v3sub(world, cam.pos);
    return v3dot(d, cam.fwd);
}

// ── Main Draw Function ──────────────────────────────────────────────

void DrawPlatformViz(ImDrawList* dl, ImVec2 origin, ImVec2 size,
                     Entity& e, VizCamera& cam,
                     bool hovered, bool active)
{
    const PlatformDef& plat = e.config.platform;
    float home_h = plat.home_height;

    // ── Auto-init camera distance ──
    if (cam.distance <= 0.0f) {
        float max_r = 1.0f;
        for (int k = 0; k < 6; k++) {
            float r = sqrtf(plat.actuators[k].base_pos[0] * plat.actuators[k].base_pos[0] +
                            plat.actuators[k].base_pos[1] * plat.actuators[k].base_pos[1]);
            if (r > max_r) max_r = r;
        }
        cam.distance = max_r * 4.5f;
    }

    // ── Mouse interaction ──
    if (active) {
        ImVec2 delta = ImGui::GetIO().MouseDelta;
        cam.azimuth   -= delta.x * 0.005f;
        cam.elevation += delta.y * 0.005f;
        cam.elevation  = fmaxf(-1.4f, fminf(1.4f, cam.elevation));
    }
    if (hovered) {
        float scroll = ImGui::GetIO().MouseWheel;
        if (scroll != 0.0f) {
            cam.distance *= (1.0f - scroll * 0.1f);
            cam.distance  = fmaxf(50.0f, fminf(5000.0f, cam.distance));
        }
    }

    // ── Build camera ──
    Vec3 target = {0, 0, home_h * 0.45f};
    Camera3D camera = buildCamera(target, cam.azimuth, cam.elevation, cam.distance);

    ImVec2 center = ImVec2(origin.x + size.x * 0.5f, origin.y + size.y * 0.5f);
    float half_h = size.y * 0.5f;

    // Clip to viewport
    dl->PushClipRect(origin, ImVec2(origin.x + size.x, origin.y + size.y), true);

    // Background
    dl->AddRectFilled(origin, ImVec2(origin.x + size.x, origin.y + size.y),
                      IM_COL32(10, 15, 30, 255));

    // ── Grid floor (z = 0) ──
    float max_base_r = 1.0f;
    for (int k = 0; k < 6; k++) {
        float r = sqrtf(plat.actuators[k].base_pos[0] * plat.actuators[k].base_pos[0] +
                        plat.actuators[k].base_pos[1] * plat.actuators[k].base_pos[1]);
        if (r > max_base_r) max_base_r = r;
    }
    float grid_extent = ceilf(max_base_r / 50.0f) * 50.0f + 50.0f;
    float grid_step = 50.0f;
    if (grid_extent > 300.0f) grid_step = 100.0f;

    ImU32 grid_col = IM_COL32(30, 40, 55, 80);
    for (float g = -grid_extent; g <= grid_extent + 0.1f; g += grid_step) {
        ImVec2 a = project(v3(g, -grid_extent, 0), camera, center, half_h);
        ImVec2 b = project(v3(g,  grid_extent, 0), camera, center, half_h);
        dl->AddLine(a, b, grid_col, 0.5f);
        a = project(v3(-grid_extent, g, 0), camera, center, half_h);
        b = project(v3( grid_extent, g, 0), camera, center, half_h);
        dl->AddLine(a, b, grid_col, 0.5f);
    }

    // Axis indicators
    float ax_len = grid_step;
    ImVec2 o2 = project(v3(0,0,0), camera, center, half_h);
    dl->AddLine(o2, project(v3(ax_len,0,0), camera, center, half_h), IM_COL32(200,60,60,140), 1.0f);
    dl->AddLine(o2, project(v3(0,ax_len,0), camera, center, half_h), IM_COL32(60,200,60,140), 1.0f);
    dl->AddLine(o2, project(v3(0,0,ax_len), camera, center, half_h), IM_COL32(60,60,200,140), 1.0f);

    // ── Compute geometry ──
    const float* pos    = e.state.input_physical; // [x, y, z, roll, pitch, yaw]
    const float* angles = e.state.output_angles;  // servo angles (radians)

    Vec3 base_pts[6], arm_tips[6], plat_pts[6];

    for (int k = 0; k < 6; k++) {
        const ActuatorDef& a = plat.actuators[k];

        // Base joint (on base plate, z ≈ 0)
        base_pts[k] = v3(a.base_pos[0], a.base_pos[1], a.base_pos[2]);

        // Servo arm tip
        float ang = angles[k];
        arm_tips[k] = v3(
            a.base_pos[0] + a.L1 * cosf(a.beta) * cosf(ang),
            a.base_pos[1] + a.L1 * sinf(a.beta) * cosf(ang),
            a.base_pos[2] + a.L1 * sinf(ang)
        );

        // Platform joint (rotate platform-frame position, then translate)
        Vec3 pp = v3(a.plat_pos[0], a.plat_pos[1], a.plat_pos[2]);
        Vec3 rotated = rotateZYX(pp, pos[3], pos[4], pos[5]);
        plat_pts[k] = v3(
            rotated.x + pos[0],
            rotated.y + pos[1],
            rotated.z + pos[2] + home_h
        );
    }

    // ── Collect depth-sorted lines ──
    std::vector<DepthLine> lines;
    lines.reserve(48);

    ImU32 motor_colors[6] = {
        IM_COL32(100,180,255,255), IM_COL32(100,220,140,255), IM_COL32(255,200,80,255),
        IM_COL32(200,140,255,255), IM_COL32(255,120,120,255), IM_COL32(80,220,230,255)
    };
    ImU32 base_col = IM_COL32(70, 80, 100, 200);
    ImU32 rod_col  = IM_COL32(160, 165, 175, 190);

    ImVec4 ecol(e.color[0], e.color[1], e.color[2], 1.0f);
    ImU32 plat_col = IM_COL32((int)(ecol.x*220), (int)(ecol.y*220), (int)(ecol.z*220), 230);

    // Base plate edges
    for (int k = 0; k < 6; k++) {
        int nk = (k + 1) % 6;
        Vec3 mid = v3((base_pts[k].x + base_pts[nk].x)*0.5f,
                      (base_pts[k].y + base_pts[nk].y)*0.5f,
                      (base_pts[k].z + base_pts[nk].z)*0.5f);
        lines.push_back({
            project(base_pts[k], camera, center, half_h),
            project(base_pts[nk], camera, center, half_h),
            base_col, 1.5f, camDepth(mid, camera)
        });
    }

    // Platform plate edges
    for (int k = 0; k < 6; k++) {
        int nk = (k + 1) % 6;
        Vec3 mid = v3((plat_pts[k].x + plat_pts[nk].x)*0.5f,
                      (plat_pts[k].y + plat_pts[nk].y)*0.5f,
                      (plat_pts[k].z + plat_pts[nk].z)*0.5f);
        lines.push_back({
            project(plat_pts[k], camera, center, half_h),
            project(plat_pts[nk], camera, center, half_h),
            plat_col, 2.0f, camDepth(mid, camera)
        });
    }

    // Servo arms + connecting rods
    for (int k = 0; k < 6; k++) {
        Vec3 mid_arm = v3((base_pts[k].x + arm_tips[k].x)*0.5f,
                          (base_pts[k].y + arm_tips[k].y)*0.5f,
                          (base_pts[k].z + arm_tips[k].z)*0.5f);
        lines.push_back({
            project(base_pts[k], camera, center, half_h),
            project(arm_tips[k], camera, center, half_h),
            motor_colors[k], 3.0f, camDepth(mid_arm, camera)
        });

        Vec3 mid_rod = v3((arm_tips[k].x + plat_pts[k].x)*0.5f,
                          (arm_tips[k].y + plat_pts[k].y)*0.5f,
                          (arm_tips[k].z + plat_pts[k].z)*0.5f);
        lines.push_back({
            project(arm_tips[k], camera, center, half_h),
            project(plat_pts[k], camera, center, half_h),
            rod_col, 1.5f, camDepth(mid_rod, camera)
        });
    }

    // Sort back-to-front (largest depth first)
    std::sort(lines.begin(), lines.end(), [](const DepthLine& a, const DepthLine& b) {
        return a.depth > b.depth;
    });

    // Draw sorted lines
    for (auto& ln : lines) {
        dl->AddLine(ln.a, ln.b, ln.col, ln.thickness);
    }

    // ── Joint dots (always on top) ──
    for (int k = 0; k < 6; k++) {
        ImVec2 b2 = project(base_pts[k], camera, center, half_h);
        ImVec2 t2 = project(arm_tips[k], camera, center, half_h);
        ImVec2 p2 = project(plat_pts[k], camera, center, half_h);

        dl->AddCircleFilled(b2, 4.0f, motor_colors[k]);   // base joint
        dl->AddCircleFilled(t2, 3.0f, motor_colors[k]);   // arm tip
        dl->AddCircleFilled(p2, 3.0f, plat_col);           // platform joint
    }

    // Platform center marker
    Vec3 plat_center = v3(pos[0], pos[1], pos[2] + home_h);
    ImVec2 pc = project(plat_center, camera, center, half_h);
    dl->AddCircle(pc, 5.0f, plat_col, 12, 1.5f);

    // ── Overlays ──
    dl->AddText(ImVec2(origin.x + 4, origin.y + 2),
                IM_COL32(100, 100, 100, 180), "3D Platform View");

    char info[64];
    snprintf(info, sizeof(info), "IK #%d  util %.0f%%", e.state.ik_seq, e.state.max_util);
    dl->AddText(ImVec2(origin.x + 4, origin.y + 16),
                IM_COL32(140, 140, 140, 180), info);

    if (e.state.valid_mask != 0) {
        ImVec2 ts = ImGui::CalcTextSize("OUT OF RANGE");
        dl->AddText(ImVec2(origin.x + size.x - ts.x - 6, origin.y + 2),
                    IM_COL32(255, 80, 80, 255), "OUT OF RANGE");
    }

    if (hovered) {
        dl->AddText(ImVec2(origin.x + size.x - 90, origin.y + size.y - 16),
                    IM_COL32(70, 70, 70, 140), "drag to orbit");
    }

    dl->PopClipRect();
}

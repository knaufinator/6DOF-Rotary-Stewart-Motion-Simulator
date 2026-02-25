#include "platform_viz.h"
#include "app.h"
#include "serial_port.h"
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

// ── Forward Kinematics Solver ───────────────────────────────────────
// Given 6 servo angles, find the platform pose [tx,ty,tz,roll,pitch,yaw]
// that places platform joints at distance L2 from their arm tips.
// Uses Newton-Raphson with numerical Jacobian, warm-started from previous pose.

static void computePlatJoint(const ActuatorDef& a, float home_h,
                             const float pose[6], float out[3])
{
    float cr = cosf(pose[3]), sr = sinf(pose[3]);
    float cp = cosf(pose[4]), sp = sinf(pose[4]);
    float cy = cosf(pose[5]), sy = sinf(pose[5]);
    float px = a.plat_pos[0], py = a.plat_pos[1], pz = a.plat_pos[2];
    out[0] = (cy*cp)*px + (cy*sp*sr - sy*cr)*py + (cy*sp*cr + sy*sr)*pz + pose[0];
    out[1] = (sy*cp)*px + (sy*sp*sr + cy*cr)*py + (sy*sp*cr - cy*sr)*pz + pose[1];
    out[2] = (-sp)*px   + (cp*sr)*py             + (cp*cr)*pz            + home_h + pose[2];
}

static float fkError(const PlatformDef& plat, const Vec3 arm_tips[6],
                     const float pose[6], float err[6])
{
    float max_err = 0;
    for (int k = 0; k < 6; k++) {
        float j[3];
        computePlatJoint(plat.actuators[k], plat.home_height, pose, j);
        float dx = j[0] - arm_tips[k].x;
        float dy = j[1] - arm_tips[k].y;
        float dz = j[2] - arm_tips[k].z;
        float dist = sqrtf(dx*dx + dy*dy + dz*dz);
        err[k] = dist - plat.actuators[k].L2;
        if (fabsf(err[k]) > max_err) max_err = fabsf(err[k]);
    }
    return max_err;
}

static void solveFK(const float angles[6], const PlatformDef& plat,
                    float pose[6], int max_iter = 20)
{
    // Compute arm tip positions from servo angles
    Vec3 arm_tips[6];
    for (int k = 0; k < 6; k++) {
        const ActuatorDef& a = plat.actuators[k];
        float ang = angles[k];
        arm_tips[k] = v3(
            a.base_pos[0] + a.L1 * cosf(a.beta) * cosf(ang),
            a.base_pos[1] + a.L1 * sinf(a.beta) * cosf(ang),
            a.base_pos[2] + a.L1 * sinf(ang)
        );
    }

    for (int iter = 0; iter < max_iter; iter++) {
        float err[6];
        float max_err = fkError(plat, arm_tips, pose, err);
        if (max_err < 0.05f) break;

        // Numerical Jacobian
        float J[6][6];
        float eps = 0.0005f;
        for (int j = 0; j < 6; j++) {
            float pose_p[6];
            memcpy(pose_p, pose, sizeof(float) * 6);
            pose_p[j] += eps;
            float err_p[6];
            fkError(plat, arm_tips, pose_p, err_p);
            for (int k = 0; k < 6; k++)
                J[k][j] = (err_p[k] - err[k]) / eps;
        }

        // Solve J * delta = -err  (Gaussian elimination with partial pivoting)
        float A[6][7];
        for (int i = 0; i < 6; i++) {
            for (int j = 0; j < 6; j++) A[i][j] = J[i][j];
            A[i][6] = -err[i];
        }
        for (int col = 0; col < 6; col++) {
            int pivot = col;
            for (int row = col + 1; row < 6; row++)
                if (fabsf(A[row][col]) > fabsf(A[pivot][col])) pivot = row;
            if (pivot != col)
                for (int j = 0; j < 7; j++) {
                    float tmp = A[col][j]; A[col][j] = A[pivot][j]; A[pivot][j] = tmp;
                }
            if (fabsf(A[col][col]) < 1e-12f) continue;
            for (int row = col + 1; row < 6; row++) {
                float f = A[row][col] / A[col][col];
                for (int j = col; j < 7; j++) A[row][j] -= f * A[col][j];
            }
        }
        float delta[6] = {};
        for (int i = 5; i >= 0; i--) {
            if (fabsf(A[i][i]) < 1e-12f) { delta[i] = 0; continue; }
            delta[i] = A[i][6];
            for (int j = i + 1; j < 6; j++) delta[i] -= A[i][j] * delta[j];
            delta[i] /= A[i][i];
        }

        for (int j = 0; j < 6; j++) pose[j] += 0.8f * delta[j];
    }
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
    bool hil_online = (e.type == EntityType::HIL) && e.serial && e.serial->isOpen();
    float home_angles[6] = {0, 0, 0, 0, 0, 0};
    const float* viz_angles;
    if (e.type == EntityType::SIL) {
        // SIL: always use locally computed IK angles
        viz_angles = e.state.output_angles;
    } else {
        // HIL: use telemetry angles only when handshake OK + telemetry active
        if (e.hil_tel_active && e.hil_handshake_ok) {
            viz_angles = e.state.output_angles;  // ESP32 telemetry angles
        } else if (hil_online) {
            viz_angles = home_angles;  // connected, awaiting telemetry
        } else {
            viz_angles = home_angles;  // offline
        }
    }

    Vec3 base_pts[6], arm_tips[6], plat_pts[6];

    // Compute arm tips from chosen angles
    for (int k = 0; k < 6; k++) {
        const ActuatorDef& a = plat.actuators[k];
        base_pts[k] = v3(a.base_pos[0], a.base_pos[1], a.base_pos[2]);
        float ang = viz_angles[k];
        arm_tips[k] = v3(
            a.base_pos[0] + a.L1 * cosf(a.beta) * cosf(ang),
            a.base_pos[1] + a.L1 * sinf(a.beta) * cosf(ang),
            a.base_pos[2] + a.L1 * sinf(ang)
        );
    }

    // FK solver: find rigid platform pose that satisfies L2 constraints.
    // Only recompute when IK output actually changes (new telemetry or new IK frame).
    // solveFK is Newton-Raphson iterative — up to 8 iterations × 7 fkError calls,
    // far too expensive to run every render frame at 60fps.
    {
        static int s_fk_seq[8] = {-1,-1,-1,-1,-1,-1,-1,-1};
        int ei = (e.id >= 0 && e.id < 8) ? e.id : 0;
        // When ik_seq resets to 0 (new handshake), force re-run by invalidating cache
        if (e.state.ik_seq == 0) s_fk_seq[ei] = -1;
        if (e.state.ik_seq != s_fk_seq[ei]) {
            s_fk_seq[ei] = e.state.ik_seq;
            solveFK(viz_angles, plat, e.hil_fk_pose);
            // If warm-start diverged, retry from home pose
            float err_check[6];
            Vec3 tips_check[6];
            for (int k = 0; k < 6; k++) {
                const ActuatorDef& a = plat.actuators[k];
                float ang = viz_angles[k];
                tips_check[k] = v3(
                    a.base_pos[0] + a.L1 * cosf(a.beta) * cosf(ang),
                    a.base_pos[1] + a.L1 * sinf(a.beta) * cosf(ang),
                    a.base_pos[2] + a.L1 * sinf(ang));
            }
            if (fkError(plat, tips_check, e.hil_fk_pose, err_check) > 1.0f) {
                memset(e.hil_fk_pose, 0, sizeof(e.hil_fk_pose));
                solveFK(viz_angles, plat, e.hil_fk_pose);
            }
        }
    }

    for (int k = 0; k < 6; k++) {
        float j[3];
        computePlatJoint(plat.actuators[k], home_h, e.hil_fk_pose, j);
        plat_pts[k] = v3(j[0], j[1], j[2]);
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

    // Platform center marker — from FK-solved pose (matches platform mesh)
    Vec3 plat_center = v3(e.hil_fk_pose[0], e.hil_fk_pose[1], e.hil_fk_pose[2] + home_h);
    ImVec2 pc = project(plat_center, camera, center, half_h);
    dl->AddCircle(pc, 5.0f, plat_col, 12, 1.5f);

    // ── Overlays ──
    // Mode label + compact status
    const char* mode_label;
    ImU32 mode_col;
    if (e.type == EntityType::SIL) {
        mode_label = "SIL";
        mode_col = IM_COL32(80, 180, 220, 200);
    } else if (e.hil_tel_active) {
        mode_label = "ESP32 Telemetry";
        mode_col = IM_COL32(80, 200, 160, 200);
    } else if (hil_online) {
        mode_label = "Connected";
        mode_col = IM_COL32(240, 190, 60, 200);
    } else {
        mode_label = "Offline";
        mode_col = IM_COL32(120, 120, 130, 160);
    }
    dl->AddText(ImVec2(origin.x + 4, origin.y + 2), mode_col, mode_label);

    if (e.type == EntityType::SIL || hil_online || e.hil_tel_active) {
        char info[64];
        snprintf(info, sizeof(info), "util %.0f%%", e.state.max_util);
        dl->AddText(ImVec2(origin.x + 4, origin.y + 16),
                    IM_COL32(140, 140, 140, 180), info);
    }

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

#include "passagelab/core.hpp"
#include "passagelab/io.hpp"
#include <Eigen/Core>
#include <Eigen/LU>
#include <glad/glad.h>
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include "ui_replay.hpp"

namespace {
using namespace pl;
struct Camera {
    float yaw = 0.72f, pitch = 0.42f, dist = 12;
    Eigen::Vector3f target = Eigen::Vector3f::Zero();
};
void reset_camera(Camera &camera, const Scene &scene) {
    camera = Camera{};
    Vec3 extent = scene.physical.max - scene.physical.min;
    camera.target = ((scene.physical.max + scene.physical.min) * .5).cast<float>();
    camera.dist = std::max(6.f, float(extent.norm() * 1.25));
}
struct Mat4 {
    float v[16]{};
};
Eigen::Matrix4f eigen(const Mat4 &m) {
    Eigen::Matrix4f a;
    for (int c = 0; c < 4; c++)
        for (int r = 0; r < 4; r++)
            a(r, c) = m.v[c * 4 + r];
    return a;
}
Mat4 mul(const Mat4 &a, const Mat4 &b) {
    Mat4 o;
    for (int c = 0; c < 4; c++)
        for (int r = 0; r < 4; r++)
            for (int k = 0; k < 4; k++)
                o.v[c * 4 + r] += a.v[k * 4 + r] * b.v[c * 4 + k];
    return o;
}
Mat4 perspective(float f, float aspect, float n, float z) {
    Mat4 m;
    float q = 1 / std::tan(f / 2);
    m.v[0] = q / aspect;
    m.v[5] = q;
    m.v[10] = (z + n) / (n - z);
    m.v[11] = -1;
    m.v[14] = 2 * z * n / (n - z);
    return m;
}
Mat4 view(const Camera &c) {
    Eigen::Vector3f eye = c.target + Eigen::Vector3f(c.dist * std::cos(c.pitch) * std::sin(c.yaw),
                                                     -c.dist * std::cos(c.pitch) * std::cos(c.yaw),
                                                     c.dist * std::sin(c.pitch));
    Eigen::Vector3f f = (c.target - eye).normalized(),
                    s = f.cross(Eigen::Vector3f::UnitZ()).normalized(), u = s.cross(f);
    Mat4 m;
    m.v[0] = s.x();
    m.v[4] = s.y();
    m.v[8] = s.z();
    m.v[1] = u.x();
    m.v[5] = u.y();
    m.v[9] = u.z();
    m.v[2] = -f.x();
    m.v[6] = -f.y();
    m.v[10] = -f.z();
    m.v[12] = -s.dot(eye);
    m.v[13] = -u.dot(eye);
    m.v[14] = f.dot(eye);
    m.v[15] = 1;
    return m;
}
GLuint shader(GLenum type, const char *src) {
    GLuint s = glCreateShader(type);
    glShaderSource(s, 1, &src, nullptr);
    glCompileShader(s);
    GLint ok = 0;
    glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char log[2048];
        glGetShaderInfoLog(s, sizeof log, nullptr, log);
        glDeleteShader(s);
        throw std::runtime_error(log);
    }
    return s;
}
GLuint program() {
    const char *vs = "#version 330 core\nlayout(location=0) in vec3 p; layout(location=1) in "
                     "vec3 normal; uniform mat4 mvp; uniform mat4 model; out vec3 N; void "
                     "main(){N=transpose(inverse(mat3(model)))*normal;gl_Position=mvp*vec4(p,"
                     "1);}";
    const char *fs = "#version 330 core\nin vec3 N; out vec4 c; uniform vec4 "
                     "tint; void main(){float "
                     "l=.38+.62*max(dot(normalize(N),normalize(vec3(.4,-.5,1))),"
                     "0.0);c=vec4(tint.rgb*l,tint.a);}";
    auto a = shader(GL_VERTEX_SHADER, vs);
    GLuint b = 0;
    try {
        b = shader(GL_FRAGMENT_SHADER, fs);
    } catch (...) {
        glDeleteShader(a);
        throw;
    }
    auto p = glCreateProgram();
    glAttachShader(p, a);
    glAttachShader(p, b);
    glLinkProgram(p);
    glDeleteShader(a);
    glDeleteShader(b);
    GLint ok = 0;
    glGetProgramiv(p, GL_LINK_STATUS, &ok);
    if (!ok) {
        char log[2048];
        glGetProgramInfoLog(p, sizeof log, nullptr, log);
        glDeleteProgram(p);
        throw std::runtime_error(log);
    }
    return p;
}
GLuint vao = 0, vbo = 0, sphereVao = 0, sphereVbo = 0, prog = 0;
GLint umvp = -1, umodel = -1, utint = -1;
GLsizei sphereCount = 0;
GLuint lineVao = 0, lineVbo = 0;
struct GlfwGuard {
    ~GlfwGuard() {
        glfwTerminate();
    }
};
struct WindowGuard {
    GLFWwindow *window;
    ~WindowGuard() {
        if (window)
            glfwDestroyWindow(window);
    }
};
struct GlResourcesGuard {
    ~GlResourcesGuard() {
        if (prog)
            glDeleteProgram(prog);
        if (vbo)
            glDeleteBuffers(1, &vbo);
        if (vao)
            glDeleteVertexArrays(1, &vao);
        if (sphereVbo)
            glDeleteBuffers(1, &sphereVbo);
        if (sphereVao)
            glDeleteVertexArrays(1, &sphereVao);
        if (lineVbo)
            glDeleteBuffers(1, &lineVbo);
        if (lineVao)
            glDeleteVertexArrays(1, &lineVao);
    }
};
struct ImGuiGuard {
    bool context = false, glfw_backend = false, opengl_backend = false;
    ~ImGuiGuard() {
        if (opengl_backend)
            ImGui_ImplOpenGL3_Shutdown();
        if (glfw_backend)
            ImGui_ImplGlfw_Shutdown();
        if (context)
            ImGui::DestroyContext();
    }
};
void init_mesh() {
    const float corners[8][3] = {{-.5f, -.5f, -.5f}, {.5f, -.5f, -.5f}, {.5f, .5f, -.5f},
                                 {-.5f, .5f, -.5f},  {-.5f, -.5f, .5f}, {.5f, -.5f, .5f},
                                 {.5f, .5f, .5f},    {-.5f, .5f, .5f}};
    const int idx[36] = {0, 2, 1, 0, 3, 2, 4, 5, 6, 4, 6, 7, 0, 1, 5, 0, 5, 4,
                         1, 2, 6, 1, 6, 5, 2, 3, 7, 2, 7, 6, 3, 0, 4, 3, 4, 7};
    float x[216];
    for (int i = 0; i < 36; i += 3) {
        Eigen::Vector3f p[3];
        for (int j = 0; j < 3; j++)
            p[j] = Eigen::Vector3f(corners[idx[i + j]][0], corners[idx[i + j]][1],
                                   corners[idx[i + j]][2]);
        Eigen::Vector3f n = (p[1] - p[0]).cross(p[2] - p[0]).normalized();
        for (int j = 0; j < 3; j++) {
            int k = (i + j) * 6;
            x[k] = p[j].x();
            x[k + 1] = p[j].y();
            x[k + 2] = p[j].z();
            x[k + 3] = n.x();
            x[k + 4] = n.y();
            x[k + 5] = n.z();
        }
    }
    glGenVertexArrays(1, &vao);
    glGenBuffers(1, &vbo);
    glBindVertexArray(vao);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof x, x, GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), nullptr);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
                          reinterpret_cast<void *>(3 * sizeof(float)));
    std::vector<float> s;
    constexpr int rings = 16, sides = 24;
    for (int y = 0; y < rings; y++)
        for (int x = 0; x < sides; x++) {
            auto point = [](int r, int c) {
                double a = 3.141592653589793 * r / rings, b = 2 * 3.141592653589793 * c / sides;
                return Eigen::Vector3f(float(std::sin(a) * std::cos(b)),
                                       float(std::sin(a) * std::sin(b)), float(std::cos(a)));
            };
            auto a = point(y, x), b = point(y + 1, x), c = point(y + 1, x + 1), d = point(y, x + 1);
            for (auto v : {a, b, c, a, c, d}) {
                s.insert(s.end(), {v.x(), v.y(), v.z(), v.x(), v.y(), v.z()});
            }
        }
    sphereCount = GLsizei(s.size() / 6);
    glGenVertexArrays(1, &sphereVao);
    glGenBuffers(1, &sphereVbo);
    glBindVertexArray(sphereVao);
    glBindBuffer(GL_ARRAY_BUFFER, sphereVbo);
    glBufferData(GL_ARRAY_BUFFER, GLsizeiptr(s.size() * sizeof(float)), s.data(), GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), nullptr);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
                          reinterpret_cast<void *>(3 * sizeof(float)));
    glGenVertexArrays(1, &lineVao);
    glGenBuffers(1, &lineVbo);
    glBindVertexArray(lineVao);
    glBindBuffer(GL_ARRAY_BUFFER, lineVbo);
    glBufferData(GL_ARRAY_BUFFER, 6 * sizeof(float), nullptr, GL_STREAM_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), nullptr);
    glDisableVertexAttribArray(1);
    glVertexAttrib3f(1, 0, 0, 1);
    prog = program();
    umvp = glGetUniformLocation(prog, "mvp");
    umodel = glGetUniformLocation(prog, "model");
    utint = glGetUniformLocation(prog, "tint");
}
Mat4 model(const Pose &p, const Vec3 &scale) {
    Eigen::Matrix3d r = p.q.normalized().toRotationMatrix();
    Mat4 m;
    for (int c = 0; c < 3; c++)
        for (int i = 0; i < 3; i++)
            m.v[c * 4 + i] = float(r(i, c) * scale[c]);
    m.v[12] = float(p.t.x());
    m.v[13] = float(p.t.y());
    m.v[14] = float(p.t.z());
    m.v[15] = 1;
    return m;
}
void cube(const Mat4 &vp, const Pose &p, const Vec3 &size, const Eigen::Vector4f &color,
          bool wire = false) {
    Mat4 md = model(p, size), mvp = mul(vp, md);
    glBindVertexArray(vao);
    glUniformMatrix4fv(umvp, 1, GL_FALSE, mvp.v);
    glUniformMatrix4fv(umodel, 1, GL_FALSE, md.v);
    glUniform4f(utint, color.x(), color.y(), color.z(), color.w());
    glPolygonMode(GL_FRONT_AND_BACK, wire ? GL_LINE : GL_FILL);
    glDrawArrays(GL_TRIANGLES, 0, 36);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
}
void sphere(const Mat4 &vp, const Pose &p, double radius, const Eigen::Vector4f &color,
            bool wire = false) {
    Mat4 md = model(p, Vec3::Constant(radius)), mvp = mul(vp, md);
    glUniformMatrix4fv(umvp, 1, GL_FALSE, mvp.v);
    glUniformMatrix4fv(umodel, 1, GL_FALSE, md.v);
    glUniform4f(utint, color.x(), color.y(), color.z(), color.w());
    glBindVertexArray(sphereVao);
    glPolygonMode(GL_FRONT_AND_BACK, wire ? GL_LINE : GL_FILL);
    glDrawArrays(GL_TRIANGLES, 0, sphereCount);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glBindVertexArray(vao);
}
void line(const Mat4 &vp, Vec3 a, Vec3 b, Eigen::Vector4f color) {
    float x[] = {float(a.x()), float(a.y()), float(a.z()),
                 float(b.x()), float(b.y()), float(b.z())};
    Mat4 ident;
    ident.v[0] = ident.v[5] = ident.v[10] = ident.v[15] = 1;
    glBindVertexArray(lineVao);
    glBindBuffer(GL_ARRAY_BUFFER, lineVbo);
    glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof x, x);
    glUniformMatrix4fv(umvp, 1, GL_FALSE, vp.v);
    glUniformMatrix4fv(umodel, 1, GL_FALSE, ident.v);
    glUniform4f(utint, color.x(), color.y(), color.z(), color.w());
    glDrawArrays(GL_LINES, 0, 2);
    glBindVertexArray(vao);
}
bool drag_double(const char *label, double *value, double speed, double min, double max) {
    return ImGui::DragScalar(label, ImGuiDataType_Double, value, float(speed), &min, &max, "%.4f");
}
bool drag_vector3(const char *label, double *value, double speed, double min, double max) {
    const bool changed = ImGui::DragScalarN(label, ImGuiDataType_Double, value, 3, float(speed), &min, &max,
                              "%.3f");
    if (active_replay) active_replay->track(label);
    return changed;
}
bool input_string(const char *label, std::string &v) {
    std::array<char, 512> b{};
    std::snprintf(b.data(), b.size(), "%s", v.c_str());
    if (!ImGui::InputText(label, b.data(), b.size()))
        return false;
    v = b.data();
    return true;
}
Pose pose(const Pose &p, const char *label) {
    Pose out = p;
    ImGui::Text("%s position (m)", label);
    ImGui::SetNextItemWidth(-1);
    ImGui::PushID(label);
    ImGui::DragScalarN("##translation", ImGuiDataType_Double, out.t.data(), 3, 0.01f, nullptr,
                       nullptr, "%.3f");
    double e[3];
    Eigen::Vector3d angles = out.q.toRotationMatrix().eulerAngles(2, 1, 0);
    e[0] = angles.z() * 57.2957795;
    e[1] = angles.y() * 57.2957795;
    e[2] = angles.x() * 57.2957795;
    ImGui::TextUnformatted("XYZ rotation (deg)");
    ImGui::SetNextItemWidth(-1);
    if (ImGui::DragScalarN("##rotation", ImGuiDataType_Double, e, 3, 0.5f, nullptr, nullptr,
                           "%.1f")) {
        out.q = Eigen::AngleAxisd(e[2] * .0174532925, Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(e[1] * .0174532925, Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(e[0] * .0174532925, Eigen::Vector3d::UnitX());
    }
    ImGui::PopID();
    return out;
}
struct Job {
    std::mutex m;
    std::vector<SearchEdge> live_edges;
    std::optional<PlanningResult> result;
    std::optional<Path> optimized;
    PathMetrics metrics;
    std::optional<PathMetrics> optimized_metrics;
    std::atomic<std::uint64_t> request{0};
    std::atomic<std::uint64_t> iterations{0};
    bool imported = false;
    bool invalidated_result = false;
};
struct Async {
    std::jthread thread;
    std::atomic<bool> running{false};
    std::atomic<bool> pause_after_progress{false};
    std::atomic<bool> optimizing{false};
    ~Async() {
        // Join before the atomics are destroyed, including exception unwinding.
        if (thread.joinable()) {
            thread.request_stop();
            thread.join();
        }
    }
};
void invalidate(Scene &s, Job &j, Async &async) {
    ++s.revision;
    refresh_hash(s);
    ++j.request;
    {
        std::lock_guard lock(j.m);
        j.invalidated_result = j.invalidated_result || j.result.has_value();
        j.result.reset();
        j.live_edges.clear();
        j.optimized.reset();
        j.optimized_metrics.reset();
        j.metrics = {};
        j.imported = false;
    }
    if (async.running)
        async.thread.request_stop();
}
void reap(Async &a) {
    if (a.thread.joinable() && !a.running.load())
        a.thread.join();
}
bool start_plan(const Scene &s, Job &j, Async &a) {
    reap(a);
    if (a.running)
        return false;
    const Scene snap = s;
    const auto req = ++j.request;
    {
        std::lock_guard lock(j.m);
        j.result.reset();
        j.live_edges.clear();
        j.optimized.reset();
        j.optimized_metrics.reset();
        j.metrics = {};
        j.imported = false;
        j.iterations = 0;
    }
    a.optimizing = false;
    a.running = true;
    a.thread = std::jthread([snap, req, &j, &a](std::stop_token stop) {
        PlanningResult r;
        PathMetrics metrics;
        try {
            r = solve(snap, snap.config, stop, [&j, &a, req, stop](std::uint64_t i, std::uint64_t) {
                if (req == j.request) {
                    j.iterations = i;
                    while (a.pause_after_progress.load() && !stop.stop_requested())
                        std::this_thread::sleep_for(std::chrono::milliseconds(1));
                }
            }, [&j, req](const SearchEdge &edge) {
                std::lock_guard lock(j.m);
                if (req == j.request && j.live_edges.size() < 4000)
                    j.live_edges.push_back(edge);
            });
            if (r.status == SolveStatus::Solved && !stop.stop_requested()) {
                metrics = evaluate(snap, r.path, snap.config, stop);
            }
        } catch (const std::exception &e) {
            r.status = SolveStatus::Error;
            r.reason = e.what();
        } catch (...) {
            r.status = SolveStatus::Error;
            r.reason = "Unknown planner exception";
        }
        if (stop.stop_requested()) {
            r.status = SolveStatus::Cancelled;
            r.reason = "Cancellation requested";
        }
        {
            std::lock_guard lock(j.m);
            if (req == j.request) {
                j.metrics = metrics;
                j.result = std::move(r);
                j.imported = false;
                j.invalidated_result = false;
            }
        }
        a.running = false;
        a.optimizing = false;
    });
    return true;
}
bool start_optimization(const Scene &s, Job &j, Async &a) {
    reap(a);
    if (a.running)
        return false;
    Path original;
    {
        std::lock_guard lock(j.m);
        if (!j.result || j.result->status != SolveStatus::Solved || j.imported)
            return false;
        original = j.result->path;
        j.optimized.reset();
        j.optimized_metrics.reset();
    }
    const Scene snapshot = s;
    const auto request = ++j.request;
    j.iterations = 0;
    a.optimizing = true;
    a.running = true;
    a.thread = std::jthread(
        [snapshot, original = std::move(original), request, &j, &a](std::stop_token stop) {
            std::optional<Path> optimized;
            std::optional<PathMetrics> metrics;
            try {
                Path candidate = optimize(snapshot, original, snapshot.config, stop);
                if (!stop.stop_requested() && candidate.poses.size() > 1) {
                    metrics = evaluate(snapshot, candidate, snapshot.config, stop);
                    optimized = std::move(candidate);
                }
            } catch (...) {
                // Preserve the original validated result when shortcut work fails.
            }
            {
                std::lock_guard lock(j.m);
                if (request == j.request && !stop.stop_requested()) {
                    j.optimized = std::move(optimized);
                    j.optimized_metrics = std::move(metrics);
                }
            }
            a.running = false;
            a.optimizing = false;
        });
    return true;
}
bool start_revalidation(const Scene &s, Job &j, Async &a) {
    reap(a);
    if (a.running)
        return false;
    Path p;
    {
        std::lock_guard lk(j.m);
        if (!j.result)
            return false;
        p = j.result->path;
    }
    const Scene snap = s;
    const auto req = ++j.request;
    {
        std::lock_guard lk(j.m);
        j.result.reset();
        j.optimized.reset();
        j.optimized_metrics.reset();
        j.imported = false;
    }
    a.optimizing = false;
    a.running = true;
    a.thread = std::jthread([snap, p = std::move(p), req, &j, &a](std::stop_token stop) mutable {
        PlanningResult r;
        r.path = p;
        PathMetrics metrics;
        try {
            auto check = validate_path(snap, p, snap.config.validation, stop);
            r.verification = check;
            r.status = check.status == CheckStatus::Passed      ? SolveStatus::Solved
                       : check.status == CheckStatus::Cancelled ? SolveStatus::Cancelled
                                                                : SolveStatus::ValidationIncomplete;
            r.reason = check.reason.empty() ? (r.status == SolveStatus::Solved
                                                   ? "Imported path passed discrete validation"
                                                   : "Imported path did not pass validation")
                                            : check.reason;
            if (r.status == SolveStatus::Solved && !stop.stop_requested())
                metrics = evaluate(snap, p, snap.config, stop);
        } catch (const std::exception &e) {
            r.status = SolveStatus::Error;
            r.reason = e.what();
        } catch (...) {
            r.status = SolveStatus::Error;
            r.reason = "Unknown validation exception";
        }
        if (stop.stop_requested()) {
            r.status = SolveStatus::Cancelled;
            r.reason = "Validation cancelled";
        }
        {
            std::lock_guard lk(j.m);
            if (req == j.request) {
                j.metrics = metrics;
                j.result = std::move(r);
                j.imported = false;
                j.invalidated_result = false;
            }
        }
        a.running = false;
        a.optimizing = false;
    });
    return true;
}
void require_test(bool condition, const char *message) {
    if (!condition)
        throw std::runtime_error(std::string("UI self-test failed: ") + message);
}
void wait_async(Async &async) {
    auto until = std::chrono::steady_clock::now() + std::chrono::seconds(30);
    while (async.running.load() && std::chrono::steady_clock::now() < until)
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    if (async.running.load()) {
        async.thread.request_stop();
        while (async.running.load())
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        throw std::runtime_error("UI self-test timed out waiting for worker");
    }
    reap(async);
}
Json run_ui_self_test(Scene scene) {
    Job job;
    Async async;
    Validator initial_validator(scene);
    (void)initial_validator.state(scene.start, scene.config.validation, true);
    require_test(initial_validator.stats().states > 0, "query the current pose before planning");
    scene.config.time_limit = 0;
    scene.config.max_iterations = 1000000;
    scene.config.max_nodes = 1000000;
    scene.config.max_queries = 1000000;
    refresh_hash(scene);

    require_test(start_plan(scene, job, async), "launch worker");
    invalidate(scene, job, async);
    wait_async(async);
    {
        std::lock_guard lock(job.m);
        require_test(!job.result, "discard stale worker result after revision");
    }

    async.pause_after_progress = true;
    require_test(start_plan(scene, job, async), "launch cancellable worker");
    auto progress_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (job.iterations.load() == 0 && async.running.load() &&
           std::chrono::steady_clock::now() < progress_deadline)
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    const auto cancel_start = std::chrono::steady_clock::now();
    async.thread.request_stop();
    async.pause_after_progress = false;
    wait_async(async);
    const double cancel_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - cancel_start).count();
    require_test(cancel_ms < 1000, "cancel worker within one second on reference machine");
    {
        std::lock_guard lock(job.m);
        require_test(job.result && job.result->status == SolveStatus::Cancelled,
                     "publish cancellation for current request");
    }

    Path imported;
    imported.scene_hash = scene.hash;
    imported.scene_revision = scene.revision;
    imported.validation = scene.config.validation;
    imported.source = "ui-self-test-import";
    imported.poses = {scene.start, scene.goal};
    {
        std::lock_guard lock(job.m);
        PlanningResult preview;
        preview.status = SolveStatus::Error;
        preview.reason = "Imported path is an unverified preview";
        preview.path = imported;
        job.result = std::move(preview);
        job.imported = true;
        job.invalidated_result = false;
    }
    require_test(start_revalidation(scene, job, async), "launch path validation");
    wait_async(async);
    {
        std::lock_guard lock(job.m);
        require_test(job.result && !job.imported, "finish imported preview validation");
        require_test(job.result->status != SolveStatus::Error &&
                         job.result->status != SolveStatus::Cancelled,
                     "return a validation status");
    }

    auto dir = std::filesystem::temp_directory_path() /
               ("passagelab-ui-self-test-" +
                std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    std::filesystem::create_directories(dir);
    auto scene_file = dir / "scene.json", path_file = dir / "path.json";
    save_scene(scene, scene_file);
    Scene loaded_scene = load_scene(scene_file);
    require_test(loaded_scene.hash == scene.hash, "scene export and reload");
    save_path(imported, path_file);
    Path loaded_path = load_path(path_file);
    require_test(loaded_path.poses.size() == imported.poses.size() &&
                     loaded_path.scene_hash == imported.scene_hash,
                 "path export and reload");
    std::filesystem::remove_all(dir);
    return {{"passed", true}, {"cancel_latency_ms", cancel_ms}, {"cancel_target_ms", 1000},
            {"checks", {"snapshot invalidation", "cancellation", "import revalidation", "scene and path round trip"}}};
}
bool save_ppm(const char *file, int w, int h) {
    std::vector<unsigned char> pix(std::size_t(w) * h * 3);
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glReadPixels(0, 0, w, h, GL_RGB, GL_UNSIGNED_BYTE, pix.data());
    std::ofstream f(file, std::ios::binary);
    f << "P6\n" << w << " " << h << "\n255\n";
    for (int y = h - 1; y >= 0; --y)
        f.write(reinterpret_cast<char *>(pix.data() + std::size_t(y) * w * 3), w * 3);
    return bool(f);
}
std::optional<double> ray_sphere(const Vec3 &o, const Vec3 &d, const Vec3 &c, double r) {
    Vec3 v = o - c;
    double b = v.dot(d), q = v.squaredNorm() - r * r, disc = b * b - q;
    if (disc < 0)
        return {};
    double t = -b - std::sqrt(disc);
    if (t < 0)
        t = -b + std::sqrt(disc);
    if (t < 0)
        return {};
    return t;
}
std::pair<Vec3, Vec3> mouse_ray(const Mat4 &vp, double mx, double my, int w, int h) {
    Eigen::Matrix4f inv = eigen(vp).inverse();
    float x = float(2 * mx / w - 1), y = float(1 - 2 * my / h);
    Eigen::Vector4f a = inv * Eigen::Vector4f(x, y, -1, 1), b = inv * Eigen::Vector4f(x, y, 1, 1);
    a /= a.w();
    b /= b.w();
    Vec3 origin = a.head<3>().cast<double>(),
         dir = (b.head<3>() - a.head<3>()).normalized().cast<double>();
    return {origin, dir};
}
} // namespace

int main(int argc, char **argv) {
    try {
        bool smoke = false, ui_self_test = false;
        int frames = -1, demo_index = 0;
        std::string shot, scene_file, capture_dir, interaction_report, self_test_report, performance_report;
        for (int i = 1; i < argc; i++) {
            std::string a = argv[i];
            if (a == "--smoke-test")
                smoke = true;
            else if (a == "--ui-self-test")
                ui_self_test = true;
            else if (a == "--frames" && i + 1 < argc)
                frames = std::stoi(argv[++i]);
            else if (a == "--screenshot" && i + 1 < argc)
                shot = argv[++i];
            else if (a == "--capture-dir" && i + 1 < argc)
                capture_dir = argv[++i];
            else if (a == "--demo" && i + 1 < argc)
                demo_index = std::stoi(argv[++i]);
            else if (a == "--scene" && i + 1 < argc)
                scene_file = argv[++i];
            else if (a == "--interaction-test" && i + 1 < argc)
                interaction_report = argv[++i];
            else if (a == "--self-test-report" && i + 1 < argc)
                self_test_report = argv[++i];
            else if (a == "--performance-report" && i + 1 < argc)
                performance_report = argv[++i];
            else
                throw std::runtime_error("Unknown or incomplete desktop option: " + a);
        }
        if (!capture_dir.empty() && frames < 1)
            frames = 60;
        if (!performance_report.empty() && frames < 1) frames = 180;
        if (!glfwInit())
            throw std::runtime_error("GLFW initialization failed");
        GlfwGuard glfw_guard;
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
        glfwWindowHint(GLFW_SAMPLES, 4);
        if (ui_self_test || !interaction_report.empty() || !performance_report.empty())
            glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
        GLFWwindow *w =
            glfwCreateWindow(1440, 900, "PassageLab | SE(3) Motion Lab", nullptr, nullptr);
        if (!w)
            throw std::runtime_error("OpenGL 3.3 window creation failed");
        WindowGuard window_guard{w};
        glfwSetWindowSizeLimits(w, 1000, 700, GLFW_DONT_CARE, GLFW_DONT_CARE);
        glfwMakeContextCurrent(w);
        glfwSwapInterval(interaction_report.empty() && performance_report.empty() ? 1 : 0);
        if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
            throw std::runtime_error("GLAD initialization failed");
        GlResourcesGuard gl_guard;
        init_mesh();
        IMGUI_CHECKVERSION();
        ImGui::CreateContext();
        ImGuiGuard imgui_guard;
        imgui_guard.context = true;
        ImGuiIO &io = ImGui::GetIO();
        const std::filesystem::path system_font = "C:/Windows/Fonts/segoeui.ttf";
        ImFont *ui_font = nullptr;
        if (std::filesystem::exists(system_font))
            ui_font = io.Fonts->AddFontFromFileTTF(system_font.string().c_str(), 17.0f);
        if (!ui_font)
            ui_font = io.Fonts->AddFontDefault();
        ImGui::StyleColorsDark();
        auto &sty = ImGui::GetStyle();
        sty.WindowRounding = 8;
        sty.FrameRounding = 5;
        sty.Colors[ImGuiCol_WindowBg] = ImVec4(.055f, .075f, .105f, .98f);
        sty.Colors[ImGuiCol_Header] = ImVec4(.12f, .42f, .49f, 1);
        imgui_guard.glfw_backend = ImGui_ImplGlfw_InitForOpenGL(w, true);
        imgui_guard.opengl_backend = ImGui_ImplOpenGL3_Init("#version 330 core");
        if (!imgui_guard.glfw_backend || !imgui_guard.opengl_backend)
            throw std::runtime_error("Dear ImGui backend initialization failed");
        std::vector<Scene> demos = demo_scenes();
        if (demo_index < 0 || demo_index >= int(demos.size()))
            throw std::runtime_error("--demo index is out of range");
        Scene scene = scene_file.empty() ? demos.at(size_t(demo_index)) : load_scene(scene_file);
        if (ui_self_test) {
            const auto self_test = run_ui_self_test(scene);
            if (!self_test_report.empty()) write_json(self_test, self_test_report);
            ImGui_ImplOpenGL3_NewFrame();
            ImGui_ImplGlfw_NewFrame();
            ImGui::NewFrame();
            ImGui::Begin("PassageLab UI self-test");
            ImGui::TextUnformatted("Worker, invalidation, validation and JSON checks passed.");
            ImGui::End();
            ImGui::Render();
            glViewport(0, 0, 1440, 900);
            glClear(GL_COLOR_BUFFER_BIT);
            ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
            glfwSwapBuffers(w);
            std::puts("PassageLab UI self-test passed");
            return 0;
        }
        Job job;
        Async async;
        Camera cam;
        reset_camera(cam, scene);
        bool wire = false, axes = true, tree = false, ghosts = true, optimized_view = false,
             play = false;
        bool inspect_search = false;
        int search_edge_index = 0;
        float search_edge_u = 1;
        float scrub = 0;
        int ghost_count = 5;
        std::string scene_path = scene_file.empty() ? "scene.json" : scene_file,
                    path_path = "path.json", load_path_name = "path.json",
                    report_path = "report.json", message = "Ready · discrete validation";
        int selected = -1;
        UiReplay replay;
        if (!interaction_report.empty()) {
            active_replay = &replay;
            io.ConfigInputTrickleEventQueue = false;
            replay.steps = {{"Plan path"}, {"Step forward"}, {"Play"}, {"Play"},
                {"Inspect minimum sampled clearance"}, {"Shortcut"}, {"Show optimized"},
                {"Export scene"}, {"Export path"}, {"Export report"}, {"Wireframe"},
                {"Search tree"}, {"Load path JSON"}, {"Revalidate path"}, {"Half size", true},
                {"Load JSON"}, {"Plan path"}, {"Inspect search motion"},
                {"Inspect minimum sampled clearance"}, {"Viewport orbit", true},
                {"Viewport pan", true, true}, {"Viewport zoom", false, false, 1},
                {"Viewport pick"}, {"Focus selection"}, {"Reset camera"}};
            const auto initial_hash = scene.hash;
            const Camera initial_camera = cam;
            replay.verify = [&, initial_hash, initial_camera](std::size_t stage) {
                std::lock_guard lock(job.m);
                const bool solved = !async.running && job.result && job.result->status == SolveStatus::Solved;
                switch (stage) {
                case 0: case 13: case 16: return solved && !job.imported;
                case 1: return scrub > 0;
                case 2: return play;
                case 3: return !play;
                case 4: case 18: return !play && !inspect_search && job.metrics.min_clearance_position &&
                    std::abs(scrub - *job.metrics.min_clearance_position) < 1e-6;
                case 5: return !async.running && job.optimized.has_value();
                case 6: return optimized_view;
                case 7: return std::filesystem::exists(scene_path);
                case 8: return std::filesystem::exists(path_path);
                case 9: return std::filesystem::exists(report_path);
                case 10: return wire;
                case 11: return tree;
                case 12: return job.imported;
                case 14: return !job.result && job.invalidated_result && scene.hash != initial_hash;
                case 15: return scene.hash == initial_hash;
                case 17: return inspect_search;
                case 19: return std::abs(cam.yaw - initial_camera.yaw) > .01;
                case 20: return (cam.target - initial_camera.target).norm() > .01;
                case 21: return cam.dist < initial_camera.dist;
                case 22: return selected != -1;
                case 23:
                    if (selected >= 0)
                        return (cam.target - scene.obstacles[std::size_t(selected)].pose.t.cast<float>()).norm() < 1e-5;
                    if (selected <= -2 && job.metrics.min_clearance_pose)
                        return (cam.target - compose(*job.metrics.min_clearance_pose,
                            scene.body[std::size_t(-selected-2)].pose).t.cast<float>()).norm() < 1e-4;
                    return false;
                case 24: return (cam.target - initial_camera.target).norm() < 1e-5 &&
                    std::abs(cam.yaw - initial_camera.yaw) < 1e-5;
                }
                return false;
            };
        }
        if (!shot.empty() || !capture_dir.empty() || !performance_report.empty()) {
            scene.config.time_limit = 0;
            for (std::uint64_t seed = 42; seed < 50; seed++) {
                scene.config.seed = seed;
                refresh_hash(scene);
                start_plan(scene, job, async);
                while (async.running.load())
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                reap(async);
                bool solved = false;
                {
                    std::lock_guard lk(job.m);
                    solved = job.result && job.result->status == SolveStatus::Solved;
                }
                if (solved)
                    break;
            }
            scrub = .4f;
        }
        int rendered = 0;
        bool dragging_object = false;
        double drag_plane_z = 0;
        Vec3 drag_offset = Vec3::Zero();
        std::vector<double> frame_times;
        while (!glfwWindowShouldClose(w)) {
            const auto frame_start = std::chrono::steady_clock::now();
            glfwPollEvents();
            if ((!capture_dir.empty() || !performance_report.empty()) && frames > 0)
                scrub = frames <= 1 ? 0.f : float(rendered) / float(frames - 1);
            int ww, hh;
            glfwGetFramebufferSize(w, &ww, &hh);
            if (ww < 1 || hh < 1)
                continue;
            ImGui_ImplOpenGL3_NewFrame();
            ImGui_ImplGlfw_NewFrame();
            if (active_replay) {
                io.AddFocusEvent(true);
                try { replay.frame(io); }
                catch (const std::exception &error) {
                    write_json({{"passed", false}, {"checks", replay.completed},
                        {"error", error.what()}, {"selected", selected}}, interaction_report);
                    save_ppm((interaction_report + ".ppm").c_str(), ww, hh);
                    throw;
                }
                replay.controls.clear();
            }
            ImGui::NewFrame();
            float uw = ImGui::GetIO().DisplaySize.x, uh = ImGui::GetIO().DisplaySize.y;
            float sx = float(ww) / uw, sy = float(hh) / uh;
            ImGui::SetNextWindowPos(ImVec2(14, 14), ImGuiCond_Always);
            ImGui::SetNextWindowSize(ImVec2(310, uh - 216), ImGuiCond_Always);
            ImGui::Begin("SCENE & ROBOT", nullptr,
                         ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize);
            ImGui::TextColored(ImVec4(.32f, .85f, .9f, 1), "PASSAGELAB");
            ImGui::TextDisabled("Rigid body motion planning");
            ImGui::Separator();
            if (ImGui::BeginCombo("Scene", scene.id.c_str())) {
                for (auto &d : demos)
                    if (ImGui::Selectable(d.id.c_str())) {
                        invalidate(scene, job, async);
                        scene = d;
                        {
                            std::lock_guard lock(job.m);
                            job.invalidated_result = false;
                        }
                        message = "Scene changed · previous result invalidated";
                    }
                ImGui::EndCombo();
            }
            input_string("Scene file", scene_path);
            if (ui_button("Load JSON", ImVec2(-1, 0))) {
                try {
                    invalidate(scene, job, async);
                    scene = load_scene(scene_path);
                    {
                        std::lock_guard lock(job.m);
                        job.invalidated_result = false;
                    }
                    message = "Scene loaded";
                } catch (const std::exception &e) {
                    message = e.what();
                }
            }
            if (ui_button("Save scene", ImVec2(-1, 0))) {
                try {
                    save_scene(scene, scene_path);
                    message = "Scene saved";
                } catch (const std::exception &e) {
                    message = e.what();
                }
            }
            input_string("Path file", load_path_name);
            if (ui_button("Load path JSON", ImVec2(-1, 0))) {
                try {
                    auto loaded = load_path(load_path_name);
                    ++job.request;
                    if (async.running)
                        async.thread.request_stop();
                    {
                        std::lock_guard lk(job.m);
                        job.result.reset();
                        job.optimized.reset();
                        job.optimized_metrics.reset();
                        job.metrics = {};
                        PlanningResult r;
                        r.status = SolveStatus::Error;
                        r.reason = "Imported path is an unverified preview";
                        r.path = std::move(loaded);
                        job.result = std::move(r);
                        job.imported = true;
                        job.invalidated_result = false;
                    }
                    message = "Path loaded · unverified; inspect scene hash and validate "
                              "before use";
                } catch (const std::exception &e) {
                    message = e.what();
                }
            }
            ImGui::Separator();
            ImGui::Text("BODY  /  %zu parts", scene.body.size());
            for (auto &part : scene.body) {
                ImGui::PushID(part.id.c_str());
                ImGui::Text("%s · %s", part.id.c_str(),
                            part.shape == Shape::Box ? "Box" : "Sphere");
                Pose editedPart = pose(part.pose, "Local pose");
                if ((editedPart.t - part.pose.t).norm() > 0 ||
                    (editedPart.q.coeffs() - part.pose.q.coeffs()).norm() > 0) {
                    part.pose = editedPart;
                    invalidate(scene, job, async);
                }
                if (part.shape == Shape::Box) {
                    if (drag_vector3("Half size", part.half.data(), .005, .01, 10)) {
                        invalidate(scene, job, async);
                    }
                } else if (drag_double("Radius", &part.radius, .005, .01, 10)) {
                    invalidate(scene, job, async);
                }
                ImGui::PopID();
            }
            if (ImGui::CollapsingHeader("Start / goal poses", ImGuiTreeNodeFlags_DefaultOpen)) {
                Pose a = pose(scene.start, "START"), b = pose(scene.goal, "GOAL");
                if ((a.t - scene.start.t).norm() > 0 ||
                    (a.q.coeffs() - scene.start.q.coeffs()).norm() > 0 ||
                    (b.t - scene.goal.t).norm() > 0 ||
                    (b.q.coeffs() - scene.goal.q.coeffs()).norm() > 0) {
                    scene.start = a;
                    scene.goal = b;
                    invalidate(scene, job, async);
                }
            }
            ImGui::Separator();
            if (ImGui::CollapsingHeader("Obstacles / aperture", ImGuiTreeNodeFlags_DefaultOpen)) {
                for (size_t i = 0; i < scene.obstacles.size(); i++) {
                    auto &o = scene.obstacles[i];
                    ImGui::PushID(int(i));
                    ImGui::Text("%s", o.id.c_str());
                    bool changed = false;
                    if (o.shape == Shape::Box)
                        changed |= drag_vector3("Half size", o.half.data(), .005, .01, 20);
                    else
                        changed |= drag_double("Radius", &o.radius, .005, .01, 20);
                    Pose edited = pose(o.pose, "Obstacle pose");
                    if ((edited.t - o.pose.t).norm() > 0 ||
                        (edited.q.coeffs() - o.pose.q.coeffs()).norm() > 0) {
                        o.pose = edited;
                        changed = true;
                    }
                    if (changed)
                        invalidate(scene, job, async);
                    ImGui::PopID();
                }
            }
            if (ImGui::CollapsingHeader("Planner & validation")) {
                if (ui_checkbox("Planar XY (fixed Z / yaw only)", &scene.config.planar_xy))
                    invalidate(scene, job, async);
                if (scene.config.planar_xy)
                    ImGui::TextWrapped("Anchored to start height and tilt; goal must lie in the same motion space.");
                int its = int(scene.config.max_iterations);
                if (ImGui::SliderInt("Iteration budget", &its, 100, 100000)) {
                    scene.config.max_iterations = its;
                    invalidate(scene, job, async);
                }
                std::uint64_t seed = scene.config.seed;
                if (ImGui::InputScalar("Seed", ImGuiDataType_U64, &seed)) {
                    scene.config.seed = seed;
                    invalidate(scene, job, async);
                }
                int shortcut = int(scene.config.shortcut_attempts);
                if (ImGui::SliderInt("Shortcut attempts", &shortcut, 0, 1000)) {
                    scene.config.shortcut_attempts = std::uint64_t(shortcut);
                    invalidate(scene, job, async);
                }
                double ext = scene.config.extension,
                       step = scene.config.validation.translation_step,
                       rot = scene.config.validation.rotation_step,
                       surface = scene.config.validation.surface_step,
                       weight = scene.config.rotation_weight, bias = scene.config.goal_bias,
                       margin = scene.config.validation.margin, seconds = scene.config.time_limit;
                if (drag_double("Extension (m)", &ext, .01, .01, 10)) {
                    scene.config.extension = ext;
                    invalidate(scene, job, async);
                }
                if (drag_double("Translation step (m)", &step, .005, .005, 1)) {
                    scene.config.validation.translation_step = step;
                    invalidate(scene, job, async);
                }
                if (drag_double("Rotation step (rad)", &rot, .005, .005, 1)) {
                    scene.config.validation.rotation_step = rot;
                    invalidate(scene, job, async);
                }
                if (drag_double("Surface step (m)", &surface, .005, .005, 1)) {
                    scene.config.validation.surface_step = surface;
                    invalidate(scene, job, async);
                }
                if (drag_double("Rotation weight", &weight, .01, .001, 10)) {
                    scene.config.rotation_weight = weight;
                    invalidate(scene, job, async);
                }
                if (drag_double("Goal bias", &bias, .005, 0, 1)) {
                    scene.config.goal_bias = bias;
                    invalidate(scene, job, async);
                }
                if (drag_double("Margin (m)", &margin, .001, 0, 1)) {
                    scene.config.validation.margin = margin;
                    invalidate(scene, job, async);
                }
                if (drag_double("Time limit (s, 0 = none)", &seconds, .05, 0, 600)) {
                    scene.config.time_limit = seconds;
                    invalidate(scene, job, async);
                }
            }
            ImGui::TextWrapped("%s", message.c_str());
            ImGui::End();

            ImGui::SetNextWindowPos(ImVec2(uw - 310, 14), ImGuiCond_Always);
            ImGui::SetNextWindowSize(ImVec2(296, uh - 216), ImGuiCond_Always);
            ImGui::Begin("INSPECTOR", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize);
            bool running = async.running.load();
            ImGui::TextColored(ImVec4(.32f, .85f, .9f, 1), "PLANNING");
            ImGui::BeginDisabled(running);
            if (ui_button("Plan path", ImVec2(132, 34))) {
                if (start_plan(scene, job, async))
                    message = "Planning…";
            }
            ImGui::EndDisabled();
            ImGui::SameLine();
            bool has_solved_path = false;
            {
                std::lock_guard lock(job.m);
                has_solved_path =
                    job.result && job.result->status == SolveStatus::Solved && !job.imported;
            }
            ImGui::BeginDisabled(running || !has_solved_path);
            if (ui_button("Shortcut", ImVec2(112, 34))) {
                if (start_optimization(scene, job, async))
                    message = "Shortcut optimization…";
            }
            ImGui::EndDisabled();
            running = async.running.load();
            ImGui::SameLine();
            if (running) {
                if (ui_button("Cancel", ImVec2(112, 34))) {
                    async.thread.request_stop();
                    message = "Cancellation requested";
                }
                if (async.optimizing.load()) {
                    ImGui::TextUnformatted("Shortcut optimization running");
                } else {
                    auto done = job.iterations.load();
                    auto limit = scene.config.max_iterations;
                    ImGui::Text("Iteration %llu / %llu", (unsigned long long)done,
                                (unsigned long long)limit);
                    ImGui::ProgressBar(limit ? float(done) / float(limit) : 0.f, ImVec2(-1, 0));
                }
            }
            ImGui::Separator();
            ImGui::TextColored(ImVec4(.32f, .85f, .9f, 1), "DISPLAY");
            ui_checkbox("World axes", &axes);
            ui_checkbox("Wireframe", &wire);
            ui_checkbox("Search tree", &tree);
            ui_checkbox("Pose ghosts", &ghosts);
            std::optional<Pose> search_pose;
            if (tree) {
                std::lock_guard lock(job.m);
                const auto &edges = job.result ? job.result->edges : job.live_edges;
                ImGui::Text("Search edges: %zu / 4000", edges.size());
                ImGui::TextWrapped("Orange: start / violet: goal / red: rejected");
                if (!edges.empty()) {
                    ui_checkbox("Inspect search motion", &inspect_search);
                    if (inspect_search) {
                        search_edge_index = std::clamp(search_edge_index, 0, int(edges.size()) - 1);
                        ImGui::SliderInt("Edge", &search_edge_index, 0, int(edges.size()) - 1);
                        ImGui::SliderFloat("Edge position", &search_edge_u, 0, 1);
                        const auto &edge = edges[search_edge_index];
                        search_pose = interpolate(edge.from, edge.to, search_edge_u);
                        ImGui::TextUnformatted(edge.accepted ? "Passed discrete check" : "Rejected / incomplete");
                        play = false;
                    }
                }
            }
            ImGui::Separator();
            ImGui::Text("Selected: %s",
                        selected == -1 ? "none"
                                       : (selected >= 0 && selected < int(scene.obstacles.size())
                                              ? scene.obstacles[size_t(selected)].id.c_str()
                                              : "robot component"));
            if (ui_button("Focus selection")) {
                if (selected >= 0 && selected < int(scene.obstacles.size()))
                    cam.target = scene.obstacles[size_t(selected)].pose.t.cast<float>();
                else if (selected <= -2 && size_t(-selected - 2) < scene.body.size()) {
                    Pose focus_pose = scene.start;
                    if (search_pose) focus_pose = *search_pose;
                    else {
                        std::lock_guard lock(job.m);
                        if (job.result && (job.result->status == SolveStatus::Solved || job.imported)) {
                            const auto &path = optimized_view && job.optimized ? *job.optimized : job.result->path;
                            if (!path.poses.empty()) {
                                const double x = scrub * (path.poses.size() - 1);
                                const auto i = std::min(std::size_t(x), path.poses.size() - 1);
                                focus_pose = interpolate(path.poses[i], path.poses[std::min(i + 1, path.poses.size() - 1)], x - i);
                            }
                        }
                    }
                    cam.target = compose(focus_pose, scene.body[size_t(-selected - 2)].pose).t.cast<float>();
                }
                else
                    cam.target = scene.start.t.cast<float>();
            }
            if (ui_button("Reset camera"))
                reset_camera(cam, scene);
            ImGui::Separator();
            ImGui::Text("ORIENTATION");
            ImGui::Text("Drag: orbit   Shift+drag: pan");
            ImGui::Text("Wheel: zoom   Click: pick");
            ImGui::Separator();
            ImGui::TextColored(ImVec4(.95f, .7f, .32f, 1), "DISCRETE VALIDATION");
            ImGui::Text("Translation step: %.3f m", scene.config.validation.translation_step);
            ImGui::Text("Rotation step: %.3f rad", scene.config.validation.rotation_step);
            ImGui::Text("Surface step: %.3f m", scene.config.validation.surface_step);
            ImGui::TextWrapped("Sampled checks do not certify continuous collision avoidance.");
            {
                std::lock_guard lk(job.m);
                if (job.result) {
                    ImGui::Separator();
                    ImGui::Text("STATUS  %s", job.imported ? "IMPORTED PREVIEW · UNVERIFIED"
                                                           : to_string(job.result->status).c_str());
                    ImGui::TextWrapped("%s", job.result->reason.c_str());
                    if (!job.imported) {
                        ImGui::Text("iterations %llu  nodes %llu",
                                    (unsigned long long)job.result->iterations,
                                    (unsigned long long)job.result->nodes);
                        ImGui::Text("search %.3fs  total %.3fs", job.result->search_seconds,
                                    job.result->total_seconds);
                    }
                    if (job.result->status == SolveStatus::Solved) {
                        ImGui::Text("Path samples: %llu", (unsigned long long)job.metrics.samples);
                        ImGui::Text("Original length: %.3f m", job.metrics.translation);
                        ImGui::Text("Original rotation: %.1f deg",
                                    job.metrics.rotation * 57.2957795);
                        if (job.metrics.min_sampled_clearance)
                            ImGui::Text("Sampled clearance min: %.4f m",
                                        *job.metrics.min_sampled_clearance);
                        if (job.optimized_metrics) {
                            ImGui::Separator();
                            ImGui::Text("Optimized length: %.3f m",
                                        job.optimized_metrics->translation);
                            ImGui::Text("Optimized rotation: %.1f deg",
                                        job.optimized_metrics->rotation * 57.2957795);
                            if (job.optimized_metrics->min_sampled_clearance)
                                ImGui::Text("Optimized clearance: %.4f m",
                                            *job.optimized_metrics->min_sampled_clearance);
                        }
                    }
                }
            }
            ImGui::End();

            ImGui::SetNextWindowPos(ImVec2(330, uh - 194), ImGuiCond_Always);
            ImGui::SetNextWindowSize(ImVec2(uw - 640, 180), ImGuiCond_Always);
            ImGui::Begin("PLAYBACK & EXPORT", nullptr,
                         ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize);
            std::optional<Path> shown;
            std::optional<PlanningResult> report_copy;
            PathMetrics report_metrics;
            std::optional<PathMetrics> report_optimized_metrics;
            bool imported_path = false;
            {
                std::lock_guard lk(job.m);
                if (job.result && (job.result->status == SolveStatus::Solved || job.imported))
                    shown = (optimized_view && job.optimized) ? *job.optimized : job.result->path;
                if (job.result)
                    report_copy = job.result;
                report_metrics = job.metrics;
                report_optimized_metrics = job.optimized_metrics;
                imported_path = job.imported;
                if (job.result) {
                    if (job.optimized &&
                        ui_button(optimized_view ? "Show original" : "Show optimized"))
                        optimized_view = !optimized_view;
                    if (job.optimized)
                        ImGui::SameLine();
                    ui_checkbox("Play", &play);
                    ImGui::SameLine();
                    if (ui_button("Step back"))
                        scrub = std::max(0.f, scrub - .02f);
                    ImGui::SameLine();
                    if (ui_button("Step forward"))
                        scrub = std::min(1.f, scrub + .02f);
                    ImGui::SameLine();
                    ImGui::Text("%s", shown ? shown->source.c_str() : "original");
                    if (shown && shown->poses.size() > 1) {
                        ImGui::SliderFloat("Path position", &scrub, 0, 1, "%.3f");
                        ImGui::Text("Pose %zu / %zu",
                                    size_t(scrub * float(shown->poses.size() - 1)) + 1,
                                    shown->poses.size());
                    }
                    const auto &metrics = optimized_view && job.optimized_metrics
                                              ? *job.optimized_metrics : job.metrics;
                    if (!job.imported && metrics.complete && metrics.min_clearance_position &&
                        ui_button("Inspect minimum sampled clearance")) {
                        scrub = float(*metrics.min_clearance_position);
                        play = false;
                        inspect_search = false;
                        search_pose.reset();
                    }
                }
            }
            if (imported_path) {
                ImGui::SameLine();
                if (ui_button("Revalidate path")) {
                    if (!start_revalidation(scene, job, async))
                        message = "Wait for current worker to finish before revalidation";
                }
            }
            if (play && shown)
                scrub = std::fmod(scrub + ImGui::GetIO().DeltaTime * .08f, 1.f);
            ImGui::SameLine();
            ImGui::SetNextItemWidth(115);
            input_string("##sceneout", scene_path);
            ImGui::SameLine();
            if (ui_button("Export scene"))
                try {
                    save_scene(scene, scene_path);
                } catch (const std::exception &e) {
                    message = e.what();
                }
            ImGui::SameLine();
            ImGui::SetNextItemWidth(115);
            input_string("##pathout", path_path);
            ImGui::SameLine();
            if (ui_button("Export path") && shown)
                try {
                    save_path(*shown, path_path);
                } catch (const std::exception &e) {
                    message = e.what();
                }
            ImGui::SameLine();
            if (ui_button("Export report") && report_copy)
                try {
                    write_json(report_json(scene, scene.config, *report_copy, report_metrics,
                                           report_optimized_metrics),
                               report_path);
                } catch (const std::exception &e) {
                    message = e.what();
                }
            ImGui::End();

            glViewport(int(330 * sx), int(180 * sy), std::max(1, int((uw - 640) * sx)),
                       std::max(1, int((uh - 194) * sy)));
            glEnable(GL_DEPTH_TEST);
            glEnable(GL_MULTISAMPLE);
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            glClearColor(.035f, .05f, .075f, 1);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            Mat4 vp = mul(
                perspective(.82f, float(std::max(1.f, uw - 640)) / float(std::max(1.f, uh - 194)),
                            .05f, 200),
                view(cam));
            if (active_replay) {
                replay.controls["Viewport orbit"] = ImVec2(uw * .5f, 90);
                replay.controls["Viewport pan"] = ImVec2(uw * .5f, 90);
                replay.controls["Viewport zoom"] = ImVec2(uw * .5f, 90);
                if (!scene.obstacles.empty()) {
                    Eigen::Vector4f world;
                    world << scene.obstacles.front().pose.t.cast<float>(), 1.f;
                    const Eigen::Vector4f clip = eigen(vp) * world;
                    replay.controls["Viewport pick"] = ImVec2(
                        330 + (clip.x() / clip.w() + 1) * .5f * (uw - 640),
                        14 + (1 - clip.y() / clip.w()) * .5f * (uh - 194));
                }
            }
            glUseProgram(prog);
            if (axes) {
                line(vp, Vec3::Zero(), Vec3(2, 0, 0), {.95f, .28f, .25f, 1});
                line(vp, Vec3::Zero(), Vec3(0, 2, 0), {.3f, .9f, .42f, 1});
                line(vp, Vec3::Zero(), Vec3(0, 0, 2), {.25f, .62f, 1, 1});
            }
            Vec3 extent = scene.physical.max - scene.physical.min;
            Vec3 bound_center = (scene.physical.max + scene.physical.min) * .5;
            Pose floor_pose;
            floor_pose.t = Vec3(bound_center.x(), bound_center.y(), scene.physical.min.z() - .015);
            cube(vp, floor_pose, Vec3(extent.x(), extent.y(), .03), {.10f, .15f, .20f, 1}, true);
            std::array<Vec3, 8> bound_corners;
            for (int i = 0; i < 8; ++i)
                bound_corners[size_t(i)] =
                    Vec3((i & 1) ? scene.physical.max.x() : scene.physical.min.x(),
                         (i & 2) ? scene.physical.max.y() : scene.physical.min.y(),
                         (i & 4) ? scene.physical.max.z() : scene.physical.min.z());
            for (int i = 0; i < 8; ++i)
                for (int axis = 0; axis < 3; ++axis) {
                    int j = i ^ (1 << axis);
                    if (i < j)
                        line(vp, bound_corners[size_t(i)], bound_corners[size_t(j)],
                             {.27f, .39f, .48f, .7f});
                }
            for (size_t i = 0; i < scene.obstacles.size(); i++) {
                auto &o = scene.obstacles[i];
                auto color = i == size_t(selected) ? Eigen::Vector4f(.96f, .37f, .24f, .9f)
                                                   : Eigen::Vector4f(.31f, .48f, .59f, .9f);
                if (o.shape == Shape::Sphere)
                    sphere(vp, o.pose, o.radius, color, wire);
                else
                    cube(vp, o.pose, o.half * 2, color, wire);
            }
            Pose current = scene.start;
            bool haspath = false;
            std::string current_path_key;
            {
                std::lock_guard lk(job.m);
                if (job.result && (job.result->status == SolveStatus::Solved || job.imported)) {
                    Path const *path = &job.result->path;
                    if (optimized_view && job.optimized)
                        path = &*job.optimized;
                    current_path_key = path->scene_hash + ":" +
                                       std::to_string(path->scene_revision) + ":" +
                                       std::to_string(path->poses.size()) + ":" + path->source;
                    haspath = true;
                    if (path->poses.size() > 1) {
                        double x = scrub * (path->poses.size() - 1);
                        size_t i = size_t(x);
                        current = interpolate(path->poses[i],
                                              path->poses[std::min(i + 1, path->poses.size() - 1)],
                                              x - i);
                        for (size_t i = 1; i < path->poses.size(); i++)
                            line(vp, path->poses[i - 1].t, path->poses[i].t, {.26f, .88f, .88f, 1});
                        if (ghosts) {
                            size_t stride = std::max<size_t>(
                                1, path->poses.size() / size_t(std::max(1, ghost_count)));
                            for (size_t i = 0; i < path->poses.size(); i += stride)
                                for (auto &part : scene.body) {
                                    Pose gp = compose(path->poses[i], part.pose);
                                    if (part.shape == Shape::Sphere)
                                        sphere(vp, gp, part.radius, {.3f, .8f, .88f, .22f}, true);
                                    else
                                        cube(vp, gp, part.half * 2, {.3f, .8f, .88f, .22f}, true);
                                }
                        }
                    }
                } else
                    current = scene.start;
            }
            bool check_invalid_endpoint = false;
            if (search_pose)
                current = *search_pose;
            {
                std::lock_guard lk(job.m);
                if (job.result && job.result->status == SolveStatus::InvalidStart) {
                    current = scene.start;
                    check_invalid_endpoint = true;
                } else if (job.result && job.result->status == SolveStatus::InvalidGoal) {
                    current = scene.goal;
                    check_invalid_endpoint = true;
                }
            }
            static std::uint64_t queried_revision = 0, queried_request = 0;
            static std::string queried_path_key;
            static Pose queried_pose;
            static bool query_cache_valid = false, queried_path = false, queried_optimized = false;
            static std::optional<StateResult> playback_state;
            const auto request = job.request.load();
            bool pose_changed = !query_cache_valid || (current.t - queried_pose.t).norm() > 1e-10 ||
                                (current.q.coeffs() - queried_pose.q.coeffs()).norm() > 1e-10;
            if (!query_cache_valid || queried_revision != scene.revision ||
                queried_request != request || queried_path != haspath ||
                queried_path_key != current_path_key || queried_optimized != optimized_view ||
                pose_changed) {
                Validator validator(scene);
                playback_state = validator.state(current, scene.config.validation, true);
                queried_revision = scene.revision;
                queried_request = request;
                queried_path_key = current_path_key;
                queried_pose = current;
                queried_path = haspath;
                queried_optimized = optimized_view;
                query_cache_valid = true;
            }
            std::string fail_body, fail_obstacle;
            {
                std::lock_guard lk(job.m);
                if (job.result && job.result->status != SolveStatus::Solved) {
                    fail_body = job.result->verification.failure.query.body_id;
                    fail_obstacle = job.result->verification.failure.query.obstacle_id;
                }
            }
            if (playback_state && !playback_state->valid) {
                fail_body = playback_state->query.body_id;
                fail_obstacle = playback_state->query.obstacle_id;
            }
            auto draw_body_pose = [&](const Pose &body_pose, const Eigen::Vector4f &color) {
                for (const auto &part : scene.body) {
                    Pose world = compose(body_pose, part.pose);
                    if (part.shape == Shape::Sphere)
                        sphere(vp, world, part.radius, color, true);
                    else
                        cube(vp, world, part.half * 2, color, true);
                }
            };
            draw_body_pose(scene.start, {.25f, .88f, .46f, .42f});
            draw_body_pose(scene.goal, {.98f, .63f, .24f, .42f});
            auto draw_pose_axes = [&](const Pose &p) {
                line(vp, p.t, p.t + p.q * Vec3(.5, 0, 0), {1, .30f, .25f, 1});
                line(vp, p.t, p.t + p.q * Vec3(0, .5, 0), {.32f, .9f, .45f, 1});
                line(vp, p.t, p.t + p.q * Vec3(0, 0, .5), {.28f, .62f, 1, 1});
            };
            draw_pose_axes(scene.start);
            draw_pose_axes(scene.goal);
            draw_pose_axes(current);
            if (tree) {
                std::lock_guard lk(job.m);
                const auto &edges = job.result ? job.result->edges : job.live_edges;
                    for (const auto &e : edges)
                        line(vp, e.from.t, e.to.t,
                             !e.accepted ? Eigen::Vector4f(.95f, .2f, .2f, .55f) :
                             e.start_tree ? Eigen::Vector4f(.8f, .54f, .26f, .55f)
                                          : Eigen::Vector4f(.47f, .55f, .85f, .55f));
            }
            for (auto &part : scene.body) {
                Pose world = compose(current, part.pose);
                bool fail = part.id == fail_body;
                if (part.shape == Shape::Sphere)
                    sphere(vp, world, part.radius,
                           fail ? Eigen::Vector4f(1, .15f, .12f, 1)
                                : Eigen::Vector4f(.18f, .78f, .72f, 1),
                           wire);
                else
                    cube(vp, world, part.half * 2,
                         fail ? Eigen::Vector4f(1, .15f, .12f, 1)
                              : Eigen::Vector4f(.18f, .78f, .72f, 1),
                         wire);
            }
            for (auto &o : scene.obstacles)
                if (o.id == fail_obstacle) {
                    if (o.shape == Shape::Sphere)
                        sphere(vp, o.pose, o.radius, {1, .12f, .08f, 1}, true);
                    else
                        cube(vp, o.pose, o.half * 2, {1, .12f, .08f, 1}, true);
                }
            if (playback_state && playback_state->query.on_body &&
                playback_state->query.on_obstacle)
                line(vp, *playback_state->query.on_body, *playback_state->query.on_obstacle,
                     {1, .83f, .2f, 1});
            {
                std::lock_guard lk(job.m);
                if (job.result && job.result->status != SolveStatus::Solved) {
                    auto &q = job.result->verification.failure.query;
                    if (q.on_body && q.on_obstacle)
                        line(vp, *q.on_body, *q.on_obstacle, {1, .2f, .1f, 1});
                }
            }
            if (ImGui::IsMouseDragging(ImGuiMouseButton_Left) && !ImGui::GetIO().WantCaptureMouse &&
                !dragging_object) {
                ImVec2 d = ImGui::GetIO().MouseDelta;
                if (ImGui::GetIO().KeyShift) {
                    cam.target.x() -= d.x * .008f;
                    cam.target.z() += d.y * .008f;
                } else {
                    cam.yaw -= d.x * .008f;
                    cam.pitch = std::clamp(cam.pitch + d.y * .008f, -1.45f, 1.45f);
                }
            }
            if (!ImGui::GetIO().WantCaptureMouse && ImGui::GetIO().MouseWheel != 0)
                cam.dist =
                    std::clamp(cam.dist * std::pow(.88f, ImGui::GetIO().MouseWheel), 1.f, 80.f);
            if (ImGui::IsMouseClicked(ImGuiMouseButton_Left) && !ImGui::GetIO().WantCaptureMouse) {
                auto [ro, rd] =
                    mouse_ray(vp, ImGui::GetIO().MousePos.x - 330, ImGui::GetIO().MousePos.y - 14,
                              int(uw - 640), int(uh - 194));
                double nearest = std::numeric_limits<double>::infinity();
                int hit = -1;
                for (size_t i = 0; i < scene.obstacles.size(); i++) {
                    auto &o = scene.obstacles[i];
                    auto t = o.shape == Shape::Box ? ray_box(ro, rd, o)
                                                   : ray_sphere(ro, rd, o.pose.t, o.radius);
                    if (t && *t < nearest) {
                        nearest = *t;
                        hit = int(i);
                    }
                }
                for (size_t i = 0; i < scene.body.size(); i++) {
                    auto &p = scene.body[i];
                    Pose world = compose(current, p.pose);
                    auto t = p.shape == Shape::Box ? ray_box(ro, rd, p, current)
                                                   : ray_sphere(ro, rd, world.t, p.radius);
                    if (t && *t < nearest) {
                        nearest = *t;
                        hit = -2 - int(i);
                    }
                }
                selected = hit;
                dragging_object = hit != -1;
                if (dragging_object) {
                    Vec3 center;
                    if (hit >= 0)
                        center = scene.obstacles[size_t(hit)].pose.t;
                    else
                        center = compose(current, scene.body[size_t(-hit - 2)].pose).t;
                    drag_plane_z = center.z();
                    drag_offset = center - (ro + rd * nearest);
                }
            }
            if (ImGui::IsMouseDragging(ImGuiMouseButton_Left) && dragging_object &&
                !ImGui::GetIO().WantCaptureMouse) {
                auto [ro, rd] =
                    mouse_ray(vp, ImGui::GetIO().MousePos.x - 330, ImGui::GetIO().MousePos.y - 14,
                              int(uw - 640), int(uh - 194));
                if (std::abs(rd.z()) > 1e-8) {
                    Vec3 pos = ro + rd * ((drag_plane_z - ro.z()) / rd.z()) + drag_offset;
                    if (selected >= 0) {
                        scene.obstacles[size_t(selected)].pose.t = pos;
                    } else {
                        auto &part = scene.body[size_t(-selected - 2)];
                        part.pose.t = current.q.conjugate() * (pos - current.t);
                    }
                    invalidate(scene, job, async);
                }
            }
            if (ImGui::IsMouseReleased(ImGuiMouseButton_Left))
                dragging_object = false;
            {
                std::lock_guard lk(job.m);
                if (job.imported && job.result) {
                    bool stale = job.result->path.scene_hash != scene.hash ||
                                 job.result->path.scene_revision != scene.revision;
                    auto *draw = ImGui::GetForegroundDrawList();
                    draw->AddText(ImVec2(330, 22),
                                  stale ? IM_COL32(255, 150, 95, 255)
                                        : IM_COL32(255, 205, 110, 255),
                                  stale ? "IMPORTED PATH · STALE / UNVERIFIED"
                                        : "IMPORTED PATH · UNVERIFIED");
                }
            }
            auto *overlay = ImGui::GetForegroundDrawList();
            bool show_invalidated = false;
            {
                std::lock_guard lock(job.m);
                show_invalidated = job.invalidated_result;
            }
            if (show_invalidated)
                overlay->AddText(ImVec2(330, 64), IM_COL32(255, 188, 102, 255),
                                 "Inputs changed - previous result invalidated. Plan again.");
            overlay->AddText(ImVec2(330, 8), IM_COL32(145, 183, 203, 255),
                             "PHYSICAL BOUNDS · Z UP");
            overlay->AddText(ImVec2(330, 24), IM_COL32(195, 216, 227, 255),
                             "START: green wireframe    GOAL: amber wireframe");
            if (playback_state) {
                auto *draw = ImGui::GetForegroundDrawList();
                std::string q = playback_state->valid ? "Playback state: valid"
                                                      : "Playback state: collision / uncertain";
                if (playback_state->query.distance)
                    q += " · clearance " + std::to_string(*playback_state->query.distance) + " m";
                draw->AddText(ImVec2(330, 44),
                              playback_state->valid ? IM_COL32(135, 230, 190, 255)
                                                    : IM_COL32(255, 115, 90, 255),
                              q.c_str());
            }
            ImGui::Render();
            ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
            if (!performance_report.empty()) {
                glFinish();
                if (rendered >= 10)
                    frame_times.push_back(std::chrono::duration<double, std::milli>(
                        std::chrono::steady_clock::now() - frame_start).count());
            }
            if (active_replay && replay.stage == replay.steps.size()) {
                write_json({{"passed", true}, {"checks", replay.completed}, {"scene", scene.id},
                    {"source_sha256", PL_SOURCE_HASH}, {"frames", rendered},
                    {"driver", reinterpret_cast<const char *>(glGetString(GL_RENDERER))},
                    {"opengl", reinterpret_cast<const char *>(glGetString(GL_VERSION))}}, interaction_report);
                if (!save_ppm((interaction_report + ".ppm").c_str(), ww, hh))
                    throw std::runtime_error("Could not capture interaction replay");
                break;
            }
            if (!shot.empty() && rendered == 0 && !save_ppm(shot.c_str(), ww, hh))
                throw std::runtime_error("Could not write screenshot");
            if (!capture_dir.empty()) {
                std::filesystem::create_directories(capture_dir);
                char name[64];
                std::snprintf(name, sizeof name, "frame%04d.ppm", rendered);
                auto file = std::filesystem::path(capture_dir) / name;
                if (!save_ppm(file.string().c_str(), ww, hh))
                    throw std::runtime_error("Could not write capture frame");
            }
            glfwSwapBuffers(w);
            ++rendered;
            if (active_replay) std::this_thread::sleep_for(std::chrono::milliseconds(1));
            if (!shot.empty())
                break;
            if ((smoke && rendered >= 4) || (frames >= 0 && rendered >= frames))
                break;
        }
        if (async.thread.joinable()) {
            async.thread.request_stop();
            while (async.running.load())
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            reap(async);
        }
        active_replay = nullptr;
        if (!performance_report.empty() && !frame_times.empty()) {
            std::sort(frame_times.begin(), frame_times.end());
            const double median = frame_times[frame_times.size() / 2];
            write_json({{"scene", scene.id}, {"source_sha256", PL_SOURCE_HASH},
                {"frames_after_warmup", frame_times.size()}, {"median_render_ms", median},
                {"max_render_ms", frame_times.back()}, {"median_target_ms", 33.333333},
                {"target_passed", median <= 33.333333}, {"width", 1440}, {"height", 900},
                {"measurement", "CPU frame start through glFinish, vsync disabled, first 10 excluded"},
                {"gpu", reinterpret_cast<const char *>(glGetString(GL_RENDERER))},
                {"opengl", reinterpret_cast<const char *>(glGetString(GL_VERSION))}}, performance_report);
        }
        return 0;
    } catch (const std::exception &e) {
        std::fprintf(stderr, "PassageLab desktop: %s\n", e.what());
        return 2;
    }
}

#include <windows.h>
#include <bcrypt.h>
#include "dependency_versions.hpp"
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <passagelab/io.hpp>
#include <sstream>
#include <thread>


namespace pl {
namespace {
Json vec(const Vec3 &v) {
    return Json::array({v.x(), v.y(), v.z()});
}
Vec3 read_vec(const Json &j) {
    if (!j.is_array() || j.size() != 3)
        throw std::runtime_error("Expected finite vector [x,y,z]");
    Vec3 v(j.at(0).get<double>(), j.at(1).get<double>(), j.at(2).get<double>());
    if (!v.allFinite())
        throw std::runtime_error("Nonfinite vector");
    return v;
}
Json pose_json(const Pose &p) {
    return {{"position", vec(p.t)}, {"quaternion", {p.q.x(), p.q.y(), p.q.z(), p.q.w()}}};
}
Pose read_pose(const Json &j) {
    Pose p;
    p.t = read_vec(j.at("position"));
    const auto &q = j.at("quaternion");
    if (!q.is_array() || q.size() != 4)
        throw std::runtime_error("Expected quaternion [x,y,z,w]");
    p.q = Quat(q.at(3).get<double>(), q.at(0).get<double>(), q.at(1).get<double>(),
               q.at(2).get<double>());
    if (!p.q.coeffs().allFinite() || std::abs(p.q.norm() - 1) > 1e-3)
        throw std::runtime_error("Quaternion must be unit length (normalization tolerance 0.001)");
    p.q.normalize();
    return p;
}
Json bounds_json(const Bounds &b) {
    return {{"min", vec(b.min)}, {"max", vec(b.max)}};
}
Bounds read_bounds(const Json &j) {
    return {read_vec(j.at("min")), read_vec(j.at("max"))};
}
Json validation_json(const ValidationConfig &v) {
    return {{"translation_step", v.translation_step},
            {"rotation_step", v.rotation_step},
            {"surface_step", v.surface_step},
            {"margin", v.margin},
            {"tolerance", v.tolerance},
            {"max_samples", v.max_samples},
            {"use_bvh", v.use_bvh}};
}
double finite_number(const Json &j, const char *key) {
    double v = j.at(key).get<double>();
    if (!std::isfinite(v))
        throw std::runtime_error(std::string("Nonfinite ") + key);
    return v;
}
std::uint64_t count(const Json &j, const char *key) {
    const auto &v = j.at(key);
    if (!v.is_number_integer() || v.get<double>() < 0)
        throw std::runtime_error(std::string("Invalid unsigned integer ") + key);
    return v.get<std::uint64_t>();
}
ValidationConfig read_validation(const Json &j) {
    ValidationConfig v;
    v.translation_step = finite_number(j, "translation_step");
    v.rotation_step = finite_number(j, "rotation_step");
    v.surface_step = finite_number(j, "surface_step");
    v.margin = finite_number(j, "margin");
    v.tolerance = finite_number(j, "tolerance");
    v.max_samples = count(j, "max_samples");
    v.use_bvh = j.at("use_bvh").get<bool>();
    if (v.translation_step <= 0 || v.rotation_step <= 0 || v.surface_step <= 0 || v.margin < 0 ||
        v.tolerance <= 0 || v.max_samples < 2)
        throw std::runtime_error(
            "Invalid validation resolution, margin, tolerance or sample limit");
    return v;
}
PlannerConfig read_config(const Json &j) {
    PlannerConfig c;
    c.seed = count(j, "seed");
    c.max_iterations = count(j, "max_iterations");
    c.max_nodes = count(j, "max_nodes");
    c.max_queries = count(j, "max_queries");
    c.shortcut_attempts = count(j, "shortcut_attempts");
    c.time_limit = finite_number(j, "time_limit");
    c.extension = finite_number(j, "extension");
    c.rotation_weight = finite_number(j, "rotation_weight");
    c.goal_bias = finite_number(j, "goal_bias");
    c.validation = read_validation(j.at("validation"));
    const auto space = j.value("motion_space", std::string("se3"));
    if (space != "se3" && space != "planar-xy")
        throw std::runtime_error("Unsupported motion_space");
    c.planar_xy = space == "planar-xy";
    if (c.extension <= 0 || c.rotation_weight <= 0 || c.goal_bias < 0 || c.goal_bias > 1 ||
        c.max_nodes < 2)
        throw std::runtime_error("Invalid planner configuration");
    return c;
}
Json part_json(const Part &p) {
    return {{"id", p.id},
            {"shape", p.shape == Shape::Box ? "box" : "sphere"},
            {"half_extents", vec(p.half)},
            {"radius", p.radius},
            {"pose", pose_json(p.pose)}};
}
Part read_part(const Json &j) {
    Part p;
    p.id = j.at("id").get<std::string>();
    auto shape = j.at("shape").get<std::string>();
    if (shape != "box" && shape != "sphere")
        throw std::runtime_error("Unsupported shape: " + shape);
    p.shape = shape == "box" ? Shape::Box : Shape::Sphere;
    p.half = read_vec(j.at("half_extents"));
    p.radius = finite_number(j, "radius");
    p.pose = read_pose(j.at("pose"));
    if ((p.half.array() <= 0).any() || p.radius <= 0 || p.id.empty())
        throw std::runtime_error("Part needs a nonempty ID and positive dimensions");
    return p;
}
Json read_json(const std::filesystem::path &p) {
    std::ifstream f(p);
    if (!f)
        throw std::runtime_error("Cannot open " + p.string());
    return Json::parse(f); // Reject trailing documents or garbage as well as malformed JSON.
}
Json metric_json(const PathMetrics &m) {
    Json j = {{"translation_m", m.translation}, {"rotation_rad", m.rotation},
              {"objective_m", m.objective},     {"samples", m.samples},
              {"complete", m.complete},         {"min_sampled_clearance_m", nullptr}};
    if (m.min_sampled_clearance)
        j["min_sampled_clearance_m"] = *m.min_sampled_clearance;
    if (m.min_clearance_pose)
        j["min_clearance_pose"] = pose_json(*m.min_clearance_pose);
    if (m.complete && m.min_clearance_position)
        j["min_clearance_position"] = *m.min_clearance_position;
    return j;
}
} // namespace
Json config_json(const PlannerConfig &c) {
    Json j = {{"seed", c.seed},
            {"max_iterations", c.max_iterations},
            {"max_nodes", c.max_nodes},
            {"max_queries", c.max_queries},
            {"time_limit", c.time_limit},
            {"extension", c.extension},
            {"rotation_weight", c.rotation_weight},
            {"goal_bias", c.goal_bias},
            {"shortcut_attempts", c.shortcut_attempts},
            {"validation", validation_json(c.validation)}};
    // Omit the default to retain hashes of existing schema-1 SE(3) scenes.
    if (c.planar_xy)
        j["motion_space"] = "planar-xy";
    return j;
}
Json scene_json(const Scene &s) {
    Json j = {{"schema_version", 1},
              {"scene_id", s.id},
              {"scene_revision", s.revision},
              {"scene_hash", s.hash},
              {"units", "m"},
              {"axis_convention", "right-handed-z-up"},
              {"quaternion_order", "xyzw"},
              {"sampling_bounds", bounds_json(s.sampling)},
              {"physical_bounds", bounds_json(s.physical)},
              {"start_pose", pose_json(s.start)},
              {"goal_pose", pose_json(s.goal)},
              {"default_planning_config", config_json(s.config)}};
    j["moving_body"] = Json::array();
    j["obstacles"] = Json::array();
    for (const auto &p : s.body)
        j["moving_body"].push_back(part_json(p));
    for (const auto &p : s.obstacles)
        j["obstacles"].push_back(part_json(p));
    return j;
}
std::string scene_hash(const Scene &s) {
    Json j = scene_json(s);
    j.erase("scene_hash");
    j.erase("scene_revision");
    const std::string bytes = j.dump();
    unsigned char digest[32];
    NTSTATUS status = BCryptHash(BCRYPT_SHA256_ALG_HANDLE, nullptr, 0,
                                 reinterpret_cast<PUCHAR>(const_cast<char *>(bytes.data())),
                                 static_cast<ULONG>(bytes.size()), digest, 32);
    if (status < 0)
        throw std::runtime_error("SHA256 failed");
    std::ostringstream out;
    out << "sha256:" << std::hex << std::setfill('0');
    for (auto b : digest)
        out << std::setw(2) << int(b);
    return out.str();
}
void refresh_hash(Scene &s) {
    s.hash = scene_hash(s);
}
Scene scene_from_json(const Json &j) {
    if (!j.at("schema_version").is_number_integer() || j.at("schema_version") != 1 || j.at("units") != "m" ||
        j.at("axis_convention") != "right-handed-z-up" || j.at("quaternion_order") != "xyzw")
        throw std::runtime_error("Unsupported scene version or coordinate convention");
    Scene s;
    s.id = j.at("scene_id").get<std::string>();
    if (s.id.empty() || !j.at("moving_body").is_array() || !j.at("obstacles").is_array())
        throw std::runtime_error("Scene needs a nonempty ID and arrays for moving_body and obstacles");
    s.revision = count(j, "scene_revision");
    s.sampling = read_bounds(j.at("sampling_bounds"));
    s.physical = read_bounds(j.at("physical_bounds"));
    s.start = read_pose(j.at("start_pose"));
    s.goal = read_pose(j.at("goal_pose"));
    s.config = read_config(j.at("default_planning_config"));
    for (const auto &p : j.at("moving_body"))
        s.body.push_back(read_part(p));
    for (const auto &p : j.at("obstacles"))
        s.obstacles.push_back(read_part(p));
    auto errors = scene_errors(s);
    if (!errors.empty())
        throw std::runtime_error("Invalid scene: " + errors.front());
    refresh_hash(s); // Recompute from inputs. A stale stored hash never grants validity.
    return s;
}
Scene load_scene(const std::filesystem::path &p) {
    return scene_from_json(read_json(p));
}
void write_json(const Json &j, const std::filesystem::path &p) {
    if (p.has_parent_path())
        std::filesystem::create_directories(p.parent_path());
    std::ofstream f(p);
    if (!f)
        throw std::runtime_error("Cannot write " + p.string());
    f << std::setw(2) << j << '\n';
    if (!f)
        throw std::runtime_error("Write failed: " + p.string());
}
void save_scene(const Scene &s, const std::filesystem::path &p) {
    auto copy = s;
    refresh_hash(copy);
    write_json(scene_json(copy), p);
}
Json path_json(const Path &p) {
    Json j = {{"schema_version", 1},
              {"scene_hash", p.scene_hash},
              {"scene_revision", p.scene_revision},
              {"quaternion_order", "xyzw"},
              {"interpolation", "linear-translation-shortest-arc-slerp"},
              {"source", p.source},
              {"request_id", p.request_id},
              {"endpoint_translation_tolerance_m", 1e-8},
              {"endpoint_rotation_tolerance_rad", 1e-8},
              {"validation", validation_json(p.validation)},
              {"validation_claim", "Requires revalidation on load; discrete sampling only"},
              {"poses", Json::array()}};
    for (const auto &pose : p.poses)
        j["poses"].push_back(pose_json(pose));
    return j;
}
Path path_from_json(const Json &j) {
    if (!j.at("schema_version").is_number_integer() || j.at("schema_version") != 1 || j.at("quaternion_order") != "xyzw" ||
        j.at("interpolation") != "linear-translation-shortest-arc-slerp")
        throw std::runtime_error("Unsupported path version or interpolation");
    Path p;
    p.scene_hash = j.at("scene_hash").get<std::string>();
    p.scene_revision = count(j, "scene_revision");
    p.validation = read_validation(j.at("validation"));
    p.source = j.at("source").get<std::string>();
    p.request_id = j.value("request_id", std::string{});
    for (const char *key : {"endpoint_translation_tolerance_m", "endpoint_rotation_tolerance_rad"})
        if (j.contains(key) && finite_number(j, key) != 1e-8)
            throw std::runtime_error("Unsupported endpoint tolerance");
    if (!j.at("poses").is_array() || j.at("poses").empty())
        throw std::runtime_error("Path must contain poses");
    for (const auto &v : j.at("poses"))
        p.poses.push_back(read_pose(v));
    return p;
}
Path load_path(const std::filesystem::path &p) {
    return path_from_json(read_json(p));
}
void save_path(const Path &p, const std::filesystem::path &f) {
    write_json(path_json(p), f);
}
Json report_json(const Scene &s, const PlannerConfig &c, const PlanningResult &r,
                 const PathMetrics &m, const std::optional<PathMetrics> &opt) {
    MEMORYSTATUSEX mem{};
    mem.dwLength = sizeof(mem);
    GlobalMemoryStatusEx(&mem);
    char cpu[512] = {};
    GetEnvironmentVariableA("PROCESSOR_IDENTIFIER", cpu, 512);
    const auto now = std::chrono::system_clock::now();
    const auto ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
    Json j = {{"schema_version", 1},
              {"run_id", s.id + "-" + std::to_string(c.seed) + "-" + std::to_string(ms)},
              {"unix_time_ms", ms},
              {"software_version", PL_VERSION},
              {"git_commit", PL_COMMIT},
              {"source_sha256", PL_SOURCE_HASH},
              {"git_tracked_dirty", bool(PL_DIRTY)},
              {"build", "" PL_BUILD ""},
              {"compiler_msvc", _MSC_VER},
              {"os", "Windows"},
              {"cpu", cpu},
              {"logical_processors", std::thread::hardware_concurrency()},
              {"memory_bytes", mem.ullTotalPhys},
              {"dependency_baseline", "15b12b99a227d51e21c86dffd40385439eac87a0"},
              {"dependency_versions", Json::parse(PL_DEPENDENCIES_JSON)},
              {"scene_id", s.id},
              {"scene_hash", s.hash},
              {"scene_revision", s.revision},
              {"request_id", r.path.request_id},
              {"motion_space", c.planar_xy ? "planar-xy" : "se3"},
              {"parameters", config_json(c)},
              {"status", to_string(r.status)},
              {"reason", r.reason},
              {"nodes", r.nodes},
              {"iterations", r.iterations},
              {"search_seconds", r.search_seconds},
              {"validation_seconds", r.validation_seconds},
              {"total_seconds", r.total_seconds},
              {"queries",
               {{"states", r.queries.states},
                {"motions", r.queries.motions},
                {"narrow", r.queries.narrow},
                {"broad", r.queries.broad},
                {"samples", r.queries.samples}}},
              {"verification",
               {{"status", int(r.verification.status)},
                {"samples", r.verification.samples},
                {"reason", r.verification.reason},
                {"method", "discrete interpolation samples; no continuous safety certificate"}}},
              {"metrics", metric_json(m)}};
    if (opt)
        j["optimized_metrics"] = metric_json(*opt);
    else
        j["optimized_metrics"] = nullptr;
    j["verification"]["intervals"] = r.verification.intervals;
    j["verification"]["maximum_actual_steps"] = {
        {"translation_m", r.verification.translation_step},
        {"rotation_rad", r.verification.rotation_step},
        {"surface_m", r.verification.surface_step}};
    j["verification"]["failed_edge"] = r.verification.failed_edge
        ? Json(*r.verification.failed_edge) : Json(nullptr);
    j["verification"]["failed_u"] = r.verification.failed_u;
    j["verification"]["failure"] = {{"reason", r.verification.failure.reason},
        {"body_part_id", r.verification.failure.query.body_id},
        {"obstacle_id", r.verification.failure.query.obstacle_id}};
    j["verification"]["tolerance_policy"] = "absolute-length-plus-32-epsilon-scale-v1; contact-invalid";
    return j;
}
} // namespace pl

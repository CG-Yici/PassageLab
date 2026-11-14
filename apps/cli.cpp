#include <chrono>
#include <charconv>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <map>
#include <set>
#include <passagelab/cad.hpp>
#include <passagelab/io.hpp>

namespace {
using namespace pl;
struct Args {
    std::string command;
    std::map<std::string, std::string> opts;
    Args(int argc, char **argv) {
        const std::set<std::string> flags = {"--brute", "--no-optimize", "--sweep"};
        const std::set<std::string> values = {"--scene", "--demo", "--out", "--seed",
            "--seconds", "--iterations", "--nodes", "--queries", "--goal-bias",
            "--step", "--planner", "--runs", "--path", "--motion-space",
            "--cad", "--part", "--deflection", "--angle", "--scale"};
        if (argc > 1)
            command = argv[1];
        for (int i = 2; i < argc; ++i) {
            std::string k = argv[i];
            if (opts.contains(k))
                throw std::runtime_error("Duplicate option: " + k);
            if (flags.contains(k))
                opts[k] = "true";
            else if (values.contains(k)) {
                if (i + 1 >= argc || std::string(argv[i + 1]).starts_with("--"))
                    throw std::runtime_error("Missing value for " + k);
                opts[k] = argv[++i];
            } else
                throw std::runtime_error("Unknown option: " + k);
        }
        for (const auto *key : {"--seed", "--iterations", "--nodes", "--queries", "--runs", "--demo"})
            if (has(key))
                (void)number(key);
        for (const auto *key : {"--seconds", "--goal-bias", "--step", "--deflection", "--angle", "--scale"})
            if (has(key))
                (void)real(key);
        if (has("--planner") && get("--planner") != "ompl" && get("--planner") != "passagelab")
            throw std::runtime_error("--planner must be passagelab or ompl");
        if (has("--motion-space") && get("--motion-space") != "se3" && get("--motion-space") != "planar-xy")
            throw std::runtime_error("--motion-space must be se3 or planar-xy");
    }
    std::string get(const std::string &k, std::string def = "") const {
        auto i = opts.find(k);
        return i == opts.end() ? def : i->second;
    }
    bool has(const std::string &k) const {
        return opts.contains(k);
    }
    std::uint64_t number(const std::string &key, const std::string &fallback = "0") const {
        const auto value = get(key, fallback);
        std::uint64_t result = 0;
        const auto [end, error] = std::from_chars(value.data(), value.data() + value.size(), result);
        if (error != std::errc{} || end != value.data() + value.size())
            throw std::runtime_error("Expected unsigned integer for " + key);
        return result;
    }
    double real(const std::string &key) const {
        const auto value = get(key);
        std::size_t used = 0;
        const double result = std::stod(value, &used);
        if (used != value.size() || !std::isfinite(result))
            throw std::runtime_error("Expected finite number for " + key);
        return result;
    }
};
void configure(Scene &s, const Args &a) {
    auto &c = s.config;
    if (a.has("--seed"))
        c.seed = a.number("--seed");
    if (a.has("--seconds"))
        c.time_limit = a.real("--seconds");
    if (a.has("--iterations"))
        c.max_iterations = a.number("--iterations");
    if (a.has("--nodes"))
        c.max_nodes = a.number("--nodes");
    if (a.has("--queries"))
        c.max_queries = a.number("--queries");
    if (a.has("--goal-bias"))
        c.goal_bias = a.real("--goal-bias");
    if (a.has("--step"))
        c.validation.translation_step = c.validation.rotation_step = c.validation.surface_step =
            a.real("--step");
    if (a.has("--brute"))
        c.validation.use_bvh = false;
    if (a.has("--motion-space"))
        c.planar_xy = a.get("--motion-space") == "planar-xy";
    refresh_hash(s);
}
Scene input_scene(const Args &a) {
    if (a.has("--scene"))
        return load_scene(a.get("--scene"));
    const auto i = a.number("--demo");
    auto scenes = demo_scenes();
    if (i >= scenes.size())
        throw std::runtime_error("--demo must be 0, 1 or 2");
    return scenes[i];
}
Json run(Scene s, const Args &a, const std::filesystem::path &dir) {
    for (const auto *name : {"scene.json", "path.json", "optimized.json", "report.json"})
        if (std::filesystem::exists(dir / name))
            throw std::runtime_error("Output already contains run artifacts; choose a new --out directory: " + dir.string());
    configure(s, a);
    const bool ompl = a.get("--planner") == "ompl";
    auto result = ompl ? solve_ompl(s, s.config) : solve(s, s.config);
    PathMetrics metrics;
    std::optional<PathMetrics> simplified;
    double opt_seconds = 0, eval_seconds = 0;
    save_scene(s, dir / "scene.json");
    if (result.status == SolveStatus::Solved) {
        const auto eval_start = std::chrono::steady_clock::now();
        metrics = evaluate(s, result.path, s.config);
        eval_seconds =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - eval_start).count();
        save_path(result.path, dir / "path.json");
        if (!a.has("--no-optimize")) {
            const auto start = std::chrono::steady_clock::now();
            auto improved = optimize(s, result.path, s.config);
            opt_seconds =
                std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
            simplified = evaluate(s, improved, s.config);
            save_path(improved, dir / "optimized.json");
        }
    }
    auto report = report_json(s, s.config, result, metrics, simplified);
    report["planner"] = ompl ? "ompl-rrt-connect" : "passagelab-rrt-connect";
    report["optimization_seconds"] = opt_seconds;
    report["evaluation_seconds"] = eval_seconds;
    write_json(report, dir / "report.json");
    std::cout << s.id << " seed=" << s.config.seed << " " << to_string(result.status)
              << " nodes=" << result.nodes << " seconds=" << result.total_seconds << " "
              << result.reason << '\n';
    return report;
}
std::filesystem::path cad_path(const Args &a) {
    if (!a.has("--cad"))
        throw std::runtime_error("--cad file.step is required");
    const auto text = a.get("--cad");
    return std::filesystem::path(std::u8string(text.begin(), text.end()));
}
MeshSource cad_source(const Args &a) {
    MeshSource source;
    if (a.has("--deflection"))
        source.linear_deflection = a.real("--deflection");
    if (a.has("--angle"))
        source.angular_deflection = a.real("--angle");
    if (a.has("--scale"))
        source.scale = a.real("--scale");
    return source;
}
// One tessellation: what the kernel reported, what the welded mesh looks like, and how far its
// area and volume are from the exact B-rep integrals.
Json cad_mesh_row(const std::filesystem::path &file, const MeshSource &source, bool &watertight) {
    const auto cad = tessellate_step(file, source);
    const MeshShape shape(cad.mesh);
    const auto &r = shape.report();
    watertight = r.closed && r.oriented && r.degenerate_triangles == 0;
    Json row = {{"linear_deflection_m", source.linear_deflection},
                {"angular_deflection_rad", source.angular_deflection},
                {"scale", source.scale},
                {"brep",
                 {{"solids", cad.report.solids},
                  {"shells", cad.report.shells},
                  {"faces", cad.report.faces},
                  {"edges", cad.report.edges},
                  {"untriangulated_faces", cad.report.untriangulated_faces},
                  {"volume_m3", cad.report.brep_volume},
                  {"area_m2", cad.report.brep_area}}},
                {"weld",
                 {{"tolerance_m", cad.report.weld_tolerance},
                  {"input_vertices", cad.report.weld.input_vertices},
                  {"output_vertices", cad.report.weld.output_vertices},
                  {"input_triangles", cad.report.weld.input_triangles},
                  {"output_triangles", cad.report.weld.output_triangles},
                  {"collapsed_triangles", cad.report.weld.collapsed_triangles}}},
                {"mesh", mesh_report_json(r)},
                {"solid", shape.solid()},
                {"watertight", watertight},
                {"bvh_nodes", shape.bvh_nodes()},
                {"mesh_sha256", shape.digest()},
                {"seconds",
                 {{"read", cad.report.read_seconds},
                  {"tessellate", cad.report.tessellate_seconds},
                  {"assemble", cad.report.assemble_seconds}}}};
    row["volume_relative_error"] = nullptr;
    row["area_relative_error"] = nullptr;
    if (cad.report.brep_volume > 0 && r.closed && r.oriented)
        row["volume_relative_error"] =
            std::abs(std::abs(r.volume) - cad.report.brep_volume) / cad.report.brep_volume;
    if (cad.report.brep_area > 0)
        row["area_relative_error"] = std::abs(r.area - cad.report.brep_area) / cad.report.brep_area;
    return row;
}
} // namespace
int main(int argc, char **argv) {
    try {
        Args a(argc, argv);
        if (a.command == "query-benchmark") {
            Scene s;
            s.id = "dense-query-grid";
            s.physical = {{-12, -12, -4}, {12, 12, 4}};
            s.sampling = s.physical;
            Part body;
            body.id = "query-body";
            body.half = {.25, .13, .17};
            s.body = {body};
            for (int x = 0; x < 12; ++x)
                for (int y = 0; y < 12; ++y) {
                    Part o;
                    o.id = "grid-" + std::to_string(x) + "-" + std::to_string(y);
                    o.pose.t = {-8.25 + 1.5 * x, -8.25 + 1.5 * y, 0};
                    o.half = {.2, .2, .8};
                    s.obstacles.push_back(o);
                }
            refresh_hash(s);
            std::vector<Pose> poses;
            for (int i = 0; i < 1000; ++i) {
                Pose p;
                p.t = {9 * std::sin(i * .618), 9 * std::cos(i * .413), 1.4 * std::sin(i * .113)};
                p.q = Quat(Eigen::AngleAxisd(i * .21, Vec3(1, 2, 3).normalized()));
                poses.push_back(p);
            }
            Json data = {{"samples", poses.size()},
                         {"obstacles", s.obstacles.size()},
                         {"scene_hash", s.hash},
                         {"git_commit", PL_COMMIT},
                         {"build", PL_BUILD}};
            std::vector<StateResult> baseline;
            for (bool bvh : {false, true}) {
                Validator v(s);
                auto cfg = s.config.validation;
                cfg.use_bvh = bvh;
                const auto start = std::chrono::steady_clock::now();
                bool agree = true;
                for (std::size_t i = 0; i < poses.size(); ++i) {
                    auto state = v.state(poses[i], cfg, true);
                    if (!bvh)
                        baseline.push_back(state);
                    else {
                        const auto &old = baseline[i];
                        agree &= old.valid == state.valid;
                        if (old.valid && state.valid) {
                            agree &=
                                old.query.distance.has_value() == state.query.distance.has_value();
                            if (old.query.distance && state.query.distance)
                                agree &=
                                    std::abs(*old.query.distance - *state.query.distance) < 1e-8;
                        }
                    }
                }
                const auto seconds =
                    std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
                auto stats = v.stats();
                data[bvh ? "bvh" : "brute"] = {{"seconds", seconds},
                                               {"narrow_queries", stats.narrow},
                                               {"broad_nodes", stats.broad},
                                               {"agrees", agree}};
                if (!agree) {
                    write_json(data, a.get("--out", "results/query-benchmark.json"));
                    return 2;
                }
            }
            write_json(data, a.get("--out", "results/query-benchmark.json"));
            std::cout << data.dump(2) << '\n';
            return 0;
        }
        if (a.command == "cad-sample") {
            const auto part = a.get("--part", "plate-with-hole");
            const auto text = a.get("--out", "results/" + part + ".step");
            const std::filesystem::path out(std::u8string(text.begin(), text.end()));
            write_sample_step(part, out);
            std::cout << Json({{"part", part}, {"file", text}, {"units", "mm"}}).dump(2) << '\n';
            return 0;
        }
        if (a.command == "cad-mesh") {
            const auto file = cad_path(a);
            const auto base = cad_source(a);
            Json data = {{"file", a.get("--cad")},
                         {"git_commit", PL_COMMIT},
                         {"build", PL_BUILD},
                         {"tessellations", Json::array()}};
            bool all_watertight = true;
            // The sweep halves the chordal error each step so convergence of area and volume
            // toward the B-rep values can be read off directly.
            std::vector<double> deflections = {base.linear_deflection};
            if (a.has("--sweep"))
                deflections = {0.016, 0.008, 0.004, 0.002, 0.001, 0.0005};
            for (double deflection : deflections) {
                auto source = base;
                source.linear_deflection = deflection;
                bool watertight = false;
                data["tessellations"].push_back(cad_mesh_row(file, source, watertight));
                all_watertight &= watertight;
            }
            data["all_watertight"] = all_watertight;
            if (a.has("--out"))
                write_json(data, a.get("--out"));
            std::cout << data.dump(2) << '\n';
            return all_watertight ? 0 : 2;
        }
        if (a.command == "mesh-benchmark") {
            const auto file = cad_path(a);
            const auto source = cad_source(a);
            const MeshShape shape(tessellate_step(file, source).mesh);
            const Vec3 centre = 0.5 * (shape.bounds().min + shape.bounds().max),
                       extent = 0.5 * (shape.bounds().max - shape.bounds().min);
            Part box, ball;
            box.id = "box";
            box.half = 0.05 * extent.maxCoeff() * Vec3(1.0, 0.6, 0.8);
            ball.id = "sphere";
            ball.shape = Shape::Sphere;
            ball.radius = 0.05 * extent.maxCoeff();
            std::vector<Pose> poses;
            for (int i = 0; i < 1000; ++i) {
                Pose p;
                // Deterministic poses filling 1.5x the mesh bounds: some separated, some in
                // contact, some inside the solid.
                p.t = centre + 1.5 * extent.cwiseProduct(Vec3(std::sin(i * .618), std::cos(i * .413),
                                                              std::sin(i * .113 + 1)));
                p.q = Quat(Eigen::AngleAxisd(i * .21, Vec3(1, 2, 3).normalized()));
                poses.push_back(p);
            }
            Json data = {{"file", a.get("--cad")},
                         {"samples", poses.size()},
                         {"triangles", shape.report().triangles},
                         {"bvh_nodes", shape.bvh_nodes()},
                         {"solid", shape.solid()},
                         {"linear_deflection_m", source.linear_deflection},
                         {"mesh_sha256", shape.digest()},
                         {"git_commit", PL_COMMIT},
                         {"build", PL_BUILD}};
            bool all_agree = true;
            for (const Part *body : {&box, &ball}) {
                std::vector<QueryResult> baseline;
                Json row = Json::object();
                std::map<std::string, std::uint64_t> relations;
                for (bool bvh : {false, true}) {
                    MeshQueryStats stats;
                    bool agree = true;
                    const auto start = std::chrono::steady_clock::now();
                    for (std::size_t i = 0; i < poses.size(); ++i) {
                        auto q = shape.query(*body, poses[i], 1e-8, bvh, &stats);
                        if (!bvh) {
                            static const char *names[] = {"separated", "touching", "overlapping",
                                                          "indeterminate"};
                            ++relations[names[int(q.relation)]];
                            baseline.push_back(std::move(q));
                            continue;
                        }
                        const auto &old = baseline[i];
                        agree &= old.relation == q.relation &&
                                 old.distance.has_value() == q.distance.has_value();
                        if (old.distance && q.distance)
                            agree &= std::abs(*old.distance - *q.distance) < 1e-9;
                    }
                    const auto seconds =
                        std::chrono::duration<double>(std::chrono::steady_clock::now() - start)
                            .count();
                    row[bvh ? "bvh" : "brute"] = {{"seconds", seconds},
                                                  {"triangle_tests", stats.triangles},
                                                  {"nodes_visited", stats.nodes}};
                    if (bvh)
                        row["agrees"] = agree;
                    all_agree &= agree;
                }
                row["relations"] = relations;
                data[body->id] = row;
            }
            data["agrees"] = all_agree;
            write_json(data, a.get("--out", "results/mesh-benchmark.json"));
            std::cout << data.dump(2) << '\n';
            return all_agree ? 0 : 2;
        }
        if (a.command == "generate-scenes") {
            auto scenes = demo_scenes();
            auto refs = reference_paths(scenes);
            std::filesystem::path dir = a.get("--out", "assets/scenes");
            bool pass = true;
            for (std::size_t i = 0; i < scenes.size(); ++i) {
                save_scene(scenes[i], dir / (scenes[i].id + ".json"));
                auto check = validate_path(scenes[i], refs[i], refs[i].validation);
                save_path(refs[i], dir / (scenes[i].id + ".reference.json"));
                write_json({{"scene", scenes[i].id},
                            {"status", int(check.status)},
                            {"samples", check.samples},
                            {"reason", check.reason},
                            {"source", "hand-constructed-reference"}},
                           dir / (scenes[i].id + ".reference-check.json"));
                auto negative = scenes[i];
                negative.id += "-negative-start";
                negative.start.t = negative.obstacles.front()
                                       .pose.t.cwiseMax(negative.sampling.min)
                                       .cwiseMin(negative.sampling.max);
                refresh_hash(negative);
                Validator nv(negative);
                auto invalid = nv.state(negative.start, negative.config.validation, true);
                save_scene(negative, dir / (negative.id + ".json"));
                write_json({{"expected", "invalid_start"},
                            {"valid", invalid.valid},
                            {"reason", invalid.reason},
                            {"body", invalid.query.body_id},
                            {"obstacle", invalid.query.obstacle_id}},
                           dir / (negative.id + ".check.json"));
                std::cout << scenes[i].id << " reference=" << int(check.status)
                          << " negative_valid=" << invalid.valid << " " << check.reason << '\n';
                pass &= check.status == CheckStatus::Passed && !invalid.valid;
            }
            return pass ? 0 : 2;
        }
        if (a.command == "validate-scene") {
            auto s = input_scene(a);
            auto errors = scene_errors(s);
            Validator v(s);
            auto start = v.state(s.start, s.config.validation, true),
                 goal = v.state(s.goal, s.config.validation, true);
            std::cout << Json({{"errors", errors},
                               {"start_valid", start.valid},
                               {"start_reason", start.reason},
                               {"goal_valid", goal.valid},
                               {"goal_reason", goal.reason},
                               {"scene_hash", s.hash}})
                             .dump(2)
                      << '\n';
            return errors.empty() && start.valid && goal.valid ? 0 : 2;
        }
        if (a.command == "validate-path") {
            auto s = input_scene(a);
            auto p = load_path(a.get("--path"));
            auto cfg = p.validation;
            if (a.has("--step")) {
                double step = a.real("--step");
                cfg.translation_step = cfg.rotation_step = cfg.surface_step = step;
            }
            auto check = validate_path(s, p, cfg);
            Json j = {{"status", int(check.status)},
                      {"samples", check.samples},
                      {"reason", check.reason},
                      {"failed_u", check.failed_u},
                      {"failed_edge", check.failed_edge ? Json(*check.failed_edge) : Json(nullptr)},
                      {"body_part_id", check.failure.query.body_id},
                      {"obstacle_id", check.failure.query.obstacle_id},
                      {"intervals", check.intervals},
                      {"maximum_actual_translation_step", check.translation_step},
                      {"maximum_actual_rotation_step", check.rotation_step},
                      {"maximum_actual_surface_step", check.surface_step},
                      {"source_sha256", PL_SOURCE_HASH},
                      {"request_id", p.request_id}};
            std::cout << j.dump(2) << '\n';
            if (a.has("--out"))
                write_json(j, a.get("--out"));
            return check.status == CheckStatus::Passed ? 0 : 2;
        }
        if (a.command == "plan") {
            auto report = run(input_scene(a), a, a.get("--out", "results/latest"));
            return report.at("status") == "solved" ? 0 : 2;
        }
        if (a.command == "benchmark") {
            const auto runs = a.number("--runs", "30");
            if (runs < 1 || runs > 10000)
                throw std::runtime_error("Invalid run count");
            const auto base = a.number("--seed", "1000");
            if (base > std::numeric_limits<std::uint64_t>::max() - (runs - 1))
                throw std::runtime_error("Seed range overflows uint64");
            Json rows = Json::array();
            std::filesystem::path root = a.get("--out", "results/benchmark");
            auto s = input_scene(a);
            for (std::uint64_t i = 0; i < runs; ++i) {
                Args one = a;
                one.opts["--seed"] = std::to_string(base + i);
                rows.push_back(run(s, one, root / std::to_string(base + i)));
                write_json(rows, root / "runs.json");
            }
            return 0;
        }
        std::cout << "PassageLab " << PL_VERSION << " (" << PL_COMMIT
                  << ")\nCommands: generate-scenes, validate-scene, plan, validate-path, "
                     "benchmark, cad-sample, cad-mesh, mesh-benchmark\nCAD: --cad file.step, "
                     "--deflection m, --angle rad, --scale N, --sweep, --part name\nOptions: --scene file | --demo 0|1|2, --out path, --seed N, "
                     "--seconds N (0=operation budget), --iterations N, --nodes N, --queries N, "
                     "--step N, --brute, --planner ompl, --motion-space se3|planar-xy, --no-optimize, --runs N\n";
        return a.command.empty() || a.command == "--help" ? 0 : 1;
    } catch (const std::exception &e) {
        std::cerr << "error: " << e.what() << '\n';
        return 1;
    }
}

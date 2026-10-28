#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <map>
#include <passagelab/io.hpp>

namespace {
using namespace pl;
struct Args {
    std::string command;
    std::map<std::string, std::string> opts;
    Args(int argc, char **argv) {
        if (argc > 1)
            command = argv[1];
        for (int i = 2; i < argc; ++i) {
            std::string k = argv[i];
            if (!k.starts_with("--"))
                throw std::runtime_error("Expected option: " + k);
            if (i + 1 < argc && !std::string(argv[i + 1]).starts_with("--"))
                opts[k] = argv[++i];
            else
                opts[k] = "true";
        }
    }
    std::string get(const std::string &k, std::string def = "") const {
        auto i = opts.find(k);
        return i == opts.end() ? def : i->second;
    }
    bool has(const std::string &k) const {
        return opts.contains(k);
    }
};
void configure(Scene &s, const Args &a) {
    auto &c = s.config;
    if (a.has("--seed"))
        c.seed = std::stoull(a.get("--seed"));
    if (a.has("--seconds"))
        c.time_limit = std::stod(a.get("--seconds"));
    if (a.has("--iterations"))
        c.max_iterations = std::stoull(a.get("--iterations"));
    if (a.has("--nodes"))
        c.max_nodes = std::stoull(a.get("--nodes"));
    if (a.has("--queries"))
        c.max_queries = std::stoull(a.get("--queries"));
    if (a.has("--goal-bias"))
        c.goal_bias = std::stod(a.get("--goal-bias"));
    if (a.has("--step"))
        c.validation.translation_step = c.validation.rotation_step = c.validation.surface_step =
            std::stod(a.get("--step"));
    if (a.has("--brute"))
        c.validation.use_bvh = false;
    refresh_hash(s);
}
Scene input_scene(const Args &a) {
    if (a.has("--scene"))
        return load_scene(a.get("--scene"));
    const int i = std::stoi(a.get("--demo", "0"));
    auto scenes = demo_scenes();
    if (i < 0 || i >= int(scenes.size()))
        throw std::runtime_error("--demo must be 0, 1 or 2");
    return scenes[i];
}
Json run(Scene s, const Args &a, const std::filesystem::path &dir) {
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
                double step = std::stod(a.get("--step"));
                cfg.translation_step = cfg.rotation_step = cfg.surface_step = step;
            }
            auto check = validate_path(s, p, cfg);
            Json j = {{"status", int(check.status)},
                      {"samples", check.samples},
                      {"reason", check.reason},
                      {"failed_u", check.failed_u}};
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
            const int runs = std::stoi(a.get("--runs", "30"));
            if (runs < 1 || runs > 10000)
                throw std::runtime_error("Invalid run count");
            const auto base = std::stoull(a.get("--seed", "1000"));
            Json rows = Json::array();
            std::filesystem::path root = a.get("--out", "results/benchmark");
            auto s = input_scene(a);
            for (int i = 0; i < runs; ++i) {
                Args one = a;
                one.opts["--seed"] = std::to_string(base + i);
                rows.push_back(run(s, one, root / std::to_string(base + i)));
                write_json(rows, root / "runs.json");
            }
            return 0;
        }
        std::cout << "PassageLab " << PL_VERSION << " (" << PL_COMMIT
                  << ")\nCommands: generate-scenes, validate-scene, plan, validate-path, "
                     "benchmark\nOptions: --scene file | --demo 0|1|2, --out path, --seed N, "
                     "--seconds N (0=operation budget), --iterations N, --nodes N, --queries N, "
                     "--step N, --brute, --planner ompl, --no-optimize, --runs N\n";
        return a.command.empty() || a.command == "--help" ? 0 : 1;
    } catch (const std::exception &e) {
        std::cerr << "error: " << e.what() << '\n';
        return 1;
    }
}

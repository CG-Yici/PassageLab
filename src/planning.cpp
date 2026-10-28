#include "passagelab/core.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <numbers>
#include <numeric>
#include <random>

namespace pl {
namespace {
using Clock = std::chrono::steady_clock;
struct Node {
  Pose pose;
  std::size_t parent;
};
struct Tree {
  std::vector<Node> nodes;
  bool start_root;
};
enum class Extend { Trapped, Advanced, Reached, Stopped };

double distance(const Pose &a, const Pose &b, const PlannerConfig &c) {
  return pose_distance(a, b, c.rotation_weight);
}
Pose steer(const Pose &a, const Pose &b, double eta, const PlannerConfig &c) {
  const double d = distance(a, b, c);
  return d <= eta || d <= 1e-14 ? b : interpolate(a, b, eta / d);
}
std::size_t nearest(const Tree &t, const Pose &p, const PlannerConfig &c) {
  std::size_t best = 0;
  double bd = distance(t.nodes[0].pose, p, c);
  for (std::size_t i = 1; i < t.nodes.size(); ++i) {
    const double d = distance(t.nodes[i].pose, p, c);
    if (d < bd) {
      bd = d;
      best = i;
    }
  }
  return best;
}
Pose sample(std::mt19937_64 &rng, const Scene &s) {
  std::uniform_real_distribution<double> u(0.0, 1.0);
  Pose p;
  for (int i = 0; i < 3; ++i)
    p.t[i] =
        s.sampling.min[i] + u(rng) * (s.sampling.max[i] - s.sampling.min[i]);
  // Shoemake's construction is uniform on SO(3).
  const double u1 = u(rng), u2 = u(rng), u3 = u(rng);
  const double a = std::sqrt(1 - u1), b = std::sqrt(u1);
  const double x = a * std::sin(2 * std::numbers::pi * u2),
               y = a * std::cos(2 * std::numbers::pi * u2);
  const double z = b * std::sin(2 * std::numbers::pi * u3),
               w = b * std::cos(2 * std::numbers::pi * u3);
  p.q = Quat(w, x, y, z).normalized();
  return p;
}
std::vector<Pose> chain(const Tree &t, std::size_t i) {
  std::vector<Pose> p;
  while (true) {
    p.push_back(t.nodes[i].pose);
    if (i == t.nodes[i].parent)
      break;
    i = t.nodes[i].parent;
  }
  std::reverse(p.begin(), p.end());
  return p;
}
std::vector<Pose> joined(const Tree &a, std::size_t ai, const Tree &b,
                         std::size_t bi) {
  const Tree &st = a.start_root ? a : b;
  const Tree &gt = a.start_root ? b : a;
  const std::size_t si = a.start_root ? ai : bi, gi = a.start_root ? bi : ai;
  auto x = chain(st, si), y = chain(gt, gi);
  std::reverse(y.begin(), y.end());
  if (!x.empty() && !y.empty() && (x.back().t - y.front().t).norm() < 1e-12 &&
      rotation_distance(x.back().q, y.front().q) < 1e-12)
    y.erase(y.begin());
  x.insert(x.end(), y.begin(), y.end());
  return x;
}
void append_edges(PlanningResult &r, const Pose &a, const Pose &b,
                  bool start_tree) {
  if (r.edges.size() < 4000)
    r.edges.push_back({a, b, start_tree});
}
std::string stop_reason(const std::stop_token &token) {
  return token.stop_requested() ? "cancelled" : "budget exhausted";
}

bool metadata_matches(const Scene &s, const Path &p) {
  return p.scene_hash == s.hash && p.scene_revision == s.revision;
}
bool valid_validation(const ValidationConfig &v) {
  return std::isfinite(v.translation_step) && v.translation_step > 0 &&
         std::isfinite(v.rotation_step) && v.rotation_step > 0 &&
         std::isfinite(v.surface_step) && v.surface_step > 0 &&
         std::isfinite(v.margin) && v.margin >= 0 &&
         std::isfinite(v.tolerance) && v.tolerance >= 0 && v.max_samples >= 2;
}
bool valid_config(const PlannerConfig &c) {
  return std::isfinite(c.time_limit) && std::isfinite(c.extension) &&
         c.extension > 0 && std::isfinite(c.rotation_weight) &&
         c.rotation_weight > 0 && std::isfinite(c.goal_bias) &&
         c.goal_bias >= 0 && c.goal_bias <= 1 && valid_validation(c.validation);
}
bool valid_sampling_bounds(const Bounds &b) {
  return b.min.allFinite() && b.max.allFinite() &&
         (b.min.array() <= b.max.array()).all();
}
bool no_weaker(const ValidationConfig &requested,
               const ValidationConfig &stored) {
  if (!valid_validation(requested) || !valid_validation(stored))
    return false;
  return requested.translation_step <= stored.translation_step &&
         requested.rotation_step <= stored.rotation_step &&
         requested.surface_step <= stored.surface_step &&
         requested.margin >= stored.margin &&
         requested.tolerance >= stored.tolerance;
}
SolveStatus endpoint_status(const StateResult &state, bool start) {
  if (state.reason == "geometry query budget exhausted")
    return SolveStatus::BudgetExhausted;
  if (state.reason.rfind("invalid scene", 0) == 0 ||
      state.reason == "invalid input")
    return SolveStatus::Error;
  if (state.query.relation == Relation::Indeterminate &&
      state.reason != "sampling bounds")
    return SolveStatus::ValidationIncomplete;
  return start ? SolveStatus::InvalidStart : SolveStatus::InvalidGoal;
}
double path_objective(const std::vector<Pose> &p, double rw,
                      double *tr = nullptr, double *rot = nullptr) {
  double t = 0, r = 0;
  for (std::size_t i = 1; i < p.size(); ++i) {
    t += (p[i].t - p[i - 1].t).norm();
    r += rotation_distance(p[i - 1].q, p[i].q);
  }
  if (tr)
    *tr = t;
  if (rot)
    *rot = r;
  return t + rw * r;
}
struct PathValidation {
  MotionResult result;
  QueryStats queries;
};
void add_stats(QueryStats &a, const QueryStats &b) {
  a.states += b.states;
  a.motions += b.motions;
  a.narrow += b.narrow;
  a.broad += b.broad;
  a.samples += b.samples;
}
PathValidation validate_path_impl(const Scene &scene, const Path &path,
                                  const ValidationConfig &cfg,
                                  std::stop_token stop,
                                  std::optional<std::uint64_t> query_budget) {
  PathValidation out;
  auto &r = out.result;
  if (!valid_validation(cfg)) {
    r.status = CheckStatus::Failed;
    r.reason = "invalid validation configuration";
    return out;
  }
  if (!no_weaker(cfg, path.validation)) {
    r.status = CheckStatus::Failed;
    r.reason =
        "requested validation is weaker than the path validation metadata";
    return out;
  }
  if (stop.stop_requested()) {
    r.status = CheckStatus::Cancelled;
    r.reason = "cancelled";
    return out;
  }
  if (!metadata_matches(scene, path)) {
    r.status = CheckStatus::Failed;
    r.reason = "path scene hash or revision is stale";
    return out;
  }
  if (path.poses.empty()) {
    r.status = CheckStatus::Failed;
    r.reason = "path has no poses";
    return out;
  }
  for (const auto &p : path.poses)
    if (!valid_pose(p)) {
      r.status = CheckStatus::Failed;
      r.reason = "path contains non-finite or invalid pose";
      return out;
    }
  if ((path.poses.front().t - scene.start.t).norm() > 1e-8 ||
      rotation_distance(path.poses.front().q, scene.start.q) > 1e-8 ||
      (path.poses.back().t - scene.goal.t).norm() > 1e-8 ||
      rotation_distance(path.poses.back().q, scene.goal.q) > 1e-8) {
    r.status = CheckStatus::Failed;
    r.reason = "path endpoints do not match scene start and goal";
    return out;
  }
  Validator validator(scene);
  if (query_budget)
    validator.set_query_budget(*query_budget);
  auto first = validator.state(path.poses.front(), cfg);
  r.samples = 1;
  if (!first.valid) {
    r.status = first.query.relation == Relation::Indeterminate
                   ? CheckStatus::Incomplete
                   : CheckStatus::Failed;
    r.reason = "path start is invalid: " + first.reason;
    r.failure = first;
    out.queries = validator.stats();
    return out;
  }
  for (std::size_t i = 1; i < path.poses.size(); ++i) {
    if (stop.stop_requested()) {
      r.status = CheckStatus::Cancelled;
      r.reason = "cancelled";
      break;
    }
    auto m = validator.motion(path.poses[i - 1], path.poses[i], cfg, stop);
    r.samples += m.samples;
    if (m.status != CheckStatus::Passed) {
      r.status = m.status;
      r.failed_u = m.failed_u;
      r.reason = "path edge " + std::to_string(i - 1) + " failed: " + m.reason;
      r.failure = m.failure;
      break;
    }
    r.status = CheckStatus::Passed;
  }
  if (path.poses.size() == 1)
    r.status =
        stop.stop_requested() ? CheckStatus::Cancelled : CheckStatus::Passed;
  out.queries = validator.stats();
  return out;
}
} // namespace

std::string to_string(SolveStatus s) {
  switch (s) {
  case SolveStatus::Solved:
    return "solved";
  case SolveStatus::InvalidStart:
    return "invalid_start";
  case SolveStatus::InvalidGoal:
    return "invalid_goal";
  case SolveStatus::BudgetExhausted:
    return "budget_exhausted";
  case SolveStatus::Cancelled:
    return "cancelled";
  case SolveStatus::ValidationIncomplete:
    return "validation_incomplete";
  case SolveStatus::Error:
    return "error";
  }
  return "error";
}

MotionResult validate_path(const Scene &scene, const Path &path,
                           const ValidationConfig &cfg, std::stop_token stop) {
  return validate_path_impl(scene, path, cfg, stop, std::nullopt).result;
}

PlanningResult solve(const Scene &scene, const PlannerConfig &cfg,
                     std::stop_token stop, Progress progress) {
  PlanningResult out;
  const auto begin = Clock::now();
  auto finish = [&]() {
    out.total_seconds =
        std::chrono::duration<double>(Clock::now() - begin).count();
  };
  if (!valid_pose(scene.start)) {
    out.status = SolveStatus::InvalidStart;
    out.reason = "invalid start pose";
    finish();
    return out;
  }
  if (!valid_pose(scene.goal)) {
    out.status = SolveStatus::InvalidGoal;
    out.reason = "invalid goal pose";
    finish();
    return out;
  }
  if (stop.stop_requested()) {
    out.status = SolveStatus::Cancelled;
    out.reason = "cancelled";
    finish();
    return out;
  }
  if (!valid_config(cfg)) {
    out.status = SolveStatus::Error;
    out.reason = "invalid planner or validation configuration";
    finish();
    return out;
  }
  if (!valid_sampling_bounds(scene.sampling)) {
    out.status = SolveStatus::Error;
    out.reason = "invalid sampling bounds";
    finish();
    return out;
  }
  if (cfg.max_iterations == 0 || cfg.max_nodes < 2 || cfg.max_queries < 2) {
    out.status = SolveStatus::BudgetExhausted;
    out.reason = "operation budget is zero or cannot hold both roots and "
                 "validate endpoints";
    finish();
    return out;
  }
  Validator validator(scene);
  validator.set_query_budget(cfg.max_queries);
  auto a = validator.state(scene.start, cfg.validation);
  if (!a.valid) {
    out.status = endpoint_status(a, true);
    out.reason = a.reason;
    out.queries = validator.stats();
    finish();
    return out;
  }
  auto b = validator.state(scene.goal, cfg.validation);
  if (!b.valid) {
    out.status = endpoint_status(b, false);
    out.reason = b.reason;
    out.queries = validator.stats();
    finish();
    return out;
  }
  out.status = SolveStatus::BudgetExhausted;
  out.reason =
      "planning operation budget exhausted before a verified path was found";
  std::uint64_t query_used = validator.stats().narrow;
  QueryStats verification_queries;
  auto make_path = [&](std::vector<Pose> ps) {
    out.path.poses = std::move(ps);
    out.path.scene_hash = scene.hash;
    out.path.scene_revision = scene.revision;
    out.path.validation = cfg.validation;
  };
  auto check_candidate = [&](std::vector<Pose> ps) -> bool {
    make_path(std::move(ps));
    if (query_used > cfg.max_queries) {
      out.verification.status = CheckStatus::Incomplete;
      out.verification.reason = "geometry query budget exhausted before "
                                "independent path verification";
      out.status = SolveStatus::BudgetExhausted;
      out.reason = out.verification.reason;
      return true;
    }
    const auto v0 = Clock::now();
    auto validation = validate_path_impl(scene, out.path, cfg.validation, stop,
                                         cfg.max_queries - query_used);
    out.verification = validation.result;
    out.validation_seconds +=
        std::chrono::duration<double>(Clock::now() - v0).count();
    query_used += validation.queries.narrow;
    add_stats(verification_queries, validation.queries);
    if (out.verification.status == CheckStatus::Incomplete &&
        out.verification.reason.find("geometry query budget exhausted") !=
            std::string::npos) {
      out.status = SolveStatus::BudgetExhausted;
      out.reason = out.verification.reason;
      return true;
    }
    if (out.verification.status == CheckStatus::Passed) {
      out.status = SolveStatus::Solved;
      out.reason = "path independently validated";
      return true;
    }
    if (out.verification.status == CheckStatus::Cancelled) {
      out.status = SolveStatus::Cancelled;
      out.reason = out.verification.reason;
      return true;
    }
    if (out.verification.status == CheckStatus::Incomplete) {
      out.status = SolveStatus::ValidationIncomplete;
      out.reason = out.verification.reason;
      return true;
    }
    out.status = SolveStatus::Error;
    out.reason =
        "independent path validation failed: " + out.verification.reason;
    return true;
  };
  if (distance(scene.start, scene.goal, cfg) < 1e-12) {
    check_candidate({scene.start});
    out.queries = validator.stats();
    add_stats(out.queries, verification_queries);
    finish();
    return out;
  }
  Tree ta{{Node{scene.start, 0}}, true}, tb{{Node{scene.goal, 0}}, false};
  std::mt19937_64 rng(cfg.seed);
  const auto search_begin = Clock::now();
  auto deadline = [&]() {
    return cfg.time_limit > 0 &&
           std::chrono::duration<double>(Clock::now() - begin).count() >=
               cfg.time_limit;
  };
  bool validation_incomplete = false;
  bool query_budget_exhausted = false;
  auto budget = [&]() {
    return out.iterations >= cfg.max_iterations ||
           ta.nodes.size() + tb.nodes.size() >= cfg.max_nodes ||
           query_used >= cfg.max_queries || deadline() || stop.stop_requested();
  };
  auto extend = [&](Tree &t,
                    const Pose &target) -> std::pair<Extend, std::size_t> {
    if (budget())
      return {Extend::Stopped, 0};
    ++out.iterations;
    auto ni = nearest(t, target, cfg);
    auto d = distance(t.nodes[ni].pose, target, cfg);
    if (d < 1e-12)
      return {Extend::Reached, ni};
    Pose next =
        steer(t.nodes[ni].pose, target, std::max(cfg.extension, 1e-9), cfg);
    if (distance(t.nodes[ni].pose, next, cfg) < 1e-12)
      return {Extend::Trapped, ni};
    if (out.edges.size() < 4000)
      append_edges(out, t.nodes[ni].pose, next, t.start_root);
    if (query_used >= cfg.max_queries)
      return {Extend::Stopped, ni};
    auto m = validator.motion(t.nodes[ni].pose, next, cfg.validation, stop);
    auto after = validator.stats();
    query_used = after.narrow;
    if (m.status == CheckStatus::Incomplete) {
      query_budget_exhausted =
          m.reason.find("geometry query budget exhausted") != std::string::npos;
      validation_incomplete = !query_budget_exhausted;
      return {Extend::Stopped, ni};
    }
    if (m.status != CheckStatus::Passed)
      return {m.status == CheckStatus::Cancelled ? Extend::Stopped
                                                 : Extend::Trapped,
              ni};
    auto idx = t.nodes.size();
    t.nodes.push_back({next, ni});
    ++out.nodes;
    return {distance(next, target, cfg) < 1e-10 ? Extend::Reached
                                                : Extend::Advanced,
            idx};
  };
  for (; !budget();) {
    if (progress)
      progress(out.iterations, out.nodes);
    Pose target = std::uniform_real_distribution<double>(0, 1)(rng) <
                          std::clamp(cfg.goal_bias, 0.0, 1.0)
                      ? (ta.start_root ? scene.goal : scene.start)
                      : sample(rng, scene);
    auto [es, ei] = extend(ta, target);
    if (es == Extend::Stopped)
      break;
    if (es != Extend::Trapped) {
      Extend cs = Extend::Advanced;
      std::size_t ci = 0;
      while (cs == Extend::Advanced) {
        if (budget()) {
          cs = Extend::Stopped;
          break;
        }
        auto x = extend(tb, ta.nodes[ei].pose);
        cs = x.first;
        ci = x.second;
      }
      if (cs == Extend::Stopped &&
          (validation_incomplete || query_budget_exhausted ||
           stop.stop_requested() || deadline() ||
           query_used >= cfg.max_queries ||
           out.iterations >= cfg.max_iterations ||
           ta.nodes.size() + tb.nodes.size() >= cfg.max_nodes))
        break;
      if (cs == Extend::Reached) {
        auto ps =
            joined(ta, ei, tb, ci); // verify the actual remaining bridge too
        if (check_candidate(std::move(ps)))
          break;
      }
    }
    std::swap(ta, tb);
  }
  out.search_seconds = std::max(
      0.0, std::chrono::duration<double>(Clock::now() - search_begin).count() -
               out.validation_seconds);
  if (out.status != SolveStatus::Solved &&
      out.status != SolveStatus::ValidationIncomplete &&
      out.status != SolveStatus::Error) {
    if (query_budget_exhausted) {
      out.status = SolveStatus::BudgetExhausted;
      out.reason = "geometry query budget exhausted during motion validation";
    } else if (validation_incomplete) {
      out.status = SolveStatus::ValidationIncomplete;
      out.reason = "motion validation did not complete within configured "
                   "sampling resolution";
    } else {
      out.status = stop.stop_requested() ? SolveStatus::Cancelled
                                         : SolveStatus::BudgetExhausted;
      out.reason = stop_reason(stop);
    }
  }
  out.nodes = ta.nodes.size() + tb.nodes.size();
  out.queries = validator.stats();
  add_stats(out.queries, verification_queries);
  finish();
  return out;
}

Path optimize(const Scene &scene, const Path &original,
              const PlannerConfig &cfg, std::stop_token stop) {
  if (!valid_config(cfg) || cfg.max_queries == 0)
    return original;
  auto initial = validate_path_impl(scene, original, cfg.validation, stop,
                                    cfg.max_queries);
  if (initial.result.status != CheckStatus::Passed)
    return original;
  std::uint64_t query_used = initial.queries.narrow;
  Path result = original;
  bool changed = false;
  std::mt19937_64 rng(cfg.seed);
  const auto start = Clock::now();
  for (std::uint64_t k = 0;
       k < cfg.shortcut_attempts && result.poses.size() > 2; ++k) {
    if (stop.stop_requested() ||
        (cfg.time_limit > 0 &&
         std::chrono::duration<double>(Clock::now() - start).count() >=
             cfg.time_limit))
      break;
    std::uniform_int_distribution<std::size_t> pick(0, result.poses.size() - 1);
    auto i = pick(rng), j = pick(rng);
    if (i > j)
      std::swap(i, j);
    if (j <= i + 1)
      continue;
    std::vector<Pose> candidate(result.poses.begin(),
                                result.poses.begin() +
                                    static_cast<std::ptrdiff_t>(i + 1));
    candidate.insert(candidate.end(),
                     result.poses.begin() + static_cast<std::ptrdiff_t>(j),
                     result.poses.end());
    double before = path_objective(result.poses, cfg.rotation_weight),
           after = path_objective(candidate, cfg.rotation_weight);
    if (after + 1e-12 >= before)
      continue;
    if (query_used >= cfg.max_queries)
      break;
    Validator v(scene);
    v.set_query_budget(cfg.max_queries - query_used);
    auto m = v.motion(result.poses[i], result.poses[j], cfg.validation, stop);
    query_used += v.stats().narrow;
    if (m.status != CheckStatus::Passed)
      continue;
    Path trial = result;
    trial.poses = std::move(candidate);
    trial.validation = cfg.validation;
    trial.source = "shortcut";
    if (query_used >= cfg.max_queries)
      break;
    auto check = validate_path_impl(scene, trial, cfg.validation, stop,
                                    cfg.max_queries - query_used);
    query_used += check.queries.narrow;
    if (check.result.status == CheckStatus::Passed) {
      result = std::move(trial);
      changed = true;
    } else if (check.result.status == CheckStatus::Incomplete &&
               check.result.reason.find("geometry query budget exhausted") !=
                   std::string::npos)
      break;
  }
  if (changed) {
    if (query_used >= cfg.max_queries)
      return original;
    auto final = validate_path_impl(scene, result, cfg.validation, stop,
                                    cfg.max_queries - query_used);
    if (final.result.status != CheckStatus::Passed)
      return original;
  }
  return result;
}

PathMetrics evaluate(const Scene &scene, const Path &path,
                     const PlannerConfig &cfg, std::stop_token stop) {
  PathMetrics m;
  if (!metadata_matches(scene, path) || path.poses.empty() ||
      !valid_config(cfg) || !no_weaker(cfg.validation, path.validation))
    return m;
  m.objective = path_objective(path.poses, cfg.rotation_weight, &m.translation,
                               &m.rotation);
  Validator v(scene);
  v.set_query_budget(cfg.max_queries);
  double best = std::numeric_limits<double>::infinity();
  auto measure = [&](const Pose &p) -> bool {
    if (stop.stop_requested())
      return false;
    auto state = v.state(p, cfg.validation, true);
    ++m.samples;
    if (!state.valid)
      return false;
    double clearance = std::numeric_limits<double>::infinity();
    for (const auto &part : scene.body) {
      auto box = world_aabb(part, p);
      for (int axis = 0; axis < 3; ++axis)
        clearance =
            std::min({clearance, box.min[axis] - scene.physical.min[axis],
                      scene.physical.max[axis] - box.max[axis]});
    }
    if (!scene.obstacles.empty()) {
      if (!state.query.distance)
        return false;
      clearance = std::min(clearance, *state.query.distance);
    }
    if (!std::isfinite(clearance))
      return false;
    if (clearance < best) {
      best = clearance;
      m.min_clearance_pose = p;
    }
    return true;
  };
  for (std::size_t i = 0; i < path.poses.size(); ++i) {
    if (stop.stop_requested()) {
      m.complete = false;
      return m;
    }
    if (i == 0) {
      if (!measure(path.poses[i])) {
        m.complete = false;
        return m;
      }
    } else {
      const double L = (path.poses[i].t - path.poses[i - 1].t).norm();
      const double theta =
          rotation_distance(path.poses[i - 1].q, path.poses[i].q);
      const double travel = L + body_radius(scene) * theta;
      const double nd =
          std::max({1.0, std::ceil(L / cfg.validation.translation_step),
                    std::ceil(theta / cfg.validation.rotation_step),
                    std::ceil(travel / cfg.validation.surface_step)});
      if (!std::isfinite(nd) || nd > double(cfg.validation.max_samples) ||
          m.samples >
              cfg.validation.max_samples - static_cast<std::uint64_t>(nd)) {
        m.complete = false;
        return m;
      }
      const auto n = static_cast<std::uint64_t>(nd);
      for (std::uint64_t j = 1; j <= n; ++j) {
        auto p = interpolate(path.poses[i - 1], path.poses[i],
                             double(j) / double(n));
        if (!measure(p)) {
          m.complete = false;
          return m;
        }
      }
    }
  }
  if (std::isfinite(best))
    m.min_sampled_clearance = best;
  m.complete = true;
  return m;
}
} // namespace pl

#include "passagelab/core.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <numbers>
#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateSampler.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <random>

namespace pl {
namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace {
Pose from_state(const ob::State *state) {
  const auto *s = state->as<ob::SE3StateSpace::StateType>();
  Pose p;
  p.t = Vec3(s->getX(), s->getY(), s->getZ());
  p.q =
      Quat(s->rotation().w, s->rotation().x, s->rotation().y, s->rotation().z);
  return p;
}
void to_state(const Pose &p, ob::State *state) {
  auto *s = state->as<ob::SE3StateSpace::StateType>();
  s->setXYZ(p.t.x(), p.t.y(), p.t.z());
  s->rotation().x = p.q.x();
  s->rotation().y = p.q.y();
  s->rotation().z = p.q.z();
  s->rotation().w = p.q.w();
}
class LocalSE3Sampler final : public ob::StateSampler {
public:
  LocalSE3Sampler(const ob::StateSpace *space, std::uint_fast32_t seed, std::optional<Pose> anchor)
      : ob::StateSampler(space), anchor_(std::move(anchor)) {
    rng_.setLocalSeed(seed);
  }
  void sampleUniform(ob::State *state) override {
    auto *s = state->as<ob::SE3StateSpace::StateType>();
    const auto *space = space_->as<ob::SE3StateSpace>();
    const auto &bounds = space->getBounds();
    s->setXYZ(bounds.low[0] + u() * (bounds.high[0] - bounds.low[0]),
              bounds.low[1] + u() * (bounds.high[1] - bounds.low[1]),
              bounds.low[2] + u() * (bounds.high[2] - bounds.low[2]));
    if (anchor_) {
      Pose p;
      p.t = Vec3(s->getX(), s->getY(), anchor_->t.z());
      p.q = Quat(Eigen::AngleAxisd(2 * std::numbers::pi * u(), Vec3::UnitZ())) * anchor_->q;
      to_state(p, state);
      return;
    }
    const double u1 = u(), u2 = u(), u3 = u(), a = std::sqrt(1 - u1),
                 b = std::sqrt(u1);
    auto &q = s->rotation();
    q.x = a * std::cos(2 * std::numbers::pi * u2);
    q.y = a * std::sin(2 * std::numbers::pi * u2);
    q.z = b * std::cos(2 * std::numbers::pi * u3);
    q.w = b * std::sin(2 * std::numbers::pi * u3);
  }
  void sampleUniformNear(ob::State *state, const ob::State *near,
                         double distance) override {
    ob::State *tmp = space_->allocState();
    sampleUniform(tmp);
    double d = space_->distance(near, tmp);
    space_->interpolate(
        near, tmp, d <= distance || d == 0 ? 1.0 : std::max(0.0, distance / d),
        state);
    space_->freeState(tmp);
  }
  void sampleGaussian(ob::State *state, const ob::State *mean,
                      double stddev) override {
    const auto *m = mean->as<ob::SE3StateSpace::StateType>();
    auto *s = state->as<ob::SE3StateSpace::StateType>();
    auto normal = [&]() {
      const double r = std::sqrt(-2 * std::log(std::max(u(), 1e-15)));
      return r * std::cos(2 * std::numbers::pi * u()) * std::abs(stddev);
    };
    s->setXYZ(m->getX() + normal(), m->getY() + normal(), m->getZ() + normal());
    if (anchor_) {
      Pose p = from_state(mean);
      p.t = Vec3(s->getX(), s->getY(), anchor_->t.z());
      p.q = Quat(Eigen::AngleAxisd(normal(), Vec3::UnitZ())) * p.q;
      to_state(p, state);
      return;
    }
    Eigen::Vector3d axis(normal(), normal(), normal());
    if (axis.squaredNorm() < 1e-20)
      axis = Eigen::Vector3d::UnitX();
    else
      axis.normalize();
    const Eigen::AngleAxisd delta(normal(), axis);
    Eigen::Quaterniond q(m->rotation().w, m->rotation().x, m->rotation().y,
                         m->rotation().z);
    q = (q * Eigen::Quaterniond(delta)).normalized();
    s->rotation().x = q.x();
    s->rotation().y = q.y();
    s->rotation().z = q.z();
    s->rotation().w = q.w();
  }

private:
  double u() { return rng_.uniformReal(0.0, 1.0); }
  ompl::RNG rng_;
  std::optional<Pose> anchor_;
};
class WeightedSE3Space final : public ob::SE3StateSpace {
public:
  WeightedSE3Space(double rw, std::uint_fast32_t seed, std::optional<Pose> anchor)
      : rotation_weight_(rw), seed_(seed), anchor_(std::move(anchor)) {}
  double distance(const ob::State *a, const ob::State *b) const override {
    return pose_distance(from_state(a), from_state(b), rotation_weight_);
  }
  void interpolate(const ob::State *from, const ob::State *to, double t,
                   ob::State *state) const override {
    to_state(pl::interpolate(from_state(from), from_state(to), t), state);
  }
  ob::StateSamplerPtr allocDefaultStateSampler() const override {
    return std::make_shared<LocalSE3Sampler>(this, seed_, anchor_);
  }

private:
  double rotation_weight_;
  std::uint_fast32_t seed_;
  std::optional<Pose> anchor_;
};
class SeededRRTConnect final : public og::RRTConnect {
public:
  SeededRRTConnect(const ob::SpaceInformationPtr &si, std::uint_fast32_t seed)
      : og::RRTConnect(si) {
    rng_.setLocalSeed(seed);
  }
};
class SharedMotionValidator final : public ob::MotionValidator {
public:
  SharedMotionValidator(const ob::SpaceInformationPtr &si, const Scene &scene,
                        const ValidationConfig &cfg, std::stop_token stop,
                        std::atomic<std::uint64_t> &operations,
                        std::atomic<std::uint64_t> &motion_calls)
      : ob::MotionValidator(si), validator_(scene), cfg_(cfg), stop_(stop),
        operations_(operations), motion_calls_(motion_calls) {}
  bool checkMotion(const ob::State *a, const ob::State *b) const override {
    if (stop_.stop_requested() || motion_calls_.load() >= motion_limit_)
      return false;
    ++motion_calls_;
    return run(a, b, nullptr);
  }
  bool checkMotion(const ob::State *a, const ob::State *b,
                   std::pair<ob::State *, double> &last) const override {
    if (stop_.stop_requested() || motion_calls_.load() >= motion_limit_)
      return false;
    ++motion_calls_;
    return run(a, b, &last);
  }
  QueryStats stats() const { return validator_.stats(); }

private:
  bool run(const ob::State *a, const ob::State *b,
           std::pair<ob::State *, double> *last) const {
    auto local = cfg_;
    const auto used = operations_.load();
    const auto local_used = validator_.stats().narrow;
    validator_.set_query_budget(
        query_limit_ > used ? local_used + (query_limit_ - used) : local_used);
    auto r = validator_.motion(from_state(a), from_state(b), local, stop_);
    const auto local_after = validator_.stats().narrow;
    operations_ += local_after - local_used;
    if (last) {
      double u = std::clamp(r.failed_u, 0.0, 1.0);
      if (r.status != CheckStatus::Passed && r.samples > 1 && u > 0) {
        const auto intervals =
            std::max<std::uint64_t>(1, static_cast<std::uint64_t>(std::llround(
                                           double(r.samples - 1) / u)));
        u = std::max(0.0, u - 1.0 / double(intervals));
      }
      last->second = u;
      si_->getStateSpace()->interpolate(a, b, u, last->first);
    }
    return r.status == CheckStatus::Passed;
  }

public:
  void setLimits(std::uint64_t query_limit, std::uint64_t motion_limit) {
    query_limit_ = query_limit;
    motion_limit_ = motion_limit;
  }

private:
  mutable Validator validator_;
  ValidationConfig cfg_;
  std::stop_token stop_;
  std::atomic<std::uint64_t> &operations_;
  std::atomic<std::uint64_t> &motion_calls_;
  std::uint64_t query_limit_ = 0;
  std::uint64_t motion_limit_ = 0;
};
bool valid_config(const PlannerConfig &c) {
  const auto &v = c.validation;
  return std::isfinite(c.time_limit) && std::isfinite(c.extension) &&
         c.extension > 0 && std::isfinite(c.rotation_weight) &&
         c.rotation_weight > 0 && std::isfinite(c.goal_bias) &&
         c.goal_bias >= 0 && c.goal_bias <= 1 &&
         std::isfinite(v.translation_step) && v.translation_step > 0 &&
         std::isfinite(v.rotation_step) && v.rotation_step > 0 &&
         std::isfinite(v.surface_step) && v.surface_step > 0 &&
         std::isfinite(v.margin) && v.margin >= 0 &&
         std::isfinite(v.tolerance) && v.tolerance >= 0 && v.max_samples >= 2;
}
bool valid_sampling_bounds(const Bounds &b) {
  return b.min.allFinite() && b.max.allFinite() &&
         (b.min.array() <= b.max.array()).all();
}
void add_stats(QueryStats &a, const QueryStats &b) {
  a.states += b.states;
  a.motions += b.motions;
  a.narrow += b.narrow;
  a.broad += b.broad;
  a.samples += b.samples;
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
} // namespace

PlanningResult solve_ompl(const Scene &scene, const PlannerConfig &cfg,
                          std::stop_token stop) {
  PlanningResult out;
  const auto begin = std::chrono::steady_clock::now();
  auto finish = [&]() {
    out.total_seconds =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - begin)
            .count();
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
  if (!valid_config(cfg) || !valid_sampling_bounds(scene.sampling) ||
      cfg.planar_xy != scene.config.planar_xy) {
    out.status = SolveStatus::Error;
    out.reason = "invalid configuration or sampling bounds";
    finish();
    return out;
  }
  if (cfg.max_queries < 2 || cfg.max_iterations == 0 || cfg.max_nodes < 2) {
    out.status = SolveStatus::BudgetExhausted;
    out.reason = "operation budget cannot hold both roots and endpoint checks";
    finish();
    return out;
  }
  Validator endpoint_validator(scene);
  endpoint_validator.set_query_budget(cfg.max_queries);
  auto start_check = endpoint_validator.state(scene.start, cfg.validation);
  if (!start_check.valid) {
    out.status = endpoint_status(start_check, true);
    out.reason = start_check.reason;
    out.queries = endpoint_validator.stats();
    finish();
    return out;
  }
  auto goal_check = endpoint_validator.state(scene.goal, cfg.validation);
  if (!goal_check.valid) {
    out.status = endpoint_status(goal_check, false);
    out.reason = goal_check.reason;
    out.queries = endpoint_validator.stats();
    finish();
    return out;
  }
  auto space = std::make_shared<WeightedSE3Space>(
      cfg.rotation_weight,
      static_cast<std::uint_fast32_t>(cfg.seed ^ 0x6a09e667f3bcc909ULL),
      cfg.planar_xy ? std::optional<Pose>(scene.start) : std::nullopt);
  ob::RealVectorBounds bounds(3);
  for (int i = 0; i < 3; ++i) {
    bounds.setLow(i, scene.sampling.min[i]);
    bounds.setHigh(i, scene.sampling.max[i]);
  }
  space->setBounds(bounds);
  og::SimpleSetup setup(space);
  auto si = setup.getSpaceInformation();
  Validator state_validator(scene);
  std::atomic<std::uint64_t> operations{endpoint_validator.stats().narrow},
      motion_calls{0};
  setup.setStateValidityChecker([&](const ob::State *s) {
    const auto before = state_validator.stats().narrow;
    const auto used = operations.load();
    state_validator.set_query_budget(
        used < cfg.max_queries ? before + (cfg.max_queries - used) : before);
    auto valid = state_validator.state(from_state(s), cfg.validation).valid;
    operations += state_validator.stats().narrow - before;
    return valid;
  });
  auto motion_validator = std::make_shared<SharedMotionValidator>(
      si, scene, cfg.validation, stop, operations, motion_calls);
  const auto planner_cap = std::min<std::uint64_t>(
      cfg.max_iterations, cfg.max_nodes > 2 ? cfg.max_nodes - 2 : 0);
  motion_validator->setLimits(cfg.max_queries, planner_cap);
  si->setMotionValidator(motion_validator);
  ob::ScopedState<ob::SE3StateSpace> start(space), goal(space);
  to_state(scene.start, start.get());
  to_state(scene.goal, goal.get());
  setup.setStartAndGoalStates(start, goal);
  auto planner = std::make_shared<SeededRRTConnect>(
      si, static_cast<std::uint_fast32_t>(cfg.seed));
  planner->setRange(cfg.extension);
  setup.setPlanner(planner);
  const auto search_begin = std::chrono::steady_clock::now();
  // OMPL exposes neither RRT outer iterations nor live node count. Use
  // conservative query ceilings: every new node needs a successful motion
  // check, so this cap bounds both; it may stop before the self-written
  // planner's equivalent budget.
  const auto query_cap = cfg.max_queries;
  ob::PlannerTerminationCondition ptc([&]() {
    return stop.stop_requested() ||
           (cfg.time_limit > 0 && std::chrono::duration<double>(
                                      std::chrono::steady_clock::now() - begin)
                                          .count() >= cfg.time_limit) ||
           operations.load() >= query_cap || motion_calls.load() >= planner_cap;
  });
  const auto status = setup.solve(ptc);
  out.search_seconds = std::chrono::duration<double>(
                           std::chrono::steady_clock::now() - search_begin)
                           .count();
  ob::PlannerData planner_data(si);
  planner->getPlannerData(planner_data);
  out.nodes = planner_data.numVertices();
  out.iterations = motion_calls.load();
  out.queries = endpoint_validator.stats();
  const auto vs = state_validator.stats();
  out.queries.states += vs.states;
  out.queries.motions += vs.motions;
  out.queries.samples += vs.samples;
  out.queries.narrow += vs.narrow;
  out.queries.broad += vs.broad;
  const auto ms = motion_validator->stats();
  out.queries.states += ms.states;
  out.queries.motions += ms.motions;
  out.queries.samples += ms.samples;
  out.queries.narrow += ms.narrow;
  out.queries.broad += ms.broad;
  if (status == ob::PlannerStatus::EXACT_SOLUTION) {
    auto &path = setup.getSolutionPath();
    std::vector<Pose> poses;
    poses.reserve(path.getStateCount());
    for (unsigned int i = 0; i < path.getStateCount(); ++i)
      poses.push_back(from_state(path.getState(i)));
    out.path = {std::move(poses), scene.hash, scene.revision, cfg.validation,
                "ompl-rrt-connect"};
    out.path.request_id = new_request_id();
    const auto v = std::chrono::steady_clock::now();
    const auto remaining = operations.load() < cfg.max_queries
                               ? cfg.max_queries - operations.load()
                               : 0;
    Validator verifier(scene);
    verifier.set_query_budget(remaining);
    auto first = verifier.state(out.path.poses.front(), cfg.validation);
    out.verification.samples = 1;
    if (!first.valid) {
      out.verification.status =
          first.reason == "geometry query budget exhausted"
              ? CheckStatus::Incomplete
          : first.query.relation == Relation::Indeterminate
              ? CheckStatus::Incomplete
              : CheckStatus::Failed;
      out.verification.reason = "path start is invalid: " + first.reason;
      out.verification.failure = first;
    } else {
      out.verification.status = CheckStatus::Passed;
      for (std::size_t i = 1; i < out.path.poses.size(); ++i) {
        if (stop.stop_requested()) {
          out.verification.status = CheckStatus::Cancelled;
          out.verification.reason = "cancelled";
          break;
        }
        auto edge = verifier.motion(out.path.poses[i - 1], out.path.poses[i],
                                    cfg.validation, stop);
        out.verification.samples += edge.samples;
        out.verification.intervals += edge.intervals;
        out.verification.translation_step = std::max(out.verification.translation_step, edge.translation_step);
        out.verification.rotation_step = std::max(out.verification.rotation_step, edge.rotation_step);
        out.verification.surface_step = std::max(out.verification.surface_step, edge.surface_step);
        if (edge.status != CheckStatus::Passed) {
          out.verification.status = edge.status;
          out.verification.failed_u = edge.failed_u;
          out.verification.reason =
              "path edge " + std::to_string(i - 1) + " failed: " + edge.reason;
          out.verification.failure = edge.failure;
          out.verification.failed_edge = i - 1;
          break;
        }
      }
    }
    add_stats(out.queries, verifier.stats());
    out.validation_seconds =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - v)
            .count();
    if (out.verification.status == CheckStatus::Passed) {
      out.status = SolveStatus::Solved;
      out.reason =
          "OMPL RRT-Connect exact solution independently validated. "
          "WeightedSE3Space uses PassageLab's pose_distance and interpolation. "
          "OMPL maps iteration and node budgets conservatively to motion-check "
          "ceilings (may stop early); cfg.goal_bias is not applied.";
    } else {
      const bool exhausted =
          out.verification.reason.find("geometry query budget exhausted") !=
          std::string::npos;
      out.status = out.verification.status == CheckStatus::Cancelled
                       ? SolveStatus::Cancelled
                   : exhausted ? SolveStatus::BudgetExhausted
                   : out.verification.status == CheckStatus::Failed
                       ? SolveStatus::Error
                       : SolveStatus::ValidationIncomplete;
      out.reason = out.verification.reason;
    }
  } else {
    out.status = stop.stop_requested() ? SolveStatus::Cancelled
                                       : SolveStatus::BudgetExhausted;
    out.reason =
        stop.stop_requested()
            ? "cancelled"
            : "OMPL termination condition reached without an exact solution";
  }
  finish();
  return out;
}
} // namespace pl

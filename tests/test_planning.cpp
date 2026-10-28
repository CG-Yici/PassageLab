#include "passagelab/core.hpp"
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <limits>

namespace {
pl::Scene clear_scene() {
  pl::Scene s;
  s.hash = "planning-test";
  s.revision = 7;
  pl::Part body;
  body.id = "body";
  body.shape = pl::Shape::Sphere;
  body.radius = .1;
  s.body = {body};
  s.start.t = pl::Vec3(-1, 0, 0);
  s.goal.t = pl::Vec3(1, 0, 0);
  s.sampling.min = pl::Vec3::Constant(-2);
  s.sampling.max = pl::Vec3::Constant(2);
  s.config.validation.translation_step = .1;
  s.config.validation.rotation_step = .1;
  s.config.validation.surface_step = .1;
  return s;
}
} // namespace

TEST_CASE("RRT Connect handles an identical start and goal") {
  auto s = clear_scene();
  s.goal = s.start;
  auto r = pl::solve(s, s.config);
  REQUIRE(r.status == pl::SolveStatus::Solved);
  REQUIRE(r.path.poses.size() == 1);
  REQUIRE(r.verification.status == pl::CheckStatus::Passed);
}
TEST_CASE("planar sampling and validation preserve height and yaw-relative tilt") {
  auto s = clear_scene();
  s.config.planar_xy = true;
  s.config.time_limit = 0;
  s.start.q = pl::Quat(Eigen::AngleAxisd(.3, pl::Vec3::UnitX()));
  s.goal.q = pl::Quat(Eigen::AngleAxisd(.8, pl::Vec3::UnitZ())) * s.start.q;
  for (bool ompl : {false, true}) {
    auto r = ompl ? pl::solve_ompl(s, s.config) : pl::solve(s, s.config);
    REQUIRE(r.status == pl::SolveStatus::Solved);
    REQUIRE_FALSE(r.path.request_id.empty());
    for (const auto &pose : r.path.poses)
      CHECK(pl::in_motion_space(s, pose));
    REQUIRE(pl::validate_path(s, r.path, s.config.validation).status == pl::CheckStatus::Passed);
    auto bad = r.path;
    auto middle = pl::interpolate(s.start, s.goal, .5);
    middle.t.z() += .1;
    bad.poses.insert(bad.poses.begin() + 1, middle);
    CHECK(pl::validate_path(s, bad, s.config.validation).status == pl::CheckStatus::Failed);
  }
  s.goal.t.z() += .1;
  CHECK(pl::solve(s, s.config).status == pl::SolveStatus::InvalidGoal);
  CHECK(pl::solve_ompl(s, s.config).status == pl::SolveStatus::InvalidGoal);
  s.goal.t.z() = s.start.t.z();
  s.goal.q = pl::Quat(Eigen::AngleAxisd(.6, pl::Vec3::UnitX()));
  CHECK(pl::solve(s, s.config).status == pl::SolveStatus::InvalidGoal);
}

TEST_CASE("live search edges match the bounded result and include validated orientation") {
  auto s = clear_scene();
  s.config.time_limit = 0;
  std::vector<pl::SearchEdge> live;
  auto r = pl::solve(s, s.config, {}, {}, [&](const pl::SearchEdge &edge) {
    live.push_back(edge);
  });
  REQUIRE(r.status == pl::SolveStatus::Solved);
  REQUIRE_FALSE(live.empty());
  REQUIRE(live.size() == r.edges.size());
  REQUIRE(live.size() <= 4000);
  pl::Validator validator(s);
  for (std::size_t i = 0; i < live.size(); ++i) {
    REQUIRE(pl::valid_pose(live[i].to));
    REQUIRE(live[i].accepted == r.edges[i].accepted);
    if (live[i].accepted)
      REQUIRE(validator.motion(live[i].from, live[i].to, s.config.validation).status ==
              pl::CheckStatus::Passed);
  }
}

TEST_CASE("minimum clearance playback position reproduces the measured pose") {
  auto s = clear_scene();
  pl::Path p;
  p.scene_hash = s.hash;
  p.scene_revision = s.revision;
  p.validation = s.config.validation;
  auto middle = s.start;
  middle.t = pl::Vec3(0, 1.9, 0);
  middle.q = pl::Quat(Eigen::AngleAxisd(.7, pl::Vec3::UnitZ()));
  p.poses = {s.start, middle, s.goal};
  auto m = pl::evaluate(s, p, s.config);
  REQUIRE(m.complete);
  REQUIRE(m.min_clearance_position.has_value());
  REQUIRE(m.min_clearance_pose.has_value());
  const double x = *m.min_clearance_position * (p.poses.size() - 1);
  const auto i = std::min(std::size_t(x), p.poses.size() - 2);
  const auto current = pl::interpolate(p.poses[i], p.poses[i + 1], x - i);
  REQUIRE((current.t - m.min_clearance_pose->t).norm() < 1e-12);
  REQUIRE(pl::rotation_distance(current.q, m.min_clearance_pose->q) < 1e-12);
}

TEST_CASE("solve distinguishes invalid endpoints") {
  auto s = clear_scene();
  pl::Part wall;
  wall.id = "wall";
  wall.shape = pl::Shape::Sphere;
  wall.radius = .5;
  wall.pose.t = s.start.t;
  s.obstacles = {wall};
  REQUIRE(pl::solve(s, s.config).status == pl::SolveStatus::InvalidStart);
  REQUIRE(pl::solve_ompl(s, s.config).status == pl::SolveStatus::InvalidStart);
  s.obstacles[0].pose.t = s.goal.t;
  REQUIRE(pl::solve(s, s.config).status == pl::SolveStatus::InvalidGoal);
  REQUIRE(pl::solve_ompl(s, s.config).status == pl::SolveStatus::InvalidGoal);
}

TEST_CASE("RRT Connect finds and verifies a detour around an obstacle") {
  auto s = clear_scene();
  pl::Part obstacle;
  obstacle.id = "center";
  obstacle.shape = pl::Shape::Sphere;
  obstacle.radius = .35;
  obstacle.pose.t = pl::Vec3::Zero();
  s.obstacles = {obstacle};
  s.start.t = pl::Vec3(-1, 0, 0);
  s.goal.t = pl::Vec3(1, 0, 0);
  s.config.seed = 19;
  s.config.time_limit = 0;
  s.config.max_iterations = 12000;
  s.config.max_nodes = 30000;
  s.config.max_queries = 1000000;
  s.config.extension = .35;
  s.config.goal_bias = .15;
  auto r = pl::solve(s, s.config);
  REQUIRE(r.status == pl::SolveStatus::Solved);
  REQUIRE(r.path.poses.size() > 2);
  REQUIRE((r.path.poses.front().t - s.start.t).norm() < 1e-8);
  REQUIRE((r.path.poses.back().t - s.goal.t).norm() < 1e-8);
  REQUIRE(r.verification.status == pl::CheckStatus::Passed);
}

TEST_CASE("path validation rejects stale metadata and endpoint changes") {
  auto s = clear_scene();
  pl::Path p;
  p.poses = {s.start, s.goal};
  p.scene_hash = s.hash;
  p.scene_revision = s.revision;
  p.validation = s.config.validation;
  REQUIRE(pl::validate_path(s, p, s.config.validation).status ==
          pl::CheckStatus::Passed);
  ++s.revision;
  REQUIRE(pl::validate_path(s, p, s.config.validation).status ==
          pl::CheckStatus::Failed);
}

TEST_CASE("search operation budgets and cancellation terminate") {
  auto s = clear_scene();
  pl::Part obstacle;
  obstacle.id = "obstacle";
  obstacle.shape = pl::Shape::Sphere;
  obstacle.radius = .3;
  obstacle.pose.t = pl::Vec3::Zero();
  s.obstacles = {obstacle};
  s.config.time_limit = 0;
  s.config.max_iterations = 1;
  s.config.max_nodes = 20;
  auto r = pl::solve(s, s.config);
  REQUIRE(r.status == pl::SolveStatus::BudgetExhausted);
  REQUIRE(r.iterations <= 1);
  std::stop_source source;
  source.request_stop();
  REQUIRE(pl::solve(s, s.config, source.get_token()).status ==
          pl::SolveStatus::Cancelled);
}

TEST_CASE("path metrics retain incomplete clearance as absent") {
  auto s = clear_scene();
  pl::Path p;
  p.poses = {s.start, s.goal};
  p.scene_hash = s.hash;
  p.scene_revision = s.revision;
  p.validation = s.config.validation;
  auto cfg = s.config;
  cfg.validation.max_samples = 1;
  auto m = pl::evaluate(s, p, cfg);
  REQUIRE_FALSE(m.complete);
  REQUIRE_FALSE(m.min_sampled_clearance.has_value());
}

TEST_CASE(
    "validation refuses weaker precision and honors singleton cancellation") {
  auto s = clear_scene();
  pl::Path p;
  p.poses = {s.start, s.goal};
  p.scene_hash = s.hash;
  p.scene_revision = s.revision;
  p.validation = s.config.validation;
  p.validation.translation_step = .05;
  p.validation.rotation_step = .05;
  p.validation.surface_step = .05;
  REQUIRE(pl::validate_path(s, p, s.config.validation).status ==
          pl::CheckStatus::Failed);
  p.validation = s.config.validation;
  auto limited = s.config.validation;
  limited.max_samples = 2;
  p.validation = s.config.validation;
  REQUIRE(pl::validate_path(s, p, limited).status ==
          pl::CheckStatus::Incomplete);
  p.validation = s.config.validation;
  p.validation.tolerance = 1e-5;
  REQUIRE(pl::validate_path(s, p, s.config.validation).status ==
          pl::CheckStatus::Failed);
  p.validation = s.config.validation;
  auto strict = s.config.validation;
  strict.tolerance = 1e-5;
  REQUIRE(pl::validate_path(s, p, strict).status == pl::CheckStatus::Passed);
  p.validation = s.config.validation;
  p.validation.tolerance = std::numeric_limits<double>::quiet_NaN();
  REQUIRE(pl::validate_path(s, p, s.config.validation).status ==
          pl::CheckStatus::Failed);
  p.validation = s.config.validation;
  p.poses = {s.start};
  s.goal = s.start;
  std::stop_source stop;
  stop.request_stop();
  REQUIRE(pl::validate_path(s, p, p.validation, stop.get_token()).status ==
          pl::CheckStatus::Cancelled);
}

TEST_CASE("path metrics include physical boundary clearance when obstacles are "
          "absent") {
  auto s = clear_scene();
  pl::Path p;
  p.poses = {s.start, s.goal};
  p.scene_hash = s.hash;
  p.scene_revision = s.revision;
  p.validation = s.config.validation;
  auto m = pl::evaluate(s, p, s.config);
  REQUIRE(m.complete);
  REQUIRE(m.min_sampled_clearance.has_value());
  REQUIRE(*m.min_sampled_clearance == Catch::Approx(3.9).margin(1e-6));
}

TEST_CASE("optimizer preserves scene metadata and declares shortcut source") {
  auto s = clear_scene();
  pl::Path p;
  p.poses = {s.start, pl::Pose{}, s.goal};
  p.poses[1].t = pl::Vec3(0, .2, 0);
  p.scene_hash = s.hash;
  p.scene_revision = s.revision;
  p.validation = s.config.validation;
  p.source = "test-input";
  auto result = pl::optimize(s, p, s.config);
  REQUIRE(result.poses.size() == 2);
  REQUIRE(result.source == "shortcut");
  REQUIRE(result.scene_hash == p.scene_hash);
  REQUIRE(result.scene_revision == p.scene_revision);
  REQUIRE(result.validation.translation_step == p.validation.translation_step);
}

TEST_CASE("max iteration budget counts extensions, including connect") {
  auto s = clear_scene();
  pl::Part obstacle;
  obstacle.id = "obstacle";
  obstacle.shape = pl::Shape::Sphere;
  obstacle.radius = .3;
  obstacle.pose.t = pl::Vec3::Zero();
  s.obstacles = {obstacle};
  s.config.time_limit = 0;
  s.config.max_iterations = 1;
  s.config.max_nodes = 20;
  auto r = pl::solve(s, s.config);
  REQUIRE(r.status == pl::SolveStatus::BudgetExhausted);
  REQUIRE(r.iterations == 1);
  REQUIRE(r.nodes <= 2 + r.iterations);
}

TEST_CASE("narrow geometry query budget never exceeds its hard limit") {
  auto s = clear_scene();
  pl::Part obstacle;
  obstacle.id = "center";
  obstacle.shape = pl::Shape::Sphere;
  obstacle.radius = .25;
  obstacle.pose.t = pl::Vec3::Zero();
  s.obstacles = {obstacle};
  s.config.time_limit = 0;
  s.config.max_iterations = 100;
  s.config.max_queries = 3;
  s.config.extension = 2;
  s.config.goal_bias = 1;
  s.config.validation.use_bvh = false;
  auto r = pl::solve(s, s.config);
  REQUIRE(r.status == pl::SolveStatus::BudgetExhausted);
  REQUIRE(r.queries.narrow <= s.config.max_queries);
}

TEST_CASE(
    "OMPL planning returns only independently validated exact solutions") {
  auto s = clear_scene();
  s.start.t = pl::Vec3(-.5, 0, 0);
  s.goal.t = pl::Vec3(.5, 0, 0);
  s.config.time_limit = 0;
  s.config.max_iterations = 400;
  s.config.max_nodes = 1000;
  s.config.max_queries = 20000;
  auto first = pl::solve_ompl(s, s.config),
       second = pl::solve_ompl(s, s.config);
  REQUIRE(first.status == pl::SolveStatus::Solved);
  REQUIRE(second.status == pl::SolveStatus::Solved);
  REQUIRE(first.verification.status == pl::CheckStatus::Passed);
  REQUIRE(first.path.source == "ompl-rrt-connect");
  REQUIRE(first.path.poses.size() == second.path.poses.size());
  REQUIRE(first.nodes == second.nodes);
  for (std::size_t i = 0; i < first.path.poses.size(); ++i) {
    REQUIRE((first.path.poses[i].t - second.path.poses[i].t).norm() < 1e-12);
    REQUIRE(pl::rotation_distance(first.path.poses[i].q,
                                  second.path.poses[i].q) < 1e-12);
  }
}

TEST_CASE("OMPL obeys cancellation and tiny node budgets") {
  auto s = clear_scene();
  s.config.time_limit = 0;
  s.config.max_iterations = 100;
  s.config.max_nodes = 100;
  s.config.max_queries = 1000;
  std::stop_source stop;
  stop.request_stop();
  REQUIRE(pl::solve_ompl(s, s.config, stop.get_token()).status ==
          pl::SolveStatus::Cancelled);
  s.config.max_nodes = 2;
  REQUIRE(pl::solve_ompl(s, s.config).status ==
          pl::SolveStatus::BudgetExhausted);
}

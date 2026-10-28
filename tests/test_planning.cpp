#include "passagelab/core.hpp"
#include <catch2/catch_test_macros.hpp>

namespace {
pl::Scene clear_scene() {
    pl::Scene s;s.hash="planning-test";s.revision=7;
    pl::Part body;body.id="body";body.shape=pl::Shape::Sphere;body.radius=.1;s.body={body};
    s.start.t=pl::Vec3(-1,0,0);s.goal.t=pl::Vec3(1,0,0);
    s.sampling.min=pl::Vec3::Constant(-2);s.sampling.max=pl::Vec3::Constant(2);
    s.config.validation.translation_step=.1;s.config.validation.rotation_step=.1;s.config.validation.surface_step=.1;
    return s;
}
}

TEST_CASE("RRT Connect handles an identical start and goal") {
    auto s=clear_scene();s.goal=s.start;
    auto r=pl::solve(s,s.config);
    REQUIRE(r.status==pl::SolveStatus::Solved);
    REQUIRE(r.path.poses.size()==1);
    REQUIRE(r.verification.status==pl::CheckStatus::Passed);
}

TEST_CASE("solve distinguishes invalid endpoints") {
    auto s=clear_scene();pl::Part wall;wall.id="wall";wall.shape=pl::Shape::Sphere;wall.radius=.5;wall.pose.t=s.start.t;s.obstacles={wall};
    REQUIRE(pl::solve(s,s.config).status==pl::SolveStatus::InvalidStart);
    s.obstacles[0].pose.t=s.goal.t;
    REQUIRE(pl::solve(s,s.config).status==pl::SolveStatus::InvalidGoal);
}

TEST_CASE("RRT Connect finds and verifies a detour around an obstacle") {
    auto s=clear_scene();pl::Part obstacle;obstacle.id="center";obstacle.shape=pl::Shape::Sphere;obstacle.radius=.35;obstacle.pose.t=pl::Vec3::Zero();s.obstacles={obstacle};
    s.start.t=pl::Vec3(-1,0,0);s.goal.t=pl::Vec3(1,0,0);
    s.config.seed=19;s.config.time_limit=0;s.config.max_iterations=12000;s.config.max_nodes=30000;s.config.max_queries=1000000;s.config.extension=.35;s.config.goal_bias=.15;
    auto r=pl::solve(s,s.config);
    REQUIRE(r.status==pl::SolveStatus::Solved);
    REQUIRE(r.path.poses.size()>2);
    REQUIRE((r.path.poses.front().t-s.start.t).norm()<1e-8);
    REQUIRE((r.path.poses.back().t-s.goal.t).norm()<1e-8);
    REQUIRE(r.verification.status==pl::CheckStatus::Passed);
}

TEST_CASE("path validation rejects stale metadata and endpoint changes") {
    auto s=clear_scene();pl::Path p;p.poses={s.start,s.goal};p.scene_hash=s.hash;p.scene_revision=s.revision;
    REQUIRE(pl::validate_path(s,p,s.config.validation).status==pl::CheckStatus::Passed);
    ++s.revision;
    REQUIRE(pl::validate_path(s,p,s.config.validation).status==pl::CheckStatus::Failed);
}

TEST_CASE("search operation budgets and cancellation terminate") {
    auto s=clear_scene();pl::Part obstacle;obstacle.id="obstacle";obstacle.shape=pl::Shape::Sphere;obstacle.radius=.3;obstacle.pose.t=pl::Vec3::Zero();s.obstacles={obstacle};
    s.config.time_limit=0;s.config.max_iterations=1;s.config.max_nodes=20;
    auto r=pl::solve(s,s.config);REQUIRE(r.status==pl::SolveStatus::BudgetExhausted);REQUIRE(r.iterations<=1);
    std::stop_source source;source.request_stop();
    REQUIRE(pl::solve(s,s.config,source.get_token()).status==pl::SolveStatus::Cancelled);
}

TEST_CASE("path metrics retain incomplete clearance as absent") {
    auto s=clear_scene();pl::Path p;p.poses={s.start,s.goal};p.scene_hash=s.hash;p.scene_revision=s.revision;
    auto cfg=s.config;cfg.validation.max_samples=1;
    auto m=pl::evaluate(s,p,cfg);
    REQUIRE_FALSE(m.complete);
    REQUIRE_FALSE(m.min_sampled_clearance.has_value());
}

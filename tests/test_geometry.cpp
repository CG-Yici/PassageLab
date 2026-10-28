#include "passagelab/core.hpp"
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <fcl/fcl.h>
#include <limits>
#include <numbers>
#include <random>

using namespace pl;
namespace {
Part box(std::string id, Vec3 half, Pose p = {}) {
    Part x;
    x.id = std::move(id);
    x.half = half;
    x.pose = p;
    return x;
}
Scene base_scene() {
    Scene s;
    s.body = {box("body", Vec3(.1, .1, .1))};
    s.sampling.min = Vec3(-5, -5, -5);
    s.sampling.max = Vec3(5, 5, 5);
    s.physical.min = Vec3(-10, -10, -10);
    s.physical.max = Vec3(10, 10, 10);
    return s;
}
bool on_box_surface(const Vec3 &point, const Part &part, const Pose &parent) {
    Pose t = compose(parent, part.pose);
    Vec3 local = t.q.conjugate() * (point - t.t);
    double eps = 1e-7 * std::max(1.0, part.half.maxCoeff());
    return (local.cwiseAbs().array() <= part.half.array() + eps).all() &&
           ((local.cwiseAbs() - part.half).cwiseAbs().minCoeff() <= eps);
}
} // namespace
TEST_CASE("point queries handle projection clamping containment and degenerate segments") {
    auto q = point_segment(Vec3(2, 3, 0), Vec3(0, 0, 0), Vec3(4, 0, 0));
    REQUIRE(q);
    CHECK(q->distance == Catch::Approx(3));
    CHECK((q->closest - Vec3(2, 0, 0)).norm() < 1e-12);
    q = point_segment(Vec3(8, 3, 0), Vec3(0, 0, 0), Vec3(4, 0, 0));
    REQUIRE(q);
    CHECK(q->distance == Catch::Approx(5));
    q = point_segment(Vec3(1, 0, 0), Vec3::Zero(), Vec3::Zero());
    REQUIRE(q);
    CHECK(q->distance == Catch::Approx(1));
    for (double scale : {1e-4, 1.0, 1e4}) {
        auto b = box("box", Vec3::Constant(scale));
        Pose t{Vec3(2, 3, 4), Quat(Eigen::AngleAxisd(.7, Vec3::UnitZ()))};
        auto exterior = point_box(t.t + t.q * Vec3(3 * scale, 0, 0), b, t);
        REQUIRE(exterior);
        CHECK(exterior->distance == Catch::Approx(2 * scale));
        CHECK_FALSE(exterior->inside);
        auto interior = point_box(t.t, b, t);
        REQUIRE(interior);
        CHECK(interior->inside);
        CHECK(interior->distance == Catch::Approx(0).margin(1e-12));
    }
    CHECK_FALSE(point_segment(Vec3::Constant(std::numeric_limits<double>::infinity()), Vec3::Zero(), Vec3::Zero()));
}
TEST_CASE("ray triangle picking handles edges misses degeneracy and transformed views") {
    const Vec3 a(0, 0, 0), b(1, 0, 0), c(0, 1, 0);
    auto hit = ray_triangle(Vec3(.25, .25, 2), Vec3(0, 0, -2), a, b, c);
    REQUIRE(hit);
    CHECK(hit->parameter == Catch::Approx(1));
    CHECK((hit->barycentric - Vec3(.5, .25, .25)).norm() < 1e-12);
    CHECK(ray_triangle(Vec3(.5, .5, 1), Vec3(0, 0, -1), a, b, c));
    CHECK_FALSE(ray_triangle(Vec3(2, 2, 1), Vec3(0, 0, -1), a, b, c));
    CHECK_FALSE(ray_triangle(Vec3(.25, .25, 1), Vec3(1, 0, 0), a, b, c));
    CHECK_FALSE(ray_triangle(Vec3(.25, .25, 1), Vec3(0, 0, -1), a, a, c));
    CHECK_FALSE(ray_triangle(Vec3(.25, .25, 1), Vec3::Zero(), a, b, c));
    Pose t{Vec3(-3, 2, 4), Quat(Eigen::AngleAxisd(.9, Vec3(1, 2, 3).normalized()))};
    auto moved = ray_triangle(t.t + t.q * Vec3(.25, .25, 2), t.q * Vec3(0, 0, -2),
                             t.t + t.q * a, t.t + t.q * b, t.t + t.q * c);
    REQUIRE(moved);
    CHECK(moved->parameter == Catch::Approx(hit->parameter));
}
TEST_CASE("motion diagnostics retain actual resolution and failure sample") {
    auto s = base_scene();
    s.obstacles = {box("wall", Vec3(.01, 1, 1))};
    Validator v(s);
    Pose a{Vec3(-1, 0, 0)}, b{Vec3(1, 0, 0)};
    auto r = v.motion(a, b, s.config.validation);
    REQUIRE(r.status == CheckStatus::Failed);
    REQUIRE(r.intervals > 0);
    CHECK(r.translation_step <= s.config.validation.translation_step);
    CHECK(r.surface_step <= s.config.validation.surface_step);
    CHECK(r.failed_u > 0);
    CHECK(r.failed_u < 1);
    CHECK(r.failure.query.obstacle_id == "wall");
}
TEST_CASE("rigid transforms and shortest arc are stable") {
    Pose a{Vec3(1, 2, 3), Quat(Eigen::AngleAxisd(.6, Vec3::UnitZ()))},
        b{Vec3(0, 1, 0), Quat::Identity()};
    auto p = compose(a, b), back = compose(inverse(a), p);
    REQUIRE((back.t - b.t).norm() < 1e-10);
    Quat negq = a.q;
    negq.coeffs() *= -1;
    REQUIRE(rotation_distance(a.q, negq) < 1e-8);
    Pose neg = a;
    neg.q.coeffs() *= -1;
    auto mid = interpolate(a, neg, .5);
    REQUIRE(rotation_distance(mid.q, a.q) < 1e-8);
    CHECK(rotation_distance(Quat::Identity(), Quat(Eigen::AngleAxisd(1e-12, Vec3::UnitX()))) ==
          Catch::Approx(1e-12).margin(1e-14));
    CHECK(rotation_distance(Quat::Identity(),
                            Quat(Eigen::AngleAxisd(std::numbers::pi, Vec3::UnitY()))) ==
          Catch::Approx(std::numbers::pi).margin(1e-12));
    Part left = box("left", Vec3(.3, .2, .1)), right = box("right", Vec3(.2, .2, .2));
    right.pose.t = Vec3(1, 0, 0);
    auto original = query_pair(left, Pose{}, right);
    Pose common{Vec3(3, -2, 1), Quat(Eigen::AngleAxisd(.8, Vec3(1, 2, 3).normalized()))};
    REQUIRE(original.distance);
    REQUIRE(original.on_body);
    REQUIRE(original.on_obstacle);
    CHECK((*original.on_body - *original.on_obstacle).norm() == Catch::Approx(*original.distance));
    CHECK(on_box_surface(*original.on_body, left, Pose{}));
    CHECK(on_box_surface(*original.on_obstacle, right, Pose{}));
    Part moved = right;
    moved.pose = compose(common, right.pose);
    Pose moved_parent = compose(common, Pose{});
    auto transformed = query_pair(left, moved_parent, moved);
    CHECK(original.relation == transformed.relation);
    REQUIRE(original.distance);
    REQUIRE(transformed.distance);
    CHECK(*original.distance == Catch::Approx(*transformed.distance));
    right.pose.q.coeffs() *= -1;
    CHECK(query_pair(left, Pose{}, right).relation == original.relation);
    auto swapped = query_pair(right, inverse(Pose{}), left);
    CHECK(swapped.relation == original.relation);
    REQUIRE(swapped.distance);
    CHECK(*swapped.distance == Catch::Approx(*original.distance));
}
TEST_CASE("sphere distances and OBB SAT classify separation contact and containment") {
    Part s;
    s.id = "sphere";
    s.shape = Shape::Sphere;
    s.radius = .5;
    Part t = s;
    t.id = "other";
    t.pose.t = Vec3(2, 0, 0);
    auto q = query_pair(s, Pose{}, t);
    REQUIRE(q.relation == Relation::Separated);
    REQUIRE(q.distance);
    CHECK(*q.distance == Catch::Approx(1.0));
    Part a = box("a", Vec3(1, 1, 1)), b = box("b", Vec3(1, 1, 1));
    b.pose.t = Vec3(2, 0, 0);
    CHECK(query_pair(a, Pose{}, b).relation == Relation::Touching);
    b.pose.t = Vec3(.2, 0, 0);
    CHECK(query_pair(a, Pose{}, b).relation == Relation::Overlapping);
    b.half = Vec3(.1, .1, .1);
    b.pose.t = Vec3::Zero();
    CHECK(query_pair(a, Pose{}, b).relation == Relation::Overlapping);
}
TEST_CASE("skew edge pair has exact symmetric closest feature distance") {
    Part a = box("edge-a", Vec3::Constant(0.5)), b = box("edge-b", Vec3::Constant(0.5));
    b.pose.q =
        (Quat(Eigen::AngleAxisd(0.4, Vec3::UnitZ())) * Quat(Eigen::AngleAxisd(0.6, Vec3::UnitY())))
            .normalized();
    Vec3 n = Vec3::UnitX().cross(b.pose.q * Vec3::UnitX()).normalized();
    Vec3 local_n = b.pose.q.conjugate() * n;
    double support_a = 0.5 * n.cwiseAbs().sum();
    double support_b = 0.5 * local_n.cwiseAbs().sum();
    constexpr double gap = 0.07;
    b.pose.t = n * (support_a + support_b + gap);
    auto q = query_pair(a, Pose{}, b);
    REQUIRE(q.relation == Relation::Separated);
    REQUIRE(q.distance);
    REQUIRE(q.on_body);
    REQUIRE(q.on_obstacle);
    CHECK(*q.distance == Catch::Approx(gap).margin(1e-9));
    CHECK((*q.on_body - *q.on_obstacle).norm() == Catch::Approx(*q.distance).margin(1e-9));
    Vec3 la = q.on_body.value();
    Vec3 lb = b.pose.q.conjugate() * (*q.on_obstacle - b.pose.t);
    auto edge_coordinates = [](const Vec3 &p) {
        return (p.cwiseAbs().array() > 0.5 - 1e-7).count();
    };
    CHECK(edge_coordinates(la) == 2);
    CHECK(edge_coordinates(lb) == 2);
    Part world_a = a;
    auto reverse = query_pair(b, Pose{}, world_a);
    REQUIRE(reverse.distance);
    CHECK(*reverse.distance == Catch::Approx(*q.distance).margin(1e-9));
}
TEST_CASE("validator rejects invalid state and physical boundary overflow") {
    auto s = base_scene();
    Validator v(s);
    ValidationConfig c;
    Pose p;
    p.t.x() = std::numeric_limits<double>::quiet_NaN();
    CHECK_FALSE(v.state(p, c).valid);
    s.sampling.min = Vec3::Constant(-10);
    s.sampling.max = Vec3::Constant(10);
    Validator boundary_validator(s);
    p = Pose{};
    p.t.x() = 9.95;
    CHECK(boundary_validator.state(p, c).reason == "physical bounds clearance");
}
TEST_CASE("discrete motion detects thin obstacle between clear endpoints") {
    auto s = base_scene();
    Part wall = box("wall", Vec3(.015, .5, .5));
    s.obstacles.push_back(wall);
    Validator v(s);
    ValidationConfig c;
    c.translation_step = .01;
    c.surface_step = .01;
    c.rotation_step = .05;
    Pose a, b;
    a.t.x() = -1;
    b.t.x() = 1;
    CHECK(v.motion(a, b, c).status == CheckStatus::Failed);
}
TEST_CASE("rotation samples the swept footprint") {
    auto s = base_scene();
    s.body = {box("arm", Vec3(1, .04, .04))};
    Part wall = box("wall", Vec3(.06, .06, .2));
    wall.pose.t = Vec3(.72, .72, 0);
    s.obstacles = {wall};
    Validator v(s);
    ValidationConfig c;
    c.rotation_step = .01;
    c.surface_step = .01;
    c.translation_step = .1;
    Pose a, b;
    b.q = Quat(Eigen::AngleAxisd(std::numbers::pi / 2, Vec3::UnitZ()));
    CHECK(v.motion(a, b, c).status == CheckStatus::Failed);
}
TEST_CASE("BVH and brute force agree including positive margins") {
    auto s = base_scene();
    for (int i = 0; i < 20; i++) {
        Pose p;
        p.t = Vec3(i == 0 ? .41 : 1.0 + i * .5, 0, 0);
        s.obstacles.push_back(box(std::to_string(i), Vec3(.1, .1, .1), p));
    }
    Validator v(s);
    ValidationConfig a;
    a.margin = .25;
    a.use_bvh = true;
    auto x = v.state(Pose{}, a, true);
    a.use_bvh = false;
    auto y = v.state(Pose{}, a, true);
    CHECK_FALSE(x.valid);
    CHECK(x.valid == y.valid);
    CHECK(x.query.distance.has_value() == y.query.distance.has_value());
    if (x.query.distance && y.query.distance)
        CHECK(*x.query.distance == Catch::Approx(*y.query.distance));
}
TEST_CASE("BVH distance traversal prunes by lower bound and matches brute force") {
    auto s = base_scene();
    for (int i = 0; i < 64; i++) {
        Pose p;
        p.t = Vec3(2.0 + i * 2.0, 0, 0);
        s.obstacles.push_back(box(std::to_string(i), Vec3(.1, .1, .1), p));
    }
    Validator accelerated(s), brute(s);
    ValidationConfig a;
    a.use_bvh = true;
    auto x = accelerated.state(Pose{}, a, true);
    a.use_bvh = false;
    auto y = brute.state(Pose{}, a, true);
    REQUIRE(x.valid);
    REQUIRE(y.valid);
    REQUIRE(x.query.distance);
    REQUIRE(y.query.distance);
    CHECK(*x.query.distance == Catch::Approx(*y.query.distance));
    CHECK(accelerated.stats().narrow < brute.stats().narrow);
}
TEST_CASE("physical boundary clearance contributes distance and rejects margin") {
    auto s = base_scene();
    s.physical.min = Vec3::Constant(-1);
    s.physical.max = Vec3::Constant(1);
    Validator v(s);
    ValidationConfig c;
    auto x = v.state(Pose{}, c, true);
    REQUIRE(x.valid);
    REQUIRE(x.query.distance);
    CHECK(*x.query.distance == Catch::Approx(.9));
    CHECK(x.query.obstacle_id == "physical-boundary");
    c.margin = .91;
    auto y = v.state(Pose{}, c, true);
    CHECK_FALSE(y.valid);
    CHECK(y.reason == "physical bounds clearance");
}
TEST_CASE("physical wall witness lies on rotated body support") {
    auto s = base_scene();
    s.body = {box("arm", Vec3(.8, .1, .1))};
    s.physical.min = Vec3::Constant(-1);
    s.physical.max = Vec3::Constant(1);
    Validator v(s);
    Pose p;
    p.q = Quat(Eigen::AngleAxisd(std::numbers::pi / 4, Vec3::UnitZ()));
    p.t = Vec3(.2, 0, 0);
    ValidationConfig c;
    auto r = v.state(p, c, true);
    REQUIRE(r.valid);
    REQUIRE(r.query.on_body);
    REQUIRE(r.query.on_obstacle);
    Vec3 local = p.q.conjugate() * (*r.query.on_body - p.t);
    CHECK((local.cwiseAbs().array() <= s.body[0].half.array() + 1e-10).all());
    CHECK((*r.query.on_obstacle - *r.query.on_body).norm() == Catch::Approx(*r.query.distance));
}
TEST_CASE("invalid scene geometry and identifiers stop validation") {
    auto s = base_scene();
    s.obstacles.push_back(box("body", Vec3(.1, .1, .1)));
    CHECK_FALSE(scene_errors(s).empty());
    Validator v(s);
    CHECK_FALSE(v.state(Pose{}, ValidationConfig{}).valid);
    auto bad = base_scene();
    bad.body[0].half.x() = std::numeric_limits<double>::quiet_NaN();
    CHECK_FALSE(Validator(bad).state(Pose{}, ValidationConfig{}).valid);
}
TEST_CASE(
    "OBB SAT collision relation agrees with FCL for randomized separated and overlapping poses") {
    std::mt19937 rng(711);
    std::uniform_real_distribution<double> pos(-2, 2), ang(-3.14, 3.14), size(.1, .8);
    Part a = box("a", Vec3(.4, .5, .6)), b = box("b", Vec3(.5, .3, .7));
    auto ga = std::make_shared<fcl::Boxd>(.8, 1.0, 1.2),
         gb = std::make_shared<fcl::Boxd>(1.0, .6, 1.4);
    for (int i = 0; i < 200; i++) {
        a.half = Vec3(size(rng), size(rng), size(rng));
        b.half = Vec3(size(rng), size(rng), size(rng));
        a.pose.q = Quat(Eigen::AngleAxisd(ang(rng), Vec3(1, 2, 3).normalized()));
        b.pose.q = Quat(Eigen::AngleAxisd(ang(rng), Vec3(2, -1, 1).normalized()));
        Pose body;
        body.t = Vec3(pos(rng), pos(rng), pos(rng));
        ga = std::make_shared<fcl::Boxd>(2 * a.half.x(), 2 * a.half.y(), 2 * a.half.z());
        gb = std::make_shared<fcl::Boxd>(2 * b.half.x(), 2 * b.half.y(), 2 * b.half.z());
        fcl::Transform3d xa = fcl::Transform3d::Identity(), xb = fcl::Transform3d::Identity();
        auto ta = compose(body, a.pose);
        xa.linear() = ta.q.toRotationMatrix();
        xa.translation() = ta.t;
        xb.linear() = b.pose.q.toRotationMatrix();
        xb.translation() = b.pose.t;
        fcl::CollisionRequestd req;
        fcl::CollisionResultd res;
        bool collided = fcl::collide(ga.get(), xa, gb.get(), xb, req, res) > 0;
        auto q = query_pair(a, body, b, 0);
        REQUIRE(q.relation != Relation::Indeterminate);
        CHECK((q.relation != Relation::Separated) == collided);
        if (q.relation == Relation::Separated) {
            REQUIRE(q.distance);
            REQUIRE(q.on_body);
            REQUIRE(q.on_obstacle);
            CHECK((*q.on_body - *q.on_obstacle).norm() == Catch::Approx(*q.distance).margin(1e-7));
            CHECK(on_box_surface(*q.on_body, a, body));
            CHECK(on_box_surface(*q.on_obstacle, b, Pose{}));
            Part world_a = a;
            world_a.pose = compose(body, a.pose);
            auto reverse = query_pair(b, Pose{}, world_a, 0);
            REQUIRE(reverse.distance);
            CHECK(*reverse.distance == Catch::Approx(*q.distance).margin(1e-7));
        }
    }
}
TEST_CASE("motion reports sample exhaustion and cancellation") {
    auto s = base_scene();
    Validator v(s);
    Pose a, b;
    b.t.x() = 10;
    ValidationConfig c;
    c.translation_step = 1;
    c.surface_step = 1;
    c.max_samples = 2;
    CHECK(v.motion(a, b, c).status == CheckStatus::Incomplete);
    c.max_samples = 100;
    std::stop_source stop;
    stop.request_stop();
    CHECK(v.motion(a, b, c, stop.get_token()).status == CheckStatus::Cancelled);
}
TEST_CASE("validator stops at hard narrow-query budget without exceeding it") {
    auto s = base_scene();
    for (int i = 0; i < 10; i++) {
        Pose p;
        p.t = Vec3(.5 + i * .3, 0, 0);
        s.obstacles.push_back(box(std::to_string(i), Vec3(.1, .1, .1), p));
    }
    Validator v(s);
    v.set_query_budget(2);
    ValidationConfig c;
    c.use_bvh = false;
    auto r = v.state(Pose{}, c);
    CHECK_FALSE(r.valid);
    CHECK(r.query.relation == Relation::Indeterminate);
    CHECK(r.reason == "geometry query budget exhausted");
    CHECK(v.stats().narrow == 2);
    auto motion = v.motion(Pose{}, Pose{}, c);
    CHECK(motion.status == CheckStatus::Incomplete);
}

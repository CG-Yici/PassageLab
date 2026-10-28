#include <catch2/catch_test_macros.hpp>
#include <limits>
#include <passagelab/io.hpp>
using namespace pl;
TEST_CASE("motion space and request metadata survive JSON round trips", "[io]") {
    auto s = demo_scenes()[0];
    const auto original = s.hash;
    s.config.planar_xy = true;
    refresh_hash(s);
    REQUIRE(s.hash != original);
    REQUIRE(scene_from_json(scene_json(s)).config.planar_xy);
    auto j = scene_json(s);
    j["default_planning_config"]["motion_space"] = "unknown";
    REQUIRE_THROWS(scene_from_json(j));
    auto r = solve(s, s.config);
    REQUIRE(r.status == SolveStatus::Solved);
    REQUIRE(path_from_json(path_json(r.path)).request_id == r.path.request_id);
    auto p = path_json(r.path);
    p["endpoint_rotation_tolerance_rad"] = 1.0;
    REQUIRE_THROWS(path_from_json(p));
}
TEST_CASE("Scene JSON rejects malformed collections and version types", "[io]") {
    const auto original = scene_json(demo_scenes()[0]);
    for (const auto *key : {"moving_body", "obstacles"}) {
        auto j = original;
        j[key] = Json::object({{"part", original[key][0]}});
        REQUIRE_THROWS(scene_from_json(j));
        j[key] = nullptr;
        REQUIRE_THROWS(scene_from_json(j));
    }
    auto j = original;
    j["scene_id"] = "";
    REQUIRE_THROWS(scene_from_json(j));
    j = original;
    j["schema_version"] = 1.0;
    REQUIRE_THROWS(scene_from_json(j));
}
TEST_CASE("Scene JSON preserves geometry and rejects invalid conventions", "[io]") {
    auto s = demo_scenes()[0];
    auto j = scene_json(s);
    auto copy = scene_from_json(j);
    REQUIRE(copy.hash == s.hash);
    REQUIRE(copy.body.size() == s.body.size());
    j["schema_version"] = 99;
    REQUIRE_THROWS(scene_from_json(j));
    j = scene_json(s);
    j["moving_body"][0]["half_extents"][0] = -1;
    REQUIRE_THROWS(scene_from_json(j));
    j = scene_json(s);
    j["start_pose"]["quaternion"] = {0, 0, 0, 0};
    REQUIRE_THROWS(scene_from_json(j));
    j = scene_json(s);
    j["default_planning_config"]["seed"] = -1;
    REQUIRE_THROWS(scene_from_json(j));
}
TEST_CASE("Hashes cover inputs and stale paths cannot be validated", "[io]") {
    auto scenes = demo_scenes();
    auto paths = reference_paths(scenes);
    auto p = path_from_json(path_json(paths[0]));
    REQUIRE(p.scene_hash == scenes[0].hash);
    REQUIRE(validate_path(scenes[0], p, p.validation).status == CheckStatus::Passed);
    auto altered = scenes[0];
    altered.body[0].half.x() += .1;
    refresh_hash(altered);
    REQUIRE(altered.hash != scenes[0].hash);
    REQUIRE(validate_path(altered, p, p.validation).status == CheckStatus::Failed);
}
TEST_CASE("All frozen demonstration reference motions pass discrete validation", "[scenes]") {
    auto scenes = demo_scenes();
    auto paths = reference_paths(scenes);
    for (std::size_t i = 0; i < scenes.size(); ++i) {
        CAPTURE(scenes[i].id);
        auto result = validate_path(scenes[i], paths[i], paths[i].validation);
        INFO(result.reason);
        REQUIRE(result.status == CheckStatus::Passed);
    }
}
TEST_CASE("Physical boundaries reject external bypass and negative scenes reject direct motion",
          "[scenes]") {
    for (auto s : demo_scenes()) {
        Validator v(s);
        Pose out = s.start;
        out.t.z() = s.physical.max.z() + 1;
        REQUIRE_FALSE(v.state(out, s.config.validation).valid);
    }
    auto a = demo_scenes()[0];
    Validator v(a);
    REQUIRE(v.motion(a.start, a.goal, a.config.validation).status == CheckStatus::Failed);
}

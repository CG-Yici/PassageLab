#include "passagelab/cad.hpp"
#include "passagelab/io.hpp"
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <numbers>
#include <random>

using namespace pl;
namespace {
// Six quads with their own four vertices each, like the per-face output of a CAD tessellator.
TriangleMesh box_soup(const Vec3 &half) {
    TriangleMesh m;
    for (int axis = 0; axis < 3; ++axis)
        for (double sign : {-1.0, 1.0}) {
            Vec3 n = Vec3::Zero(), u = Vec3::Zero(), v = Vec3::Zero();
            n[axis] = sign;
            u[(axis + 1) % 3] = 1;
            v = n.cross(u);
            const auto base = std::uint32_t(m.vertices.size());
            for (auto [a, b] : {std::pair{-1, -1}, {1, -1}, {1, 1}, {-1, 1}})
                m.vertices.push_back((n + a * u + b * v).cwiseProduct(half));
            m.triangles.push_back({base, base + 1, base + 2});
            m.triangles.push_back({base, base + 2, base + 3});
        }
    return m;
}
Part box_part(std::string id, Vec3 half, Pose pose = {}) {
    Part p;
    p.id = std::move(id);
    p.half = half;
    p.pose = pose;
    return p;
}
Part sphere_part(std::string id, double radius) {
    Part p;
    p.id = std::move(id);
    p.shape = Shape::Sphere;
    p.radius = radius;
    return p;
}
std::filesystem::path scratch() {
    static const auto dir = [] {
        auto d = std::filesystem::temp_directory_path() /
                 ("passagelab-mesh-" + std::to_string(std::random_device{}()));
        std::filesystem::create_directories(d);
        return d;
    }();
    return dir;
}
std::filesystem::path sample(const std::string &part) {
    const auto file = scratch() / (part + ".step");
    if (!std::filesystem::exists(file))
        write_sample_step(part, file);
    return file;
}
Pose random_pose(std::mt19937_64 &rng, double reach) {
    std::uniform_real_distribution<double> u(-1, 1);
    Pose p;
    p.t = reach * Vec3(u(rng), u(rng), u(rng));
    p.q = Quat(u(rng), u(rng), u(rng), u(rng)).normalized();
    return p;
}
} // namespace

TEST_CASE("welding restores shared topology and analysis reports defects", "[mesh]") {
    const auto soup = box_soup(Vec3(1, 2, 3));
    auto raw = analyze(soup);
    CHECK(raw.vertices == 24);
    CHECK_FALSE(raw.closed);
    CHECK(raw.boundary_edges == 24);
    CHECK(raw.components == 6);

    WeldStats stats;
    auto mesh = weld(soup, 1e-9, &stats);
    CHECK(stats.input_vertices == 24);
    CHECK(stats.output_vertices == 8);
    CHECK(stats.collapsed_triangles == 0);
    auto r = analyze(mesh);
    CHECK(r.closed);
    CHECK(r.oriented);
    CHECK(r.components == 1);
    CHECK(r.degenerate_triangles == 0);
    CHECK(r.volume == Catch::Approx(48));
    CHECK(r.area == Catch::Approx(88));
    CHECK(MeshShape(mesh).solid());
    CHECK(analyze(weld(soup, 0)).closed); // exact matching is enough for identical coordinates

    auto open = mesh;
    open.triangles.pop_back();
    r = analyze(open);
    CHECK_FALSE(r.closed);
    CHECK(r.boundary_edges == 3);
    CHECK_FALSE(MeshShape(open).solid());

    auto flipped = mesh;
    std::swap(flipped.triangles[0][1], flipped.triangles[0][2]);
    r = analyze(flipped);
    CHECK(r.closed);
    CHECK_FALSE(r.oriented);
    CHECK(r.inconsistent_edges == 3);
    CHECK_FALSE(MeshShape(flipped).solid());

    auto doubled = mesh;
    doubled.triangles.push_back(doubled.triangles[0]);
    r = analyze(doubled);
    CHECK(r.duplicate_triangles == 1);
    CHECK(r.nonmanifold_edges == 3);

    // Vertices within the tolerance merge and the triangle between them collapses.
    TriangleMesh sliver;
    sliver.vertices = {Vec3(0, 0, 0), Vec3(1, 0, 0), Vec3(1, 1e-12, 0), Vec3(0, 1, 0)};
    sliver.triangles = {{0, 1, 2}, {0, 2, 3}};
    auto merged = weld(sliver, 1e-9, &stats);
    CHECK(stats.collapsed_triangles == 1);
    CHECK(merged.triangles.size() == 1);
    CHECK(merged.vertices.size() == 3);

    TriangleMesh bad;
    bad.vertices = {Vec3::Zero()};
    bad.triangles = {{0, 1, 2}};
    CHECK_THROWS(analyze(bad));
    CHECK_THROWS(weld(soup, -1));
    CHECK_THROWS(MeshShape(TriangleMesh{}));
}

TEST_CASE("point to triangle reports the closest feature", "[mesh]") {
    const Vec3 a(0, 0, 0), b(2, 0, 0), c(0, 2, 0);
    auto q = point_triangle(Vec3(.5, .5, 3), a, b, c);
    CHECK(q.feature == 6);
    CHECK(q.distance == Catch::Approx(3));
    q = point_triangle(Vec3(-1, -1, 0), a, b, c);
    CHECK(q.feature == 0);
    CHECK(q.distance == Catch::Approx(std::sqrt(2.0)));
    q = point_triangle(Vec3(1, -2, 0), a, b, c);
    CHECK(q.feature == 3);
    CHECK((q.closest - Vec3(1, 0, 0)).norm() < 1e-12);
    q = point_triangle(Vec3(2, 2, 0), a, b, c);
    CHECK(q.feature == 4);
    CHECK(q.distance == Catch::Approx(std::sqrt(2.0)));
    q = point_triangle(Vec3(-3, 1, 0), a, b, c);
    CHECK(q.feature == 5);
    // A collinear triangle still answers with its nearest edge point.
    q = point_triangle(Vec3(1, 1, 0), a, Vec3(1, 0, 0), b);
    CHECK(q.distance == Catch::Approx(1));
}

TEST_CASE("mesh queries match analytic box queries on the same geometry", "[mesh]") {
    const Vec3 half(.6, .4, .3);
    Pose placement{Vec3(.3, -.2, .5), Quat(Eigen::AngleAxisd(.7, Vec3(1, 2, 3).normalized()))};
    Part analytic = box_part("obstacle", half, placement);
    Part meshed = analytic;
    meshed.shape = Shape::Mesh;
    meshed.mesh = std::make_shared<MeshShape>(weld(box_soup(half), 1e-9));
    std::mt19937_64 rng(7);
    int separated = 0, overlapping = 0;
    for (const Part &body : {box_part("body", Vec3(.2, .1, .15)), sphere_part("body", .2)})
        for (int i = 0; i < 400; ++i) {
            const Pose pose = random_pose(rng, 1.6);
            const auto expected = query_pair(body, pose, analytic);
            if (expected.relation == Relation::Touching ||
                (expected.distance && *expected.distance < 1e-6))
                continue;
            const auto actual = query_pair(body, pose, meshed);
            INFO("sample " << i << " backend " << actual.backend);
            REQUIRE(actual.relation == expected.relation);
            if (expected.relation != Relation::Separated) {
                ++overlapping;
                continue;
            }
            ++separated;
            REQUIRE(actual.distance);
            CHECK(*actual.distance == Catch::Approx(*expected.distance).margin(1e-9));
            // Witness points realise the reported distance and lie on the obstacle box surface.
            CHECK((*actual.on_body - *actual.on_obstacle).norm() ==
                  Catch::Approx(*actual.distance).margin(1e-9));
            const Vec3 local = placement.q.conjugate() * (*actual.on_obstacle - placement.t);
            CHECK((local.cwiseAbs() - half).maxCoeff() == Catch::Approx(0).margin(1e-9));
        }
    CHECK(separated > 200);
    CHECK(overlapping > 20);

    // A small body deep inside touches no triangle; only solid semantics reports the overlap.
    const auto inside = query_pair(sphere_part("body", .05), placement, meshed);
    CHECK(inside.relation == Relation::Overlapping);
    auto shell = box_soup(half);
    shell.triangles.pop_back();
    meshed.mesh = std::make_shared<MeshShape>(weld(shell, 1e-9));
    CHECK(query_pair(sphere_part("body", .05), placement, meshed).relation == Relation::Separated);

    Part mesh_body = meshed;
    CHECK(query_pair(mesh_body, Pose{}, analytic).relation == Relation::Indeterminate);
}

TEST_CASE("STEP tessellation is watertight and converges to the B-rep", "[mesh][cad]") {
    for (const auto &part : sample_parts()) {
        INFO(part);
        MeshSource coarse, fine;
        coarse.linear_deflection = 0.008;
        fine.linear_deflection = 0.001;
        // Small fillets are governed by the angular limit, so both limits are tightened.
        fine.angular_deflection = 0.1;
        const auto a = tessellate_step(sample(part), coarse), b = tessellate_step(sample(part), fine);
        for (const auto *cad : {&a, &b}) {
            const auto r = analyze(cad->mesh);
            CHECK(cad->report.solids == 1);
            CHECK(cad->report.untriangulated_faces == 0);
            CHECK(cad->report.weld.output_vertices < cad->report.weld.input_vertices);
            CHECK(r.closed);
            CHECK(r.oriented);
            CHECK(r.degenerate_triangles == 0);
            CHECK(r.duplicate_triangles == 0);
            CHECK(r.components == 1);
            CHECK(r.volume > 0); // outward normals
            CHECK(cad->mesh.faces.size() == cad->mesh.triangles.size());
        }
        const auto ra = analyze(a.mesh), rb = analyze(b.mesh);
        const double exact = a.report.brep_volume;
        CHECK(b.report.brep_volume == Catch::Approx(exact));
        CHECK(rb.triangles > ra.triangles);
        // Chords lie inside convex curved faces, so refinement can only approach the B-rep.
        CHECK(std::abs(rb.volume - exact) < std::abs(ra.volume - exact));
        CHECK(std::abs(rb.volume - exact) / exact < 2e-2);
        CHECK(std::abs(rb.area - b.report.brep_area) / b.report.brep_area < 2e-2);
    }
    // Model millimetres become scene metres, and the kernel integrals match closed forms.
    MeshSource source;
    const double pi = std::numbers::pi;
    CHECK(tessellate_step(sample("cylinder"), source).report.brep_volume ==
          Catch::Approx(pi * .25 * .25 * .8).epsilon(1e-6));
    CHECK(tessellate_step(sample("torus"), source).report.brep_volume ==
          Catch::Approx(2 * pi * pi * .5 * .15 * .15).epsilon(1e-6));
    CHECK(tessellate_step(sample("plate-with-hole"), source).report.brep_volume ==
          Catch::Approx(3 * 3 * .12 - pi * .45 * .45 * .12).epsilon(1e-6));

    // Every vertex of the sphere lies on the surface and no triangle strays past the deflection.
    source.linear_deflection = 0.002;
    const auto sphere = tessellate_step(sample("sphere"), source).mesh;
    double worst_vertex = 0, worst_chord = 0;
    for (const auto &v : sphere.vertices)
        worst_vertex = std::max(worst_vertex, std::abs(v.norm() - .4));
    for (const auto &t : sphere.triangles)
        worst_chord = std::max(worst_chord,
                               .4 - ((sphere.vertices[t[0]] + sphere.vertices[t[1]] +
                                      sphere.vertices[t[2]]) / 3).norm());
    CHECK(worst_vertex < 1e-9);
    CHECK(worst_chord < 2 * source.linear_deflection);

    source.linear_deflection = -1;
    CHECK_THROWS(tessellate_step(sample("sphere"), source));
    CHECK_THROWS(tessellate_step(scratch() / "missing.step", MeshSource{}));
    CHECK_THROWS(write_sample_step("unknown", scratch() / "unknown.step"));
    const auto garbage = scratch() / "garbage.step";
    write_json(Json::object(), garbage);
    CHECK_THROWS(tessellate_step(garbage, MeshSource{}));
}

TEST_CASE("triangle BVH agrees with exhaustive search on a tessellated torus", "[mesh][cad]") {
    MeshSource source;
    const MeshShape torus(tessellate_step(sample("torus"), source).mesh);
    REQUIRE(torus.solid());
    REQUIRE(torus.report().triangles > 500);
    std::mt19937_64 rng(11);
    std::uniform_real_distribution<double> u(-1, 1);
    MeshQueryStats fast, slow;
    int inside = 0;
    for (int i = 0; i < 300; ++i) {
        const Vec3 p(.8 * u(rng), .8 * u(rng), .3 * u(rng));
        const auto a = torus.closest(p, true, &fast), b = torus.closest(p, false, &slow);
        REQUIRE(a);
        REQUIRE(b);
        CHECK(a->distance == Catch::Approx(b->distance).margin(1e-12));
        // Analytic distance to the torus (major .5, minor .15); the mesh is inscribed within the
        // chordal deflection.
        const double exact = std::abs(std::hypot(std::hypot(p.x(), p.y()) - .5, p.z()) - .15);
        CHECK(std::abs(a->distance - exact) < 2 * source.linear_deflection + 1e-9);
        // The pseudonormal sign test agrees with the generalized winding number.
        if (a->distance > 1e-6) {
            CHECK(a->inside == (std::abs(torus.winding_number(p)) > .5));
            CHECK(a->inside == b->inside);
        }
        inside += a->inside;

        const Vec3 direction(u(rng), u(rng), u(rng));
        const auto ha = torus.ray(p, direction, true), hb = torus.ray(p, direction, false);
        REQUIRE(ha.has_value() == hb.has_value());
        if (ha)
            CHECK(ha->parameter == Catch::Approx(hb->parameter).margin(1e-12));
    }
    CHECK(inside > 10);
    CHECK(inside < 290);
    CHECK(fast.triangles * 10 < slow.triangles);

    for (const Part &body : {box_part("body", Vec3(.12, .05, .08)), sphere_part("body", .07)})
        for (int i = 0; i < 200; ++i) {
            const Pose pose = random_pose(rng, .8);
            const auto a = torus.query(body, pose, 1e-8, true), b = torus.query(body, pose, 1e-8, false);
            INFO("sample " << i);
            REQUIRE(a.relation == b.relation);
            REQUIRE(a.distance.has_value() == b.distance.has_value());
            if (a.distance)
                CHECK(*a.distance == Catch::Approx(*b.distance).margin(1e-10));
        }
    CHECK_FALSE(torus.closest(Vec3::Constant(std::numeric_limits<double>::quiet_NaN())));
    CHECK_FALSE(torus.ray(Vec3::Zero(), Vec3::Zero()));
}

TEST_CASE("a STEP obstacle takes part in validation, hashing and planning", "[mesh][cad]") {
    Scene s;
    s.id = "step-aperture";
    s.body = {box_part("bar", Vec3(.5, .12, .12))};
    s.sampling = {Vec3(-1.3, -1.6, -1.3), Vec3(1.3, 1.6, 1.3)};
    s.physical = {Vec3(-1.5, -2.2, -1.5), Vec3(1.5, 2.2, 1.5)};
    s.start.t = Vec3(-.6, -1.2, -.5);
    s.goal.t = Vec3(.6, 1.2, .5);
    Part wall;
    wall.id = "wall";
    wall.shape = Shape::Mesh;
    wall.source.file = [&] {
        const auto text = sample("plate-with-hole").generic_u8string();
        return std::string(text.begin(), text.end());
    }();
    wall.mesh = std::make_shared<MeshShape>(
        tessellate_step(sample("plate-with-hole"), wall.source).mesh);
    s.obstacles = {wall};
    refresh_hash(s);
    REQUIRE(scene_errors(s).empty());

    Validator v(s);
    const auto cfg = s.config.validation;
    // The bar is 1.0 long and the aperture 0.9 wide: it fits only when aligned with the hole axis.
    const Quat along_hole(Eigen::AngleAxisd(std::numbers::pi / 2, Vec3::UnitZ()));
    auto through = v.state(Pose{Vec3::Zero(), along_hole}, cfg, true);
    REQUIRE(through.valid);
    REQUIRE(through.query.distance);
    // Corner of the 0.24 square section to the 0.45 radius hole, within the chordal deflection.
    CHECK(*through.query.distance ==
          Catch::Approx(.45 - .12 * std::sqrt(2.0)).margin(wall.source.linear_deflection + 1e-9));
    CHECK(through.query.obstacle_id == "wall");
    auto across = v.state(Pose{}, cfg, true);
    CHECK_FALSE(across.valid);
    CHECK(across.query.obstacle_id == "wall");
    auto blocked = v.motion(Pose{Vec3(0, -1, 0), Quat::Identity()},
                            Pose{Vec3(0, 1, 0), Quat::Identity()}, cfg);
    CHECK(blocked.status == CheckStatus::Failed);

    // The scene file stores the CAD path relative to itself and reloads the same triangles.
    const auto file = scratch() / "scene" / "step-aperture.json";
    save_scene(s, file);
    const auto reloaded = load_scene(file);
    CHECK(reloaded.hash == s.hash);
    REQUIRE(reloaded.obstacles.front().mesh);
    CHECK(reloaded.obstacles.front().mesh->digest() == wall.mesh->digest());
    auto stored = scene_json(s, file.parent_path());
    CHECK(stored["obstacles"][0]["source"]["file"] == "../plate-with-hole.step");
    // A different tessellation is a different scene: paths validated on one do not carry over.
    stored["obstacles"][0]["source"]["linear_deflection"] = 0.01;
    CHECK(scene_from_json(stored, file.parent_path()).hash != s.hash);
    stored["obstacles"][0]["source"]["file"] = "missing.step";
    CHECK_THROWS(scene_from_json(stored, file.parent_path()));

    auto mesh_body = s;
    mesh_body.body.push_back(wall);
    mesh_body.body.back().id = "mesh-body";
    CHECK_FALSE(scene_errors(mesh_body).empty());

    s.config.seed = 3;
    s.config.time_limit = 0;
    s.config.max_iterations = 4000;
    refresh_hash(s);
    const auto result = solve(s, s.config);
    REQUIRE(result.status == SolveStatus::Solved);
    CHECK(validate_path(s, result.path, result.path.validation).status == CheckStatus::Passed);
}

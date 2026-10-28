#include <numbers>
#include <passagelab/io.hpp>

namespace pl {
namespace {
Part box(std::string id, Vec3 center, Vec3 half) {
    Part p;
    p.id = std::move(id);
    p.pose.t = center;
    p.half = half;
    return p;
}
Pose pose(Vec3 p, double roll = 0, double yaw = 0) {
    return {p,
            Quat(Eigen::AngleAxisd(yaw, Vec3::UnitZ()) * Eigen::AngleAxisd(roll, Vec3::UnitX()))};
}
void aperture(Scene &s, std::string id, double x, double y, double z, double hy, double hz) {
    const double yl = s.physical.min.y(), yr = s.physical.max.y(), zl = s.physical.min.z(),
                 zr = s.physical.max.z(), th = .12;
    s.obstacles.push_back(box(id + "-left", {x, (yl + y - hy) / 2, (zl + zr) / 2},
                              {th, (y - hy - yl) / 2, (zr - zl) / 2}));
    s.obstacles.push_back(box(id + "-right", {x, (yr + y + hy) / 2, (zl + zr) / 2},
                              {th, (yr - y - hy) / 2, (zr - zl) / 2}));
    s.obstacles.push_back(
        box(id + "-bottom", {x, y, (zl + z - hz) / 2}, {th, hy, (z - hz - zl) / 2}));
    s.obstacles.push_back(box(id + "-top", {x, y, (zr + z + hz) / 2}, {th, hy, (zr - z - hz) / 2}));
}
} // namespace
std::vector<Scene> demo_scenes() {
    Scene a;
    a.id = "A-offset-passage";
    a.physical = {{-4, -2.3, 0}, {4, 2.3, 3}};
    a.sampling = {{-2.8, -1.7, .5}, {2.8, 1.7, 2.5}};
    a.body = {box("bar", {0, 0, 0}, {1.05, .16, .16})};
    a.obstacles = {box("central-column", {0, 0, 1.5}, {.35, .65, 1.5})};
    a.start = pose({-2.6, -.65, 1.3});
    a.goal = pose({2.6, .65, 1.3});
    a.config.rotation_weight = 1.1;
    a.config.extension = .65;
    a.config.time_limit = 15;

    Scene b;
    b.id = "B-L-corner";
    b.physical = {{-4, -1.5, 0}, {1.5, 4.4, 3}};
    b.sampling = {{-2.6, -.9, .5}, {.9, 3.1, 2.5}};
    b.body = {box("long-arm", {.35, 0, 0}, {.85, .14, .14}),
              box("short-arm", {-.35, .42, 0}, {.14, .55, .14})};
    b.obstacles = {box("inner-corner", {-2.75, 2.95, 1.5}, {1.25, 1.45, 1.5})};
    b.start = pose({-2.55, -.25, 1.4});
    b.goal = pose({.35, 2.9, 1.4}, 0, std::numbers::pi / 2);
    b.config.rotation_weight = 1.3;
    b.config.extension = .55;
    b.config.time_limit = 20;

    Scene c;
    c.id = "C-staggered-apertures";
    c.physical = {{-4.3, -2, 0}, {4.3, 2, 3.2}};
    c.sampling = {{-3.2, -1.25, .6}, {3.2, 1.25, 2.4}};
    c.body = {box("spine", {0, 0, 0}, {.9, .12, .12}),
              box("offset-fin", {.2, .32, 0}, {.2, .48, .12}),
              box("raised-lug", {-.3, 0, .18}, {.2, .12, .25})};
    aperture(c, "vertical-opening", -1.3, -.35, 1.35, .48, .8);
    aperture(c, "horizontal-opening", 1.3, .45, 1.7, .85, .38);
    c.start = pose({-3.1, -.2, 1.05}, std::numbers::pi / 2);
    c.goal = pose({3.1, .15, 1.55});
    c.config.rotation_weight = 1.2;
    c.config.extension = .4;
    c.config.time_limit = 30;
    c.config.max_iterations = 30000;
    c.config.max_nodes = 80000;
    std::vector<Scene> scenes = {a, b, c};
    for (auto &s : scenes)
        refresh_hash(s);
    return scenes;
}
std::vector<Path> reference_paths(const std::vector<Scene> &scenes) {
    std::vector<Path> refs;
    for (const auto &s : scenes) {
        Path p;
        p.scene_hash = s.hash;
        p.scene_revision = s.revision;
        p.source = "hand-constructed-reference";
        p.validation = s.config.validation;
        if (s.id.starts_with("A-"))
            p.poses = {s.start, pose({-2.6, -1.3, 1.3}), pose({2.6, -1.3, 1.3}), s.goal};
        else if (s.id.starts_with("B-"))
            p.poses = {s.start, pose({0, -.25, 1.4}), pose({0, 0, 1.4}, 0, std::numbers::pi / 2),
                       s.goal};
        else
            p.poses = {s.start, pose({0, -.2, 1.05}, std::numbers::pi / 2), pose({0, .15, 1.55}),
                       s.goal};
        refs.push_back(std::move(p));
    }
    return refs;
}
} // namespace pl

#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <stop_token>
#include <string>
#include <vector>

namespace pl {
using Vec3 = Eigen::Vector3d;
using Quat = Eigen::Quaterniond;
struct Pose {
    Vec3 t = Vec3::Zero();
    Quat q = Quat::Identity();
};
Pose compose(const Pose &, const Pose &);
Pose inverse(const Pose &);
Pose interpolate(const Pose &, const Pose &, double);
double rotation_distance(const Quat &, const Quat &);
double pose_distance(const Pose &, const Pose &, double rotation_weight);
bool valid_pose(const Pose &);
enum class Shape { Box, Sphere };
struct Part {
    std::string id;
    Shape shape = Shape::Box;
    Vec3 half = Vec3::Constant(0.5);
    double radius = 0.5;
    Pose pose; // body-local for components, world for obstacles
};
struct Bounds {
    Vec3 min = Vec3::Constant(-5);
    Vec3 max = Vec3::Constant(5);
};
struct ValidationConfig {
    double translation_step = 0.08;
    double rotation_step = 0.08;
    double surface_step = 0.08;
    double margin = 0.0;
    double tolerance = 1e-8;
    std::uint64_t max_samples = 100000;
    bool use_bvh = true;
};
struct PlannerConfig {
    std::uint64_t seed = 42, max_iterations = 10000, max_nodes = 40000, max_queries = 2000000;
    double time_limit = 10.0, extension = 0.5, rotation_weight = 1.0, goal_bias = 0.05;
    std::uint64_t shortcut_attempts = 100;
    ValidationConfig validation;
    bool planar_xy = false; // fixed start Z, rotations about world Z relative to start orientation
};
struct Scene {
    std::string id = "scene", hash;
    std::uint64_t revision = 1;
    std::vector<Part> body, obstacles;
    Bounds sampling, physical;
    Pose start, goal;
    PlannerConfig config;
};
struct Aabb {
    Vec3 min, max;
};
Aabb world_aabb(const Part &, const Pose &parent = {});
double body_radius(const Scene &);
std::vector<std::string> scene_errors(const Scene &);
bool in_motion_space(const Scene &, const Pose &);
struct PointDistance {
    double distance = 0;
    Vec3 closest = Vec3::Zero();
    bool inside = false;
};
std::optional<PointDistance> point_segment(const Vec3 &, const Vec3 &, const Vec3 &);
std::optional<PointDistance> point_box(const Vec3 &, const Part &, const Pose &parent = {});
struct RayHit {
    double parameter = 0;
    Vec3 position = Vec3::Zero(), barycentric = Vec3::Zero();
};
std::optional<RayHit> ray_triangle(const Vec3 &origin, const Vec3 &direction,
                                   const Vec3 &, const Vec3 &, const Vec3 &);
enum class Relation { Separated, Touching, Overlapping, Indeterminate };
struct QueryResult {
    Relation relation = Relation::Indeterminate;
    std::string body_id, obstacle_id, reason, backend;
    std::optional<double> distance;
    std::optional<Vec3> on_body, on_obstacle;
};
QueryResult query_pair(const Part &body, const Pose &body_pose, const Part &obstacle,
                       double tolerance = 1e-8);
struct QueryStats {
    std::uint64_t states = 0, motions = 0, narrow = 0, broad = 0, samples = 0;
};
struct StateResult {
    bool valid = false;
    std::string reason;
    QueryResult query;
};
enum class CheckStatus { Passed, Failed, Incomplete, Cancelled };
struct MotionResult {
    CheckStatus status = CheckStatus::Incomplete;
    std::uint64_t samples = 0;
    double failed_u = 0;
    std::string reason;
    StateResult failure;
    std::uint64_t intervals = 0;
    double translation_step = 0, rotation_step = 0, surface_step = 0;
    std::optional<std::size_t> failed_edge;
};
class Validator {
  public:
    explicit Validator(const Scene &);
    ~Validator();
    Validator(const Validator &) = delete;
    Validator &operator=(const Validator &) = delete;
    StateResult state(const Pose &, const ValidationConfig &, bool distance = false);
    MotionResult motion(const Pose &, const Pose &, const ValidationConfig &, std::stop_token = {});
    QueryStats stats() const;
    void set_query_budget(std::uint64_t maximum_narrow_queries);

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
struct Path {
    std::vector<Pose> poses;
    std::string scene_hash;
    std::uint64_t scene_revision = 0;
    ValidationConfig validation;
    std::string source = "rrt-connect";
    std::string request_id;
};
struct PathMetrics {
    double translation = 0, rotation = 0, objective = 0;
    std::optional<double> min_sampled_clearance;
    std::optional<Pose> min_clearance_pose;
    std::optional<double> min_clearance_position; // normalized path-node playback position
    std::uint64_t samples = 0;
    bool complete = false;
};
enum class SolveStatus {
    Solved,
    InvalidStart,
    InvalidGoal,
    BudgetExhausted,
    Cancelled,
    ValidationIncomplete,
    Error
};
std::string to_string(SolveStatus);
struct SearchEdge {
    Pose from, to;
    bool start_tree = true;
    bool accepted = true;
};
struct PlanningResult {
    SolveStatus status = SolveStatus::Error;
    std::string reason;
    Path path;
    QueryStats queries;
    std::uint64_t iterations = 0, nodes = 0;
    double search_seconds = 0, validation_seconds = 0, total_seconds = 0;
    MotionResult verification;
    std::vector<SearchEdge> edges;
};
using Progress = std::function<void(std::uint64_t, std::uint64_t)>;
using SearchProgress = std::function<void(const SearchEdge &)>;
std::string new_request_id();
PlanningResult solve(const Scene &, const PlannerConfig &, std::stop_token = {}, Progress = {},
                     SearchProgress = {});
MotionResult validate_path(const Scene &, const Path &, const ValidationConfig &,
                           std::stop_token = {});
Path optimize(const Scene &, const Path &, const PlannerConfig &, std::stop_token = {});
PathMetrics evaluate(const Scene &, const Path &, const PlannerConfig &, std::stop_token = {});
PlanningResult solve_ompl(const Scene &, const PlannerConfig &, std::stop_token = {});
std::optional<double> ray_box(const Vec3 &origin, const Vec3 &direction, const Part &,
                              const Pose &parent = {});
} // namespace pl

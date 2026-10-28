#include "passagelab/core.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <fcl/fcl.h>
#include <limits>
#include <numeric>
#include <queue>
#include <set>

namespace pl {
namespace {
bool finite(const Vec3 &v) {
    return v.array().isFinite().all();
}
bool valid_part(const Part &p) {
    if (!valid_pose(p.pose))
        return false;
    if (p.shape == Shape::Box)
        return finite(p.half) && (p.half.array() > 0).all();
    if (p.shape == Shape::Sphere)
        return std::isfinite(p.radius) && p.radius > 0;
    return false;
}
Pose normalized(Pose p) {
    p.q.normalize();
    return p;
}
double aabb_distance(const Aabb &a, const Aabb &b) {
    Vec3 d = (a.min - b.max).cwiseMax(b.min - a.max).cwiseMax(Vec3::Zero());
    return d.norm();
}
bool aabb_overlap(const Aabb &a, const Aabb &b, double m = 0) {
    return (a.min.array() <= b.max.array() + m).all() && (b.min.array() <= a.max.array() + m).all();
}
QueryResult result(Relation r, const Part &a, const Part &b, const char *backend) {
    QueryResult q;
    q.relation = r;
    q.body_id = a.id;
    q.obstacle_id = b.id;
    q.backend = backend;
    return q;
}
double sat(const Part &a, const Pose &ap, const Part &b, double tol, Relation &rel) {
    const Pose ta = compose(ap, a.pose), tb = normalized(b.pose);
    Eigen::Matrix3d A = ta.q.toRotationMatrix(), B = tb.q.toRotationMatrix();
    const Vec3 ea = a.half, eb = b.half, delta = tb.t - ta.t;
    double min_gap = std::numeric_limits<double>::infinity();
    bool touch = false, numerical_failure = false;
    auto axis_check = [&](Vec3 axis) {
        const double n = axis.norm();
        if (n < 1e-10)
            return true;
        axis /= n;
        const double ra = (A.transpose() * axis).cwiseAbs().dot(ea);
        const double rb = (B.transpose() * axis).cwiseAbs().dot(eb);
        const double gap = std::abs(delta.dot(axis)) - ra - rb;
        min_gap = std::min(min_gap, gap);
        if (!std::isfinite(gap)) {
            numerical_failure = true;
            return false;
        }
        if (gap > tol)
            return false;
        if (gap >= -tol)
            touch = true;
        return true;
    };
    auto check = [&](const Vec3 &axis) {
        if (axis_check(axis))
            return true;
        rel = numerical_failure ? Relation::Indeterminate : Relation::Separated;
        return false;
    };
    for (int i = 0; i < 3; i++)
        if (!check(A.col(i)) || !check(B.col(i)))
            return min_gap;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            if (!check(A.col(i).cross(B.col(j))))
                return min_gap;
    rel = touch ? Relation::Touching : Relation::Overlapping;
    return min_gap;
}
QueryResult sphere_pair(const Part &a, const Pose &ap, const Part &b, double tol) {
    Pose ta = compose(ap, a.pose), tb = normalized(b.pose);
    Vec3 d = tb.t - ta.t;
    double n = d.norm(), gap = n - a.radius - b.radius;
    Relation r = gap > tol ? Relation::Separated
                           : (gap >= -tol ? Relation::Touching : Relation::Overlapping);
    auto q = result(r, a, b, "analytic-sphere");
    if (r == Relation::Separated) {
        Vec3 u = d / n;
        q.distance = gap;
        q.on_body = ta.t + u * a.radius;
        q.on_obstacle = tb.t - u * b.radius;
    }
    return q;
}
QueryResult sphere_box(const Part &s, const Pose &sp, const Part &box, bool sphere_first,
                       double tol) {
    Pose ts = compose(sp, s.pose), tb = normalized(box.pose);
    Eigen::Matrix3d R = tb.q.toRotationMatrix();
    Vec3 local = R.transpose() * (ts.t - tb.t),
         closest = local.cwiseMax(-box.half).cwiseMin(box.half);
    Vec3 diff = local - closest;
    double d = diff.norm();
    Relation r;
    double gap;
    if (d > tol) {
        gap = d - s.radius;
        r = gap > tol ? Relation::Separated
                      : (gap >= -tol ? Relation::Touching : Relation::Overlapping);
    } else {
        double face = (box.half - local.cwiseAbs()).minCoeff();
        gap = -(face + s.radius);
        r = Relation::Overlapping;
    }
    auto q = result(r, sphere_first ? s : box, sphere_first ? box : s, "analytic-sphere-box");
    if (r == Relation::Separated) {
        Vec3 u = R * (diff / d);
        Vec3 ps = ts.t - u * s.radius, pb = tb.t + R * closest;
        q.distance = gap;
        q.on_body = sphere_first ? ps : pb;
        q.on_obstacle = sphere_first ? pb : ps;
    }
    return q;
}
struct Node {
    Aabb box;
    int left = -1, right = -1;
    std::vector<size_t> ids;
};
struct Segment {
    Vec3 a, b;
};
std::array<Vec3, 8> box_vertices(const Part &p, const Pose &world) {
    std::array<Vec3, 8> vertices;
    Eigen::Matrix3d rotation = world.q.toRotationMatrix();
    for (int i = 0; i < 8; ++i) {
        Vec3 local((i & 1) ? p.half.x() : -p.half.x(), (i & 2) ? p.half.y() : -p.half.y(),
                   (i & 4) ? p.half.z() : -p.half.z());
        vertices[i] = world.t + rotation * local;
    }
    return vertices;
}
std::array<Segment, 12> box_edges(const std::array<Vec3, 8> &vertices) {
    std::array<Segment, 12> edges;
    size_t next = 0;
    for (int i = 0; i < 8; ++i)
        for (int bit : {1, 2, 4})
            if ((i & bit) == 0)
                edges[next++] = {vertices[i], vertices[i | bit]};
    return edges;
}
void update_closest(const Vec3 &a, const Vec3 &b, double &best_squared, Vec3 &on_a, Vec3 &on_b) {
    double squared = (a - b).squaredNorm();
    if (squared < best_squared) {
        best_squared = squared;
        on_a = a;
        on_b = b;
    }
}
void point_box_candidate(const Vec3 &point, const Part &box, const Pose &world,
                         double &best_squared, Vec3 &on_a, Vec3 &on_b, bool point_is_a) {
    Vec3 local = world.q.conjugate() * (point - world.t);
    Vec3 clamped = local.cwiseMax(-box.half).cwiseMin(box.half);
    Vec3 on_box = world.t + world.q * clamped;
    if (point_is_a)
        update_closest(point, on_box, best_squared, on_a, on_b);
    else
        update_closest(on_box, point, best_squared, on_a, on_b);
}
void segment_candidate(const Segment &x, const Segment &y, double &best_squared, Vec3 &on_a,
                       Vec3 &on_b) {
    const Vec3 u = x.b - x.a, v = y.b - y.a, w = x.a - y.a;
    const double a = u.dot(u), b = u.dot(v), c = v.dot(v), d = u.dot(w), e = v.dot(w);
    const double denominator = a * c - b * b;
    double s = 0, t = 0;
    if (denominator <= std::numeric_limits<double>::epsilon() * a * c) {
        s = 0;
        t = std::clamp(e / c, 0.0, 1.0);
    } else {
        s = std::clamp((b * e - c * d) / denominator, 0.0, 1.0);
        t = (b * s + e) / c;
        if (t < 0) {
            t = 0;
            s = std::clamp(-d / a, 0.0, 1.0);
        } else if (t > 1) {
            t = 1;
            s = std::clamp((b - d) / a, 0.0, 1.0);
        }
    }
    update_closest(x.a + s * u, y.a + t * v, best_squared, on_a, on_b);
}
QueryResult box_feature_distance(const Part &a, const Pose &ta, const Part &b, const Pose &tb,
                                 QueryResult q) {
    auto vertices_a = box_vertices(a, ta), vertices_b = box_vertices(b, tb);
    auto edges_a = box_edges(vertices_a), edges_b = box_edges(vertices_b);
    double best_squared = std::numeric_limits<double>::infinity();
    Vec3 on_a = Vec3::Zero(), on_b = Vec3::Zero();
    for (const auto &point : vertices_a)
        point_box_candidate(point, b, tb, best_squared, on_a, on_b, true);
    for (const auto &point : vertices_b)
        point_box_candidate(point, a, ta, best_squared, on_a, on_b, false);
    for (const auto &edge_a : edges_a)
        for (const auto &edge_b : edges_b)
            segment_candidate(edge_a, edge_b, best_squared, on_a, on_b);
    if (!std::isfinite(best_squared) || best_squared < 0 || !finite(on_a) || !finite(on_b)) {
        q.relation = Relation::Indeterminate;
        q.reason = "box feature distance failed";
        return q;
    }
    q.distance = std::sqrt(best_squared);
    q.on_body = on_a;
    q.on_obstacle = on_b;
    q.backend = "obb-sat+box-feature-distance";
    return q;
}
} // namespace

Pose compose(const Pose &a, const Pose &b) {
    Quat qa = a.q.normalized(), qb = b.q.normalized();
    return {a.t + qa * b.t, (qa * qb).normalized()};
}
Pose inverse(const Pose &a) {
    Quat q = a.q.normalized().conjugate();
    return {q * (-a.t), q};
}
Pose interpolate(const Pose &a, const Pose &b, double u) {
    Quat qa = a.q.normalized(), q = b.q.normalized();
    if (qa.dot(q) < 0)
        q.coeffs() *= -1;
    return {(1 - u) * a.t + u * b.t, qa.slerp(u, q).normalized()};
}
double rotation_distance(const Quat &a, const Quat &b) {
    Quat rel = a.normalized().conjugate() * b.normalized();
    return 2 * std::atan2(rel.vec().norm(), std::abs(rel.w()));
}
double pose_distance(const Pose &a, const Pose &b, double w) {
    return std::hypot((a.t - b.t).norm(), w * rotation_distance(a.q, b.q));
}
bool valid_pose(const Pose &p) {
    double n = p.q.norm();
    return finite(p.t) && p.q.coeffs().array().isFinite().all() && std::isfinite(n) && n > 1e-12 &&
           std::abs(n - 1) <= 1e-3;
}
Aabb world_aabb(const Part &p, const Pose &parent) {
    Pose t = compose(parent, p.pose);
    Vec3 e;
    if (p.shape == Shape::Sphere)
        e = Vec3::Constant(p.radius);
    else
        e = t.q.toRotationMatrix().cwiseAbs() * p.half;
    return {t.t - e, t.t + e};
}
Vec3 support_point(const Part &p, const Pose &parent, const Vec3 &direction) {
    Pose t = compose(parent, p.pose);
    Vec3 d = direction.normalized();
    if (p.shape == Shape::Sphere)
        return t.t + p.radius * d;
    Vec3 local = t.q.conjugate() * d;
    Vec3 corner =
        p.half.cwiseProduct(local.unaryExpr([](double v) { return v >= 0 ? 1.0 : -1.0; }));
    return t.t + t.q * corner;
}
double body_radius(const Scene &s) {
    double r = 0;
    for (const auto &p : s.body) {
        Pose t = p.pose;
        r = std::max(r, t.t.norm() + (p.shape == Shape::Box ? p.half.norm() : p.radius));
    }
    return r;
}
std::vector<std::string> scene_errors(const Scene &s) {
    std::vector<std::string> e;
    if (s.body.empty())
        e.push_back("body is empty");
    std::set<std::string> ids;
    auto check_id = [&](const Part &p) {
        if (p.id.empty())
            e.push_back("empty part id");
        else if (!ids.insert(p.id).second)
            e.push_back("duplicate part id: " + p.id);
    };
    for (const auto &p : s.body) {
        check_id(p);
        if (!valid_part(p))
            e.push_back("invalid body part: " + p.id);
    }
    for (const auto &p : s.obstacles) {
        check_id(p);
        if (!valid_part(p))
            e.push_back("invalid obstacle: " + p.id);
    }
    if (!valid_pose(s.start) || !valid_pose(s.goal))
        e.push_back("invalid start or goal pose");
    if (!finite(s.sampling.min) || !finite(s.sampling.max) || !finite(s.physical.min) ||
        !finite(s.physical.max) || (s.sampling.min.array() >= s.sampling.max.array()).any() ||
        (s.physical.min.array() >= s.physical.max.array()).any())
        e.push_back("invalid bounds");
    return e;
}
QueryResult query_pair(const Part &a, const Pose &ap, const Part &b, double tol) {
    if (!valid_part(a) || !valid_part(b) || !valid_pose(ap) || !std::isfinite(tol) || tol < 0)
        return result(Relation::Indeterminate, a, b, "invalid-input");
    const Pose aa = compose(ap, a.pose), bb = normalized(b.pose);
    const double scale = std::max({1.0, aa.t.cwiseAbs().maxCoeff(), bb.t.cwiseAbs().maxCoeff(),
                                   (a.shape == Shape::Box ? a.half.norm() : a.radius),
                                   (b.shape == Shape::Box ? b.half.norm() : b.radius)});
    tol += 32 * std::numeric_limits<double>::epsilon() * scale;
    if (a.shape == Shape::Sphere && b.shape == Shape::Sphere)
        return sphere_pair(a, ap, b, tol);
    if (a.shape == Shape::Sphere)
        return sphere_box(a, ap, b, true, tol);
    if (b.shape == Shape::Sphere) {
        Part world_body = a;
        world_body.pose = compose(ap, a.pose);
        return sphere_box(b, Pose{}, world_body, false, tol);
    }
    Relation rel;
    sat(a, ap, b, tol, rel);
    auto q = result(rel, a, b, "obb-sat");
    if (rel == Relation::Separated) {
        auto ga = std::make_shared<fcl::Boxd>(2 * a.half.x(), 2 * a.half.y(), 2 * a.half.z());
        auto gb = std::make_shared<fcl::Boxd>(2 * b.half.x(), 2 * b.half.y(), 2 * b.half.z());
        Pose ta = compose(ap, a.pose);
        fcl::Transform3d xa = fcl::Transform3d::Identity(), xb = fcl::Transform3d::Identity();
        Pose tb = normalized(b.pose);
        xa.linear() = ta.q.toRotationMatrix();
        xa.translation() = ta.t;
        xb.linear() = tb.q.toRotationMatrix();
        xb.translation() = tb.t;
        // Keep FCL as an independent audit; its GJK distance and support-point tie
        // breaking are not sufficiently symmetric for authoritative clearance.
        fcl::DistanceRequestd req(true, false, 0.0, 0.0, 1e-9, fcl::GST_INDEP);
        fcl::DistanceResultd res;
        const double fcl_distance = fcl::distance(ga.get(), xa, gb.get(), xb, req, res);
        q = box_feature_distance(a, ta, b, tb, std::move(q));
        if (q.relation != Relation::Separated || !q.distance)
            return q;
        const double audit_tolerance = std::max(1e-9, 1e-8 * std::max(1.0, *q.distance));
        if (!std::isfinite(fcl_distance) || fcl_distance < 0 ||
            std::abs(fcl_distance - *q.distance) > audit_tolerance)
            q.reason = "FCL GST_INDEP distance audit differs from exact box feature distance";
    }
    return q;
}

struct Validator::Impl {
    Scene scene;
    QueryStats stats;
    std::vector<Node> nodes;
    std::vector<Aabb> obstacle_bounds;
    int root = -1;
    double obstacle_scale = 1.0;
    std::vector<std::string> errors;
    std::uint64_t max_narrow_queries = std::numeric_limits<std::uint64_t>::max();
    explicit Impl(const Scene &s) : scene(s) {
        Scene geometry_scene = s;
        geometry_scene.start = Pose{};
        geometry_scene.goal = Pose{};
        errors = scene_errors(geometry_scene);
        if (!errors.empty())
            return;
        obstacle_bounds.reserve(s.obstacles.size());
        for (const auto &obstacle : s.obstacles) {
            Aabb b = world_aabb(obstacle);
            obstacle_bounds.push_back(b);
            obstacle_scale = std::max({obstacle_scale, b.min.cwiseAbs().maxCoeff(),
                                       b.max.cwiseAbs().maxCoeff(), (b.max - b.min).norm()});
        }
        std::vector<size_t> ids(s.obstacles.size());
        std::iota(ids.begin(), ids.end(), 0);
        std::function<int(std::vector<size_t>)> build = [&](std::vector<size_t> v) -> int {
            if (v.empty())
                return -1;
            Node n;
            n.box = obstacle_bounds[v[0]];
            for (size_t i : v) {
                const auto &b = obstacle_bounds[i];
                n.box.min = n.box.min.cwiseMin(b.min);
                n.box.max = n.box.max.cwiseMax(b.max);
            }
            int idx = (int)nodes.size();
            nodes.push_back(n);
            if (v.size() <= 4)
                nodes[idx].ids = std::move(v);
            else {
                Vec3 extent = n.box.max - n.box.min;
                Eigen::Index ax;
                extent.maxCoeff(&ax);
                std::sort(v.begin(), v.end(), [&](size_t a, size_t b) {
                    return obstacle_bounds[a].min[ax] + obstacle_bounds[a].max[ax] <
                           obstacle_bounds[b].min[ax] + obstacle_bounds[b].max[ax];
                });
                auto mid = v.begin() + v.size() / 2;
                std::vector<size_t> l(v.begin(), mid), r(mid, v.end());
                nodes[idx].left = build(std::move(l));
                nodes[idx].right = build(std::move(r));
            }
            return idx;
        };
        root = build(std::move(ids));
    }
    std::vector<size_t> candidates(const Aabb &box, double margin, bool bvh) {
        std::vector<size_t> out;
        if (!bvh || root < 0) {
            for (size_t i = 0; i < scene.obstacles.size(); ++i)
                out.push_back(i);
            return out;
        }
        std::vector<int> st{root};
        while (!st.empty()) {
            int i = st.back();
            st.pop_back();
            ++stats.broad;
            const auto &n = nodes[i];
            if (!aabb_overlap(box, n.box, margin))
                continue;
            if (n.left < 0 && n.right < 0)
                out.insert(out.end(), n.ids.begin(), n.ids.end());
            else {
                if (n.left >= 0)
                    st.push_back(n.left);
                if (n.right >= 0)
                    st.push_back(n.right);
            }
        }
        return out;
    }
    template <class F> void visit_nearest(const Aabb &box, double &best, F &&visit) {
        struct Item {
            double lower;
            int code;
        };
        auto cmp = [](const Item &a, const Item &b) { return a.lower > b.lower; };
        std::priority_queue<Item, std::vector<Item>, decltype(cmp)> pending(cmp);
        if (root >= 0)
            pending.push({aabb_distance(box, nodes[root].box), root});
        while (!pending.empty()) {
            Item item = pending.top();
            pending.pop();
            if (item.lower > best)
                break;
            if (item.code < 0) {
                size_t id = size_t(-item.code - 1);
                visit(id, item.lower, best);
                continue;
            }
            ++stats.broad;
            const Node &node = nodes[item.code];
            if (node.left < 0 && node.right < 0)
                for (size_t id : node.ids)
                    pending.push({aabb_distance(box, obstacle_bounds[id]), -int(id) - 1});
            else {
                if (node.left >= 0)
                    pending.push({aabb_distance(box, nodes[node.left].box), node.left});
                if (node.right >= 0)
                    pending.push({aabb_distance(box, nodes[node.right].box), node.right});
            }
        }
    }
};
Validator::Validator(const Scene &s) : impl_(std::make_unique<Impl>(s)) {}
Validator::~Validator() = default;
QueryStats Validator::stats() const {
    return impl_->stats;
}
void Validator::set_query_budget(std::uint64_t maximum_narrow_queries) {
    impl_->max_narrow_queries = maximum_narrow_queries;
}
StateResult Validator::state(const Pose &input, const ValidationConfig &c, bool want_distance) {
    auto &x = *impl_;
    ++x.stats.states;
    StateResult out;
    if (!x.errors.empty()) {
        out.reason = "invalid scene: " + x.errors.front();
        out.query.relation = Relation::Indeterminate;
        return out;
    }
    if (!valid_pose(input) || !std::isfinite(c.margin) || c.margin < 0 ||
        !std::isfinite(c.tolerance) || c.tolerance < 0) {
        out.reason = "invalid input";
        out.query.relation = Relation::Indeterminate;
        return out;
    }
    Pose pose = normalized(input);
    if ((pose.t.array() < x.scene.sampling.min.array()).any() ||
        (pose.t.array() > x.scene.sampling.max.array()).any()) {
        out.reason = "sampling bounds";
        out.query.relation = Relation::Separated;
        return out;
    }
    double nearest = std::numeric_limits<double>::infinity();
    bool have = false;
    for (const auto &p : x.scene.body) {
        Aabb wb = world_aabb(p, pose);
        const double scale = std::max({1.0, wb.min.cwiseAbs().maxCoeff(),
                                       wb.max.cwiseAbs().maxCoeff(), (wb.max - wb.min).norm()});
        const double tol = c.tolerance + 32 * std::numeric_limits<double>::epsilon() * scale;
        const Vec3 low_gap = wb.min - x.scene.physical.min,
                   high_gap = x.scene.physical.max - wb.max;
        Eigen::Index low_axis, high_axis;
        double low = low_gap.minCoeff(&low_axis), high = high_gap.minCoeff(&high_axis);
        double wall_gap = std::min(low, high);
        if (wall_gap <= c.margin + tol) {
            out.reason = "physical bounds clearance";
            out.query.relation = wall_gap < -tol
                                     ? Relation::Overlapping
                                     : (wall_gap <= tol ? Relation::Touching : Relation::Separated);
            out.query.body_id = p.id;
            out.query.obstacle_id = "physical-boundary";
            out.query.backend = "physical-bounds";
            if (wall_gap >= 0)
                out.query.distance = wall_gap;
            Vec3 body_witness, wall_witness;
            if (low <= high) {
                Vec3 dir = Vec3::Zero();
                dir[low_axis] = -1;
                body_witness = support_point(p, pose, dir);
                wall_witness = body_witness;
                wall_witness[low_axis] = x.scene.physical.min[low_axis];
            } else {
                Vec3 dir = Vec3::Zero();
                dir[high_axis] = 1;
                body_witness = support_point(p, pose, dir);
                wall_witness = body_witness;
                wall_witness[high_axis] = x.scene.physical.max[high_axis];
            }
            out.query.on_body = body_witness;
            out.query.on_obstacle = wall_witness;
            return out;
        }
        if (want_distance && wall_gap < nearest) {
            nearest = wall_gap;
            have = true;
            out.query.relation = Relation::Separated;
            out.query.body_id = p.id;
            out.query.obstacle_id = "physical-boundary";
            out.query.backend = "physical-bounds";
            out.query.distance = wall_gap;
            Vec3 body_witness, wall_witness;
            if (low <= high) {
                Vec3 dir = Vec3::Zero();
                dir[low_axis] = -1;
                body_witness = support_point(p, pose, dir);
                wall_witness = body_witness;
                wall_witness[low_axis] = x.scene.physical.min[low_axis];
            } else {
                Vec3 dir = Vec3::Zero();
                dir[high_axis] = 1;
                body_witness = support_point(p, pose, dir);
                wall_witness = body_witness;
                wall_witness[high_axis] = x.scene.physical.max[high_axis];
            }
            out.query.on_body = body_witness;
            out.query.on_obstacle = wall_witness;
        }
        auto evaluate = [&](size_t oi, double lower, double &best) -> StateResult {
            if (want_distance && c.use_bvh && have && lower > best)
                return StateResult{};
            if (x.stats.narrow >= x.max_narrow_queries) {
                StateResult fail;
                fail.reason = "geometry query budget exhausted";
                fail.query.relation = Relation::Indeterminate;
                fail.query.body_id = p.id;
                fail.query.obstacle_id = x.scene.obstacles[oi].id;
                fail.query.backend = "query-budget";
                return fail;
            }
            ++x.stats.narrow;
            auto q = query_pair(p, pose, x.scene.obstacles[oi], tol);
            if (q.relation == Relation::Indeterminate) {
                StateResult fail;
                fail.reason = q.reason.empty() ? "indeterminate geometry" : q.reason;
                fail.query = q;
                return fail;
            }
            if (q.relation == Relation::Separated && c.margin > 0 && !q.distance) {
                StateResult fail;
                fail.reason = "distance unavailable for clearance margin";
                fail.query = q;
                fail.query.relation = Relation::Indeterminate;
                return fail;
            }
            if (q.relation != Relation::Separated ||
                (q.distance && *q.distance <= c.margin + tol)) {
                StateResult fail;
                fail.reason =
                    q.relation == Relation::Separated ? "clearance margin" : "collision or contact";
                fail.query = q;
                return fail;
            }
            if (q.distance && *q.distance < best) {
                best = *q.distance;
                out.query = q;
                have = true;
            }
            return StateResult{};
        };
        if (want_distance && c.use_bvh) {
            // visit_nearest evaluates the exact distance in increasing AABB lower-bound order.
            // Stop as soon as every remaining lower bound exceeds the best exact distance.
            bool failed = false;
            x.visit_nearest(wb, nearest, [&](size_t id, double lower, double &best) {
                if (failed)
                    return;
                auto r = evaluate(id, lower, best);
                if (!r.reason.empty()) {
                    out = r;
                    failed = true;
                    best = 0;
                }
            });
            if (failed)
                return out;
        } else {
            double broad_scale = std::max(scale, x.obstacle_scale);
            auto ids = x.candidates(
                wb, c.margin + tol + 32 * std::numeric_limits<double>::epsilon() * broad_scale,
                c.use_bvh);
            if (want_distance && !c.use_bvh) {
                ids.resize(x.scene.obstacles.size());
                std::iota(ids.begin(), ids.end(), 0);
            }
            for (size_t oi : ids) {
                auto r = evaluate(oi, aabb_distance(wb, x.obstacle_bounds[oi]), nearest);
                if (!r.reason.empty())
                    return r;
            }
        }
    }
    if (want_distance && have)
        out.query.distance = nearest;
    out.valid = true;
    out.reason = "valid";
    return out;
}
MotionResult Validator::motion(const Pose &a, const Pose &b, const ValidationConfig &c,
                               std::stop_token stop) {
    auto &x = *impl_;
    ++x.stats.motions;
    MotionResult m;
    if (!valid_pose(a) || !valid_pose(b) || !std::isfinite(c.translation_step) ||
        c.translation_step <= 0 || !std::isfinite(c.rotation_step) || c.rotation_step <= 0 ||
        !std::isfinite(c.surface_step) || c.surface_step <= 0 || !std::isfinite(c.margin) ||
        c.margin < 0 || !std::isfinite(c.tolerance) || c.tolerance < 0 || c.max_samples < 2) {
        m.reason = "invalid motion configuration or pose";
        return m;
    }
    double L = (a.t - b.t).norm(), theta = rotation_distance(a.q, b.q),
           travel = L + body_radius(x.scene) * theta;
    double nd = std::max({1.0, std::ceil(L / c.translation_step),
                          std::ceil(theta / c.rotation_step), std::ceil(travel / c.surface_step)});
    if (!std::isfinite(nd) || nd + 1 > double(c.max_samples)) {
        m.reason = "sample limit exceeded";
        m.status = CheckStatus::Incomplete;
        return m;
    }
    auto n = static_cast<std::uint64_t>(nd);
    for (std::uint64_t i = 0; i <= n; ++i) {
        if (stop.stop_requested()) {
            m.status = CheckStatus::Cancelled;
            m.reason = "cancelled";
            return m;
        }
        auto s = state(interpolate(a, b, double(i) / n), c);
        ++m.samples;
        ++x.stats.samples;
        if (!s.valid) {
            m.status = s.query.relation == Relation::Indeterminate ? CheckStatus::Incomplete
                                                                   : CheckStatus::Failed;
            m.failed_u = double(i) / n;
            m.failure = s;
            m.reason = s.reason;
            return m;
        }
    }
    m.status = CheckStatus::Passed;
    return m;
}
std::optional<double> ray_box(const Vec3 &o, const Vec3 &d, const Part &p, const Pose &parent) {
    if (!finite(o) || !finite(d) || d.squaredNorm() < 1e-24 || p.shape != Shape::Box ||
        !valid_part(p) || !valid_pose(parent))
        return std::nullopt;
    Pose t = compose(parent, p.pose);
    Vec3 lo = t.q.conjugate() * (o - t.t), ld = t.q.conjugate() * d;
    double tmin = 0, tmax = std::numeric_limits<double>::infinity();
    for (int i = 0; i < 3; i++) {
        if (std::abs(ld[i]) < 1e-15) {
            if (lo[i] < -p.half[i] || lo[i] > p.half[i])
                return std::nullopt;
            continue;
        }
        double a = (-p.half[i] - lo[i]) / ld[i], b = (p.half[i] - lo[i]) / ld[i];
        if (a > b)
            std::swap(a, b);
        tmin = std::max(tmin, a);
        tmax = std::min(tmax, b);
        if (tmin > tmax)
            return std::nullopt;
    }
    return tmin;
}
} // namespace pl

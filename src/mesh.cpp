#include "passagelab/mesh.hpp"

#include "passagelab/io.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <map>
#include <numbers>
#include <numeric>
#include <set>
#include <stdexcept>
#include <unordered_map>

namespace pl {
namespace {
constexpr double kInf = std::numeric_limits<double>::infinity();
bool finite(const Vec3 &v) {
    return v.array().isFinite().all();
}
std::uint64_t edge_key(std::uint32_t a, std::uint32_t b) {
    return (std::uint64_t(std::min(a, b)) << 32) | std::max(a, b);
}
void check_mesh(const TriangleMesh &m) {
    if (m.vertices.size() > std::numeric_limits<std::uint32_t>::max())
        throw std::invalid_argument("mesh has too many vertices");
    for (const auto &v : m.vertices)
        if (!finite(v))
            throw std::invalid_argument("mesh has a nonfinite vertex");
    for (const auto &t : m.triangles)
        for (auto i : t)
            if (i >= m.vertices.size())
                throw std::invalid_argument("mesh triangle index out of range");
    if (!m.faces.empty() && m.faces.size() != m.triangles.size())
        throw std::invalid_argument("mesh face attribute does not match triangle count");
}
struct CellHash {
    std::size_t operator()(const std::array<std::int64_t, 3> &k) const {
        std::uint64_t h = 1469598103934665603ull;
        for (auto v : k) {
            h ^= std::uint64_t(v);
            h *= 1099511628211ull;
        }
        return std::size_t(h);
    }
};
double point_aabb_squared(const Vec3 &p, const Aabb &b) {
    return (b.min - p).cwiseMax(p - b.max).cwiseMax(Vec3::Zero()).squaredNorm();
}
double aabb_aabb_squared(const Aabb &a, const Aabb &b) {
    return (a.min - b.max).cwiseMax(b.min - a.max).cwiseMax(Vec3::Zero()).squaredNorm();
}
Vec3 closest_on_segment(const Vec3 &p, const Vec3 &a, const Vec3 &b) {
    const Vec3 e = b - a;
    const double n = e.squaredNorm();
    return n > 0 ? Vec3(a + std::clamp((p - a).dot(e) / n, 0.0, 1.0) * e) : a;
}
// Ericson, Real-Time Collision Detection 5.1.5, extended with the Voronoi feature of the result.
Vec3 closest_on_triangle(const Vec3 &p, const Vec3 &a, const Vec3 &b, const Vec3 &c, int &feature) {
    const Vec3 ab = b - a, ac = c - a, ap = p - a;
    const double d1 = ab.dot(ap), d2 = ac.dot(ap);
    if (d1 <= 0 && d2 <= 0) {
        feature = 0;
        return a;
    }
    const Vec3 bp = p - b;
    const double d3 = ab.dot(bp), d4 = ac.dot(bp);
    if (d3 >= 0 && d4 <= d3) {
        feature = 1;
        return b;
    }
    const double vc = d1 * d4 - d3 * d2;
    if (vc <= 0 && d1 >= 0 && d3 <= 0) {
        feature = 3;
        return d1 - d3 > 0 ? Vec3(a + d1 / (d1 - d3) * ab) : a;
    }
    const Vec3 cp = p - c;
    const double d5 = ab.dot(cp), d6 = ac.dot(cp);
    if (d6 >= 0 && d5 <= d6) {
        feature = 2;
        return c;
    }
    const double vb = d5 * d2 - d1 * d6;
    if (vb <= 0 && d2 >= 0 && d6 <= 0) {
        feature = 5;
        return d2 - d6 > 0 ? Vec3(a + d2 / (d2 - d6) * ac) : a;
    }
    const double va = d3 * d6 - d5 * d4;
    if (va <= 0 && d4 - d3 >= 0 && d5 - d6 >= 0) {
        feature = 4;
        const double s = (d4 - d3) + (d5 - d6);
        return s > 0 ? Vec3(b + (d4 - d3) / s * (c - b)) : b;
    }
    const double sum = va + vb + vc;
    if (!(sum > 0)) { // collinear triangle: fall back to its three edges
        const Vec3 q[3] = {closest_on_segment(p, a, b), closest_on_segment(p, b, c),
                           closest_on_segment(p, c, a)};
        int best = 0;
        for (int i = 1; i < 3; ++i)
            if ((p - q[i]).squaredNorm() < (p - q[best]).squaredNorm())
                best = i;
        feature = 3 + best;
        return q[best];
    }
    feature = 6;
    return a + ab * (vb / sum) + ac * (vc / sum);
}
void segment_pair(const Vec3 &a0, const Vec3 &a1, const Vec3 &b0, const Vec3 &b1, double &best,
                  Vec3 &on_a, Vec3 &on_b) {
    const Vec3 u = a1 - a0, v = b1 - b0, w = a0 - b0;
    const double a = u.dot(u), b = u.dot(v), c = v.dot(v), d = u.dot(w), e = v.dot(w);
    const double denominator = a * c - b * b;
    double s = 0, t = 0;
    if (a <= 0 && c <= 0) {
        s = t = 0;
    } else if (a <= 0) {
        t = std::clamp(e / c, 0.0, 1.0);
    } else if (c <= 0) {
        s = std::clamp(-d / a, 0.0, 1.0);
    } else if (denominator <= std::numeric_limits<double>::epsilon() * a * c) {
        // Parallel: every endpoint projection is a candidate, so keep the best of the four.
        const Vec3 candidates[4][2] = {{a0, closest_on_segment(a0, b0, b1)},
                                       {a1, closest_on_segment(a1, b0, b1)},
                                       {closest_on_segment(b0, a0, a1), b0},
                                       {closest_on_segment(b1, a0, a1), b1}};
        for (const auto &pair : candidates) {
            const double squared = (pair[0] - pair[1]).squaredNorm();
            if (squared < best) {
                best = squared;
                on_a = pair[0];
                on_b = pair[1];
            }
        }
        return;
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
    const Vec3 pa = a0 + s * u, pb = b0 + t * v;
    const double squared = (pa - pb).squaredNorm();
    if (squared < best) {
        best = squared;
        on_a = pa;
        on_b = pb;
    }
}
struct BoxFrame {
    Vec3 center, half;
    Eigen::Matrix3d axes;
    std::array<Vec3, 8> vertices;
    std::array<std::array<int, 2>, 12> edges;
    Aabb bounds;
};
BoxFrame box_frame(const Vec3 &half, const Pose &pose) {
    BoxFrame f;
    f.center = pose.t;
    f.half = half;
    f.axes = pose.q.normalized().toRotationMatrix();
    for (int i = 0; i < 8; ++i)
        f.vertices[size_t(i)] =
            f.center + f.axes * Vec3((i & 1) ? half.x() : -half.x(), (i & 2) ? half.y() : -half.y(),
                                     (i & 4) ? half.z() : -half.z());
    size_t next = 0;
    for (int i = 0; i < 8; ++i)
        for (int bit : {1, 2, 4})
            if ((i & bit) == 0)
                f.edges[next++] = {i, i | bit};
    const Vec3 extent = f.axes.cwiseAbs() * half;
    f.bounds = {f.center - extent, f.center + extent};
    return f;
}
// Separating-axis test of a solid box against a triangle: three box face normals, the triangle
// normal and nine edge cross products.
Relation triangle_box_sat(const BoxFrame &box, const Vec3 v[3], double tol) {
    bool touch = false, failed = false;
    const Vec3 r[3] = {v[0] - box.center, v[1] - box.center, v[2] - box.center};
    auto overlaps = [&](Vec3 axis) {
        const double n = axis.norm();
        if (n < 1e-10)
            return true;
        axis /= n;
        const double radius = (box.axes.transpose() * axis).cwiseAbs().dot(box.half);
        const double p0 = r[0].dot(axis), p1 = r[1].dot(axis), p2 = r[2].dot(axis);
        const double gap =
            std::max(std::min({p0, p1, p2}) - radius, -std::max({p0, p1, p2}) - radius);
        if (!std::isfinite(gap)) {
            failed = true;
            return false;
        }
        if (gap > tol)
            return false;
        if (gap >= -tol)
            touch = true;
        return true;
    };
    const Vec3 edges[3] = {(v[1] - v[0]).normalized(), (v[2] - v[1]).normalized(),
                           (v[0] - v[2]).normalized()};
    bool separated = false;
    for (int i = 0; i < 3 && !separated; ++i)
        separated = !overlaps(box.axes.col(i));
    if (!separated)
        separated = !overlaps(edges[0].cross(edges[1]));
    for (int i = 0; i < 3 && !separated; ++i)
        for (int j = 0; j < 3 && !separated; ++j)
            separated = !overlaps(box.axes.col(i).cross(edges[j]));
    if (failed)
        return Relation::Indeterminate;
    if (separated)
        return Relation::Separated;
    return touch ? Relation::Touching : Relation::Overlapping;
}
// For a separated box and triangle the minimum is attained by a vertex against the other shape or
// by an edge pair, so enumerating those features is exact.
void triangle_box_distance(const BoxFrame &box, const Vec3 v[3], double &best, Vec3 &on_box,
                           Vec3 &on_mesh) {
    for (int i = 0; i < 3; ++i) {
        const Vec3 local = box.axes.transpose() * (v[i] - box.center);
        const Vec3 q = box.center + box.axes * local.cwiseMax(-box.half).cwiseMin(box.half);
        const double squared = (q - v[i]).squaredNorm();
        if (squared < best) {
            best = squared;
            on_box = q;
            on_mesh = v[i];
        }
    }
    for (const auto &corner : box.vertices) {
        int feature = 6;
        const Vec3 q = closest_on_triangle(corner, v[0], v[1], v[2], feature);
        const double squared = (q - corner).squaredNorm();
        if (squared < best) {
            best = squared;
            on_box = corner;
            on_mesh = q;
        }
    }
    for (const auto &edge : box.edges)
        for (int i = 0; i < 3; ++i)
            segment_pair(box.vertices[size_t(edge[0])], box.vertices[size_t(edge[1])], v[i],
                         v[(i + 1) % 3], best, on_box, on_mesh);
}
bool ray_aabb(const Vec3 &o, const Vec3 &inv, const Aabb &b, double limit) {
    double tmin = 0, tmax = limit;
    for (int i = 0; i < 3; ++i) {
        double a = (b.min[i] - o[i]) * inv[i], c = (b.max[i] - o[i]) * inv[i];
        if (std::isnan(a) || std::isnan(c)) // origin on a slab plane of a parallel ray
            continue;
        if (a > c)
            std::swap(a, c);
        tmin = std::max(tmin, a);
        tmax = std::min(tmax, c);
        if (tmin > tmax)
            return false;
    }
    return true;
}
} // namespace

TriangleMesh weld(const TriangleMesh &in, double tolerance, WeldStats *stats) {
    if (!std::isfinite(tolerance) || tolerance < 0)
        throw std::invalid_argument("weld tolerance must be finite and nonnegative");
    check_mesh(in);
    std::vector<Vec3> merged;
    std::vector<std::uint32_t> remap(in.vertices.size());
    if (tolerance == 0) {
        std::map<std::array<double, 3>, std::uint32_t> seen;
        for (size_t i = 0; i < in.vertices.size(); ++i) {
            const Vec3 &v = in.vertices[i];
            // Adding zero folds -0.0 into +0.0 so both map to one key.
            auto [it, added] =
                seen.try_emplace({v.x() + 0.0, v.y() + 0.0, v.z() + 0.0}, std::uint32_t(merged.size()));
            if (added)
                merged.push_back(v);
            remap[i] = it->second;
        }
    } else {
        // Uniform grid with cells twice the tolerance: any vertex within the tolerance lies in the
        // 27 cells around the query.
        const double cell = 2 * tolerance;
        std::unordered_map<std::array<std::int64_t, 3>, std::vector<std::uint32_t>, CellHash> grid;
        for (size_t i = 0; i < in.vertices.size(); ++i) {
            const Vec3 &v = in.vertices[i];
            if ((v.cwiseAbs().array() / cell > 4e18).any())
                throw std::invalid_argument("weld tolerance is too small for the mesh extent");
            const std::array<std::int64_t, 3> key = {std::int64_t(std::floor(v.x() / cell)),
                                                     std::int64_t(std::floor(v.y() / cell)),
                                                     std::int64_t(std::floor(v.z() / cell))};
            std::uint32_t found = std::numeric_limits<std::uint32_t>::max();
            double best = tolerance * tolerance;
            for (std::int64_t dx = -1; dx <= 1; ++dx)
                for (std::int64_t dy = -1; dy <= 1; ++dy)
                    for (std::int64_t dz = -1; dz <= 1; ++dz) {
                        auto it = grid.find({key[0] + dx, key[1] + dy, key[2] + dz});
                        if (it == grid.end())
                            continue;
                        for (auto candidate : it->second) {
                            const double squared = (merged[candidate] - v).squaredNorm();
                            if (squared <= best) {
                                best = squared;
                                found = candidate;
                            }
                        }
                    }
            if (found == std::numeric_limits<std::uint32_t>::max()) {
                found = std::uint32_t(merged.size());
                merged.push_back(v);
                grid[key].push_back(found);
            }
            remap[i] = found;
        }
    }
    TriangleMesh out;
    std::vector<std::uint32_t> compact(merged.size(), std::numeric_limits<std::uint32_t>::max());
    std::size_t collapsed = 0;
    for (size_t i = 0; i < in.triangles.size(); ++i) {
        std::array<std::uint32_t, 3> t = {remap[in.triangles[i][0]], remap[in.triangles[i][1]],
                                          remap[in.triangles[i][2]]};
        if (t[0] == t[1] || t[1] == t[2] || t[2] == t[0]) {
            ++collapsed;
            continue;
        }
        for (auto &index : t) {
            if (compact[index] == std::numeric_limits<std::uint32_t>::max()) {
                compact[index] = std::uint32_t(out.vertices.size());
                out.vertices.push_back(merged[index]);
            }
            index = compact[index];
        }
        out.triangles.push_back(t);
        if (!in.faces.empty())
            out.faces.push_back(in.faces[i]);
    }
    if (stats)
        *stats = {in.vertices.size(), out.vertices.size(), in.triangles.size(),
                  out.triangles.size(), collapsed};
    return out;
}

MeshReport analyze(const TriangleMesh &m) {
    check_mesh(m);
    MeshReport r;
    r.vertices = m.vertices.size();
    r.triangles = m.triangles.size();
    if (m.triangles.empty())
        return r;
    Vec3 low = Vec3::Constant(kInf), high = Vec3::Constant(-kInf);
    std::vector<std::uint32_t> parent(m.vertices.size());
    std::iota(parent.begin(), parent.end(), 0u);
    auto root = [&](std::uint32_t i) {
        while (parent[i] != i)
            i = parent[i] = parent[parent[i]];
        return i;
    };
    std::vector<char> used(m.vertices.size(), 0);
    for (const auto &t : m.triangles)
        for (auto i : t) {
            used[i] = 1;
            low = low.cwiseMin(m.vertices[i]);
            high = high.cwiseMax(m.vertices[i]);
        }
    r.bounds = {low, high};
    const Vec3 origin = 0.5 * (low + high); // reduces cancellation in the signed volume
    struct Uses {
        std::uint32_t forward = 0, backward = 0;
    };
    std::unordered_map<std::uint64_t, Uses> edges;
    edges.reserve(m.triangles.size() * 2);
    std::set<std::array<std::uint32_t, 3>> unique;
    r.min_edge = kInf;
    for (const auto &t : m.triangles) {
        const Vec3 a = m.vertices[t[0]] - origin, b = m.vertices[t[1]] - origin,
                   c = m.vertices[t[2]] - origin;
        const double twice_area = (b - a).cross(c - a).norm();
        const double longest =
            std::max({(b - a).squaredNorm(), (c - b).squaredNorm(), (a - c).squaredNorm()});
        const bool repeated = t[0] == t[1] || t[1] == t[2] || t[2] == t[0];
        if (repeated || !(twice_area > 1e-12 * longest))
            ++r.degenerate_triangles;
        r.area += 0.5 * twice_area;
        r.volume += a.dot(b.cross(c)) / 6;
        auto sorted = t;
        std::sort(sorted.begin(), sorted.end());
        if (!unique.insert(sorted).second)
            ++r.duplicate_triangles;
        for (int i = 0; i < 3; ++i) {
            const std::uint32_t from = t[size_t(i)], to = t[size_t((i + 1) % 3)];
            if (from == to)
                continue;
            const double length = (m.vertices[from] - m.vertices[to]).norm();
            r.min_edge = std::min(r.min_edge, length);
            r.max_edge = std::max(r.max_edge, length);
            auto &uses = edges[edge_key(from, to)];
            ++(from < to ? uses.forward : uses.backward);
            parent[root(from)] = root(to);
        }
    }
    if (!std::isfinite(r.min_edge))
        r.min_edge = 0;
    for (const auto &[key, uses] : edges) {
        const auto total = uses.forward + uses.backward;
        if (total == 1)
            ++r.boundary_edges;
        else if (total > 2)
            ++r.nonmanifold_edges;
        else if (uses.forward != uses.backward)
            ++r.inconsistent_edges;
    }
    for (std::uint32_t i = 0; i < m.vertices.size(); ++i)
        if (used[i] && root(i) == i)
            ++r.components;
    r.closed = r.boundary_edges == 0 && r.nonmanifold_edges == 0;
    r.oriented = r.inconsistent_edges == 0;
    return r;
}

TrianglePoint point_triangle(const Vec3 &p, const Vec3 &a, const Vec3 &b, const Vec3 &c) {
    TrianglePoint out;
    out.closest = closest_on_triangle(p, a, b, c, out.feature);
    out.distance = (p - out.closest).norm();
    return out;
}

MeshShape::MeshShape(TriangleMesh mesh) : mesh_(std::move(mesh)) {
    report_ = analyze(mesh_);
    if (mesh_.triangles.empty())
        throw std::invalid_argument("mesh has no triangles");
    const auto count = mesh_.triangles.size();
    solid_ = report_.closed && report_.oriented && report_.volume != 0;
    orientation_ = report_.volume < 0 ? -1 : 1;

    // Content digest: vertices then indices, so the scene hash changes with the tessellation.
    std::vector<unsigned char> bytes(mesh_.vertices.size() * 24 + count * 12);
    auto *cursor = bytes.data();
    for (const auto &v : mesh_.vertices) {
        std::memcpy(cursor, v.data(), 24);
        cursor += 24;
    }
    for (const auto &t : mesh_.triangles) {
        std::memcpy(cursor, t.data(), 12);
        cursor += 12;
    }
    digest_ = sha256_hex(bytes.data(), bytes.size());

    // Angle-weighted vertex pseudonormals and summed edge pseudonormals (Baerentzen and Aanaes):
    // the sign of (p - closest) against the pseudonormal of the closest feature decides inside.
    face_normals_.resize(count);
    vertex_normals_.assign(mesh_.vertices.size(), Vec3::Zero());
    edge_normals_.assign(count, {Vec3::Zero(), Vec3::Zero(), Vec3::Zero()});
    std::unordered_map<std::uint64_t, Vec3> edge_sum;
    edge_sum.reserve(count * 2);
    std::vector<Aabb> boxes(count);
    std::vector<Vec3> centroids(count);
    for (size_t i = 0; i < count; ++i) {
        const auto &t = mesh_.triangles[i];
        const Vec3 &a = mesh_.vertices[t[0]], &b = mesh_.vertices[t[1]], &c = mesh_.vertices[t[2]];
        face_normals_[i] = (b - a).cross(c - a).normalized();
        const Vec3 corner[3] = {a, b, c};
        for (int k = 0; k < 3; ++k) {
            const Vec3 u = (corner[(k + 1) % 3] - corner[k]).normalized(),
                       w = (corner[(k + 2) % 3] - corner[k]).normalized();
            vertex_normals_[t[size_t(k)]] +=
                std::acos(std::clamp(u.dot(w), -1.0, 1.0)) * face_normals_[i];
            edge_sum[edge_key(t[size_t(k)], t[size_t((k + 1) % 3)])] += face_normals_[i];
        }
        boxes[i] = {a.cwiseMin(b).cwiseMin(c), a.cwiseMax(b).cwiseMax(c)};
        centroids[i] = (a + b + c) / 3;
    }
    for (size_t i = 0; i < count; ++i)
        for (int k = 0; k < 3; ++k)
            edge_normals_[i][size_t(k)] = edge_sum[edge_key(
                mesh_.triangles[i][size_t(k)], mesh_.triangles[i][size_t((k + 1) % 3)])];

    // Median-split BVH over triangle centroids, at most four triangles per leaf.
    order_.resize(count);
    std::iota(order_.begin(), order_.end(), 0u);
    nodes_.reserve(count);
    struct Range {
        std::uint32_t node, first, last;
    };
    nodes_.push_back({});
    std::vector<Range> pending{{0, 0, std::uint32_t(count)}};
    while (!pending.empty()) {
        const Range range = pending.back();
        pending.pop_back();
        Aabb box{Vec3::Constant(kInf), Vec3::Constant(-kInf)};
        Aabb spread = box;
        for (auto i = range.first; i < range.last; ++i) {
            const auto id = order_[i];
            box.min = box.min.cwiseMin(boxes[id].min);
            box.max = box.max.cwiseMax(boxes[id].max);
            spread.min = spread.min.cwiseMin(centroids[id]);
            spread.max = spread.max.cwiseMax(centroids[id]);
        }
        nodes_[range.node].box = box;
        if (range.last - range.first <= 4) {
            nodes_[range.node].first = range.first;
            nodes_[range.node].count = range.last - range.first;
            continue;
        }
        Eigen::Index axis;
        (spread.max - spread.min).maxCoeff(&axis);
        const auto middle = range.first + (range.last - range.first) / 2;
        std::nth_element(order_.begin() + range.first, order_.begin() + middle,
                         order_.begin() + range.last, [&](std::uint32_t x, std::uint32_t y) {
                             return centroids[x][axis] < centroids[y][axis];
                         });
        const auto left = std::uint32_t(nodes_.size());
        nodes_.push_back({});
        nodes_.push_back({});
        nodes_[range.node].left = left;
        nodes_[range.node].right = left + 1;
        pending.push_back({left, range.first, middle});
        pending.push_back({left + 1, middle, range.last});
    }
}

bool MeshShape::inside(const Vec3 &p, const Vec3 &closest, std::uint32_t triangle,
                       int feature) const {
    if (!solid_)
        return false;
    const Vec3 offset = p - closest;
    if (offset.squaredNorm() == 0)
        return false; // on the surface: reported as contact, not containment
    const Vec3 normal = feature < 3   ? vertex_normals_[mesh_.triangles[triangle][size_t(feature)]]
                        : feature < 6 ? edge_normals_[triangle][size_t(feature - 3)]
                                      : face_normals_[triangle];
    if (!(normal.squaredNorm() > 0)) // degenerate neighbourhood: use the slower exact audit
        return std::abs(winding_number(p)) > 0.5;
    return orientation_ * offset.dot(normal) < 0;
}

std::optional<MeshPoint> MeshShape::closest(const Vec3 &p, bool use_bvh,
                                            MeshQueryStats *stats) const {
    if (!finite(p))
        return std::nullopt;
    MeshQueryStats local;
    double best = kInf;
    MeshPoint out;
    int best_feature = 6;
    auto visit = [&](std::uint32_t id) {
        ++local.triangles;
        const auto &t = mesh_.triangles[id];
        int feature = 6;
        const Vec3 q = closest_on_triangle(p, mesh_.vertices[t[0]], mesh_.vertices[t[1]],
                                           mesh_.vertices[t[2]], feature);
        const double squared = (p - q).squaredNorm();
        if (squared < best) {
            best = squared;
            out.closest = q;
            out.triangle = id;
            best_feature = feature;
        }
    };
    if (!use_bvh) {
        for (std::uint32_t id = 0; id < mesh_.triangles.size(); ++id)
            visit(id);
    } else {
        std::vector<std::pair<std::uint32_t, double>> stack{{0, point_aabb_squared(p, nodes_[0].box)}};
        while (!stack.empty()) {
            const auto [index, lower] = stack.back();
            stack.pop_back();
            if (lower > best)
                continue;
            ++local.nodes;
            const Node &node = nodes_[index];
            if (node.count > 0) {
                for (std::uint32_t i = 0; i < node.count; ++i)
                    visit(order_[node.first + i]);
                continue;
            }
            const double l = point_aabb_squared(p, nodes_[node.left].box),
                         r = point_aabb_squared(p, nodes_[node.right].box);
            // The nearer child is pushed last so it is refined first and tightens the bound.
            if (l < r) {
                stack.push_back({node.right, r});
                stack.push_back({node.left, l});
            } else {
                stack.push_back({node.left, l});
                stack.push_back({node.right, r});
            }
        }
    }
    out.distance = std::sqrt(best);
    out.inside = inside(p, out.closest, out.triangle, best_feature);
    if (stats) {
        stats->nodes += local.nodes;
        stats->triangles += local.triangles;
    }
    return out;
}

std::optional<MeshRayHit> MeshShape::ray(const Vec3 &origin, const Vec3 &direction, bool use_bvh,
                                         MeshQueryStats *stats) const {
    if (!finite(origin) || !finite(direction) || !(direction.squaredNorm() > 0))
        return std::nullopt;
    MeshQueryStats local;
    std::optional<MeshRayHit> best;
    auto visit = [&](std::uint32_t id) {
        ++local.triangles;
        const auto &t = mesh_.triangles[id];
        auto hit = ray_triangle(origin, direction, mesh_.vertices[t[0]], mesh_.vertices[t[1]],
                                mesh_.vertices[t[2]]);
        if (hit && (!best || hit->parameter < best->parameter))
            best = MeshRayHit{hit->parameter, hit->position, id};
    };
    if (!use_bvh) {
        for (std::uint32_t id = 0; id < mesh_.triangles.size(); ++id)
            visit(id);
    } else {
        const Vec3 inverse = direction.cwiseInverse();
        std::vector<std::uint32_t> stack{0};
        while (!stack.empty()) {
            const Node &node = nodes_[stack.back()];
            stack.pop_back();
            if (!ray_aabb(origin, inverse, node.box, best ? best->parameter : kInf))
                continue;
            ++local.nodes;
            if (node.count > 0) {
                for (std::uint32_t i = 0; i < node.count; ++i)
                    visit(order_[node.first + i]);
            } else {
                stack.push_back(node.left);
                stack.push_back(node.right);
            }
        }
    }
    if (stats) {
        stats->nodes += local.nodes;
        stats->triangles += local.triangles;
    }
    return best;
}

double MeshShape::winding_number(const Vec3 &p) const {
    // Van Oosterom and Strackee solid angle of each triangle seen from p.
    double total = 0;
    for (const auto &t : mesh_.triangles) {
        const Vec3 a = mesh_.vertices[t[0]] - p, b = mesh_.vertices[t[1]] - p,
                   c = mesh_.vertices[t[2]] - p;
        const double la = a.norm(), lb = b.norm(), lc = c.norm();
        total += 2 * std::atan2(a.dot(b.cross(c)),
                                la * lb * lc + a.dot(b) * lc + b.dot(c) * la + c.dot(a) * lb);
    }
    return total / (4 * std::numbers::pi);
}

QueryResult MeshShape::sphere_query(double radius, const Pose &pose, double tol, bool use_bvh,
                                    MeshQueryStats &stats) const {
    QueryResult q;
    q.backend = "mesh-bvh-sphere";
    const auto nearest = closest(pose.t, use_bvh, &stats);
    if (!nearest || !std::isfinite(nearest->distance)) {
        q.reason = "mesh closest-point query failed";
        return q;
    }
    const double gap = nearest->distance - radius;
    if (nearest->inside) {
        q.relation = Relation::Overlapping;
        q.reason = "centre inside solid mesh";
        return q;
    }
    q.relation = gap > tol ? Relation::Separated
                           : (gap >= -tol ? Relation::Touching : Relation::Overlapping);
    if (q.relation == Relation::Separated) {
        q.distance = gap;
        q.on_body = pose.t + (nearest->closest - pose.t) / nearest->distance * radius;
        q.on_obstacle = nearest->closest;
    }
    return q;
}

QueryResult MeshShape::box_query(const Vec3 &half, const Pose &pose, double tol, bool use_bvh,
                                 MeshQueryStats &stats) const {
    QueryResult q;
    q.backend = "mesh-bvh-box-sat+feature-distance";
    const BoxFrame box = box_frame(half, pose);
    double best = kInf;
    Vec3 on_box = Vec3::Zero(), on_mesh = Vec3::Zero();
    bool touching = false, overlapping = false, failed = false;
    auto visit = [&](std::uint32_t id) {
        ++stats.triangles;
        const auto &t = mesh_.triangles[id];
        const Vec3 v[3] = {mesh_.vertices[t[0]], mesh_.vertices[t[1]], mesh_.vertices[t[2]]};
        switch (triangle_box_sat(box, v, tol)) {
        case Relation::Overlapping:
            overlapping = true;
            break;
        case Relation::Touching:
            touching = true;
            best = 0; // only triangles whose bounds reach the box can still change the answer
            break;
        case Relation::Indeterminate:
            failed = true;
            break;
        case Relation::Separated:
            if (!touching)
                triangle_box_distance(box, v, best, on_box, on_mesh);
            break;
        }
    };
    if (!use_bvh) {
        for (std::uint32_t id = 0; id < mesh_.triangles.size() && !overlapping && !failed; ++id)
            visit(id);
    } else {
        std::vector<std::pair<std::uint32_t, double>> stack{
            {0, aabb_aabb_squared(box.bounds, nodes_[0].box)}};
        while (!stack.empty() && !overlapping && !failed) {
            const auto [index, lower] = stack.back();
            stack.pop_back();
            if (lower > best)
                continue;
            ++stats.nodes;
            const Node &node = nodes_[index];
            if (node.count > 0) {
                for (std::uint32_t i = 0; i < node.count && !overlapping && !failed; ++i)
                    visit(order_[node.first + i]);
                continue;
            }
            const double l = aabb_aabb_squared(box.bounds, nodes_[node.left].box),
                         r = aabb_aabb_squared(box.bounds, nodes_[node.right].box);
            if (l < r) {
                stack.push_back({node.right, r});
                stack.push_back({node.left, l});
            } else {
                stack.push_back({node.left, l});
                stack.push_back({node.right, r});
            }
        }
    }
    if (failed) {
        q.reason = "triangle-box separating-axis test failed";
        return q;
    }
    if (overlapping) {
        q.relation = Relation::Overlapping;
        return q;
    }
    if (touching) {
        q.relation = Relation::Touching;
        return q;
    }
    if (solid_) {
        // No triangle reaches the box, so the box is entirely inside or entirely outside.
        const auto centre = closest(box.center, use_bvh, &stats);
        if (centre && centre->inside) {
            q.relation = Relation::Overlapping;
            q.reason = "box inside solid mesh";
            return q;
        }
    }
    if (!std::isfinite(best)) {
        q.reason = "mesh box distance failed";
        return q;
    }
    q.relation = Relation::Separated;
    q.distance = std::sqrt(best);
    q.on_body = on_box;
    q.on_obstacle = on_mesh;
    return q;
}

QueryResult MeshShape::query(const Part &body, const Pose &pose, double tolerance, bool use_bvh,
                             MeshQueryStats *stats) const {
    MeshQueryStats local;
    QueryResult q;
    if (!valid_pose(pose) || !std::isfinite(tolerance) || tolerance < 0) {
        q.backend = "invalid-input";
    } else if (body.shape == Shape::Sphere && std::isfinite(body.radius) && body.radius > 0) {
        q = sphere_query(body.radius, pose, tolerance, use_bvh, local);
    } else if (body.shape == Shape::Box && finite(body.half) && (body.half.array() > 0).all()) {
        q = box_query(body.half, pose, tolerance, use_bvh, local);
    } else {
        q.backend = "invalid-input";
    }
    if (stats) {
        stats->nodes += local.nodes;
        stats->triangles += local.triangles;
    }
    return q;
}
} // namespace pl

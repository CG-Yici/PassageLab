#pragma once
#include "core.hpp"
#include <array>

namespace pl {
// Indexed triangle mesh in the local frame of the part that owns it. Lengths are scene units (m).
struct TriangleMesh {
    std::vector<Vec3> vertices;
    std::vector<std::array<std::uint32_t, 3>> triangles;
    std::vector<std::uint32_t> faces; // optional: source CAD face of each triangle
};
struct WeldStats {
    std::size_t input_vertices = 0, output_vertices = 0, input_triangles = 0,
                output_triangles = 0, collapsed_triangles = 0;
};
// Merges vertices closer than `tolerance` and removes triangles that collapse to an edge or point.
// CAD tessellation produces one vertex set per face; welding restores the shared topology.
TriangleMesh weld(const TriangleMesh &, double tolerance, WeldStats * = nullptr);
struct MeshReport {
    std::size_t vertices = 0, triangles = 0, degenerate_triangles = 0, duplicate_triangles = 0,
                boundary_edges = 0, nonmanifold_edges = 0, inconsistent_edges = 0, components = 0;
    bool closed = false;   // every edge is shared by exactly two triangles
    bool oriented = false; // every shared edge is traversed in opposite directions
    double area = 0, volume = 0; // volume is signed and only meaningful when closed and oriented
    double min_edge = 0, max_edge = 0;
    Aabb bounds{Vec3::Zero(), Vec3::Zero()};
};
// Throws std::invalid_argument for out-of-range indices or nonfinite vertices.
MeshReport analyze(const TriangleMesh &);
struct TrianglePoint {
    double distance = 0;
    Vec3 closest = Vec3::Zero();
    int feature = 6; // 0-2 vertex, 3-5 edge (ab, bc, ca), 6 interior
};
TrianglePoint point_triangle(const Vec3 &p, const Vec3 &a, const Vec3 &b, const Vec3 &c);
struct MeshQueryStats {
    std::uint64_t nodes = 0, triangles = 0;
};
struct MeshPoint {
    double distance = 0;
    Vec3 closest = Vec3::Zero();
    std::uint32_t triangle = 0;
    bool inside = false; // only set for solid meshes
};
struct MeshRayHit {
    double parameter = 0;
    Vec3 position = Vec3::Zero();
    std::uint32_t triangle = 0;
};
// Immutable mesh with a triangle BVH. A closed, consistently oriented mesh is treated as a solid:
// a body completely inside it overlaps even though no triangle is touched. Any other mesh is a
// surface, and only triangle contact counts.
class MeshShape {
  public:
    explicit MeshShape(TriangleMesh);
    const TriangleMesh &mesh() const {
        return mesh_;
    }
    const MeshReport &report() const {
        return report_;
    }
    const Aabb &bounds() const {
        return report_.bounds;
    }
    bool solid() const {
        return solid_;
    }
    const std::string &digest() const {
        return digest_;
    }
    std::size_t bvh_nodes() const {
        return nodes_.size();
    }
    std::optional<MeshPoint> closest(const Vec3 &, bool use_bvh = true,
                                     MeshQueryStats * = nullptr) const;
    std::optional<MeshRayHit> ray(const Vec3 &origin, const Vec3 &direction, bool use_bvh = true,
                                  MeshQueryStats * = nullptr) const;
    // Generalized winding number: about +-1 inside a closed oriented mesh and 0 outside. Linear in
    // the triangle count; kept as an independent audit of the pseudonormal inside test.
    double winding_number(const Vec3 &) const;
    // Box or sphere against this mesh. `pose` is the body part expressed in the mesh frame and
    // witnesses are returned in the mesh frame. IDs are left for the caller to fill.
    QueryResult query(const Part &body, const Pose &pose, double tolerance, bool use_bvh = true,
                      MeshQueryStats * = nullptr) const;

  private:
    struct Node {
        Aabb box;
        std::uint32_t left = 0, right = 0, first = 0, count = 0; // count > 0 marks a leaf
    };
    bool inside(const Vec3 &p, const Vec3 &closest, std::uint32_t triangle, int feature) const;
    QueryResult sphere_query(double radius, const Pose &, double tol, bool, MeshQueryStats &) const;
    QueryResult box_query(const Vec3 &half, const Pose &, double tol, bool, MeshQueryStats &) const;
    TriangleMesh mesh_;
    MeshReport report_;
    bool solid_ = false;
    double orientation_ = 1;
    std::string digest_;
    std::vector<Node> nodes_;
    std::vector<std::uint32_t> order_;
    std::vector<Vec3> face_normals_, vertex_normals_;
    std::vector<std::array<Vec3, 3>> edge_normals_;
};
} // namespace pl

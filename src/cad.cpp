#include "passagelab/cad.hpp"

#include <BRepAlgoAPI_Cut.hxx>
#include <BRepFilletAPI_MakeFillet.hxx>
#include <BRepGProp.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <BRepPrimAPI_MakeTorus.hxx>
#include <BRep_Tool.hxx>
#include <GProp_GProps.hxx>
#include <Message.hxx>
#include <Message_Messenger.hxx>
#include <Message_PrinterOStream.hxx>
#include <NCollection_IndexedMap.hxx>
#include <Poly_Triangulation.hxx>
#include <STEPControl_Reader.hxx>
#include <STEPControl_Writer.hxx>
#include <Standard_Failure.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopExp.hxx>
#include <TopLoc_Location.hxx>
#include <TopTools_ShapeMapHasher.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <chrono>
#include <cmath>
#include <fstream>
#include <gp_Ax2.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>
#include <stdexcept>

namespace pl {
namespace {
using ShapeMap = NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher>;
using Clock = std::chrono::steady_clock;
double seconds_since(Clock::time_point start) {
    return std::chrono::duration<double>(Clock::now() - start).count();
}
std::string utf8(const std::filesystem::path &p) {
    const auto s = p.u8string();
    return {s.begin(), s.end()};
}
void silence_kernel_messages() {
    // The STEP translator prints statistics to stdout, which would corrupt JSON written there.
    static const int removed =
        Message::DefaultMessenger()->RemovePrinters(STANDARD_TYPE(Message_PrinterOStream));
    (void)removed;
}
std::size_t count(const TopoDS_Shape &shape, TopAbs_ShapeEnum type) {
    ShapeMap map;
    TopExp::MapShapes(shape, type, map);
    return std::size_t(map.Extent());
}
TopoDS_Shape read_step(const std::filesystem::path &path) {
    std::ifstream in(path, std::ios::binary);
    if (!in)
        throw std::runtime_error("Cannot open CAD file " + utf8(path));
    STEPControl_Reader reader;
    // Reading from a stream keeps non-ASCII paths out of the kernel's file layer.
    if (reader.ReadStream(utf8(path.filename()).c_str(), in) != IFSelect_RetDone)
        throw std::runtime_error("Not a readable STEP file: " + utf8(path));
    if (reader.TransferRoots() <= 0)
        throw std::runtime_error("STEP file has no transferable shape: " + utf8(path));
    TopoDS_Shape shape = reader.OneShape();
    if (shape.IsNull() || count(shape, TopAbs_FACE) == 0)
        throw std::runtime_error("STEP file has no faces: " + utf8(path));
    return shape;
}
TopoDS_Shape sample_shape(const std::string &part) {
    if (part == "cylinder")
        return BRepPrimAPI_MakeCylinder(250.0, 800.0).Shape();
    if (part == "sphere")
        return BRepPrimAPI_MakeSphere(400.0).Shape();
    if (part == "torus")
        return BRepPrimAPI_MakeTorus(500.0, 150.0).Shape();
    if (part == "filleted-block") {
        const TopoDS_Shape block = BRepPrimAPI_MakeBox(gp_Pnt(-300, -200, -150), 600, 400, 300).Shape();
        BRepFilletAPI_MakeFillet fillet(block);
        ShapeMap edges;
        TopExp::MapShapes(block, TopAbs_EDGE, edges);
        for (int i = 1; i <= edges.Extent(); ++i)
            fillet.Add(40.0, TopoDS::Edge(edges.FindKey(i)));
        return fillet.Shape();
    }
    if (part == "plate-with-hole") {
        // A wall in the XZ plane with a round aperture along Y: the passage a body must cross.
        const TopoDS_Shape plate = BRepPrimAPI_MakeBox(gp_Pnt(-1500, -60, -1500), 3000, 120, 3000).Shape();
        const TopoDS_Shape hole =
            BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(0, -100, 0), gp_Dir(0, 1, 0)), 450.0, 200.0).Shape();
        BRepAlgoAPI_Cut cut(plate, hole);
        if (!cut.IsDone())
            throw std::runtime_error("boolean cut failed for plate-with-hole");
        return cut.Shape();
    }
    throw std::runtime_error("Unknown sample part: " + part);
}
} // namespace

CadMesh tessellate_step(const std::filesystem::path &path, const MeshSource &source) {
    if (!std::isfinite(source.linear_deflection) || source.linear_deflection <= 0 ||
        !std::isfinite(source.angular_deflection) || source.angular_deflection <= 0 ||
        !std::isfinite(source.scale) || source.scale <= 0)
        throw std::runtime_error("Tessellation deflections and scale must be positive");
    silence_kernel_messages();
    CadMesh out;
    try {
        auto start = Clock::now();
        const TopoDS_Shape shape = read_step(path);
        out.report.read_seconds = seconds_since(start);
        out.report.solids = count(shape, TopAbs_SOLID);
        out.report.shells = count(shape, TopAbs_SHELL);
        out.report.edges = count(shape, TopAbs_EDGE);
        GProp_GProps volume, area;
        BRepGProp::VolumeProperties(shape, volume);
        BRepGProp::SurfaceProperties(shape, area);
        out.report.brep_volume = volume.Mass() * std::pow(source.scale, 3);
        out.report.brep_area = area.Mass() * std::pow(source.scale, 2);

        start = Clock::now();
        // Absolute chordal deflection in model units. Edges are discretised once and shared by
        // the faces on both sides, which is what makes the welded result watertight.
        BRepMesh_IncrementalMesh mesher(shape, source.linear_deflection / source.scale, false,
                                        source.angular_deflection, true);
        if (!mesher.IsDone())
            throw std::runtime_error("Tessellation did not complete: " + utf8(path));
        out.report.tessellate_seconds = seconds_since(start);

        start = Clock::now();
        ShapeMap faces;
        TopExp::MapShapes(shape, TopAbs_FACE, faces);
        out.report.faces = std::size_t(faces.Extent());
        TriangleMesh soup;
        for (int f = 1; f <= faces.Extent(); ++f) {
            const TopoDS_Face &face = TopoDS::Face(faces.FindKey(f));
            TopLoc_Location location;
            const auto &triangulation = BRep_Tool::Triangulation(face, location);
            if (triangulation.IsNull() || triangulation->NbTriangles() == 0) {
                ++out.report.untriangulated_faces;
                continue;
            }
            const gp_Trsf transform = location.Transformation();
            // Triangles follow the surface normal; a reversed face or a mirroring location flips
            // which side is material, so the winding is flipped to keep normals pointing outward.
            const bool flip = (face.Orientation() == TopAbs_REVERSED) != transform.IsNegative();
            const auto base = std::uint32_t(soup.vertices.size());
            for (int i = 1; i <= triangulation->NbNodes(); ++i) {
                const gp_Pnt p = triangulation->Node(i).Transformed(transform);
                soup.vertices.push_back(source.scale * Vec3(p.X(), p.Y(), p.Z()));
            }
            for (int i = 1; i <= triangulation->NbTriangles(); ++i) {
                int a = 0, b = 0, c = 0;
                triangulation->Triangle(i).Get(a, b, c);
                if (flip)
                    std::swap(b, c);
                soup.triangles.push_back({base + std::uint32_t(a - 1), base + std::uint32_t(b - 1),
                                          base + std::uint32_t(c - 1)});
                soup.faces.push_back(std::uint32_t(f - 1));
            }
        }
        if (soup.triangles.empty())
            throw std::runtime_error("Tessellation produced no triangles: " + utf8(path));
        Vec3 low = soup.vertices.front(), high = low;
        for (const auto &v : soup.vertices) {
            low = low.cwiseMin(v);
            high = high.cwiseMax(v);
        }
        // Far below any deflection, far above the rounding of coordinates at this extent.
        out.report.weld_tolerance = std::max(1e-9, 1e-10 * (high - low).norm());
        out.mesh = weld(soup, out.report.weld_tolerance, &out.report.weld);
        out.report.assemble_seconds = seconds_since(start);
    } catch (const Standard_Failure &failure) {
        throw std::runtime_error("CAD kernel error for " + utf8(path) + ": " + failure.what());
    }
    if (out.mesh.triangles.empty())
        throw std::runtime_error("Tessellation collapsed to no triangles: " + utf8(path));
    return out;
}

std::vector<std::string> sample_parts() {
    return {"cylinder", "sphere", "torus", "filleted-block", "plate-with-hole"};
}

void write_sample_step(const std::string &part, const std::filesystem::path &path) {
    silence_kernel_messages();
    try {
        const TopoDS_Shape shape = sample_shape(part);
        STEPControl_Writer writer;
        if (writer.Transfer(shape, STEPControl_AsIs) != IFSelect_RetDone)
            throw std::runtime_error("STEP transfer failed for " + part);
        if (path.has_parent_path())
            std::filesystem::create_directories(path.parent_path());
        std::ofstream out(path, std::ios::binary);
        if (!out || writer.WriteStream(out) != IFSelect_RetDone || !out)
            throw std::runtime_error("Cannot write " + utf8(path));
    } catch (const Standard_Failure &failure) {
        throw std::runtime_error(std::string("CAD kernel error: ") + failure.what());
    }
}
} // namespace pl

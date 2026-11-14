#pragma once
#include "mesh.hpp"
#include <filesystem>

namespace pl {
// What the CAD kernel reported for the B-rep and what happened while turning it into one mesh.
struct CadReport {
    std::size_t solids = 0, shells = 0, faces = 0, edges = 0, untriangulated_faces = 0;
    WeldStats weld;
    double weld_tolerance = 0;             // scene units
    double brep_volume = 0, brep_area = 0; // integrated on the exact B-rep, scene units
    double read_seconds = 0, tessellate_seconds = 0, assemble_seconds = 0;
};
struct CadMesh {
    TriangleMesh mesh;
    CadReport report;
};
// STEP B-rep -> per-face triangulation (OpenCASCADE BRepMesh) -> one welded indexed mesh.
// The model is read in millimetres and multiplied by `source.scale`; the deflections in `source`
// are given in scene units. Throws std::runtime_error when the file cannot be read or meshed.
CadMesh tessellate_step(const std::filesystem::path &, const MeshSource &source);
// Analytic test parts written as STEP, in millimetres.
std::vector<std::string> sample_parts();
void write_sample_step(const std::string &part, const std::filesystem::path &);
} // namespace pl

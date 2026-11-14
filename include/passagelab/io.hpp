#pragma once
#include "mesh.hpp"
#include <filesystem>
#include <nlohmann/json.hpp>
namespace pl {
using Json = nlohmann::json;
Json scene_json(const Scene &);
// Mesh sources are written relative to `directory`, the folder the scene file lives in.
Json scene_json(const Scene &, const std::filesystem::path &directory);
// Relative mesh sources are resolved against `directory`.
Scene scene_from_json(const Json &, const std::filesystem::path &directory = {});
Scene load_scene(const std::filesystem::path &);
void save_scene(const Scene &, const std::filesystem::path &);
std::string scene_hash(const Scene &);
void refresh_hash(Scene &);
Json path_json(const Path &);
Path path_from_json(const Json &);
Path load_path(const std::filesystem::path &);
void save_path(const Path &, const std::filesystem::path &);
Json config_json(const PlannerConfig &);
Json report_json(const Scene &, const PlannerConfig &, const PlanningResult &, const PathMetrics &,
                 const std::optional<PathMetrics> &optimized = {});
std::string sha256_hex(const void *, std::size_t);
Json mesh_report_json(const MeshReport &);
void write_json(const Json &, const std::filesystem::path &);
std::vector<Scene> demo_scenes();
std::vector<Path> reference_paths(const std::vector<Scene> &);
} // namespace pl

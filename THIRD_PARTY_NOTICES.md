# Third-party software

PassageLab's original source is MIT licensed. Dependencies retain their own licenses; the release `licenses/` folder contains the notices distributed by the pinned vcpkg packages, including transitive dependencies.

| Dependency | Role | License family (see bundled exact notice) |
|---|---|---|
| Eigen | Vector, matrix and quaternion algebra | MPL-2.0 |
| FCL | Box-distance audits and geometry comparison | BSD-3-Clause |
| libccd | Convex distance support used by FCL | BSD-3-Clause |
| OctoMap | FCL transitive dependency | BSD-3-Clause |
| OMPL | Independent RRT-Connect implementation for comparison | BSD-3-Clause |
| Boost | OMPL transitive dependencies | BSL-1.0 |
| GLFW | Window and OpenGL context | Zlib |
| GLAD | OpenGL entry-point loader | MIT / Apache-2.0 components |
| Dear ImGui | Immediate mode controls and platform backends | MIT |
| nlohmann/json | JSON serialization | MIT |
| Catch2 | Tests only | BSL-1.0 |

The rendered geometry and scenes are generated from project parameters. No external model or image assets are used. RRT-Connect is an established algorithm (Kuffner and LaValle, 2000); PassageLab implements it rather than claiming to invent it. Microsoft Visual C++ runtime redistribution follows Microsoft's runtime terms; the package requires the supported x64 Visual C++ runtime if absent on the host.

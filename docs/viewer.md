# PassageLab desktop viewer

[中文演示与操作指南](demo.zh-CN.md)

The desktop application uses OpenGL 3.3 core with GLFW, GLAD, Dear ImGui and Eigen. It opens `demo_scenes()[0]` by default. Pass `--scene path/to/scene.json` to load another scene at startup. Scene and path exports use the JSON formats exposed by `passagelab/io.hpp`.

## Controls

- Drag empty viewport space to orbit; Shift-drag to pan; use the wheel to zoom. Drag a picked box/sphere to edit its world position (body components move in body-local coordinates).
- Click the viewport to select an obstacle, use **Focus selection** to center the camera, or choose **Reset camera**.
- Edit body dimensions, start/goal poses, obstacle dimensions and positions, or planning/validation settings in the left panel. Any such edit increments the scene revision and invalidates the current plan.
- **Planar XY** fixes the start height and tilt and allows XY translation plus world-Z yaw. Disable it for full SE(3); incompatible goals are reported as invalid.
- **Plan path** runs the planner asynchronously. **Cancel** requests cooperative cancellation. A successful path is displayed only when the planner reports `Solved`.
- **Search tree** displays at most 4000 edges as the worker produces them. Start and goal trees use separate colors; rejected or incomplete motions are red. **Inspect search motion** selects an edge and its interpolation position, displaying the complete body orientation and the normal collision/distance query at that pose.
- **Inspect minimum sampled clearance** pauses playback and jumps to the measured minimum on the currently selected original or optimized path. It is available only after complete metric evaluation.
- Playback supports scrubbing and single-step navigation. The original and optimized paths remain separately selectable; optimization is requested after planning and is kept only when it returns a path.

The viewer labels its guarantee **discrete validation**. A solved path has passed the configured sampled checks; it is not a continuous collision certificate. Search-budget exhaustion and cancellation remain visible as non-solved statuses. Reference paths are not displayed as planner results.

On Windows the UI loads the system Segoe UI font at 17 px when available; it falls back to Dear ImGui's built-in font. No font asset is bundled.

## Command line

```text
passagelab_desktop.exe [--scene scene.json] [--demo 0|1|2] [--smoke-test]
passagelab_desktop.exe --screenshot capture.ppm [--demo 0|1|2]
passagelab_desktop.exe --capture-dir frames --frames 60 [--demo 0|1|2]
passagelab_desktop.exe --ui-self-test
```

`--smoke-test` creates a context, renders four frames and exits. `--frames N` exits after N rendered frames. `--screenshot` solves the selected demo using a fixed sequence of seeds and operation budgets, sets playback to 40%, then captures the visible application frame as binary PPM before buffer swap. `--capture-dir` renders playback from start to goal across N frames and writes `frame0000.ppm`, `frame0001.ppm`, and so on. A capture contains only the planner's validated result; if planning does not solve the scene, the visible budget/error status remains truthful. Reference motion is never substituted.

`--ui-self-test` creates a hidden OpenGL window and exercises worker launch/cancel, stale-result invalidation, imported-path revalidation, and scene/path JSON round trips. It exits nonzero when an assertion fails.

`--interaction-test report.json` runs 25 input-driven checks through the actual ImGui widgets and viewport handlers in a hidden window. Run it from a fresh working directory, because the export/import checks create JSON files there. It checks planning, playback, shortcut, exports, import/revalidation, dimension editing and invalidation, scene reload, search/minimum-clearance inspection, orbit/pan/zoom, picking, focus and camera reset. This is application-level mouse-event replay, not a claim that every physical input device or display scale has been manually tested. Focus on a moving body uses the displayed pose, including playback and search-edge inspection.

`--self-test-report report.json --ui-self-test` records cooperative cancellation latency. `--performance-report report.json --demo 2 --frames 180` measures rendered playback in a hidden 1440x900 OpenGL window, with vsync disabled, GPU completion included and ten warm-up frames excluded. The reference-host target is median frame time at most 33.333 ms. `tools/verify.ps1` also checks observed peak process working set against 512 MiB; cancellation must complete within one second. These are reference-workload targets, not guarantees for arbitrary scenes or hardware.

The viewport renders the configured box and sphere geometry, the current playback-state query, available distance witnesses, path ghosts, search edges, and collision pairs. Imported paths are labeled as unverified previews until explicitly revalidated against the current scene. Picking uses camera-unprojected rays and nearest analytic geometry hits.

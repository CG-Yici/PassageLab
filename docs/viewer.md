# PassageLab desktop viewer

The desktop application uses OpenGL 3.3 core with GLFW, GLAD, Dear ImGui and Eigen. It opens `demo_scenes()[0]` by default. Pass `--scene path/to/scene.json` to load another scene at startup. Scene and path exports use the JSON formats exposed by `passagelab/io.hpp`.

## Controls

- Drag empty viewport space to orbit; Shift-drag to pan; use the wheel to zoom. Drag a picked box/sphere to edit its world position (body components move in body-local coordinates).
- Click the viewport to select an obstacle, use **Focus selection** to center the camera, or choose **Reset camera**.
- Edit body dimensions, start/goal poses, obstacle dimensions and positions, or planning/validation settings in the left panel. Any such edit increments the scene revision and invalidates the current plan.
- **Plan path** runs the planner asynchronously. **Cancel** requests cooperative cancellation. A successful path is displayed only when the planner reports `Solved`.
- Playback supports scrubbing and single-step navigation. The original and optimized paths remain separately selectable; optimization is requested after planning and is kept only when it returns a path.

The viewer labels its guarantee **discrete validation**. A solved path has passed the configured sampled checks; it is not a continuous collision certificate. Search-budget exhaustion and cancellation remain visible as non-solved statuses. Reference paths are not displayed as planner results.

## Command line

```text
PassageLabDesktop [--scene scene.json] [--demo 0|1|2] [--smoke-test]
PassageLabDesktop --screenshot capture.ppm [--demo 0|1|2]
PassageLabDesktop --capture-dir frames --frames 60 [--demo 0|1|2]
```

`--smoke-test` creates a context, renders four frames and exits. `--frames N` exits after N rendered frames. `--screenshot` solves the selected demo using a fixed sequence of seeds and operation budgets, sets playback to 40%, then captures the visible application frame as binary PPM before buffer swap. `--capture-dir` renders playback from start to goal across N frames and writes `frame0000.ppm`, `frame0001.ppm`, and so on. A capture contains only the planner's validated result; if planning does not solve the scene, the visible budget/error status remains truthful. Reference motion is never substituted.

The viewport renders the configured box and sphere geometry, the current playback-state query, available distance witnesses, path ghosts, search edges, and collision pairs. Imported paths are labeled as unverified previews until explicitly revalidated against the current scene. Picking uses camera-unprojected rays and nearest analytic geometry hits.

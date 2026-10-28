# PassageLab desktop viewer

The desktop application uses OpenGL 3.3 core with GLFW, GLAD, Dear ImGui and Eigen. It opens `demo_scenes()[0]` by default. Pass `--scene path/to/scene.json` to load another scene at startup. Scene and path exports use the JSON formats exposed by `passagelab/io.hpp`.

## Controls

- Drag in the viewport to orbit; Shift-drag to pan; use the wheel to zoom.
- Click the viewport to select an obstacle, use **Focus selection** to center the camera, or choose **Reset camera**.
- Edit body dimensions, start/goal poses, obstacle dimensions and positions, or planning/validation settings in the left panel. Any such edit increments the scene revision and invalidates the current plan.
- **Plan path** runs the planner asynchronously. **Cancel** requests cooperative cancellation. A successful path is displayed only when the planner reports `Solved`.
- Playback supports scrubbing and single-step navigation. The original and optimized paths remain separately selectable; optimization is requested after planning and is kept only when it returns a path.

The viewer labels its guarantee **discrete validation**. A solved path has passed the configured sampled checks; it is not a continuous collision certificate. Search-budget exhaustion and cancellation remain visible as non-solved statuses. Reference paths are not displayed as planner results.

## Command line

```text
PassageLabDesktop [--scene scene.json] [--smoke-test]
PassageLabDesktop --screenshot capture.ppm
PassageLabDesktop --frames 4
```

`--smoke-test` creates a context, renders four frames and exits. `--frames N` exits after N rendered frames. `--screenshot` solves the first demo scene using a fixed seed and operation budget, then captures the visible application frame as binary PPM. If planning does not produce a validated solution, the screenshot shows that truthful result; no reference animation is substituted.

The rendering layer currently draws box meshes for all analytic shapes, using their dimensions for a quick engineering visualization. Sphere geometry rendering and per-frame nearest-clearance witness overlays are not yet implemented. The planning result's sampled-clearance metric is shown when available. Picking currently selects the first obstacle when the viewport is clicked; geometric ray picking is still pending.

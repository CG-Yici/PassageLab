# Implementation and verification status

Version **0.2.0**, verified locally on 2026-10-28: design phases **P0–P6** are implemented. P7 is explicitly optional in the design. Windows x64, MSVC 19.37, Release, Windows build 26200, AMD Radeon RX 9070 XT. See the [Chinese acceptance report](acceptance-0.2.0.zh-CN.md) for the deliverable checklist and experiment totals.

Production-source SHA-256: `55ddafaf7c410f5e2916476d46028f25f24c5d64a3368fabd67875a5f1486789`. Verification used base commit `c9881a2` with uncommitted changes before publication. The release tag records those verified production-source bytes; the embedded base commit is historical build metadata. The source fingerprint covers production source, headers (including the UI replay adapter), build definitions and dependency metadata. The clean-source rebuild produced the same fingerprint.

| Requirement | Implementation | Verification |
|---|---|---|
| F01 scenes and JSON | Strict scene/path IO, three parameterized demos | Round trips, malformed-input rejection, three reference motions |
| F02 shapes and compound body | Shared box/sphere geometry, compound transforms | Analytic/randomized regressions and rendered A/B/C |
| F03 pose editing | Pose, dimension and obstacle editors | Actual widget edit invalidates result; scene reload restores input |
| F04 state/nearest queries | Analytic queries, SAT, feature distance, BVH | FCL comparisons, witness tests, point/segment/box and ray/triangle tests |
| F05 motion validation | Translation, rotation and surface subdivisions | Thin-wall and rotational-sweep regressions; six finer path rechecks |
| F06 planning | Self-written RRT-Connect, budgets, cancellation | Frozen seed 42 solves A/B/C; invalid endpoints and termination tests |
| F07 shortcut | Preserve original, validate simplified path | Raw/optimized objective comparisons and independent 0.02 checks |
| F08 3D interaction | Camera, picking, editing, focus | 25 input-driven checks per demo, including orbit/pan/zoom/pick |
| F09 result inspection | Playback, ghosts, live search edges, minimum clearance | Actual controls and full-pose inspection exercised in replay |
| F10 stale-result rejection | Hash/revision and background request identity | Edited scene invalidation and cancelled-worker late-result self-test |
| F11 headless execution | CLI, batch runners, portable binaries | CLI integration, three demos and benchmark execution |
| F12 reproducible reports | Source/hash/request/tolerance/step diagnostics | Full raw reports, seeds, configuration, hardware and failure counts retained |

**40/40 CTest cases**, **75/75 desktop input checks**, desktop smoke/concurrency self-tests and all six original/optimized path checks at 0.02 resolution passed. The latter use one quarter of the default search step. These are discrete checks, not continuous collision certificates. [Evidence](../benchmarks/data/worktree-0.2.0/verification/).

The explicit `planar-xy` mode is supported by both planners and enforced by the shared validator. It keeps initial Z and tilt and permits world-Z yaw. Incompatible goals are rejected. The comparison experiment records a distinct C variant with common compatible endpoints; it never silently projects the original demonstration endpoints.

Reference-host targets were fixed for the final verification: cancellation within 1000 ms, median rendered frame within 33.333 ms at 1440x900, and observed peak process working set below 512 MiB. Results were 31.436 ms, 0.419 ms and 98.40 MiB respectively. Rendering includes GPU completion with vsync disabled and ten warm-up frames excluded. These workload-specific measurements do not predict arbitrary-scene or other-GPU performance.

P6 includes a source-only copy built in a fresh directory, all tests and a desktop self-test, actual release-binary screenshots/video, notices and portable-package verification tooling. The clean build reused the pinned dependency installation on the same host; it is not a claim of fresh-machine/VM validation. Publication is tracked by the [v0.2.0 release](https://github.com/CG-Yici/PassageLab/releases/tag/v0.2.0). Other GPUs, display scales and physical input devices remain outside the tested support matrix.

The box-distance implementation deliberately differs from the original FCL-backend proposal: symmetric box-feature distances are authoritative after reproduced FCL discrepancies, with FCL retained for independent audits. See [algorithm notes](algorithms.md). The historical 0.1.0 and 0.1.1 assets/evidence are preserved.

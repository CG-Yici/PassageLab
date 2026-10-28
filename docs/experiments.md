# Experiments

The release experiment configuration is `benchmarks/configs/release.json`. Runs are headless Release builds. Each run exports its exact scene/configuration, path if solved, simplified path if available and report. Reference motions never seed the planner.

The baseline and comparison runner uses 30 predeclared seeds starting at 1000. Comparisons change full traversal versus BVH, 0.08 versus 0.04 sampling steps, or planner implementation. The raw scene hashes differ when computation parameters change; geometric content remains fixed for each comparison.

Report both success fraction (with 95% Wilson interval) and timing conditional on success. Preserve every failed run and status. A time budget depends on hardware and scheduling; fixed operation budgets are used for deterministic regression cases. Repeated configurations must not silently replace difficult cases with easier geometry.

Timing includes the phases indicated by each field. Search, final verification, clearance evaluation and shortcut are separate costs. No GPU performance claims are made from headless data. Small scene obstacle counts may provide little or no speedup from a BVH; conclusions must follow measurements.

Executed results and environment details will be linked here after verification.

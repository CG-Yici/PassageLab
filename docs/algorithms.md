# Algorithms and engineering choices

## State and metric

A pose has a translation and unit quaternion. Quaternion sign is not a physical distinction. The rotation distance uses the shortest relative angle. Planning uses `hypot(translation_distance, rotation_weight * rotation_angle)`. Path shortcut acceptance uses the separately declared objective `translation_length + rotation_weight * accumulated_rotation`; these are different operations, not interchangeable measures of safety.

All motion uses linear translation and shortest-arc SLERP. Uniform orientation sampling avoids uniformly sampling Euler angles. Seeds are deterministic within a fixed implementation/toolchain and operation-budget run; they do not imply identical random sequences across standard libraries or planners.

## Geometry

Sphere queries are analytical. OBB collision classification checks six face axes and nine edge cross-product axes, with a conservative contact tolerance. SAT projection gaps are never reported as Euclidean distance. Compound bodies are unions of local parts; parts do not collide against each other.

The original design selected FCL as the box-distance backend. Testing found argument-order-dependent distances in the pinned FCL 0.7.0 LIBCCD and independent GJK paths, including an axis-aligned expected 0.5 m case returning about 0.5831 m with the default backend. The implementation therefore enumerates both sets of eight vertex-to-box projections and all 12-by-12 edge segment pairs. For separated boxes, a minimum is attained by one of those feature pairs; its witnesses lie on the actual shapes. This is a bounded box-specific method, not a general convex-body solver. FCL GST_INDEP (distance tolerance 1e-9) remains an audit; disagreement is retained in `QueryResult.reason`. Randomized reversal, rigid-transform, witness-surface and known skew-edge cases cover this deviation from the initial design.

An obstacle BVH stores world AABBs. Collision traversal expands by the clearance/contact threshold. Distance traversal uses AABB distance lower bounds to prune nodes only when they cannot improve the current minimum. The full-traversal mode is retained for comparison. Physical boundary clearance participates in state validity and distance evaluation.

## RRT-Connect

Each tree retains its start/goal identity when active trees swap. Extend searches the nearest node, steers using the shared pose interpolation, and validates the complete proposed edge. Connect repeats extension until reached, trapped or stopped. Parent chains are assembled in start-to-goal order, and the candidate is independently checked before `Solved` is returned.

No-progress extensions terminate. Nodes use stable indices. Budgets and stop requests are checked inside Connect. The saved search tree is bounded for display and is only a projection of the full six-dimensional search.

## Shortcut and evaluation

Shortcut attempts remove intermediate vertices only when the replacement segment passes validation and the explicit objective decreases. The full resulting path is checked again. Original paths remain available. Metrics retain translation, rotation and sampled clearance separately. A sampled minimum is not a lower bound on clearance between samples.

OMPL uses the same geometry and discrete motion validator. Its internal growth and termination structure differs; experiment notes and report reasons describe budget mappings. Agreement between two planners sharing a validator is not independent evidence that the validator is correct.

# Validation contract

For segment endpoints a,b, translation L, shortest rotation theta and local body radius r:

```text
N = max(1, ceil(L / translation_step), ceil(theta / rotation_step),
           ceil((L + r * theta) / surface_step))
samples = { interpolate(a,b,i/N) | i=0,...,N }
```

Every sample checks valid input, reference-point sampling bounds, whole-body physical bounds, collisions/contact and configured clearance. If N exceeds the sample limit, validation is incomplete; the resolution is never silently reduced. Stop requests and exhausted query budgets cannot produce a valid result.

Final path verification also checks scene hash/revision, exact requested endpoints within numerical tolerance, all poses and all adjacent edges. Revalidation must use no weaker resolution or clearance requirement than the path records. A malformed, stale or approximate path cannot become solved simply by loading it.

The test suite includes known analytical distances, transform composition, quaternion sign/near-zero/pi cases, SAT containment/contact, randomized FCL comparisons, BVH/full-traversal equivalence, positive clearance, midpoint thin-wall crossing, rotational sweep collisions, cancellation, operation budgets, stale metadata and scene-reference regressions.

Randomized comparisons avoid intentionally ambiguous contact bands; dedicated boundary cases test conservative decisions there. Differences are investigated rather than resolved by relaxing thresholds. The algorithms use double precision, finite-input checks and scale-aware numerical allowances.

This validation does not account for shape uncertainty, forces, grasping, acceleration or human/robot execution constraints. Continuous collision detection and conservative interval certification are future work.

# Team Asclepius: NIDAR Rescueswarm

Search-and-rescue drone swarm tooling for the NIDAR challenge: field coverage planning,
fleet management, and onboard person detection.

| Path | What it is |
| --- | --- |
| [`swarm_lab/`](swarm_lab/) | Coverage planner: medial axis, four sweep planners compared side by side, per-drone areas, flight paths, nadir camera plan |
| [`swarm_lab/MANUAL.md`](swarm_lab/MANUAL.md) | **Field-to-flight manual:** draw the KML, generate missions, set up and upload to each drone, fly, and RTL |
| [`swarm_lab/ARDUPILOT.md`](swarm_lab/ARDUPILOT.md) | **Fly it:** KML/KMZ/.poly ? per-drone ArduCopter `.waypoints` (takeoff, synced speeds, camera trigger distance, RTL) + optional pymavlink upload with read-back |
| `Fleet_manager.py` | Fleet/lane assignment, camera parameters, and delivery-drone dispatch |
| `docs/` | GitHub Pages version of the comparison page. A pure JS port (`planner.js`, `medial.js`, `nadir.js`) checked against the Python planners by `node docs/verify.mjs` |
| [`paper/`](paper/) | Technical report ([LaTeX](paper/coverage_theory.tex), [Markdown](paper/coverage_theory.md)): the mathematics, the solvers that failed and why, and the method in use. Includes equation checks (`verify_derivations.py`), figures (`figures.py`) and tables (`results.py`) |
| `NIDAR_YOLO.py` | YOLO-World open-vocabulary detection (`person`, `tree`, ...) |
| `weights/`, `yolov8s-worldv2.pt` | Model weights (Git LFS) |

**Live comparison page:** <https://avinashistoocool.github.io/asclepius/>. It runs entirely
in your browser.

## Status (October 2026)

| Area | State |
| --- | --- |
| Coverage planner (`swarm_lab/`) | Working. Four planners; 100% nadir coverage on every test field; 8 unit tests pass |
| Browser demo (`docs/`) | Live on GitHub Pages, Obsidian & Lime UI, runs in a Web Worker in 0.8?2.3 s |
| Technical report (`paper/`) | LaTeX + Markdown, 4 figures, 2 result tables, 8/8 equation checks pass |
| Exact chord-bisecting trajectory | Solved for convex fields and symmetric presets; shown not to exist in general for strongly non-convex fields (see the report) |
| ArduPilot export | `ardupilot_export.py`: Mission Planner `.waypoints` per drone, KML preview, MAVLink upload verified against simulated vehicles |
| Fleet manager / YOLO detection | Unchanged in this round |

**Key finding.** The harmonic planner (`?T = 0`) needs the lowest maximum altitude on
every test field. The lawnmower and geodesic planners balance drone workload best.

**Next steps** (detailed in the report): shooting search for exact centre lines in near-convex fields,
`p`-Laplacian sweeps to minimise the worst altitude, workload-balanced harmonic fields,
and turn-rate, terrain and GPS constraints for real flight.

---

## Coverage planning: the problem

A swarm of **N drones** flies as a flexible line over a field of arbitrary shape. Each drone
carries a **nadir (straight-down) camera**, and each drone can fly at its own altitude, so
its ground footprint (swath) is adjustable. The swarm may split around obstacles and
rejoin later. Goal: **image every point of the field**, keeping altitude, path length, and
workload imbalance low.

### Unifying model: arrival-time fields

Every planner below is written as a scalar **arrival time** `T(x)` over the field, scaled
to `[0, 1]`:

- The formation at time `t` is the **front** `{x : T(x) = t}`, extracted with marching squares.
- If the front has several pieces, the swarm has **split** into sub-swarms.
- Drones are spaced evenly along the front. Per-drone swath is `front length / N`.
- A point is swept once the front passes it. Coverage is the fraction of cells with a finite `T`.

This puts very different strategies on the same footing, so they can be compared on the
same metrics.

---

## Algorithms

### 1. Medial axis transform (MAT): the backbone

The medial axis is the set of centres of maximal inscribed disks (disks touching the
boundary at two or more points). Every simple closed region has one, so this step cannot
fail with "no solution found".

1. Resample the boundary at about 1 unit spacing (400 to 6000 samples).
2. Compute the **Voronoi diagram** of the samples. Interior Voronoi vertices approximate the MAT.
3. **Prune** each Voronoi ridge unless its two generating samples are farther apart along the
   boundary than `prune × perimeter` (Ogniewicz residual). This removes spurs from corners
   and boundary noise.
4. Build a **minimum spanning tree** of the largest component and take its **longest path**
   (two Dijkstra passes) as the **trunk**.
5. Smooth the trunk with a cubic spline that stays inside the field, then extend both ends
   tangentially to the boundary.
6. Fallback for disk-like shapes, where the MAT collapses to a point: use the principal
   axis through the largest inscribed disk.

The trunk endpoints define the **start and end caps** used by the harmonic and geodesic
planners. The inscribed radius along the trunk feeds the complexity metrics.

> The MAT is *equidistant* from the boundary. It bisects its perpendicular chord only where
> the two walls are locally parallel. Solving for an exact chord-bisecting trajectory was
> attempted three ways: spline collocation, boundary-pair continuation, and ODE shooting.
> The report shows why such a trajectory exists only for convex or near-convex fields,
> which is why the sweep is relaxed to arrival-time fields.

### 2. Sweep planners

| Planner | Arrival time `T(x)` | Strengths | Weaknesses |
| --- | --- | --- | --- |
| **MAT + chord** | Arc length along the trunk at which the perpendicular chord through `x` passes | One rigid line, never splits | Chords **fold** (cross each other) in bends and lobes; some areas get swept backwards or twice |
| **Lawnmower** | Projection onto the principal axis (PCA) | Every drone flies at the same speed; balanced areas | Splits around every concavity; straight line ignores the shape |
| **Harmonic** | Solution of Laplace's equation `∇²T = 0`, with `T = 0` on the start cap and `T = 1` on the end cap (linear along each side wall) | Smooth fronts, no folds or cusps; follows bends | Fronts bunch up in narrow parts and spread in lobes, so drone speeds vary a lot |
| **Geodesic** | Shortest-path (fast-marching, 8-connected) distance from the start cap | Constant front speed; short missions | Fronts can develop cusps and split into many pieces |

Implementation details:

- **Grid:** 200 × 200 cells over a 600 × 600 canvas. The field mask comes from polygon rasterisation.
- **Harmonic:** a 5-point Laplacian on interior cells with Dirichlet boundary values, solved with a sparse direct solver.
- **Geodesic:** `skimage.graph.MCP_Geometric` from the start-cap cell.
- **MAT + chord:** each quad between consecutive perpendicular chords is rasterised and given the earlier time.
- **Fronts:** 90 levels from `t = 0.005` to `0.995`. Pieces shorter than 4 cells are dropped.

### 3. Per-drone areas and flight paths

1. **Orient fronts consistently in time.** Each front is resampled. Its pieces are ordered
   and flipped so that their nearest points on the previous front have increasing lane
   coordinate. The pieces are then concatenated and parameterised by normalised arc length
   `s ∈ [0, 1]`.
2. **Lanes.** Drone `i` owns `s ∈ [i/N, (i+1)/N)`. Each swept cell is matched to its
   nearest front frame and nearest front point, and takes that point's lane. This
   **partitions the field** into N coloured areas.
3. **Paths.** Drone `i` flies `s = (i + 0.5)/N` on every front. A step longer than 25 units
   is a **transit jump** to another piece of a split front.

### 4. Nadir camera plan: guaranteed coverage

A nadir camera at altitude `h`, with fields of view `HFOV` and `VFOV`, images a rectangle
centred under the drone:

```
across-track half-width  = h · tan(HFOV / 2)
along-track  half-length = h · tan(VFOV / 2)
```

The rectangle is rotated to the drone's **heading**, the direction of travel averaged over
the adjacent path segments (transit jumps are ignored).

1. **Initial altitude.** For each cell owned by drone `i`, take the nearest path sample and
   express the cell in that sample's body frame (along, across), padded by half a cell
   diagonal. The required altitude is `max(across / tan(HFOV/2), (along − half step) / tan(VFOV/2))`.
   The maximum is taken per sample, followed by a 3-sample max filter. Samples with no
   cells are interpolated from their neighbours. Then multiply by `(1 + overlap)`.
2. **Verify.** Stamp the footprint at every sample and densely between samples (at most
   one footprint length apart, using the lower of the two altitudes). Mark every imaged cell.
3. **Repair.** For any field cell still unimaged, raise the altitude of the owning drone
   around its nearest sample by `×1.15 + 1 cell`, then verify again (up to 8 rounds).

Result: **Nadir coverage = 100%** and **unimaged cells = 0** for every planner, which the
test suite asserts. Altitude is in canvas units. Scale it by the real field size.

### 5. Shape complexity

| Metric | Definition |
| --- | --- |
| Circularity | `4πA / P²` (1 for a disk) |
| Solidity | area / convex hull area |
| Convexity | hull perimeter / perimeter |
| Reflex vertices | vertices with a negative turning angle |
| Bending | total absolute turning / 2π (1 for a convex shape) |
| Elongation | ratio of principal singular values |
| Width ratio | max / min inscribed radius along the inner 80% of the trunk |
| Necks | runs of the trunk where the radius drops below 75% of the widest point on **both** sides |
| **Score (0–100)** | 30% non-convexity + 25% roughness + 15% elongation + 15% width variation + 15% necks |

---

## Comparison metrics

| Metric | Meaning | Better |
| --- | --- | --- |
| Coverage % | Cells swept by the front | higher |
| Peak swath | Largest per-drone front share, which sets the max altitude without a camera model | lower |
| Mission | Path length of the fastest-moving drone | lower |
| Max sub-swarms | Most front pieces at any time | lower |
| Speed spread | 95th / 5th percentile drone speed on the front | closer to 1 |
| Chord folds % | MAT only: steps where consecutive chords cross | lower |
| Area balance | Smallest / largest drone area | closer to 1 |
| Longest drone path | Excluding transit jumps | lower |
| Transit jumps | Hops between pieces of a split front | lower |
| Nadir coverage % / unimaged cells | After the camera plan | 100 / 0 |
| Max / mean altitude | From the camera plan | lower |
| Mean image overlap | Average number of drones imaging a cell | close to 1 |
| Compute ms | Time spent by that planner | lower |

### Typical results (12 drones)

Maximum altitude for 100% nadir coverage (HFOV 70?, VFOV 55?, 10% overlap; canvas units, lower is better):

| Field | MAT + chord | Lawnmower | Harmonic | Geodesic |
| --- | ---: | ---: | ---: | ---: |
| Ellipse | 21.4 | **21.3** | **21.3** | 24.0 |
| Star | 140.7 | 58.4 | **31.0** | 77.0 |
| C-bend | 31.8 | 55.9 | **15.0** | 23.8 |
| Narrow neck | 93.5 | 84.1 | **21.1** | 159.2 |
| Blob | 27.9 | 28.1 | **24.9** | 79.1 |

Area balance (smallest / largest drone area): lawnmower 0.73?0.87, geodesic 0.73?0.86,
harmonic 0.32?0.79, MAT 0.12?0.94. MAT chords fold on 23?52% of steps in non-convex fields.
Regenerate with `py paper/results.py`.

Rule of thumb: **harmonic** gives the lowest altitudes and no splits on bent or necked
fields, at the cost of uneven drone speeds. **Geodesic** and **lawnmower** give balanced
workloads and short missions but split more. **MAT + chord** is only reliable on
near-convex or tube-like fields.

---

## Running locally

```powershell
cd swarm_lab
py -m pip install -r requirements.txt
py server.py              # http://127.0.0.1:8765  (MAT lab)
                          # http://127.0.0.1:8765/compare  (planner comparison)
py -m unittest test_medial -v
cd ..
py paper/verify_derivations.py   # equation checks
py paper/figures.py              # report figures
py paper/results.py              # result tables
```

The server streams NDJSON progress events, and both pages show a live progress bar.
The [GitHub Pages version](https://avinashistoocool.github.io/asclepius/) runs the same
pipeline in the browser with no server.

### `swarm_lab/` files

| File | Role |
| --- | --- |
| `medial_axis.py` | Voronoi MAT, pruning, trunk, inscribed radii |
| `sweep.py` | Arrival-time fields, fronts, lanes, drone paths, metrics |
| `nadir.py` | Headings, altitudes, footprint stamping, coverage repair |
| `complexity.py` | Shape complexity descriptors |
| `boundary.py` / `exact.py` | Polygon validation and intersections; preset shapes |
| `server.py` | Local HTTP server with streamed progress |
| `ardupilot_export.py` | KML/KMZ/.poly ? ArduCopter missions and MAVLink upload ([guide](swarm_lab/ARDUPILOT.md)) |
| `examples/` | Sample field (`field.kml`) and a fake MAVLink vehicle for upload tests |
| `index.html` / `compare.html` | MAT lab UI / comparison UI |
| `test_medial.py` | Unit tests (MAT, complexity, lane partition, 100% nadir coverage) |
| `test_ardupilot.py` | Exporter tests (projection, mission structure, altitude cap, trigger spacing, coverage) |

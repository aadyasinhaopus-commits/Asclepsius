"""
RescueSwarm — Phase 1 Simulation
=================================
Core logic for:
  1. FleetManager  — polygon -> parallel search lanes, balanced across N search drones
  2. TargetManager  — survivor detections -> delivery-drone dispatch (continuous / two-wave)

Scope note (read before extending):
  This module is a GENERAL lane-generation planner driven by `swath_width`. It is
  deliberately separate from the "5 fixed parallel strips flown once as a single
  formation pass" design used elsewhere in the RescueSwarm engineering docs — that
  design assumes the 5-drone combined swath already spans the polygon width in one
  pass. This planner instead computes however many lanes `swath_width` actually
  requires and balances them across the fleet, which is the right tool if that
  single-pass assumption doesn't hold (narrower footprint, different polygon,
  different fleet size). Keep the two mental models separate.

Dependencies: shapely, numpy (both geometry-only, no simulation-time dependency).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum
from typing import List, Optional, Tuple

import numpy as np
from shapely.affinity import rotate as shp_rotate
from shapely.geometry import LineString, MultiLineString, Point, Polygon
from scipy.interpolate import splprep, splev
from scipy.optimize import linear_sum_assignment
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches


# =============================================================================
# Camera / footprint model
# =============================================================================

@dataclass
class CameraParams:
    """
    Search-drone camera characteristics.

    resolution_mp / horizontal_fov_deg reflect the actual hardware (12 MP, 120
    degree wide-angle, non-nadir mount). `nadir_assumption` is the Phase-1 TOGGLE
    requested: True uses a simple nadir footprint formula as a placeholder so the
    lane-planning geometry can be built and tested now, before the non-nadir
    frustum-to-ground-plane model is finalized.
    """
    altitude_m: float = 20.0
    horizontal_fov_deg: float = 120.0
    resolution_mp: float = 12.0
    nadir_assumption: bool = True     # Phase-1 placeholder toggle (see docstring)
    tilt_deg: float = 0.0             # forward tilt from nadir; only used if nadir_assumption=False

    def footprint_width(self) -> float:
        """
        Ground swath width used by the lane planner.

        NADIR (placeholder, nadir_assumption=True):
            W = 2 * h * tan(FOV/2)   — standard flat-ground nadir footprint.

        NON-NADIR (nadir_assumption=False):
            A tilted-forward FOV footprint is actually a trapezoid (near edge
            narrower than far edge) whose centroid is offset forward of the
            nadir point, not a simple rectangle. Modeling that correctly needs
            full frustum/ground-plane intersection geometry, which is NOT yet
            implemented. The line below is an ENGINEERING ASSUMPTION placeholder
            (a flat derate of the nadir footprint) — OPEN QUESTION: replace with
            real frustum geometry once the mount tilt angle is finalized.
        """
        theta = math.radians(self.horizontal_fov_deg)
        w_nadir = 2.0 * self.altitude_m * math.tan(theta / 2.0)
        if self.nadir_assumption:
            return w_nadir
        derate = math.cos(math.radians(self.tilt_deg)) if self.tilt_deg else 0.8
        return w_nadir * derate


# =============================================================================
# Fleet Manager — lane generation & balancing
# =============================================================================

@dataclass
class Lane:
    """A single straight search lane, in the ORIGINAL (unrotated) coordinate frame."""
    lane_id: int
    start: Tuple[float, float]
    end: Tuple[float, float]

    @property
    def length(self) -> float:
        (x1, y1), (x2, y2) = self.start, self.end
        return math.hypot(x2 - x1, y2 - y1)


@dataclass
class CurvedLane:
    """
    A curved formation track for one drone, in the ORIGINAL (unrotated) frame.
    Unlike `Lane`, this isn't two endpoints — it's a sampled polyline, because
    the whole point of this sweep model is that the track bends as the local
    polygon width changes along the travel direction.
    """
    drone_id: int
    points: List[Tuple[float, float]]

    @property
    def start(self) -> Tuple[float, float]:
        return self.points[0]

    @property
    def end(self) -> Tuple[float, float]:
        return self.points[-1]

    @property
    def length(self) -> float:
        total = 0.0
        for (x1, y1), (x2, y2) in zip(self.points, self.points[1:]):
            total += math.hypot(x2 - x1, y2 - y1)
        return total


@dataclass
class ChordSample:
    """
    One step of the medial-axis march: a point ON the centerline, plus the
    two boundary points of the chord perpendicular to the LOCAL centerline
    direction at that point (not a fixed global axis). `center` is the
    midpoint of (chord_lo, chord_hi) — i.e. exactly the construction you
    described: connect the midpoints of chords perpendicular to the path.
    """
    center: Tuple[float, float]
    chord_lo: Tuple[float, float]
    chord_hi: Tuple[float, float]

    @property
    def chord_width(self) -> float:
        (x1, y1), (x2, y2) = self.chord_lo, self.chord_hi
        return math.hypot(x2 - x1, y2 - y1)


def _diameter_endpoints(points: List[Tuple[float, float]]) -> Tuple[Tuple[float, float], Tuple[float, float]]:
    """
    The two boundary points that are farthest apart from each other — the
    geometric "diameter" of the shape. Used as the start/end of the medial
    march. O(n^2) over boundary sample points; fine for a few hundred points.
    """
    best_d, best_a, best_b = -1.0, points[0], points[0]
    n = len(points)
    for i in range(n):
        xi, yi = points[i]
        for j in range(i + 1, n):
            xj, yj = points[j]
            d = math.hypot(xi - xj, yi - yj)
            if d > best_d:
                best_d, best_a, best_b = d, points[i], points[j]
    return best_a, best_b


@dataclass
class DronePlan:
    """The ordered set of lanes assigned to one search drone, plus transit overhead."""
    drone_id: int
    lanes: List[Lane] = field(default_factory=list)

    @property
    def lane_distance(self) -> float:
        return sum(l.length for l in self.lanes)

    @property
    def transit_distance(self) -> float:
        """Straight-line hops between the end of one assigned lane and the start of the next."""
        total = 0.0
        for a, b in zip(self.lanes, self.lanes[1:]):
            (ex, ey) = a.end
            (sx, sy) = b.start
            total += math.hypot(sx - ex, sy - ey)
        return total

    @property
    def total_distance(self) -> float:
        return self.lane_distance + self.transit_distance


class FleetManager:
    """
    Converts an organiser-provided mission polygon into a set of parallel search
    lanes, oriented to minimize turns, and balances them across the search fleet
    so no single drone becomes the bottleneck (minimizes makespan).
    """

    def __init__(
        self,
        polygon_coords: List[Tuple[float, float]],
        camera: CameraParams,
        num_search_drones: int = 5,
        overlap_fraction: float = 0.2,
    ):
        if len(polygon_coords) < 3:
            raise ValueError("Polygon needs at least 3 vertices")
        self.polygon = Polygon(polygon_coords)
        if not self.polygon.is_valid:
            raise ValueError("Input polygon is not valid (self-intersecting?)")
        self.camera = camera
        self.num_search_drones = num_search_drones
        self.overlap_fraction = overlap_fraction
        self._effective_swath = camera.footprint_width() * (1.0 - overlap_fraction)
        if self._effective_swath <= 0:
            raise ValueError("Effective swath width must be positive — check camera params/overlap")

        self._sweep_angle_deg: Optional[float] = None
        self.lanes: List[Lane] = []          # boustrophedon-ordered, original frame
        self.drone_plans: List[DronePlan] = []
        self.formation_curves: List[CurvedLane] = []
        self._formation_max_band_width: float = 0.0
        self.medial_path: List[ChordSample] = []
        self.formation_curves_medial: List[CurvedLane] = []
        self._formation_medial_max_band_width: float = 0.0

    # ------------------------------------------------------------------
    # Step 1: sweep direction — parallel to the longest edge of the
    # minimum rotated bounding rectangle, to minimize turn count.
    # ------------------------------------------------------------------
    def _compute_sweep_angle(self) -> float:
        mrr = self.polygon.minimum_rotated_rectangle
        coords = list(mrr.exterior.coords)[:-1]  # drop closing duplicate vertex
        edges = []
        for (x1, y1), (x2, y2) in zip(coords, coords[1:] + coords[:1]):
            length = math.hypot(x2 - x1, y2 - y1)
            angle = math.degrees(math.atan2(y2 - y1, x2 - x1))
            edges.append((length, angle))
        edges.sort(key=lambda e: e[0], reverse=True)
        longest_edge_angle = edges[0][1]
        # Normalize to [0, 180) — a line and its reverse are the same sweep direction.
        return longest_edge_angle % 180.0

    # ------------------------------------------------------------------
    # Step 2: generate lanes as horizontal scanlines in a rotated frame
    # where the sweep direction is aligned to the x-axis, then clip each
    # scanline to the polygon boundary and rotate back.
    # ------------------------------------------------------------------
    def generate_lanes(self) -> List[Lane]:
        self._sweep_angle_deg = self._compute_sweep_angle()
        origin = self.polygon.centroid

        # Rotate polygon so the sweep direction becomes horizontal (parallel to x-axis).
        rotated_poly = shp_rotate(self.polygon, -self._sweep_angle_deg, origin=origin, use_radians=False)
        minx, miny, maxx, maxy = rotated_poly.bounds

        raw_lanes: List[Tuple[float, float, float]] = []  # (y, x_start, x_end) in rotated frame
        y = miny + self._effective_swath / 2.0
        pad = max(maxx - minx, 1.0)  # generous horizontal overshoot so intersection isn't truncated
        while y <= maxy:
            scan = LineString([(minx - pad, y), (maxx + pad, y)])
            clipped = rotated_poly.intersection(scan)
            if clipped.is_empty:
                y += self._effective_swath
                continue
            segments = [clipped] if isinstance(clipped, LineString) else list(clipped.geoms)
            for seg in segments:
                if seg.length <= 0:
                    continue
                (sx, sy0), (ex, ey0) = seg.coords[0], seg.coords[-1]
                x_lo, x_hi = sorted((sx, ex))
                raw_lanes.append((y, x_lo, x_hi))
            y += self._effective_swath

        # Order top-to-bottom by y, then boustrophedon alternate left<->right direction
        # so consecutive lanes' transit hops are short (end of lane i is near start of lane i+1).
        raw_lanes.sort(key=lambda t: (t[0], t[1]))
        lanes: List[Lane] = []
        for idx, (y_r, x_lo, x_hi) in enumerate(raw_lanes):
            if idx % 2 == 0:
                start_r, end_r = (x_lo, y_r), (x_hi, y_r)
            else:
                start_r, end_r = (x_hi, y_r), (x_lo, y_r)
            # Rotate each lane endpoint back to the original frame.
            start = self._rotate_point_back(start_r, origin)
            end = self._rotate_point_back(end_r, origin)
            lanes.append(Lane(lane_id=idx, start=start, end=end))

        self.lanes = lanes
        return lanes

    def _rotate_point_back(self, pt: Tuple[float, float], origin: Point) -> Tuple[float, float]:
        p = Point(pt)
        rotated = shp_rotate(p, self._sweep_angle_deg, origin=origin, use_radians=False)
        return (rotated.x, rotated.y)

    # ------------------------------------------------------------------
    # Step 3: balance the boustrophedon-ordered lane list across drones.
    # Contiguous partition (no re-ordering) minimizing the MAKESPAN (the
    # longest single drone's total distance) — this is what actually
    # determines mission completion time when drones fly simultaneously,
    # even though total summed distance is ~fixed regardless of partition.
    # ------------------------------------------------------------------
    def balance_across_fleet(self) -> List[DronePlan]:
        if not self.lanes:
            self.generate_lanes()
        n = self.num_search_drones
        lane_lengths = [l.length for l in self.lanes]

        if len(self.lanes) <= n:
            # Fewer lanes than drones: one lane per drone (or fewer drones used).
            partitions = [[i] for i in range(len(self.lanes))]
        else:
            partitions = self._partition_min_makespan(lane_lengths, n)

        plans = []
        for drone_id, idx_group in enumerate(partitions):
            plan = DronePlan(drone_id=drone_id, lanes=[self.lanes[i] for i in idx_group])
            plans.append(plan)
        self.drone_plans = plans
        return plans

    @staticmethod
    def _partition_min_makespan(weights: List[float], k: int) -> List[List[int]]:
        """
        Classic 'split an array into k contiguous groups minimizing the maximum
        group sum' — solved via binary search on the answer + greedy feasibility.
        Returns a list of k index groups (some may be empty if k > len(weights)).
        """
        n = len(weights)
        lo, hi = max(weights), sum(weights)

        def groups_needed(capacity: float) -> List[List[int]]:
            groups, current, current_sum = [], [], 0.0
            for i, w in enumerate(weights):
                if current and current_sum + w > capacity:
                    groups.append(current)
                    current, current_sum = [], 0.0
                current.append(i)
                current_sum += w
            groups.append(current)
            return groups

        best_groups = groups_needed(hi)
        while lo < hi:
            mid = (lo + hi) / 2.0
            groups = groups_needed(mid)
            if len(groups) <= k:
                hi = mid
                best_groups = groups
            else:
                lo = mid + 1e-6

        while len(best_groups) < k:
            best_groups.append([])
        return best_groups[:k]

    # ------------------------------------------------------------------
    # ALTERNATIVE sweep model: perpendicular formation, curved lanes.
    #
    # Instead of fixed-spacing parallel straight lines, this models 5 drones
    # flying as a rigid array perpendicular to the travel direction. At every
    # point along the sweep, the local polygon cross-section (measured
    # perpendicular to travel direction) is divided into `num_search_drones`
    # EQUAL-WIDTH bands, and each drone tracks the centerline of its own band.
    # Because the polygon's width changes along the travel axis, each drone's
    # offset from center has to change too — that's what makes the tracks
    # curves rather than straight lines.
    #
    # This does NOT use `swath_width` as a spacing rule the way generate_lanes
    # does — band width here is purely geometric (local_width / n). That means
    # coverage is only guaranteed if the camera's actual footprint is wide
    # enough to cover the widest band the sweep ever produces. This method
    # checks that and prints a warning if it isn't — don't ignore it.
    # ------------------------------------------------------------------
    def generate_formation_curves(self, n_samples: int = 300) -> List[CurvedLane]:
        n = self.num_search_drones
        origin = self.polygon.centroid
        self._sweep_angle_deg = self._compute_sweep_angle()
        rotated_poly = shp_rotate(self.polygon, -self._sweep_angle_deg, origin=origin, use_radians=False)
        minx, miny, maxx, maxy = rotated_poly.bounds
        pad = max(maxy - miny, 1.0)

        xs = np.linspace(minx, maxx, n_samples)
        drone_points_rotated: List[List[Tuple[float, float]]] = [[] for _ in range(n)]
        max_band_width = 0.0
        skipped = 0

        for x in xs:
            bounds = self._vertical_slice_bounds(rotated_poly, x, miny - pad, maxy + pad)
            if bounds is None:
                skipped += 1
                continue
            y_lo, y_hi = bounds
            local_width = y_hi - y_lo
            band_width = local_width / n
            max_band_width = max(max_band_width, band_width)
            for i in range(n):
                y_i = y_lo + (i + 0.5) / n * local_width
                drone_points_rotated[i].append((x, y_i))

        if skipped:
            print(f"[generate_formation_curves] note: {skipped}/{n_samples} sample columns "
                  f"had no polygon intersection (likely near a pointed vertex) and were skipped.")

        # Coverage check: this sweep model doesn't derive spacing from swath_width,
        # so verify the camera can actually see across the widest band it produced.
        if max_band_width > self._effective_swath:
            print(f"[generate_formation_curves] WARNING: max band width "
                  f"{max_band_width:.1f} m exceeds the camera's effective swath "
                  f"{self._effective_swath:.1f} m — there will be an uncovered gap "
                  f"between adjacent drones' bands at the widest point of the sweep. "
                  f"Either the polygon is too wide for a 5-drone single pass at this "
                  f"altitude/FOV, or this formation model needs a fallback (extra pass, "
                  f"lower altitude, or fall back to generate_lanes()).")

        curves = []
        for i in range(n):
            original_points = [self._rotate_point_back(p, origin) for p in drone_points_rotated[i]]
            curves.append(CurvedLane(drone_id=i, points=original_points))

        self.formation_curves = curves
        self._formation_max_band_width = max_band_width
        return curves

    @staticmethod
    def _vertical_slice_bounds(
        rotated_poly: Polygon, x: float, y_min: float, y_max: float
    ) -> Optional[Tuple[float, float]]:
        """
        Intersects a vertical line at the given x with the (rotated) polygon and
        returns (y_lo, y_hi) of the LARGEST resulting segment.

        LIMITATION (flagged, not silently handled): if the polygon is concave
        enough that a single vertical slice crosses it in more than one disjoint
        segment, this takes only the largest segment and discards the others —
        the formation follows one continuous band, it does not split to cover a
        secondary disjoint piece at the same x. Fine for mildly irregular shapes;
        revisit if the actual competition polygon turns out to be badly concave.
        """
        scan = LineString([(x, y_min), (x, y_max)])
        clipped = rotated_poly.intersection(scan)
        if clipped.is_empty:
            return None
        segments = [clipped] if isinstance(clipped, LineString) else (
            list(clipped.geoms) if isinstance(clipped, MultiLineString) else []
        )
        segments = [s for s in segments if s.length > 0]
        if not segments:
            return None
        best = max(segments, key=lambda s: s.length)
        (_, y1), (_, y2) = best.coords[0], best.coords[-1]
        return (min(y1, y2), max(y1, y2))

    # ------------------------------------------------------------------
    # GENERAL sweep model: medial-axis march, for genuinely curved/bending
    # polygons. This is the one to use once the boundary itself has real
    # curvature — generate_formation_curves above assumes one FIXED global
    # sweep direction, which is only exactly correct for a convex/simple
    # shape whose two sides are close to straight lines.
    #
    # Algorithm (marching, "diameter"-anchored):
    #   1. Find the two boundary points farthest apart (the shape's
    #      "diameter") — these anchor the start and end of the centerline.
    #   2. March from one to the other in small steps. At each step, take
    #      the chord perpendicular to the CURRENT LOCAL DIRECTION of travel
    #      (not a fixed global axis), clip it to the polygon, and record its
    #      midpoint as the next centerline point.
    #   3. Update the local direction from the last two centerline points
    #      before taking the next step — this is what makes the chord follow
    #      the shape's bend instead of a straight line.
    #   4. Each drone's track is then the locus of points at a fixed
    #      fractional position along these chords (same "divide local width
    #      into n equal bands" idea as before, just now measured along a
    #      curving chord instead of a straight one).
    #
    # LIMITATION (flagged, not silently handled): this marching approach
    # assumes the polygon is simple enough that a perpendicular chord to the
    # local direction crosses the boundary in essentially one segment near
    # the current point. A badly concave/self-crossing-adjacent shape can
    # break this (the march can exit the polygon or jump to the wrong
    # segment) — it stops rather than producing a wrong answer in that case.
    # Revisit with a proper skeletonization library (e.g. a Voronoi-based
    # medial axis) if the real competition polygon turns out to need it.
    # ------------------------------------------------------------------
    def compute_medial_path(self, n_steps: int = 150, n_relax_iters: int = 5,
                             edge_trim_fraction: float = 0.08) -> List[ChordSample]:
        """
        Builds the medial centerline via RELAXATION, not sequential marching.

        (An earlier sequential-marching version of this — step forward, take
        a perpendicular chord, repeat — turned out to be numerically unstable
        on real curved boundaries: small direction errors compound step after
        step and the march can drift and never actually reach the far end,
        producing wildly wrong outer-band track lengths. Caught by testing
        against a real curved shape, not a synthetic straight one — worth
        remembering when extending this further.)

        TOPOLOGY ASSUMPTION (flagged, not silently handled): this whole
        perpendicular-chord approach — marching OR relaxation — assumes the
        polygon is a "channel": roughly two parallel-ish boundary curves with
        a well-defined local width along one dominant direction, like a bent
        river or a curved road. Tested against a hand-drawn blob-like shape
        (a rounded polygon without that channel structure) and it broke badly
        — a perpendicular chord on a blob can cut across a completely
        different part of the shape from one step to the next, producing the
        same kind of wild jump the marching version had, for a different
        underlying reason (wrong shape assumption, not numerical instability).
        If the real organiser polygon isn't channel-shaped, this method needs
        a proper skeletonization approach (e.g. Voronoi-based medial axis)
        instead — don't trust this output without checking the shape first.

        EDGE-CAP ARTIFACT (found by testing, fixed by trimming, not by
        pretending it doesn't happen): near the very ends of a channel that's
        closed off by a short "cap" edge (rather than continuing as two long
        rails), the perpendicular-chord logic can briefly grab the cap edge
        itself or cut diagonally across the corner instead of the true local
        width — chord widths spike erratically for the first/last few samples
        (confirmed: e.g. jumping 6m -> 80m -> 42m -> 155m within 4 samples on
        the test channel). Physically, the "5 drones split the local width
        evenly" concept is ill-defined right at a channel's tip anyway (same
        reasoning as the pointed-vertex convergence case earlier), so
        `edge_trim_fraction` drops that many samples off both ends rather
        than reporting a track through geometry that isn't real.

        This version instead:
          1. Takes an initial guess: points evenly spaced along the straight
             line between the shape's two "diameter" endpoints, each snapped
             to the midpoint of the polygon chord perpendicular to that line.
          2. Relaxes it for a few passes: each interior point's chord is
             recomputed perpendicular to the LOCAL tangent estimated from its
             (previous-iteration) neighbors, letting the centerline bend to
             actually follow the shape.
        Each relaxation pass is a fresh, well-conditioned local computation —
        errors don't compound across hundreds of sequential steps the way
        they did in the marching version.
        """
        boundary = list(self.polygon.exterior.coords)[:-1]
        point_a, point_b = _diameter_endpoints(boundary)
        a = np.array(point_a, dtype=float)
        b = np.array(point_b, dtype=float)
        base_dir = b - a
        base_norm = np.linalg.norm(base_dir)
        if base_norm <= 0:
            raise ValueError("Degenerate polygon — diameter endpoints coincide")
        base_dir = base_dir / base_norm

        big = max(self.polygon.bounds[2] - self.polygon.bounds[0],
                   self.polygon.bounds[3] - self.polygon.bounds[1]) * 2.0

        def chord_at(point: np.ndarray, direction: np.ndarray):
            """Perpendicular chord through `point`, perpendicular to `direction`. Returns (mid, lo, hi) or None."""
            n_perp = np.array([-direction[1], direction[0]])
            chord = LineString([tuple(point - n_perp * big), tuple(point + n_perp * big)])
            clipped = self.polygon.intersection(chord)
            if clipped.is_empty:
                return None
            segs = [clipped] if isinstance(clipped, LineString) else (
                list(clipped.geoms) if isinstance(clipped, MultiLineString) else []
            )
            segs = [s for s in segs if s.length > 0]
            if not segs:
                return None
            cpt = Point(tuple(point))
            best_seg = min(segs, key=lambda s: s.distance(cpt))
            (x1, y1), (x2, y2) = best_seg.coords[0], best_seg.coords[-1]
            p1, p2 = np.array([x1, y1]), np.array([x2, y2])
            if np.dot(p1 - point, n_perp) <= np.dot(p2 - point, n_perp):
                lo, hi = (x1, y1), (x2, y2)
            else:
                lo, hi = (x2, y2), (x1, y1)
            mid = (p1 + p2) / 2.0
            return mid, lo, hi

        # --- Step 1: initial guess along the straight A-B line ---
        centers = []
        for t in np.linspace(0.0, 1.0, n_steps):
            guess = a + t * (b - a)
            result = chord_at(guess, base_dir)
            if result is not None:
                centers.append(result[0])
        if len(centers) < 3:
            self.medial_path = []
            return self.medial_path
        centers = np.array(centers)

        # --- Step 2: relax — bend the centerline to follow local tangents ---
        for _ in range(n_relax_iters):
            new_centers = centers.copy()
            for i in range(1, len(centers) - 1):
                tangent = centers[i + 1] - centers[i - 1]
                norm = np.linalg.norm(tangent)
                if norm < 1e-9:
                    continue
                tangent = tangent / norm
                result = chord_at(centers[i], tangent)
                if result is not None:
                    new_centers[i] = result[0]
            centers = new_centers

        # --- Step 3: final chord (with lo/hi) at each relaxed point ---
        samples: List[ChordSample] = []
        n = len(centers)
        for i in range(n):
            if i == 0:
                tangent = centers[1] - centers[0]
            elif i == n - 1:
                tangent = centers[i] - centers[i - 1]
            else:
                tangent = centers[i + 1] - centers[i - 1]
            norm = np.linalg.norm(tangent)
            if norm < 1e-9:
                continue
            tangent = tangent / norm
            result = chord_at(centers[i], tangent)
            if result is None:
                continue
            mid, lo, hi = result
            samples.append(ChordSample(center=(float(mid[0]), float(mid[1])), chord_lo=lo, chord_hi=hi))

        # Drop the erratic edge-cap samples (see docstring) rather than report them.
        trim = max(2, int(round(len(samples) * edge_trim_fraction)))
        if len(samples) > 2 * trim:
            samples = samples[trim:-trim]

        self.medial_path = samples
        return samples

    def generate_formation_curves_medial(self, n_steps: int = 150) -> List[CurvedLane]:
        if not self.medial_path:
            self.compute_medial_path(n_steps=n_steps)
        n = self.num_search_drones
        curve_points: List[List[Tuple[float, float]]] = [[] for _ in range(n)]
        max_band_width = 0.0

        for sample in self.medial_path:
            lo = np.array(sample.chord_lo)
            hi = np.array(sample.chord_hi)
            local_width = sample.chord_width
            band = local_width / n
            max_band_width = max(max_band_width, band)
            for i in range(n):
                t = (i + 0.5) / n
                p = lo + t * (hi - lo)
                curve_points[i].append((float(p[0]), float(p[1])))

        curves = [CurvedLane(drone_id=i, points=curve_points[i]) for i in range(n)]
        self.formation_curves_medial = curves
        self._formation_medial_max_band_width = max_band_width
        if max_band_width > self._effective_swath:
            print(f"[generate_formation_curves_medial] WARNING: max band width "
                  f"{max_band_width:.1f} m exceeds the camera's effective swath "
                  f"{self._effective_swath:.1f} m — coverage gap risk at the widest "
                  f"point of the medial sweep. Lower altitude, narrower FOV derate, "
                  f"or accept the gap.")
        return curves

    def summary_formation_medial(self) -> dict:
        """
        Reports the medial-path length vs. the straight long-axis length, to
        make the altitude/resolution tradeoff concrete: a longer (curved)
        path implies a smaller average width for the same enclosed area,
        which is exactly the "smaller swath -> lower altitude -> better
        resolution" argument.
        """
        if not self.formation_curves_medial:
            self.generate_formation_curves_medial()
        path_length = sum(
            math.hypot(b.center[0] - a.center[0], b.center[1] - a.center[1])
            for a, b in zip(self.medial_path, self.medial_path[1:])
        )
        avg_width = self.polygon.area / path_length if path_length > 0 else float("nan")

        mrr = self.polygon.minimum_rotated_rectangle
        coords = list(mrr.exterior.coords)[:-1]
        edge_lengths = [
            math.hypot(coords[(k + 1) % len(coords)][0] - coords[k][0],
                       coords[(k + 1) % len(coords)][1] - coords[k][1])
            for k in range(len(coords))
        ]
        straight_long_axis_length = max(edge_lengths)
        straight_avg_width = self.polygon.area / straight_long_axis_length if straight_long_axis_length > 0 else float("nan")

        return {
            "medial_path_length_m": round(path_length, 2),
            "avg_width_along_medial_m": round(avg_width, 2),
            "straight_long_axis_length_m": round(straight_long_axis_length, 2),
            "avg_width_if_flown_straight_m": round(straight_avg_width, 2),
            "max_band_width_m": round(self._formation_medial_max_band_width, 2),
            "effective_swath_m": round(self._effective_swath, 2),
            "coverage_ok": self._formation_medial_max_band_width <= self._effective_swath,
        }

    # ------------------------------------------------------------------
    # Reporting
    # ------------------------------------------------------------------
    def summary(self) -> dict:
        if not self.drone_plans:
            self.balance_across_fleet()
        total_lane_distance = sum(p.lane_distance for p in self.drone_plans)
        total_transit_distance = sum(p.transit_distance for p in self.drone_plans)
        makespan = max((p.total_distance for p in self.drone_plans), default=0.0)
        return {
            "sweep_angle_deg": round(self._sweep_angle_deg, 2) if self._sweep_angle_deg is not None else None,
            "effective_swath_m": round(self._effective_swath, 2),
            "num_lanes": len(self.lanes),
            "total_lane_distance_m": round(total_lane_distance, 2),
            "total_transit_distance_m": round(total_transit_distance, 2),
            "total_path_distance_m": round(total_lane_distance + total_transit_distance, 2),
            "makespan_m": round(makespan, 2),
        }

    def summary_formation(self) -> dict:
        """Reporting for the curved-formation sweep (generate_formation_curves)."""
        if not self.formation_curves:
            self.generate_formation_curves()
        total_distance = sum(c.length for c in self.formation_curves)
        makespan = max((c.length for c in self.formation_curves), default=0.0)
        return {
            "sweep_angle_deg": round(self._sweep_angle_deg, 2) if self._sweep_angle_deg is not None else None,
            "effective_swath_m": round(self._effective_swath, 2),
            "max_band_width_m": round(self._formation_max_band_width, 2),
            "coverage_ok": self._formation_max_band_width <= self._effective_swath,
            "total_path_distance_m": round(total_distance, 2),
            "makespan_m": round(makespan, 2),
        }


# =============================================================================
# Visualization — see the lane segregation, not just read coordinates
# =============================================================================

# One distinct, high-contrast color per drone. Extend this list if you ever
# run with num_search_drones > 8 — it will otherwise start recycling colors.
DRONE_COLORS = ["#1D9E75", "#D85A30", "#378ADD", "#BA7517", "#993556", "#7F77DD", "#639922", "#A32D2D"]


def plot_lane_assignment(fleet: "FleetManager", save_path: Optional[str] = None, show: bool = False) -> None:
    """
    Plots the mission polygon plus every generated lane, color-coded by which
    search drone it's assigned to, with an arrow on each lane showing the fly
    direction (boustrophedon: alternating). This is the "lane segregation"
    view — at a glance you can see which drone covers which part of the area.
    """
    if not fleet.drone_plans:
        fleet.balance_across_fleet()

    fig, ax = plt.subplots(figsize=(9, 7))

    # Mission boundary
    poly_x, poly_y = fleet.polygon.exterior.xy
    ax.plot(list(poly_x), list(poly_y), color="black", linewidth=1.5, label="Mission boundary")
    ax.fill(list(poly_x), list(poly_y), color="black", alpha=0.03)

    for plan in fleet.drone_plans:
        color = DRONE_COLORS[plan.drone_id % len(DRONE_COLORS)]
        for lane in plan.lanes:
            (x1, y1), (x2, y2) = lane.start, lane.end
            ax.annotate(
                "", xy=(x2, y2), xytext=(x1, y1),
                arrowprops=dict(arrowstyle="-|>", color=color, lw=2, mutation_scale=15),
            )
        # One legend entry per drone (only label the first lane so the legend doesn't repeat).
        if plan.lanes:
            ax.plot([], [], color=color, lw=2, marker=">",
                    label=f"Search drone {plan.drone_id} ({len(plan.lanes)} lanes, "
                          f"{plan.total_distance:.0f} m)")

    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title("RescueSwarm — lane segregation across search fleet")
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.08), ncol=2, fontsize=9)
    fig.tight_layout()

    if save_path:
        fig.savefig(save_path, dpi=150, bbox_inches="tight")
        print(f"[plot saved to {save_path}]")
    if show:
        plt.show()
    plt.close(fig)


def plot_formation_curves(fleet: "FleetManager", curves: Optional[List[CurvedLane]] = None,
                           save_path: Optional[str] = None, show: bool = False,
                           title: str = "RescueSwarm — perpendicular-formation curved sweep") -> None:
    """
    Plots the mission polygon plus curved formation tracks. Pass `curves`
    explicitly to choose which sweep model to visualize — e.g.
    fleet.formation_curves (fixed global axis) or fleet.formation_curves_medial
    (self-consistent medial-axis march). Defaults to fleet.formation_curves
    if not given.
    """
    if curves is None:
        if not fleet.formation_curves:
            fleet.generate_formation_curves()
        curves = fleet.formation_curves

    fig, ax = plt.subplots(figsize=(9, 7))
    poly_x, poly_y = fleet.polygon.exterior.xy
    ax.plot(list(poly_x), list(poly_y), color="black", linewidth=1.5, label="Mission boundary")
    ax.fill(list(poly_x), list(poly_y), color="black", alpha=0.03)

    for curve in curves:
        if not curve.points:
            continue
        color = DRONE_COLORS[curve.drone_id % len(DRONE_COLORS)]
        xs = [p[0] for p in curve.points]
        ys = [p[1] for p in curve.points]
        ax.plot(xs, ys, color=color, lw=2,
                label=f"Search drone {curve.drone_id} ({curve.length:.0f} m)")
        if len(curve.points) >= 2:
            (x1, y1), (x2, y2) = curve.points[-2], curve.points[-1]
            ax.annotate("", xy=(x2, y2), xytext=(x1, y1),
                        arrowprops=dict(arrowstyle="-|>", color=color, lw=2, mutation_scale=15))

    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title(title)
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.08), ncol=2, fontsize=9)
    fig.tight_layout()

    if save_path:
        fig.savefig(save_path, dpi=150, bbox_inches="tight")
        print(f"[plot saved to {save_path}]")
    if show:
        plt.show()
    plt.close(fig)


def make_bending_channel_polygon(
    length: float = 350.0,
    base_width: float = 60.0,
    n_points: int = 200,
    bend_amplitude: float = 80.0,
    bend_periods: float = 1.3,
) -> Tuple[List[Tuple[float, float]], List[Tuple[float, float]]]:
    """
    Builds a genuinely bending CHANNEL-shaped polygon (roughly constant width,
    following a sine-wave centerline) — the shape type the medial-axis method
    below actually assumes. Also returns the TRUE centerline used to build it,
    so the medial-axis output can be checked against ground truth rather than
    just visually inspected.

    This exists because a hand-drawn/spline-interpolated blob can easily end
    up topologically wrong for this method (see compute_medial_path's
    docstring for what went wrong when that was tried) — this generator
    guarantees the right topology by construction.
    """
    s = np.linspace(0.0, length, n_points)
    centerline_y = bend_amplitude * np.sin(2 * np.pi * bend_periods * s / length)
    true_centerline = list(zip(s.tolist(), centerline_y.tolist()))

    dx = np.gradient(s)
    dy = np.gradient(centerline_y)
    norm = np.hypot(dx, dy)
    norm[norm < 1e-9] = 1.0
    nx, ny = -dy / norm, dx / norm  # unit perpendicular to the centerline tangent

    half_w = base_width / 2.0
    left = list(zip((s + nx * half_w).tolist(), (centerline_y + ny * half_w).tolist()))
    right = list(zip((s - nx * half_w).tolist(), (centerline_y - ny * half_w).tolist()))
    polygon_coords = left + right[::-1]
    if not Polygon(polygon_coords).is_valid:
        raise ValueError(
            "Generated channel self-intersects — base_width is too large relative to "
            "how sharply it bends (the offset curves crossed at the bend). Reduce "
            "base_width, reduce bend_amplitude, or increase bend_periods/length."
        )
    return polygon_coords, true_centerline


def interpolate_closed_curve(
    control_points: List[Tuple[float, float]],
    n_samples: int = 300,
    smoothing: float = 0.0,
) -> List[Tuple[float, float]]:
    """
    Fits a smooth PERIODIC (closed-loop) cubic spline through a handful of
    control points and densely samples it. This is what actually gives you a
    curved polygon boundary — a straight-edge polygon with more vertices is
    still straight-edged; this produces genuine curvature between control
    points.

    control_points: at least 4 points, in order around the boundary, NOT
                     repeating the first point at the end (periodic=True
                     closes the loop automatically).
    n_samples:       how densely to sample the resulting curve — this becomes
                      your polygon's vertex count, so higher = smoother.
    smoothing:       0.0 = interpolate exactly through every control point;
                      >0.0 = allow the spline to deviate slightly for a
                      smoother result (passed straight to scipy's `s` param).
    """
    if len(control_points) < 4:
        raise ValueError(
            f"Need at least 4 control points for a periodic smooth curve, got {len(control_points)}"
        )
    pts = np.array(control_points)
    tck, _ = splprep([pts[:, 0], pts[:, 1]], s=smoothing, per=True)
    u_fine = np.linspace(0.0, 1.0, n_samples, endpoint=False)
    x_fine, y_fine = splev(u_fine, tck)
    return [(float(x), float(y)) for x, y in zip(x_fine, y_fine)]


def draw_polygon_interactively(
    xlim: Tuple[float, float] = (0, 400),
    ylim: Tuple[float, float] = (0, 300),
    smooth: bool = True,
    n_curve_samples: int = 300,
) -> List[Tuple[float, float]]:
    """
    Opens an interactive matplotlib window so you can draw the mission
    boundary with the mouse instead of hand-typing coordinates.

      - Left-click  : place the next boundary CONTROL POINT, in order
      - Right-click : undo the last point
      - Enter / middle-click : finish

    If smooth=True (default): your clicked points are treated as control
    points for a periodic cubic spline (see interpolate_closed_curve) and the
    function returns a densely-sampled SMOOTH CURVE through them — a real
    curved boundary, not just a many-sided straight polygon. Needs at least
    4 clicks. If smooth=False: returns your raw clicks as a straight-edge
    polygon (the old behavior).

    REQUIRES A LOCAL DISPLAY (matplotlib GUI event loop / ginput) — will not
    work over a headless/remote/SSH-without-X session.
    """
    fig, ax = plt.subplots(figsize=(8, 6))
    ax.set_xlim(*xlim)
    ax.set_ylim(*ylim)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, linestyle="--", alpha=0.4)
    mode_note = "control points of a smooth curve" if smooth else "straight-edge vertices"
    ax.set_title(
        f"Left-click to place {mode_note}, in order.\n"
        "Right-click undoes the last point. Press ENTER when done."
    )
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")

    raw_points = plt.ginput(n=-1, timeout=0, mouse_add=1, mouse_pop=3, mouse_stop=2)
    plt.close(fig)

    min_needed = 4 if smooth else 3
    if len(raw_points) < min_needed:
        raise ValueError(f"Need at least {min_needed} points — you placed {len(raw_points)}")

    control_points = [(round(x, 2), round(y, 2)) for x, y in raw_points]
    print(f"[placed {len(control_points)} control points]")
    for i, (x, y) in enumerate(control_points):
        print(f"    control point {i}: ({x}, {y})")

    if smooth:
        polygon_coords = interpolate_closed_curve(control_points, n_samples=n_curve_samples)
        print(f"[fit a smooth closed curve through them -> {len(polygon_coords)}-point polygon]")
        print(f"\n# Paste this back into the INPUT block to reuse this exact control-point shape:")
        print(f"dummy_control_points = {control_points}\n")
    else:
        polygon_coords = control_points
        print(f"\n# Paste this back into the INPUT block to reuse this exact shape:")
        print(f"dummy_polygon = {polygon_coords}\n")

    return polygon_coords


# =============================================================================
# Target Manager — detection intake & delivery dispatch
# =============================================================================

class DispatchPolicy(Enum):
    CONTINUOUS = "continuous"
    TWO_WAVE = "two_wave"


class DroneStatus(Enum):
    IDLE_AT_GCS = "idle_at_gcs"
    EN_ROUTE = "en_route"
    HOLDING = "holding"           # two-wave only: done with wave 1, waiting for wave 2 trigger


@dataclass
class Target:
    target_id: int
    x: float
    y: float
    confidence: float = 1.0
    assigned_drone: Optional[int] = None
    delivered: bool = False


@dataclass
class DeliveryDrone:
    drone_id: int
    kits_remaining: int
    position: Tuple[float, float]          # current position; resets to GCS position when idle
    status: DroneStatus = DroneStatus.IDLE_AT_GCS

    def is_available(self) -> bool:
        return self.status == DroneStatus.IDLE_AT_GCS and self.kits_remaining > 0


class TargetManager:
    """
    Receives survivor detections from the search fleet and assigns them to
    delivery drones per a configurable dispatch policy.

      CONTINUOUS — greedy nearest-available-drone assignment as each target
                   is confirmed (matches the Task Allocator design: dedicated
                   delivery-drone pool only, cost = travel distance proxy).

      TWO_WAVE   — dispatch continuously for the first `wave1_limit` targets
                   (default: one per delivery drone), then HOLD any further
                   targets. Once `notify_search_complete()` is called (all
                   search drones report SEARCH_COMPLETE), all held/queued
                   targets are dispatched in a single final wave.

    OPEN QUESTION: the two-wave trigger (wave1_limit) is exposed as a
    parameter rather than hardcoded — confirm against the actual kit loadout
    (3+3+4 = 10) before treating any particular value as final.
    """

    def __init__(
        self,
        delivery_drones: List[DeliveryDrone],
        gcs_position: Tuple[float, float],
        dispatch_policy: DispatchPolicy = DispatchPolicy.CONTINUOUS,
        wave1_limit: Optional[int] = None,
    ):
        self.drones = {d.drone_id: d for d in delivery_drones}
        self.gcs_position = gcs_position
        self.policy = dispatch_policy
        self.wave1_limit = wave1_limit if wave1_limit is not None else len(delivery_drones)

        self.targets: List[Target] = []
        self.pending_queue: List[Target] = []   # undispatched targets (two-wave hold, or no drone free)
        self.wave1_dispatch_count = 0
        self.wave1_closed = False
        self.search_complete = False

    # ------------------------------------------------------------------
    def register_detection(self, target: Target) -> None:
        self.targets.append(target)
        self._try_dispatch(target)

    def notify_search_complete(self) -> None:
        """Called once every search drone reports SEARCH_COMPLETE."""
        self.search_complete = True
        if self.policy == DispatchPolicy.TWO_WAVE:
            self.run_optimal_final_assignment()

    def mark_delivered(self, drone_id: int) -> None:
        drone = self.drones[drone_id]
        for t in self.targets:
            if t.assigned_drone == drone_id and not t.delivered:
                t.delivered = True
                break
        drone.kits_remaining -= 1
        drone.position = self.gcs_position

        if self.policy == DispatchPolicy.CONTINUOUS:
            drone.status = DroneStatus.IDLE_AT_GCS
            self._drain_pending_queue()
        else:  # TWO_WAVE
            if not self.wave1_closed:
                drone.status = DroneStatus.IDLE_AT_GCS
                self._drain_pending_queue()
            else:
                # Wave 1 is done for this drone; hold at GCS until wave 2 trigger.
                drone.status = DroneStatus.HOLDING

    # ------------------------------------------------------------------
    def _try_dispatch(self, target: Target) -> None:
        if self.policy == DispatchPolicy.CONTINUOUS:
            # Fixed: previously, a target detected while every drone was busy
            # was silently dropped — never queued, never retried. Found by
            # testing with more targets than drones could handle at once.
            # Now it queues and gets picked up by _drain_pending_queue() the
            # moment any drone frees up, same mechanism the two-wave policy
            # already used correctly.
            if not self._dispatch_if_possible(target):
                self.pending_queue.append(target)
            return

        # TWO_WAVE
        if not self.wave1_closed:
            dispatched = self._dispatch_if_possible(target)
            if dispatched:
                self.wave1_dispatch_count += 1
            else:
                self.pending_queue.append(target)
            if self.wave1_dispatch_count >= self.wave1_limit:
                self.wave1_closed = True
                if self.search_complete:
                    self.run_optimal_final_assignment()
        else:
            self.pending_queue.append(target)
            if self.search_complete:
                self.run_optimal_final_assignment()

    def _dispatch_if_possible(self, target: Target) -> bool:
        drone = self._nearest_available_drone(target)
        if drone is None:
            return False
        target.assigned_drone = drone.drone_id
        drone.status = DroneStatus.EN_ROUTE
        drone.position = (target.x, target.y)
        return True

    def _drain_pending_queue(self) -> None:
        still_pending = []
        for t in self.pending_queue:
            if t.assigned_drone is None and self._dispatch_if_possible(t):
                continue
            still_pending.append(t)
        self.pending_queue = still_pending

    def run_optimal_final_assignment(self) -> None:
        """
        ALGORITHM 2 — final-wave optimal assignment.

        Called once search is complete: every remaining un-delivered target
        and every drone's remaining capacity are now fully known, so this is
        a static, offline problem — assign all pending targets to available
        delivery-drone kit-slots to MINIMIZE THE SUM of drone-to-target
        distances, solved exactly with the Hungarian algorithm
        (scipy.optimize.linear_sum_assignment).

        This is deliberately different from the greedy nearest-drone
        dispatch used during active search (register_detection /
        _dispatch_if_possible) — greedy is the right call there because
        targets arrive one at a time with incomplete future information;
        Hungarian is the right call here because nothing more is coming and
        the whole problem is visible at once. Same project, two different
        regimes, two different algorithms — not a contradiction.

        SIMPLIFICATION (flagged, not hidden): a drone with multiple kits gets
        multiple "virtual slots", all priced from its CURRENT position — this
        assigns which targets go to which drone optimally, but does not solve
        the sub-problem of what ORDER a multi-kit drone should visit its
        assigned targets in (that's a per-drone routing/TSP problem, out of
        scope for Phase 1). Fine when a drone's assigned targets are close
        together; revisit if that's not the case in practice.
        """
        for drone in self.drones.values():
            if drone.status == DroneStatus.HOLDING:
                drone.status = DroneStatus.IDLE_AT_GCS

        pending = [t for t in self.pending_queue if t.assigned_drone is None]
        if not pending:
            return

        slots: List[int] = []  # drone_id per virtual slot
        for d in self.drones.values():
            if d.status == DroneStatus.IDLE_AT_GCS:
                slots.extend([d.drone_id] * d.kits_remaining)

        if not slots:
            return  # nobody available yet; targets stay pending

        drone_positions = {d.drone_id: d.position for d in self.drones.values()}
        cost = np.zeros((len(pending), len(slots)))
        for i, t in enumerate(pending):
            for j, drone_id in enumerate(slots):
                px, py = drone_positions[drone_id]
                cost[i, j] = math.hypot(px - t.x, py - t.y)

        row_idx, col_idx = linear_sum_assignment(cost)  # minimizes total SUM of cost, exactly

        assigned_ids = set()
        for r, c in zip(row_idx, col_idx):
            target = pending[r]
            drone_id = slots[c]
            target.assigned_drone = drone_id
            self.drones[drone_id].status = DroneStatus.EN_ROUTE
            assigned_ids.add(target.target_id)

        self.pending_queue = [t for t in self.pending_queue if t.target_id not in assigned_ids]

    def _nearest_available_drone(self, target: Target) -> Optional[DeliveryDrone]:
        candidates = [d for d in self.drones.values() if d.is_available()]
        if not candidates:
            return None
        return min(
            candidates,
            key=lambda d: math.hypot(d.position[0] - target.x, d.position[1] - target.y),
        )

    # ------------------------------------------------------------------
    def status_report(self) -> dict:
        return {
            "policy": self.policy.value,
            "targets_total": len(self.targets),
            "targets_dispatched": sum(1 for t in self.targets if t.assigned_drone is not None),
            "targets_delivered": sum(1 for t in self.targets if t.delivered),
            "targets_pending": len(self.pending_queue),
            "drone_status": {d.drone_id: d.status.value for d in self.drones.values()},
        }


# =============================================================================
# Test block
# =============================================================================

if __name__ == "__main__":
    # =========================================================================
    # >>> INPUT YOUR MISSION SHAPE HERE <<<
    # Two ways to provide the polygon — pick one by setting USE_INTERACTIVE_DRAWING.
    #
    #   False (default): use the hardcoded `dummy_polygon` list below — edit
    #                     those (x, y) vertices directly to test a specific shape.
    #
    #   True: a mouse-driven canvas opens (requires a local display — see
    #         draw_polygon_interactively's docstring). Left-click to place each
    #         boundary vertex in order, right-click to undo, Enter to finish.
    #         The clicked shape is used directly AND printed in paste-able
    #         form so you can hardcode it below next time and skip redrawing.
    # =========================================================================
    USE_INTERACTIVE_DRAWING = True

    dummy_polygon = [
        (0, 0),
        (300, 20),
        (280, 200),
        (10, 180),
    ]

    if USE_INTERACTIVE_DRAWING:
        dummy_polygon = draw_polygon_interactively(xlim=(0, 400), ylim=(0, 300))

    camera = CameraParams(
        altitude_m=20.0,
        horizontal_fov_deg=120.0,
        resolution_mp=12.0,
        nadir_assumption=True,   # Phase-1 placeholder toggle, per task spec
    )

    print("=" * 70)
    print("TASK 1 — Fleet Manager: lane generation")
    print("=" * 70)

    fleet = FleetManager(dummy_polygon, camera=camera, num_search_drones=5, overlap_fraction=0.2)
    fleet.generate_lanes()
    fleet.balance_across_fleet()

    for plan in fleet.drone_plans:
        print(f"\nSearch drone {plan.drone_id}  |  lanes: {len(plan.lanes)}  |  "
              f"lane distance: {plan.lane_distance:.1f} m  |  transit: {plan.transit_distance:.1f} m  |  "
              f"total: {plan.total_distance:.1f} m")
        for lane in plan.lanes:
            print(f"    lane {lane.lane_id:>3}: "
                  f"start=({lane.start[0]:7.2f}, {lane.start[1]:7.2f})  "
                  f"end=({lane.end[0]:7.2f}, {lane.end[1]:7.2f})  "
                  f"length={lane.length:6.2f} m")

    print("\n" + "-" * 70)
    print("Mission summary:", fleet.summary())

    # Visual output: color-coded lane segregation across the 5 search drones.
    # Saved next to this script — open it to see which drone flies which part
    # of the polygon. Set show=True instead if you're running this locally
    # with a display (it will pop up a window rather than only saving a file).
    plot_lane_assignment(fleet, save_path="lane_segregation.png", show=False)

    print("\n" + "=" * 70)
    print("TASK 1b — Fleet Manager: perpendicular-formation curved sweep")
    print("=" * 70)
    print("(fixed global-axis version — will be ~straight, since dummy_polygon above")
    print(" is a straight-edge quadrilateral; see the medial-axis demo below for a")
    print(" genuinely curved boundary and genuinely curved tracks)")
    fleet.generate_formation_curves()
    for curve in fleet.formation_curves:
        print(f"  drone {curve.drone_id}: {len(curve.points)} pts, length={curve.length:.1f} m, "
              f"start={curve.start}, end={curve.end}")
    print("Formation summary:", fleet.summary_formation())
    plot_formation_curves(fleet, curves=fleet.formation_curves,
                           save_path="formation_curves.png", show=False,
                           title="RescueSwarm — fixed-axis formation (straight boundary in -> straight tracks out)")

    # =========================================================================
    # TASK 1c — medial-axis formation on a GENUINELY CURVED polygon.
    # Control points for a bending, elongated shape (a smoothed spline is fit
    # through these — see interpolate_closed_curve). This is what actually
    # demonstrates curved tracks, since the boundary itself now has curvature.
    # =========================================================================
    print("\n" + "=" * 70)
    print("TASK 1c — Fleet Manager: medial-axis formation on a CURVED polygon")
    print("=" * 70)

    # A genuine bending CHANNEL shape (see make_bending_channel_polygon's
    # docstring for why a hand-drawn blob was tried first and rejected).
    curved_polygon, true_centerline = make_bending_channel_polygon(
        length=400.0, base_width=40.0, n_points=200, bend_amplitude=50.0, bend_periods=1.0
    )

    curved_fleet = FleetManager(curved_polygon, camera=camera, num_search_drones=5, overlap_fraction=0.2)
    curved_fleet.generate_formation_curves_medial(n_steps=150)
    for curve in curved_fleet.formation_curves_medial:
        print(f"  drone {curve.drone_id}: {len(curve.points)} pts, length={curve.length:.1f} m")

    # Sanity check against ground truth: the middle drone (drone 2 of 5) should
    # track close to the TRUE centerline we built the channel from, since it
    # sits at the exact midpoint of every chord. This is the actual proof the
    # algorithm is doing the right thing, not just "the plot looks curvy".
    mid_drone = curved_fleet.formation_curves_medial[2]
    max_dev = 0.0
    for px, py in mid_drone.points:
        nearest = min(math.hypot(px - tx, py - ty) for tx, ty in true_centerline)
        max_dev = max(max_dev, nearest)
    print(f"  validation: drone 2 (middle) max deviation from true sine centerline = {max_dev:.2f} m")

    med_summary = curved_fleet.summary_formation_medial()
    print("Medial formation summary:", med_summary)
    if med_summary["medial_path_length_m"] > med_summary["straight_long_axis_length_m"]:
        print(f"  -> medial path is {med_summary['medial_path_length_m'] - med_summary['straight_long_axis_length_m']:.1f} m "
              f"LONGER than the straight long-axis, which is why avg width along it "
              f"({med_summary['avg_width_along_medial_m']} m) is SMALLER than the straight-line "
              f"case ({med_summary['avg_width_if_flown_straight_m']} m) — exactly the "
              f"lower-altitude / better-resolution tradeoff you're going for.")
    plot_formation_curves(curved_fleet, curves=curved_fleet.formation_curves_medial,
                           save_path="formation_curves_medial.png", show=False,
                           title="RescueSwarm — medial-axis formation on a bending channel")

    # --- Task 2: Target Manager dispatch policy demo ---
    print("\n" + "=" * 70)
    print("TASK 2 — Target Manager: dispatch policy demo")
    print("=" * 70)

    for policy in (DispatchPolicy.CONTINUOUS, DispatchPolicy.TWO_WAVE):
        print(f"\n--- Policy: {policy.value} ---")
        gcs_pos = (0.0, 0.0)
        drones = [
            DeliveryDrone(drone_id=0, kits_remaining=3, position=gcs_pos),
            DeliveryDrone(drone_id=1, kits_remaining=3, position=gcs_pos),
            DeliveryDrone(drone_id=2, kits_remaining=4, position=gcs_pos),
        ]
        # wave1_limit=5 per "first 5 drops" — dispatched via greedy nearest-drone
        # as they're detected (same as CONTINUOUS). Anything detected AFTER the
        # 5th dispatch queues in pending_queue untouched until search completes,
        # at which point Algorithm 2 (Hungarian, run_optimal_final_assignment)
        # assigns everything remaining in one optimal batch.
        tm = TargetManager(drones, gcs_position=gcs_pos, dispatch_policy=policy, wave1_limit=5)

        # Simulate detections arriving mid-search (7 targets: first 5 go out
        # immediately under two-wave, the last 2 queue for Algorithm 2).
        detections = [
            Target(0, 50, 40), Target(1, 120, 90), Target(2, 200, 150),
            Target(3, 60, 160), Target(4, 250, 60), Target(5, 90, 200), Target(6, 30, 100),
        ]
        for t in detections:
            tm.register_detection(t)
            print(f"  detected target {t.target_id} -> assigned_drone={t.assigned_drone}")

        print("  status after detections:", tm.status_report())

        # Simulate the wave-1 assigned deliveries completing.
        for t in detections:
            if t.assigned_drone is not None:
                tm.mark_delivered(t.assigned_drone)

        print("  status after wave-1 deliveries:", tm.status_report())

        # Search completes -> TWO_WAVE triggers Algorithm 2 (Hungarian optimal
        # assignment) for whatever is still pending; CONTINUOUS has nothing
        # pending by design (it dispatches immediately on every detection).
        tm.notify_search_complete()
        print("  status after search_complete (Algorithm 2, if TWO_WAVE):", tm.status_report())
        assigned_now = [t for t in tm.targets if t.assigned_drone is not None and not t.delivered]
        if assigned_now:
            print("  final-wave assignments:", [(t.target_id, t.assigned_drone) for t in assigned_now])
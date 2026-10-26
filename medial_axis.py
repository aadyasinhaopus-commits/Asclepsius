"""Medial axis transform (MAT) trajectory extraction with progress reporting.

The medial axis is the locus of centres of maximal inscribed disks. It is built
from the Voronoi diagram of a dense boundary sampling, pruned by the boundary
arc length between the two generating samples (Ogniewicz residual), reduced to
a tree, and its longest path ("trunk") is smoothed into a C2 cubic spline.
The MAT always exists for a simple closed region, so this never fails for lack
of a solution.
"""

import time

import numpy as np
from scipy import ndimage
from scipy.interpolate import splev, splprep
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components, dijkstra, minimum_spanning_tree
from scipy.spatial import Voronoi, cKDTree
from skimage.draw import polygon2mask
from skimage.measure import points_in_poly

from boundary import cross, intersection_parameters, validate_polygon


class Cancelled(Exception):
    pass


def _noop(fraction, message):
    pass


def resample(polygon, spacing):
    closed = np.vstack((polygon, polygon[:1]))
    lengths = np.linalg.norm(np.diff(closed, axis=0), axis=1)
    arc = np.r_[0., np.cumsum(lengths)]
    perimeter = arc[-1]
    count = int(np.clip(perimeter / spacing, 400, 6000))
    sites = np.union1d(np.arange(count) * perimeter / count, arc[:-1])
    samples = np.c_[np.interp(sites, arc, closed[:, 0]), np.interp(sites, arc, closed[:, 1])]
    keep = np.r_[True, np.linalg.norm(np.diff(samples, axis=0), axis=1) > 1e-6]
    return samples[keep], sites[keep], perimeter


def first_hit(origin, direction, polygon):
    hits = intersection_parameters(origin, direction, polygon)
    hits = hits[hits > 1e-9]
    return float(np.min(hits)) if len(hits) else 0.


def longest_path(graph, nodes):
    distances = dijkstra(graph, directed=False, indices=nodes[0])
    start = nodes[np.argmax(np.where(np.isfinite(distances[nodes]), distances[nodes], -1))]
    distances, predecessors = dijkstra(graph, directed=False, indices=start, return_predecessors=True)
    end = nodes[np.argmax(np.where(np.isfinite(distances[nodes]), distances[nodes], -1))]
    path = [end]
    while path[-1] != start:
        path.append(predecessors[path[-1]])
    return np.array(path[::-1])


def smooth_path(points, polygon, sigma=1.):
    lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
    keep = np.r_[True, lengths > 1e-6]
    points = points[keep]
    if len(points) < 4:
        return points, np.gradient(points, axis=0) if len(points) > 1 else np.zeros_like(points)
    for scale in (sigma, sigma / 4, 0.):
        tck, _ = splprep(points.T, s=len(points) * scale ** 2, k=3)
        u = np.linspace(0, 1, 801)
        path = np.column_stack(splev(u, tck))
        if scale == 0. or np.all(points_in_poly(path, polygon)):
            return path, np.column_stack(splev(u, tck, der=1))
    return path, np.column_stack(splev(u, tck, der=1))


def perpendicular_chords(path, velocity, polygon, count=81):
    index = np.linspace(0, len(path) - 1, count + 2)[1:-1].round().astype(int)
    chords = []
    for i in index:
        speed = np.linalg.norm(velocity[i])
        if speed < 1e-12:
            continue
        normal = np.array([-velocity[i, 1], velocity[i, 0]]) / speed
        hits = intersection_parameters(path[i], normal, polygon)
        below, above = hits[hits < -1e-9], hits[hits > 1e-9]
        if not len(below) or not len(above):
            continue
        lo, hi = float(np.max(below)), float(np.min(above))
        chords.append({"center": path[i].tolist(), "a": (path[i] + lo * normal).tolist(),
                       "b": (path[i] + hi * normal).tolist(), "error": abs(lo + hi) / 2})
    return chords


def analyze(points, resolution=240, prune=.05, extend=True, progress=_noop):
    started = time.perf_counter()
    progress(.02, "Validating boundary")
    polygon = validate_polygon(points)
    if isinstance(resolution, bool) or resolution not in (180, 240, 320):
        raise ValueError("Field resolution must be 180, 240, or 320.")
    if isinstance(prune, bool) or not isinstance(prune, (int, float)) or not 0 <= prune <= .4:
        raise ValueError("Branch pruning must be between 0 and 0.4.")

    progress(.08, "Resampling boundary")
    samples, sites, perimeter = resample(polygon, 1.)

    progress(.15, f"Voronoi diagram of {len(samples)} boundary samples")
    voronoi = Voronoi(samples)
    vertices = voronoi.vertices

    progress(.40, "Selecting interior Voronoi vertices")
    inside = np.zeros(len(vertices), dtype=bool)
    candidate = np.all((vertices > 0) & (vertices < 600), axis=1)
    inside[candidate] = points_in_poly(vertices[candidate], polygon)

    progress(.50, "Pruning medial branches")
    ridges = np.array(voronoi.ridge_vertices)
    generators = voronoi.ridge_points
    finite = np.all(ridges >= 0, axis=1)
    ridges, generators = ridges[finite], generators[finite]
    arc = np.abs(sites[generators[:, 0]] - sites[generators[:, 1]])
    arc = np.minimum(arc, perimeter - arc)
    keep = inside[ridges[:, 0]] & inside[ridges[:, 1]] & (arc > max(prune * perimeter, 3.))
    edges = ridges[keep]
    tree = cKDTree(samples)
    radii = np.zeros(len(vertices))
    used = np.unique(edges)
    if len(used):
        radii[used] = tree.query(vertices[used])[0]

    progress(.62, "Building medial graph")
    trunk_nodes = np.array([], dtype=int)
    branch_segments = []
    if len(edges):
        weights = np.linalg.norm(vertices[edges[:, 0]] - vertices[edges[:, 1]], axis=1) + 1e-9
        size = len(vertices)
        graph = coo_matrix((weights, (edges[:, 0], edges[:, 1])), shape=(size, size)).tocsr()
        graph = graph.maximum(graph.T)
        _, labels = connected_components(graph, directed=False)
        totals = np.bincount(labels[edges[:, 0]], weights=weights, minlength=labels.max() + 1)
        main = np.argmax(totals)
        mst = minimum_spanning_tree(graph).tocoo()
        component = labels[mst.row] == main
        rows, cols = mst.row[component], mst.col[component]
        branch_segments = [[vertices[a].tolist(), vertices[b].tolist()] for a, b in zip(rows, cols)]
        mst = coo_matrix((mst.data[component], (rows, cols)), shape=(size, size)).tocsr()
        progress(.72, "Extracting longest medial path")
        trunk_nodes = longest_path(mst, np.unique(np.r_[rows, cols]))

    warnings = []
    trunk = vertices[trunk_nodes] if len(trunk_nodes) else np.empty((0, 2))
    trunk_length = float(np.sum(np.linalg.norm(np.diff(trunk, axis=0), axis=1))) if len(trunk) > 1 else 0.
    area = abs(float(np.sum(cross(polygon, np.roll(polygon, -1, axis=0))))) / 2
    if trunk_length < .02 * np.sqrt(area):
        # Disk-like region: the medial axis collapses to (nearly) a point.
        centers = vertices[inside] if np.any(inside) else polygon.mean(axis=0, keepdims=True)
        center = centers[np.argmax(tree.query(centers)[0])]
        _, _, axes = np.linalg.svd(samples - samples.mean(axis=0), full_matrices=False)
        direction = axes[0]
        trunk = np.array([center - direction, center + direction])
        warnings.append("The medial axis is (nearly) a single point; the principal axis through the "
                        "largest inscribed disk centre is used as the trajectory.")

    progress(.80, "Smoothing trunk into a cubic spline")
    path, velocity = smooth_path(trunk, polygon)
    if extend and len(path) > 1:
        progress(.84, "Extending trunk ends to the boundary")
        pieces = []
        for end, sign in ((0, -1.), (-1, 1.)):
            tangent = sign * velocity[end] / max(np.linalg.norm(velocity[end]), 1e-12)
            reach = first_hit(path[end], tangent, polygon)
            steps = max(int(reach / 2), 1)
            pieces.append(path[end] + np.linspace(0, reach * (1 - 1e-7), steps + 1)[1:, None] * tangent)
        path = np.vstack((pieces[0][::-1], path, pieces[1]))
        velocity = np.gradient(path, axis=0)

    progress(.88, "Measuring disks and perpendicular chords")
    path_radius = tree.query(path)[0]
    chords = perpendicular_chords(path, velocity, polygon)

    progress(.93, "Computing distance field")
    spacing = 600 / (resolution - 1)
    mask = polygon2mask((resolution, resolution), polygon[:, ::-1] / spacing)
    heat = ndimage.distance_transform_edt(mask)[::3, ::3] * spacing
    incoming, outgoing = polygon - np.roll(polygon, 1, axis=0), np.roll(polygon, -1, axis=0) - polygon
    turning = np.arctan2(cross(incoming, outgoing), np.sum(incoming * outgoing, axis=1))
    curvature = turning / ((np.linalg.norm(incoming, axis=1) + np.linalg.norm(outgoing, axis=1)) / 2)

    errors = [chord["error"] for chord in chords]
    warnings += [
        "Medial axis: centres of maximal inscribed disks, from the Voronoi diagram of the dense boundary sampling.",
        f"Branches whose generating boundary arc is shorter than {prune:.0%} of the perimeter are pruned.",
        "The trajectory is the longest path through the pruned medial tree, smoothed by a cubic spline"
        + (" and extended tangentially to the boundary." if extend else "."),
        "The perpendicular chord midpoint error is shown as a diagnostic only; the MAT is equidistant to the "
        "boundary, which equals chord bisection only where the two sides are locally parallel.",
    ]
    progress(1., "Done")
    return {
        "mode": "medial",
        "boundary": polygon.tolist(),
        "path": path.tolist(),
        "radius": path_radius.tolist(),
        "branches": branch_segments,
        "chords": chords,
        "curvature": curvature.tolist(),
        "heat": {"width": heat.shape[1], "height": heat.shape[0], "values": np.round(heat, 2).ravel().tolist()},
        "stats": {
            "length": round(float(np.sum(np.linalg.norm(np.diff(path, axis=0), axis=1))), 3),
            "branches": len(branch_segments),
            "maxRadius": float(np.max(path_radius)),
            "minRadius": float(np.min(path_radius)),
            "maxChordError": float(np.max(errors)) if errors else 0.,
            "meanChordError": float(np.mean(errors)) if errors else 0.,
            "samples": len(samples),
            "concaveVertices": int(np.count_nonzero(curvature < -1e-6)),
            "elapsedMs": round((time.perf_counter() - started) * 1000),
        },
        "warnings": warnings,
    }

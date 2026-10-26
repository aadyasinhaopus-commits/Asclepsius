"""Polygon predicates and line intersections shared by solver and validation."""

import numpy as np


def cross(a, b):
    return a[..., 0] * b[..., 1] - a[..., 1] * b[..., 0]


def inside(point, polygon):
    a, b = polygon, np.roll(polygon, -1, axis=0)
    edge, rel = b - a, np.asarray(point) - a
    on = (np.abs(cross(edge, rel)) < 1e-7) & (
        np.sum(rel * (np.asarray(point) - b), axis=1) <= 1e-7
    )
    if np.any(on):
        return True
    ids = np.flatnonzero((a[:, 1] > point[1]) != (b[:, 1] > point[1]))
    x = a[ids, 0] + (point[1] - a[ids, 1]) * edge[ids, 0] / edge[ids, 1]
    return bool(np.count_nonzero(x > point[0]) % 2)


def intersection_parameters(origin, direction, polygon):
    edges = np.roll(polygon, -1, axis=0) - polygon
    rel = polygon - origin
    den = cross(direction, edges)
    usable = np.abs(den) > 1e-10
    t = cross(rel[usable], edges[usable]) / den[usable]
    u = cross(rel[usable], direction) / den[usable]
    hits = list(t[(u >= -1e-9) & (u <= 1 + 1e-9)])
    collinear = (~usable) & (np.abs(cross(rel, direction)) < 1e-8)
    norm2 = float(np.dot(direction, direction))
    if norm2 > 1e-16 and np.any(collinear):
        hits.extend((rel[collinear] @ direction / norm2).tolist())
        hits.extend(((rel[collinear] + edges[collinear]) @ direction / norm2).tolist())
    return np.asarray(hits)


def segment_inside(a, b, polygon):
    if not inside(a, polygon) or not inside(b, polygon):
        return False
    direction = np.asarray(b) - a
    if np.linalg.norm(direction) < 1e-10:
        return True
    hits = intersection_parameters(np.asarray(a), direction, polygon)
    cuts = np.unique(np.r_[0.0, hits[(hits > 0) & (hits < 1)], 1.0])
    return all(inside(a + direction * ((lo + hi) / 2), polygon)
               for lo, hi in zip(cuts[:-1], cuts[1:]))


def validate_polygon(points):
    try:
        polygon = np.asarray(points, dtype=float)
    except (ValueError, TypeError) as error:
        raise ValueError("Boundary coordinates must be numeric pairs.") from error
    if polygon.ndim != 2 or polygon.shape[1] != 2 or not 3 <= len(polygon) <= 384:
        raise ValueError("Use between 3 and 384 boundary vertices.")
    if not np.all(np.isfinite(polygon)) or np.any(polygon < 5) or np.any(polygon > 595):
        raise ValueError("Keep all vertices within the drawing margin (5 to 595).")
    next_points = np.roll(polygon, -1, axis=0)
    edges = next_points - polygon
    if np.any(np.linalg.norm(edges, axis=1) < .1):
        raise ValueError("Adjacent vertices are too close together.")
    incoming = polygon - np.roll(polygon, 1, axis=0)
    if np.any((np.abs(cross(incoming, edges)) < 1e-8) & (np.sum(incoming * edges, axis=1) < 0)):
        raise ValueError("The boundary doubles back over itself.")
    for i, (a, b) in enumerate(zip(polygon, next_points)):
        for j in range(i + 1, len(polygon)):
            if j == i + 1 or (i == 0 and j == len(polygon) - 1):
                continue
            c, d = polygon[j], next_points[j]
            if np.any(np.maximum(np.minimum(a, b), np.minimum(c, d)) >
                      np.minimum(np.maximum(a, b), np.maximum(c, d)) + 1e-9):
                continue
            if cross(b - a, c - a) * cross(b - a, d - a) <= 1e-9 and cross(d - c, a - c) * cross(d - c, b - c) <= 1e-9:
                raise ValueError("The boundary crosses or touches itself. Move its vertices apart.")
    area = float(np.sum(cross(polygon, next_points)) / 2)
    if abs(area) < 100:
        raise ValueError("The shape is too small. Draw a larger closed region.")
    return polygon if area > 0 else polygon[::-1].copy()

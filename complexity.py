"""Shape complexity descriptors for a simple polygon field."""

import numpy as np
from scipy.spatial import ConvexHull

from boundary import cross


def describe(polygon, medial):
    nxt = np.roll(polygon, -1, axis=0)
    area = abs(float(np.sum(cross(polygon, nxt)))) / 2
    edges = nxt - polygon
    perimeter = float(np.sum(np.linalg.norm(edges, axis=1)))
    hull = ConvexHull(polygon)
    hull_area, hull_perimeter = float(hull.volume), float(hull.area)

    incoming = polygon - np.roll(polygon, 1, axis=0)
    turning = np.arctan2(cross(incoming, edges), np.sum(incoming * edges, axis=1))
    reflex = int(np.count_nonzero(turning < -1e-3))
    # Total absolute turning is 2*pi for convex shapes; the excess measures wiggliness.
    bending = float(np.sum(np.abs(turning)) / (2 * np.pi))

    centred = polygon - polygon.mean(axis=0)
    singular = np.linalg.svd(centred, compute_uv=False)
    radius = np.asarray(medial["radius"])
    inner = radius[len(radius) // 10: len(radius) - len(radius) // 10] if len(radius) > 20 else radius
    # A neck is a run of trunk points whose inscribed radius is below 75% of the widest
    # point on both sides of it: the field pinches, then widens again.
    necks = 0
    if len(inner) > 5:
        left = np.maximum.accumulate(inner)
        right = np.maximum.accumulate(inner[::-1])[::-1]
        pinched = inner < .75 * np.minimum(left, right)
        necks = int(np.count_nonzero(np.diff(np.r_[0, pinched.astype(int)]) == 1))
    disk = any("single point" in note for note in medial["warnings"])

    circularity = 4 * np.pi * area / perimeter ** 2
    solidity = area / hull_area
    convexity = hull_perimeter / perimeter
    elongation = float(singular[0] / max(singular[1], 1e-9))
    width_ratio = 1. if disk or not len(inner) else float(inner.max() / max(inner.min(), 1e-9))
    branches = int(medial["stats"]["branches"])
    # Composite 0-100 score: each term is 0 for a disk and saturates for very complex fields.
    terms = {
        "nonConvexity": 1 - solidity,
        "boundaryRoughness": min((bending - 1) / 3, 1),
        "elongation": min((elongation - 1) / 4, 1),
        "widthVariation": min((width_ratio - 1) / 4, 1),
        "necks": min(necks / 3, 1),
    }
    weights = {"nonConvexity": .3, "boundaryRoughness": .25, "elongation": .15, "widthVariation": .15, "necks": .15}
    score = 100 * sum(weights[k] * max(v, 0.) for k, v in terms.items())
    return {
        "score": round(float(score), 1),
        "area": round(area, 1),
        "perimeter": round(perimeter, 1),
        "circularity": round(float(circularity), 3),
        "solidity": round(float(solidity), 3),
        "convexity": round(float(convexity), 3),
        "reflexVertices": reflex,
        "bending": round(bending, 2),
        "elongation": round(elongation, 2),
        "maxInscribed": round(float(radius.max()), 1),
        "widthRatio": round(width_ratio, 2),
        "necks": necks,
        "medialEdges": branches,
        "terms": {k: round(float(max(v, 0.)), 3) for k, v in terms.items()},
    }

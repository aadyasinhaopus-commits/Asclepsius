"""Analytic constructions for explicitly declared, unedited shape families."""

import math
from fractions import Fraction

import numpy as np

from boundary import inside, intersection_parameters

KINDS = ("circle", "ellipse", "star", "flower", "crescent", "neck")


def preset_boundary(model):
    if not isinstance(model, dict) or model.get("kind") not in KINDS:
        raise ValueError("Unknown analytic boundary model.")
    kind = model["kind"]
    depth, arms = model.get("depth", .55), model.get("arms", 5)
    if isinstance(depth, bool) or not isinstance(depth, (float, int)) or not math.isfinite(depth) or not .15 <= depth <= .8:
        raise ValueError("Preset indentation must be between 0.15 and 0.8.")
    if isinstance(arms, bool) or not isinstance(arms, int) or not 3 <= arms <= 12:
        raise ValueError("Preset arm count must be an integer from 3 to 12.")
    if kind == "star":
        t = -np.pi / 2 + np.arange(2 * arms) * np.pi / arms
        r = np.where(np.arange(2 * arms) % 2, 220 * (1 - depth), 220)
    elif kind == "crescent":
        t = np.pi * .23 + np.arange(65) / 64 * np.pi * 1.54
        outer = np.c_[300 + 220 * np.cos(t), 300 + 220 * np.sin(t)]
        inner = 80 + 90 * (1 - depth)
        return np.vstack((outer, np.c_[300 + inner * np.cos(t[::-1]), 300 + inner * np.sin(t[::-1])]))
    elif kind == "neck":
        h = 18 + 65 * (1 - depth)
        return np.array([[80, 155], [220, 155], [220, 300 - h], [380, 300 - h],
                         [380, 155], [520, 155], [520, 445], [380, 445],
                         [380, 300 + h], [220, 300 + h], [220, 445], [80, 445]], dtype=float)
    else:
        t = np.arange(160) / 160 * 2 * np.pi
        r = 170 * (1 + depth * .42 * np.cos(arms * t)) if kind == "flower" else 205
    xscale, yscale = (1.12, .63) if kind == "ellipse" else (1, 1)
    return np.c_[300 + r * np.cos(t) * xscale, 300 + r * np.sin(t) * yscale]


def line_piece(a, b):
    return {"interval": [0., 1.], "controls": [list(a), list(b)], "weights": [1., 1.]}


def reflection_construction(polygon):
    rational = [tuple(Fraction(float(v)) for v in point) for point in polygon]
    edges = {frozenset((rational[i], rational[(i + 1) % len(rational)])) for i in range(len(rational))}
    choices = []
    for coordinate in (0, 1):
        axis = (min(p[coordinate] for p in rational) + max(p[coordinate] for p in rational)) / 2
        reflected = [tuple(2 * axis - value if j == coordinate else value for j, value in enumerate(p)) for p in rational]
        if any(frozenset((reflected[i], reflected[(i + 1) % len(reflected)])) not in edges for i in range(len(reflected))):
            continue
        origin = np.mean(polygon, axis=0)
        origin[coordinate] = float(axis)
        direction = np.array([0., 1.]) if coordinate == 0 else np.array([1., 0.])
        hits = np.unique(intersection_parameters(origin, direction, polygon))
        for lo, hi in zip(hits[:-1], hits[1:]):
            if hi - lo > 1e-7 and inside(origin + (lo + hi) / 2 * direction, polygon):
                choices.append((hi - lo, origin + lo * direction, origin + hi * direction, coordinate, axis))
    if not choices:
        return None
    _, a, b, coordinate, axis = max(choices, key=lambda choice: choice[0])
    return {
        "pieces": [line_piece(a, b)],
        "proof": "Exact rational comparison of every reflected polygon edge establishes symmetry. Reflection swaps the first normal-chord hits.",
        "boundaryDescription": "Input polygon itself; no rounding or symmetrization is applied. The longest contained interval among certified Cartesian reflection axes is selected.",
        "model": {"kind": "polygon-reflection", "coordinate": coordinate, "axisRational": str(axis)},
        "maxError": 0., "rmsError": 0., "checks": 0,
    }


def exact_construction(polygon, model):
    if model is None:
        return reflection_construction(polygon)
    expected = preset_boundary(model)
    if polygon.shape != expected.shape or not np.allclose(polygon, expected, rtol=0, atol=1e-7):
        raise ValueError("The declared analytic preset no longer matches the vertices. Clear the model after editing.")
    kind = model["kind"]
    depth, arms = model.get("depth", .55), model.get("arms", 5)
    description = "Reflection fixes the trajectory and swaps the first normal-boundary hits, so both half-chords are equal."
    boundary_type = "Declared symmetric polygon family (floating-point display coordinates)."
    if kind == "circle":
        pieces = [line_piece([95., 300.], [505., 300.])]
        boundary_type = "Exact circle, center (300, 300), radius 205; vertices are display samples."
        description = "Every perpendicular chord of a circle is bisected by this diameter."
    elif kind == "ellipse":
        pieces = [line_piece([300 - 205 * 1.12, 300.], [300 + 205 * 1.12, 300.])]
        boundary_type = "Exact ellipse, center (300, 300), semiaxes 229.6 and 129.15; vertices are display samples."
    elif kind == "crescent":
        inner = 80 + 90 * (1 - depth)
        radius = (220 + inner) / 2
        start, sweep = np.pi * .23, np.pi * 1.54
        pieces = []
        for i in range(4):
            a, b = start + sweep * i / 4, start + sweep * (i + 1) / 4
            middle = (a + b) / 2
            weight = math.cos((b - a) / 2)
            controls = [
                [300 + radius * math.cos(a), 300 + radius * math.sin(a)],
                [300 + radius * math.cos(middle) / weight, 300 + radius * math.sin(middle) / weight],
                [300 + radius * math.cos(b), 300 + radius * math.sin(b)],
            ]
            pieces.append({"interval": [i / 4, (i + 1) / 4], "controls": controls, "weights": [1., weight, 1.]})
        boundary_type = f"Exact annular sector, radii {inner:g} and 220, angles 0.23*pi to 1.77*pi; arcs are not polygon edges."
        description = "Trajectory radius is (inner + outer)/2. Its normal chords are radial, with both half-widths (outer - inner)/2."
    elif kind == "star":
        bottom = 220 if arms % 2 == 0 else 220 * (1 - depth)
        pieces = [line_piece([300., 80.], [300., 300 + bottom])]
    elif kind == "flower":
        right = 170 * (1 + depth * .42)
        left = 170 * (1 + depth * .42 * (-1) ** arms)
        pieces = [line_piece([300 - left, 300.], [300 + right, 300.])]
    else:
        pieces = [line_piece([80., 300.], [520., 300.])]
    return {"pieces": pieces, "proof": description, "boundaryDescription": boundary_type,
            "model": model, "maxError": 0., "rmsError": 0., "checks": 0}


def evaluate_pieces(pieces, parameters):
    parameters = np.asarray(parameters, dtype=float)
    positions = np.empty((len(parameters), 2))
    velocities = np.empty_like(positions)
    for index, piece in enumerate(pieces):
        a, b = piece["interval"]
        chosen = (parameters >= a) & ((parameters < b) if index < len(pieces) - 1 else (parameters <= b))
        u = (parameters[chosen] - a) / (b - a)
        controls = np.asarray(piece["controls"])
        weights = np.asarray(piece["weights"])
        degree = len(controls) - 1
        bernstein = np.column_stack([math.comb(degree, j) * u ** j * (1 - u) ** (degree - j)
                                     for j in range(degree + 1)])
        lower = np.column_stack([math.comb(degree - 1, j) * u ** j * (1 - u) ** (degree - 1 - j)
                                 for j in range(degree)])
        derivative = degree * (np.c_[np.zeros(len(u)), lower] - np.c_[lower, np.zeros(len(u))])
        denominator = bernstein @ weights
        numerator = (bernstein * weights) @ controls
        positions[chosen] = numerator / denominator[:, None]
        velocities[chosen] = (
            ((derivative * weights) @ controls) * denominator[:, None]
            - numerator * (derivative @ weights)[:, None]
        ) / denominator[:, None] ** 2 / (b - a)
    return positions, velocities


def analytic_chords(points, velocities, model):
    normals = np.c_[-velocities[:, 1], velocities[:, 0]]
    normals /= np.linalg.norm(normals, axis=1)[:, None]
    kind = model["kind"]
    if kind in ("circle", "ellipse"):
        rx, ry = (205., 205.) if kind == "circle" else (205 * 1.12, 205 * .63)
        halfwidth = ry * np.sqrt(np.maximum(0, 1 - ((points[:, 0] - 300) / rx) ** 2))
    elif kind == "crescent":
        inner = 80 + 90 * (1 - model.get("depth", .55))
        halfwidth = np.full(len(points), (220 - inner) / 2)
    else:
        raise ValueError("This preset uses polygon-boundary intersections.")
    return points - halfwidth[:, None] * normals, points + halfwidth[:, None] * normals

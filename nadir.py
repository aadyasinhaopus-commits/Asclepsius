"""Nadir camera planning: headings, altitudes, and verified ground coverage.

A nadir camera at altitude h with horizontal/vertical field of view (hfov, vfov)
images a rectangle under the drone: across-track width 2*h*tan(hfov/2) and
along-track length 2*h*tan(vfov/2), rotated to the drone's heading.

For each drone, every cell of its assigned area is matched to the nearest sample
of its path and expressed in that sample's body frame (along, across). The
footprint at that sample must reach it, which fixes the minimum altitude.
Footprints swept between consecutive samples are rasterised and any gap left
on the field raises the altitude of the drone that owns the gap. This repeats
until every field cell is imaged.
"""

import math

import numpy as np
from scipy.ndimage import maximum_filter1d
from scipy.spatial import cKDTree

JUMP = 25.


def headings(path):
    """Unit direction of travel per sample, ignoring transit jumps."""
    if len(path) < 2:
        return np.tile([1., 0.], (len(path), 1))
    step = np.diff(path, axis=0)
    length = np.linalg.norm(step, axis=1)
    usable = (length > 1e-6) & (length <= JUMP)
    direction = np.zeros_like(step)
    direction[usable] = step[usable] / length[usable, None]
    out = np.zeros_like(path)
    out[:-1] += direction
    out[1:] += direction
    norm = np.linalg.norm(out, axis=1)
    last = np.array([1., 0.])
    for i in range(len(out)):
        if norm[i] > 1e-9:
            last = out[i] / norm[i]
        out[i] = last
    first = np.flatnonzero(norm > 1e-9)
    if len(first):
        out[:first[0]] = out[first[0]]
    return out


def plan(grid, lanes, paths, hfov=70., vfov=55., overlap=.1, max_rounds=8):
    tan_x, tan_y = math.tan(math.radians(hfov) / 2), math.tan(math.radians(vfov) / 2)
    drones = len(paths)
    field = grid.mask
    rows, cols = np.nonzero(field)
    xy = grid.xy[rows, cols]
    owner = lanes[rows, cols].copy()
    # Any cell without a lane goes to the drone whose path passes closest.
    trees = [cKDTree(p) if len(p) else None for p in paths]
    orphan = np.flatnonzero(owner < 0)
    if len(orphan):
        best = np.full(len(orphan), np.inf)
        for i, tree in enumerate(trees):
            if tree is None:
                continue
            d = tree.query(xy[orphan])[0]
            better = d < best
            owner[orphan[better]], best[better] = i, d[better]
    pad = grid.spacing * math.sqrt(2) / 2
    plans = []
    for i, path in enumerate(paths):
        if not len(path):
            plans.append({"heading": np.zeros((0, 2)), "altitude": np.zeros(0)})
            continue
        heading = headings(path)
        altitude = np.zeros(len(path))
        mine = np.flatnonzero(owner == i)
        if len(mine):
            index = trees[i].query(xy[mine])[1]
            offset = xy[mine] - path[index]
            along = np.abs(np.sum(offset * heading[index], axis=1)) + pad
            across = np.abs(offset[:, 0] * -heading[index, 1] + offset[:, 1] * heading[index, 0]) + pad
            # Along-track reach is supplied by motion between samples; only half a step is needed.
            gap = np.linalg.norm(np.diff(path, axis=0), axis=1)
            gap = np.where(gap > JUMP, 0., gap)
            half_step = np.maximum(np.r_[gap, 0.], np.r_[0., gap])[index] / 2
            need = np.maximum(across / tan_x, np.maximum(along - half_step, 0) / tan_y)
            np.maximum.at(altitude, index, need)
        altitude = maximum_filter1d(altitude, 3)
        # Samples with no assigned cells still need a flyable altitude: interpolate from neighbours.
        known = altitude > 0
        if np.any(known) and not np.all(known):
            index = np.arange(len(altitude))
            altitude = np.interp(index, index[known], altitude[known])
        altitude = np.maximum(altitude, grid.spacing)
        plans.append({"heading": heading, "altitude": altitude * (1 + overlap)})

    owner_map = np.full(field.shape, -1)
    owner_map[rows, cols] = owner
    for round_index in range(max_rounds):
        hits = np.zeros(field.shape, dtype=np.int32)
        for i, path in enumerate(paths):
            sweep_footprints(grid, path, plans[i], tan_x, tan_y, hits)
        gaps = field & (hits == 0)
        if not np.any(gaps):
            break
        gap_rows, gap_cols = np.nonzero(gaps)
        gap_owner = owner_map[gap_rows, gap_cols]
        gap_xy = grid.xy[gap_rows, gap_cols]
        for i, path in enumerate(paths):
            pick = gap_owner == i
            if not np.any(pick) or not len(path):
                continue
            index = trees[i].query(gap_xy[pick])[1]
            for j in np.unique(index):
                plans[i]["altitude"][max(j - 1, 0):j + 2] *= 1.15
                plans[i]["altitude"][max(j - 1, 0):j + 2] += grid.spacing
    covered = field & (hits > 0)
    return plans, hits, {
        "cameraCoverage": round(100 * np.count_nonzero(covered) / max(np.count_nonzero(field), 1), 2),
        "gapCells": int(np.count_nonzero(field & (hits == 0))),
        "rounds": round_index + 1,
        "maxAltitude": round(float(max((p["altitude"].max() for p in plans if len(p["altitude"])), default=0.)), 1),
        "meanAltitude": round(float(np.mean(np.concatenate([p["altitude"] for p in plans if len(p["altitude"])]))), 1),
        "meanOverlap": round(float(np.mean(hits[covered])) if np.any(covered) else 0., 2),
    }


def footprint_size(altitude, tan_x, tan_y):
    """Half along-track and half across-track extent of the nadir image."""
    return altitude * tan_y, altitude * tan_x


def stamp(grid, center, heading, half_along, half_across, out):
    """Mark grid cells whose centres lie inside the heading-aligned footprint rectangle."""
    reach = math.hypot(half_along, half_across)
    c0, r0 = np.floor((center - reach) / grid.spacing).astype(int)
    c1, r1 = np.ceil((center + reach) / grid.spacing).astype(int)
    c0, r0 = max(c0, 0), max(r0, 0)
    c1, r1 = min(c1, grid.size - 1), min(r1, grid.size - 1)
    if c1 < c0 or r1 < r0:
        return
    local = grid.xy[r0:r1 + 1, c0:c1 + 1] - center
    along = local[..., 0] * heading[0] + local[..., 1] * heading[1]
    across = local[..., 1] * heading[0] - local[..., 0] * heading[1]
    out[r0:r1 + 1, c0:c1 + 1] |= (np.abs(along) <= half_along) & (np.abs(across) <= half_across)


def sweep_footprints(grid, path, plan_i, tan_x, tan_y, hits):
    """Stamp footprints densely enough along the path that consecutive images overlap."""
    if not len(path):
        return
    heading, altitude = plan_i["heading"], plan_i["altitude"]
    single = np.zeros(hits.shape, dtype=bool)
    for k in range(len(path)):
        half_along, half_across = footprint_size(max(altitude[k], 1e-3), tan_x, tan_y)
        stamp(grid, path[k], heading[k], half_along, half_across, single)
        if k + 1 >= len(path):
            continue
        length = float(np.linalg.norm(path[k + 1] - path[k]))
        if length > JUMP or length < 1e-9:
            continue
        # Conservative: while moving between samples use the lower of the two altitudes.
        half_along, half_across = footprint_size(max(min(altitude[k], altitude[k + 1]), 1e-3), tan_x, tan_y)
        steps = int(min(math.ceil(length / max(min(2 * half_along, grid.spacing), grid.spacing / 2)), 64))
        for u in np.arange(1, steps) / steps:
            direction = (1 - u) * heading[k] + u * heading[k + 1]
            direction /= max(np.linalg.norm(direction), 1e-9)
            stamp(grid, (1 - u) * path[k] + u * path[k + 1], direction, half_along, half_across, single)
    hits += single

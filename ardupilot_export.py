"""Export swarm coverage plans as ArduPilot (ArduCopter) missions.

Pipeline
--------
1. Read the field polygon from a KML/KMZ file or a Mission Planner .poly file.
2. Project it to a local east/north plane in metres, then scale it into the
   planner's 600 x 600 canvas.
3. Run the planner (sweep.compare) and pick one method (default: harmonic).
4. Convert every drone's path, heading and altitude back to lat/lon/metres.
   Apply the altitude limits and re-verify nadir coverage at the final altitudes.
5. Simplify the paths, add camera-trigger and speed commands, and write one
   Mission Planner ``.waypoints`` file per drone, plus a KML preview and a JSON
   summary.
6. Optionally upload each mission to its vehicle over MAVLink (pymavlink), and
   read it back to verify.

Example
-------
    py ardupilot_export.py field.kml --drones 6 --hfov 147 --vfov 62.22 \
        --max-alt 120 --out missions
    py ardupilot_export.py field.kml --drones 3 --connect udp:127.0.0.1:14550 \
        udp:127.0.0.1:14560 udp:127.0.0.1:14570

Fly ONE path of a multi-drone plan with a single drone (here path 2 of a 4-drone plan):
    py ardupilot_export.py field.kml --drones 4 --only 2 --test --home 77.1930 28.5466

Test mode (one or two drones, small area, low and slow):
    py ardupilot_export.py --test --home 77.1930 28.5466             # 80 m square at home
    py ardupilot_export.py field.kml --test --drones 2 --connect sitl  # clip field to 80 m around centroid
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
import zipfile
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field, replace

import numpy as np

# Fence uploads need MAVLink 2: MAVLink 1 drops `mission_type`, and a fence would overwrite the mission.
os.environ.setdefault("MAVLINK20", "1")

from boundary import validate_polygon
from nadir import JUMP, headings, sweep_footprints
from sweep import Grid, compare

EARTH_RADIUS = 6378137.0
CANVAS_MARGIN = 40.0
CANVAS_SPAN = 600.0 - 2 * CANVAS_MARGIN

# MAVLink command and frame ids (common.xml)
MAV_FRAME_GLOBAL = 0
MAV_FRAME_MISSION = 2
MAV_FRAME_GLOBAL_RELATIVE_ALT = 3
MAV_CMD_NAV_WAYPOINT = 16
MAV_CMD_NAV_RETURN_TO_LAUNCH = 20
MAV_CMD_NAV_TAKEOFF = 22
MAV_CMD_DO_CHANGE_SPEED = 178
MAV_CMD_DO_SET_CAM_TRIGG_DIST = 206
MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION = 5001
FENCE_TYPE_ALT, FENCE_TYPE_CIRCLE, FENCE_TYPE_POLYGON = 1, 2, 4
MISSION_TYPE_MISSION, MISSION_TYPE_FENCE = 0, 1


# --------------------------------------------------------------------------- KML input
def _coordinates(text):
    points = []
    for token in text.replace("\n", " ").split():
        parts = token.split(",")
        if len(parts) >= 2:
            points.append((float(parts[0]), float(parts[1])))
    if len(points) > 1 and points[0] == points[-1]:
        points.pop()
    return points


def read_poly(path):
    """Mission Planner polygon file: optional '#' comments, then one 'lat lon' pair per line."""
    ring = []
    with open(path) as handle:
        for line in handle:
            parts = line.replace(",", " ").split()
            if len(parts) >= 2 and not line.lstrip().startswith("#"):
                ring.append((float(parts[1]), float(parts[0])))
    if len(ring) > 1 and ring[0] == ring[-1]:
        ring.pop()
    if len(ring) < 3:
        raise ValueError("The .poly file needs at least 3 points.")
    return os.path.splitext(os.path.basename(path))[0], ring


def read_kml(path, placemark=None):
    """Return (name, [(lon, lat), ...]) for the outer ring of a polygon in a KML, KMZ or Mission Planner .poly file."""
    if path.lower().endswith(".poly"):
        return read_poly(path)
    if path.lower().endswith(".kmz"):
        with zipfile.ZipFile(path) as archive:
            name = next((n for n in archive.namelist() if n.lower().endswith(".kml")), None)
            if name is None:
                raise ValueError("The KMZ file contains no .kml document.")
            data = archive.read(name)
    else:
        with open(path, "rb") as handle:
            data = handle.read()
    root = ET.fromstring(data)
    strip = lambda tag: tag.rsplit("}", 1)[-1]
    found = []
    for mark in root.iter():
        if strip(mark.tag) != "Placemark":
            continue
        title = next((c.text or "" for c in mark if strip(c.tag) == "name"), "")
        for polygon in (e for e in mark.iter() if strip(e.tag) == "Polygon"):
            outer = next((e for e in polygon.iter() if strip(e.tag) == "outerBoundaryIs"), None)
            coords = next((e for e in (outer if outer is not None else polygon).iter()
                           if strip(e.tag) == "coordinates"), None)
            if coords is not None and coords.text:
                ring = _coordinates(coords.text)
                holes = sum(1 for e in polygon.iter() if strip(e.tag) == "innerBoundaryIs")
                found.append((title.strip(), ring, holes))
    if placemark is not None:
        found = [f for f in found if f[0] == placemark]
    if not found:
        raise ValueError("No polygon found in the KML" + (f" named '{placemark}'." if placemark else "."))
    # Use the largest polygon by projected area.
    def area(ring):
        xy = np.asarray(ring)
        return abs(np.sum(xy[:, 0] * np.roll(xy[:, 1], -1) - np.roll(xy[:, 0], -1) * xy[:, 1]))
    title, ring, holes = max(found, key=lambda f: area(f[1]))
    if holes:
        print(f"warning: polygon '{title}' has {holes} hole(s); holes are ignored by the planner.", file=sys.stderr)
    if len(ring) < 3:
        raise ValueError("The polygon needs at least 3 vertices.")
    return title, ring


# --------------------------------------------------------------------------- projection
@dataclass
class LocalFrame:
    """Equirectangular east/north projection about a reference point (accurate to ~0.1% over a few km)."""

    lat0: float
    lon0: float

    def to_en(self, lonlat):
        lonlat = np.asarray(lonlat, dtype=float)
        east = np.radians(lonlat[:, 0] - self.lon0) * EARTH_RADIUS * math.cos(math.radians(self.lat0))
        north = np.radians(lonlat[:, 1] - self.lat0) * EARTH_RADIUS
        return np.c_[east, north]

    def to_lonlat(self, en):
        en = np.asarray(en, dtype=float)
        lat = self.lat0 + np.degrees(en[:, 1] / EARTH_RADIUS)
        lon = self.lon0 + np.degrees(en[:, 0] / (EARTH_RADIUS * math.cos(math.radians(self.lat0))))
        return np.c_[lon, lat]


@dataclass
class CanvasMap:
    """Uniform scale + offset between local metres and planner canvas units."""

    scale: float  # metres per canvas unit
    origin: np.ndarray  # metres at canvas (CANVAS_MARGIN, CANVAS_MARGIN)
    offset: np.ndarray  # canvas offset centring the field

    @classmethod
    def fit(cls, en):
        low, high = en.min(axis=0), en.max(axis=0)
        scale = max(float(np.max(high - low)) / CANVAS_SPAN, 1e-6)
        offset = (CANVAS_SPAN - (high - low) / scale) / 2 + CANVAS_MARGIN
        return cls(scale, low, offset)

    def to_canvas(self, en):
        return (np.asarray(en) - self.origin) / self.scale + self.offset

    def to_metres(self, xy):
        return (np.asarray(xy) - self.offset) * self.scale + self.origin


def clip_to_square(ring_en, centre, half):
    """Sutherland-Hodgman clip of a polygon (east/north metres) to an axis-aligned square."""
    out = [tuple(p) for p in ring_en]
    for axis, sign in ((0, 1), (0, -1), (1, 1), (1, -1)):
        limit = centre[axis] + sign * half
        inside = lambda q: sign * (q[axis] - limit) <= 0
        src, out = out, []
        for k, current in enumerate(src):
            previous = src[k - 1]
            if inside(current) != inside(previous):
                t = (limit - previous[axis]) / (current[axis] - previous[axis])
                out.append(tuple(previous[j] + t * (current[j] - previous[j]) for j in range(2)))
            if inside(current):
                out.append(current)
        if not out:
            break
    return np.asarray(out, dtype=float).reshape(-1, 2)


def simplify_ring(points, limit=300):
    """Douglas-Peucker on a closed ring until it has at most `limit` vertices."""
    points = np.asarray(points, dtype=float)
    if len(points) <= limit:
        return points
    tolerance = 0.05
    while True:
        keep = _rdp(np.vstack((points, points[:1])), tolerance)[:-1]
        if len(keep) <= limit:
            return points[keep]
        tolerance *= 1.6


def _rdp(points, tolerance):
    keep = np.zeros(len(points), dtype=bool)
    keep[[0, -1]] = True
    stack = [(0, len(points) - 1)]
    while stack:
        a, b = stack.pop()
        if b <= a + 1:
            continue
        start, end = points[a], points[b]
        segment = end - start
        length = np.linalg.norm(segment)
        middle = points[a + 1:b]
        if length < 1e-12:
            distance = np.linalg.norm(middle - start, axis=1)
        else:
            distance = np.abs(segment[0] * (middle[:, 1] - start[1]) - segment[1] * (middle[:, 0] - start[0])) / length
        i = int(np.argmax(distance))
        if distance[i] > tolerance:
            keep[a + 1 + i] = True
            stack.extend(((a, a + 1 + i), (a + 1 + i, b)))
    return np.flatnonzero(keep)


# --------------------------------------------------------------------------- mission model
@dataclass
class MissionItem:
    command: int
    frame: int = MAV_FRAME_GLOBAL_RELATIVE_ALT
    params: tuple = (0., 0., 0., 0.)
    lat: float = 0.
    lon: float = 0.
    alt: float = 0.
    current: int = 0
    autocontinue: int = 1


@dataclass
class DroneMission:
    index: int
    items: list = field(default_factory=list)
    waypoints: np.ndarray = None  # lon, lat, alt (relative) of flown waypoints
    length_m: float = 0.
    max_alt: float = 0.
    min_alt: float = 0.
    transits: int = 0
    photos: int = 0


def _segments(path, jump):
    """Split a path into continuous runs separated by transit jumps."""
    if len(path) == 0:
        return []
    breaks = np.flatnonzero(np.linalg.norm(np.diff(path, axis=0), axis=1) > jump) + 1
    return [r for r in np.split(np.arange(len(path)), breaks) if len(r)]


def _simplify_run(en, alt, tolerance, alt_tolerance):
    """Keep path vertices needed to stay within `tolerance` metres and `alt_tolerance` metres of altitude."""
    if len(en) <= 2:
        return np.arange(len(en))
    scaled = np.c_[en, alt * (tolerance / max(alt_tolerance, 1e-9))]
    keep = np.zeros(len(en), dtype=bool)
    keep[[0, -1]] = True
    stack = [(0, len(en) - 1)]
    while stack:
        a, b = stack.pop()
        if b <= a + 1:
            continue
        u = np.linspace(0, 1, b - a + 1)[1:-1, None]
        chord = scaled[a] + u * (scaled[b] - scaled[a])
        distance = np.linalg.norm(scaled[a + 1:b] - chord, axis=1)
        i = int(np.argmax(distance))
        if distance[i] > tolerance:
            keep[a + 1 + i] = True
            stack.extend(((a, a + 1 + i), (a + 1 + i, b)))
    return np.flatnonzero(keep)


def build_mission(index, lonlat, alt, seconds, args, home):
    """Turn one drone's sampled path into ArduCopter mission items."""
    frame = LocalFrame(home[1], home[0])
    en = frame.to_en(lonlat)
    tan_y = math.tan(math.radians(args.vfov) / 2)
    mission = DroneMission(index)
    items = mission.items
    items.append(MissionItem(MAV_CMD_NAV_WAYPOINT, MAV_FRAME_GLOBAL, lat=home[1], lon=home[0], alt=0., current=1))
    runs = _segments(en, args.jump_m)
    if not runs:
        return mission
    items.append(MissionItem(MAV_CMD_NAV_TAKEOFF, params=(0., 0., 0., float("nan")), alt=float(alt[runs[0][0]])))
    flown, last_speed, last_trigger = [], None, None

    def set_speed(speed):
        nonlocal last_speed
        speed = float(np.clip(speed, args.min_speed, args.speed))
        if last_speed is None or abs(speed - last_speed) > .15 * last_speed:
            items.append(MissionItem(MAV_CMD_DO_CHANGE_SPEED, MAV_FRAME_MISSION, params=(1., round(speed, 2), -1., 0.)))
            last_speed = speed

    def set_trigger(distance):
        nonlocal last_trigger
        distance = round(float(distance), 1)
        if last_trigger is None or abs(distance - last_trigger) > max(.1 * last_trigger, .05):
            items.append(MissionItem(MAV_CMD_DO_SET_CAM_TRIGG_DIST, MAV_FRAME_MISSION, params=(distance, 0., 1., 0.)))
            last_trigger = distance

    def waypoint(i, altitude):
        lon, lat = lonlat[i]
        items.append(MissionItem(MAV_CMD_NAV_WAYPOINT, params=(0., args.accept_radius, 0., float("nan")),
                                 lat=float(lat), lon=float(lon), alt=round(float(altitude), 2)))
        flown.append((lon, lat, altitude))

    for r, run in enumerate(runs):
        keep = _simplify_run(en[run], alt[run], args.path_tolerance, args.alt_tolerance)
        if r:
            set_trigger(0.)
            mission.transits += 1
        set_speed(args.speed)
        waypoint(run[keep[0]], alt[run[keep[0]]])
        for a, b in zip(keep[:-1], keep[1:]):
            span = run[a:b + 1]
            # Never fly lower than any dropped sample needed.
            segment_alt = float(np.max(alt[span]))
            if len(flown):
                flown[-1] = (flown[-1][0], flown[-1][1], max(flown[-1][2], segment_alt))
                items[-1].alt = round(max(items[-1].alt, segment_alt), 2)
            length = float(np.sum(np.linalg.norm(np.diff(en[span], axis=0), axis=1)))
            duration = float(np.sum(seconds[span[:-1]])) if args.sync else 0.
            if args.sync and duration > 0:
                set_speed(length / duration)
            footprint_along = 2 * segment_alt * tan_y
            set_trigger(max(footprint_along * (1 - args.front_overlap), .5))
            mission.photos += int(length / max(last_trigger, .5)) + 1
            waypoint(run[b], segment_alt)
    set_trigger(0.)
    items.append(MissionItem(MAV_CMD_NAV_RETURN_TO_LAUNCH, MAV_FRAME_MISSION))
    mission.waypoints = np.asarray(flown)
    mission.length_m = float(np.sum(np.linalg.norm(np.diff(frame.to_en(mission.waypoints[:, :2]), axis=0), axis=1)))
    mission.max_alt = float(mission.waypoints[:, 2].max())
    mission.min_alt = float(mission.waypoints[:, 2].min())
    return mission


def write_waypoints(mission, path):
    """Mission Planner / QGC 'QGC WPL 110' text format."""
    with open(path, "w", newline="\n") as handle:
        handle.write("QGC WPL 110\n")
        for seq, item in enumerate(mission.items):
            p = [0. if (isinstance(v, float) and math.isnan(v)) else v for v in item.params]
            handle.write("\t".join(str(v) for v in (
                seq, item.current, item.frame, item.command, f"{p[0]:.8f}", f"{p[1]:.8f}", f"{p[2]:.8f}",
                f"{p[3]:.8f}", f"{item.lat:.8f}", f"{item.lon:.8f}", f"{item.alt:.6f}", item.autocontinue)) + "\n")


def read_waypoints(path):
    items = []
    with open(path) as handle:
        if not handle.readline().startswith("QGC WPL"):
            raise ValueError("Not a QGC WPL waypoint file.")
        for line in handle:
            f = line.split("\t")
            if len(f) == 12:
                items.append(MissionItem(int(f[3]), int(f[2]), tuple(float(v) for v in f[4:8]),
                                         float(f[8]), float(f[9]), float(f[10]), int(f[1]), int(f[11])))
    return items


FENCE_MARGIN = 2


def boundary_fence(ring_lonlat, missions, home, buffer, max_vertices=60):
    """Inclusion polygon = field grown by `buffer` m, plus home and the legs home->first and last->home.

    Leaving it (touching the field boundary + buffer) breaches the fence and triggers RTL. The take-off,
    the transit to the first waypoint and the mission-end RTL leg stay inside, so they never trip it.
    Returns (lon/lat vertices, metrics).
    """
    from shapely.geometry import LineString, Point, Polygon
    from shapely.ops import unary_union

    frame = LocalFrame(home[1], home[0])
    field_poly = Polygon(frame.to_en(ring_lonlat)).buffer(0)
    parts = [field_poly.buffer(buffer, join_style=2, mitre_limit=2.), Point(0, 0).buffer(buffer)]
    clearance = float("inf")
    for m in missions:
        if m.waypoints is None or not len(m.waypoints):
            continue
        en = frame.to_en(m.waypoints[:, :2])
        parts.append(LineString([(0, 0), tuple(en[0])]).buffer(buffer))
        parts.append(LineString([tuple(en[-1]), (0, 0)]).buffer(buffer))
        clearance = min(clearance, min(field_poly.exterior.distance(Point(*q)) if field_poly.contains(Point(*q))
                                       else -field_poly.exterior.distance(Point(*q)) for q in en))
    shape = unary_union(parts)
    if shape.geom_type != "Polygon":
        shape = max(shape.geoms, key=lambda g: g.area)
    shape = Polygon(shape.exterior)  # inclusion fences cannot have holes
    # Grow slightly before simplifying so simplification never moves the fence inward of the buffer.
    grown, tolerance = shape.buffer(.5, join_style=2), .3
    simple = grown.simplify(tolerance, preserve_topology=True)
    while len(simple.exterior.coords) - 1 > max_vertices:
        tolerance *= 1.5
        simple = grown.buffer(tolerance, join_style=2).simplify(tolerance, preserve_topology=True)
    coords = np.asarray(simple.exterior.coords)[:-1]
    if Polygon(coords).exterior.is_ccw:
        coords = coords[::-1]
    return frame.to_lonlat(coords), {"bufferM": buffer, "vertices": int(len(coords)),
                                     "pathClearanceM": None if not math.isfinite(clearance) else round(clearance, 1),
                                     "homeInsideField": bool(field_poly.buffer(buffer).contains(Point(0, 0)))}


def fence_items(vertices_lonlat):
    n = len(vertices_lonlat)
    return [MissionItem(MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION, MAV_FRAME_GLOBAL, (float(n), 0., 0., 0.),
                        float(lat), float(lon), 0.) for lon, lat in vertices_lonlat]


def write_fence_files(vertices_lonlat, stem_path):
    """Fence as a QGC WPL file (fence vertex commands) and as a Mission Planner .poly (lat lon per line)."""
    items = fence_items(vertices_lonlat)
    with open(stem_path + "_fence.waypoints", "w", newline="\n") as handle:
        handle.write("QGC WPL 110\n")
        for seq, item in enumerate(items):
            handle.write("\t".join(str(v) for v in (
                seq, 0, item.frame, item.command, f"{item.params[0]:.8f}", "0.00000000", "0.00000000", "0.00000000",
                f"{item.lat:.8f}", f"{item.lon:.8f}", "0.000000", 1)) + "\n")
    with open(stem_path + "_fence.poly", "w", newline="\n") as handle:
        handle.write("#saved by ardupilot_export.py (inclusion fence)\n")
        for lon, lat in list(vertices_lonlat) + [vertices_lonlat[0]]:
            handle.write(f"{lat:.8f} {lon:.8f}\n")


def safety_params(missions, home, args):
    """Suggested per-drone ArduCopter parameters for the return leg and a geofence around home.

    Every drone gets its own RTL altitude (above the highest survey altitude of the whole swarm, plus
    the per-drone stagger) so that drones returning together do not meet at the same height.
    """
    frame = LocalFrame(home[1], home[0])
    top = max((m.max_alt for m in missions if m.waypoints is not None and len(m.waypoints)), default=args.max_alt)
    reach = 0.
    for m in missions:
        if m.waypoints is not None and len(m.waypoints):
            reach = max(reach, float(np.max(np.linalg.norm(frame.to_en(m.waypoints[:, :2]), axis=1))))
    radius = max(30., math.ceil((reach + args.fence_margin) / 10) * 10)
    rtls = [math.ceil(top + args.rtl_margin + m.index * args.alt_stagger) if args.rtl_alt is None
            else args.rtl_alt + m.index * args.alt_stagger for m in missions]
    if args.rtl_height is not None:
        # Return layers must sit between the survey top and the ceiling (minus the fence margin),
        # or the return itself would breach the ceiling and re-trigger RTL.
        ceiling = args.rtl_height - FENCE_MARGIN - 1
        if max(rtls) > ceiling:
            spacing = min(args.alt_stagger, (ceiling - top) / len(rtls))
            if spacing < 1:
                raise SystemExit(f"--RTLheight {args.rtl_height:.0f} m leaves no room for {len(rtls)} return "
                                 f"layer(s) above the survey (top {top:.0f} m). Raise --RTLheight or lower --max-alt.")
            rtls = [math.floor(top + spacing * (k + 1)) for k in range(len(rtls))]
            print(f"note: RTL altitudes set to {', '.join(f'{r:.0f}' for r in rtls)} m "
                  f"to fit under --RTLheight {args.rtl_height:.0f} m")
    sets = []
    for m, rtl in zip(missions, rtls):
        fence_alt = args.rtl_height if args.rtl_height is not None else math.ceil(rtl + 10)
        sets.append({
            "RTL_ALT": int(round(rtl * 100)),            # cm
            "RTL_ALT_FINAL": 0,                          # land at home
            "FENCE_ENABLE": 1,
            "FENCE_TYPE": FENCE_TYPE_ALT | FENCE_TYPE_CIRCLE | (FENCE_TYPE_POLYGON if args.boundary_fence else 0),
            "FENCE_ACTION": 1,                           # RTL (or land if RTL is impossible)
            "FENCE_ALT_MAX": int(math.floor(fence_alt)) if args.rtl_height is not None else int(fence_alt),
            "FENCE_RADIUS": int(radius),                 # m from home
            "FENCE_MARGIN": FENCE_MARGIN,                # m
        })
    return sets


def write_params(params, path, index, home_given):
    with open(path, "w", newline="\n") as handle:
        handle.write(f"# Suggested safety parameters for drone {index + 1} (generated by ardupilot_export.py)\n")
        handle.write("# Mission Planner: CONFIG > Full Parameter List > Load from file > Write Params\n")
        handle.write(f"# Give each drone a unique SYSID_THISMAV (suggested: {index + 1}).\n")
        if not home_given:
            handle.write("# FENCE_RADIUS assumes take-off near the field centre; re-run with --home LON LAT otherwise.\n")
        for name, value in params.items():
            handle.write(f"{name},{value}\n")


def _append_fence_to_preview(path, vertices_lonlat):
    with open(path, encoding="utf-8") as handle:
        text = handle.read()
    coords = " ".join(f"{lon:.8f},{lat:.8f},0" for lon, lat in list(vertices_lonlat) + [vertices_lonlat[0]])
    mark = (f'<Placemark><name>Boundary fence (leave = RTL)</name><Style><LineStyle><color>ff0000ff</color>'
            f'<width>3</width></LineStyle><PolyStyle><fill>0</fill></PolyStyle></Style><Polygon><outerBoundaryIs>'
            f'<LinearRing><coordinates>{coords}</coordinates></LinearRing></outerBoundaryIs></Polygon></Placemark>')
    with open(path, "w", encoding="utf-8") as handle:
        handle.write(text.replace("</Document></kml>", mark + "</Document></kml>"))


def write_preview_kml(boundary_lonlat, missions, path, title, lanes=()):
    colours = ["ff00ffcc", "ff3399ff", "ffff66cc", "ff66ff33", "ff3333ff", "ffffcc00", "ffcc66ff", "ff00ccff"]
    out = ['<?xml version="1.0" encoding="UTF-8"?>', '<kml xmlns="http://www.opengis.net/kml/2.2"><Document>',
           f"<name>{title} swarm plan</name>"]
    ring = " ".join(f"{lon:.8f},{lat:.8f},0" for lon, lat in list(boundary_lonlat) + [boundary_lonlat[0]])
    out.append(f'<Placemark><name>Field</name><Style><LineStyle><color>ffffffff</color><width>2</width></LineStyle>'
               f'<PolyStyle><color>2200ff00</color></PolyStyle></Style><Polygon><outerBoundaryIs><LinearRing>'
               f'<coordinates>{ring}</coordinates></LinearRing></outerBoundaryIs></Polygon></Placemark>')
    for ring in lanes:
        coords = " ".join(f"{lon:.8f},{lat:.8f},0" for lon, lat in list(ring) + [ring[0]])
        out.append(f'<Placemark><name>Area covered by this drone</name><Style><LineStyle><color>ff00ffff</color>'
                   f'<width>2</width></LineStyle><PolyStyle><color>5500ffff</color></PolyStyle></Style><Polygon>'
                   f'<outerBoundaryIs><LinearRing><coordinates>{coords}</coordinates></LinearRing></outerBoundaryIs>'
                   f'</Polygon></Placemark>')
    for m in missions:
        if m.waypoints is None or not len(m.waypoints):
            continue
        coords = " ".join(f"{lon:.8f},{lat:.8f},{alt:.2f}" for lon, lat, alt in m.waypoints)
        out.append(f'<Placemark><name>Drone {m.index + 1} (max {m.max_alt:.0f} m)</name><Style><LineStyle>'
                   f'<color>{colours[m.index % len(colours)]}</color><width>3</width></LineStyle></Style>'
                   f'<LineString><altitudeMode>relativeToGround</altitudeMode><coordinates>{coords}</coordinates>'
                   f'</LineString></Placemark>')
    out.append("</Document></kml>")
    with open(path, "w", encoding="utf-8") as handle:
        handle.write("\n".join(out))


# --------------------------------------------------------------------------- planning
def lane_mask(method, index, size):
    """Grid cells that belong to drone `index` (the planner stores lanes at half resolution)."""
    lanes = method["lanes"]
    image = np.asarray(lanes["values"]).reshape(lanes["height"], lanes["width"])
    full = np.repeat(np.repeat(image, 2, axis=0), 2, axis=1)[:size, :size]
    return full == index


def lane_outlines(mask, grid, canvas, frame):
    """Lon/lat outlines of a lane mask, for the KML preview."""
    from skimage.measure import find_contours
    rings = []
    for contour in find_contours(np.pad(mask, 1).astype(float), .5):
        xy = (contour[:, ::-1] - 1) * grid.spacing
        if len(xy) > 3:
            rings.append(frame.to_lonlat(canvas.to_metres(xy)))
    return rings


def coverage_at(polygon_canvas, paths, headings_deg, altitudes, hfov, vfov, size, region=None):
    """Fraction of the field (or of `region` cells) imaged at the given altitudes, in canvas units."""
    grid = Grid(polygon_canvas, size)
    hits = np.zeros(grid.mask.shape, dtype=np.int32)
    tan_x, tan_y = math.tan(math.radians(hfov) / 2), math.tan(math.radians(vfov) / 2)
    for path, heading, alt in zip(paths, headings_deg, altitudes):
        if not len(path):
            continue
        h = np.radians(heading)
        sweep_footprints(grid, path, {"heading": np.c_[np.cos(h), np.sin(h)], "altitude": alt}, tan_x, tan_y, hits)
    target = grid.mask if region is None else grid.mask & region
    return np.count_nonzero(target & (hits > 0)) / max(np.count_nonzero(target), 1)


def plan(args, log=print):
    if args.kml:
        title, ring = read_kml(args.kml, args.placemark)
        ring = np.asarray(ring)
    else:
        # Test mode without a field file: a square of --test-size metres centred on home.
        lon, lat = args.home
        f0 = LocalFrame(lat, lon)
        half = (args.test_size or 80.) / 2
        title, ring = "Test square", f0.to_lonlat(np.array([[-half, -half], [half, -half], [half, half], [-half, half]]))
    centre = ring.mean(axis=0)
    frame = LocalFrame(float(centre[1]), float(centre[0]))
    en = frame.to_en(ring)
    if args.kml and args.test_size:
        anchor = frame.to_en(np.array([args.home]))[0] if args.home else en.mean(axis=0)
        clipped = clip_to_square(en, anchor, args.test_size / 2)
        if len(clipped) < 3 or abs(np.sum(clipped[:, 0] * np.roll(clipped[:, 1], -1)
                                          - np.roll(clipped[:, 0], -1) * clipped[:, 1])) / 2 < 100:
            raise SystemExit(f"Test area: the {args.test_size:.0f} m square around "
                             f"{'--home' if args.home else 'the field centroid'} does not overlap the field.")
        log(f"TEST: clipping the field to a {args.test_size:.0f} m square around "
            f"{'home' if args.home else 'the field centroid'}")
        ring = frame.to_lonlat(clipped)
        centre = ring.mean(axis=0)
        frame = LocalFrame(float(centre[1]), float(centre[0]))
        en = frame.to_en(ring)
    canvas = CanvasMap.fit(en)
    polygon = simplify_ring(canvas.to_canvas(en))
    polygon = validate_polygon(polygon)
    area_m2 = abs(np.sum(en[:, 0] * np.roll(en[:, 1], -1) - np.roll(en[:, 0], -1) * en[:, 1])) / 2
    if args.jump_m is None:
        # The planner's transit threshold is in canvas units.
        args.jump_m = JUMP * canvas.scale
    log(f"Field '{title or os.path.basename(args.kml or 'test')}': {len(ring)} vertices, {area_m2 / 1e4:.2f} ha, "
        f"{canvas.scale:.3f} m per canvas unit")

    drones = args.drones
    while True:
        log(f"Planning {drones} drones with the {args.planner} planner ...")
        result = compare(polygon.tolist(), drones=drones, size=args.grid, hfov=args.hfov, vfov=args.vfov,
                         overlap=args.overlap)
        method = result["methods"][args.planner]
        needed = method["stats"]["maxAltitude"] * canvas.scale
        if needed <= args.max_alt or not args.auto_drones or drones >= 64:
            break
        drones = min(64, max(drones + 1, int(math.ceil(drones * needed / args.max_alt))))
        log(f"  needs {needed:.0f} m > {args.max_alt:.0f} m; retrying with {drones} drones")

    paths = [np.asarray(d["path"], dtype=float).reshape(-1, 2) for d in method["drones"]]
    heads = [np.asarray(d["heading"], dtype=float) for d in method["drones"]]
    alts_canvas = [np.asarray(d["altitude"], dtype=float) for d in method["drones"]]
    # Each drone gets its own altitude band (+i * stagger) so converging drones stay vertically separated.
    # Raising altitude only enlarges footprints, so this never reduces coverage.
    alts_m = [np.clip(a * canvas.scale, args.min_alt, args.max_alt) for a in alts_canvas]
    if not args.only:
        alts_m = [np.minimum(a + i * args.alt_stagger, args.max_alt) for i, a in enumerate(alts_m)]
    capped = any(np.any(a * canvas.scale > args.max_alt) for a in alts_canvas)
    coverage = coverage_at(polygon, paths, heads, [a / canvas.scale for a in alts_m], args.hfov, args.vfov, args.grid)
    if capped and coverage < 1:
        hint = "use 2 drones or a smaller --test-size" if args.test else "use more drones or --auto-drones"
        log(f"warning: the plan needs up to {needed:.0f} m but --max-alt is {args.max_alt:.0f} m. "
            f"Coverage at capped altitudes: {100 * coverage:.1f}%; {hint}.")
    elif capped:
        log(f"note: altitudes capped at {args.max_alt:.0f} m (planned up to {needed:.0f} m); "
            f"coverage re-checked at the capped altitudes is still 100%.")

    # Frame timing for synchronised speeds: every drone reaches front k at the same moment.
    en_paths = [canvas.to_metres(p) for p in paths]
    steps = [np.r_[np.linalg.norm(np.diff(p, axis=0), axis=1), 0.] if len(p) else np.zeros(0) for p in en_paths]
    frames = max((len(s) for s in steps), default=0)
    seconds = np.zeros(frames)
    for s in steps:
        usable = np.where(s > args.jump_m, 0., s)
        seconds[:len(s)] = np.maximum(seconds[:len(s)], usable / args.speed)

    lane_coverage, lane_rings = None, []
    if args.only:
        # One drone flies path N alone: it must image its own lane completely; other lanes are not its job.
        k = args.only - 1
        region = lane_mask(method, k, args.grid)
        lane_coverage = coverage_at(polygon, [paths[k]], [heads[k]], [alts_m[k] / canvas.scale], args.hfov, args.vfov,
                                    args.grid, region=region)
        lane_rings = lane_outlines(region & Grid(polygon, args.grid).mask, Grid(polygon, args.grid), canvas, frame)
        lane_area = np.count_nonzero(region & Grid(polygon, args.grid).mask) * (Grid(polygon, args.grid).spacing
                                                                              * canvas.scale) ** 2
        log(f"Path {args.only}: own area {lane_area / 1e4:.2f} ha "
            f"({100 * lane_area / max(area_m2, 1e-9):.0f}% of the field); "
            f"imaged by this drone alone: {100 * lane_coverage:.1f}%")
        if lane_coverage < 1:
            log(f"warning: path {args.only} does not fully image its own area at <= {args.max_alt:.0f} m; "
                f"raise --max-alt or split the field into more paths (--drones).")

    home = (float(centre[0]), float(centre[1])) if args.home is None else tuple(args.home)
    missions = []
    for i, (p, a) in enumerate(zip(en_paths, alts_m)):
        lonlat = frame.to_lonlat(p) if len(p) else np.zeros((0, 2))
        missions.append(build_mission(i, lonlat, a, seconds[:len(p)], args, home))
    return {"title": title, "ring": ring, "frame": frame, "canvas": canvas, "result": result, "method": method,
            "missions": missions, "coverage": coverage, "capped": capped, "needed_alt": needed, "drones": drones,
            "area_m2": area_m2, "seconds": float(np.sum(seconds)), "home": home,
            "separation": min_separation(en_paths, alts_m), "lane_coverage": lane_coverage, "lane_rings": lane_rings}


def min_separation(en_paths, alts_m):
    """Smallest 3D distance between any two drones at the same formation frame (the drones fly in sync)."""
    frames = min((len(p) for p in en_paths if len(p)), default=0)
    if frames == 0 or len(en_paths) < 2:
        return float("inf")
    points = np.stack([np.c_[p[:frames], a[:frames]] for p, a in zip(en_paths, alts_m)])
    best = float("inf")
    for i in range(len(points)):
        for j in range(i + 1, len(points)):
            best = min(best, float(np.min(np.linalg.norm(points[i] - points[j], axis=1))))
    return best


# --------------------------------------------------------------------------- MAVLink upload
def _connect(connection, timeout):
    from pymavlink import mavutil

    link = mavutil.mavlink_connection(connection, source_system=255, source_component=190, autoreconnect=True)
    deadline, heartbeat = time.time() + timeout, None
    while heartbeat is None and time.time() < deadline:
        link.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        heartbeat = link.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if heartbeat is not None and heartbeat.get_srcComponent() == 190:
            heartbeat = None
    if heartbeat is None:
        raise TimeoutError(f"No vehicle heartbeat on {connection}.")
    return link, heartbeat.get_srcSystem(), heartbeat.get_srcComponent()


def _send_items(link, system, component, items, mission_type, timeout, verify, label):
    """MAVLink mission protocol (works for missions and fences), with optional item-by-item read-back."""
    from pymavlink import mavutil

    link.mav.mission_count_send(system, component, len(items), mission_type)
    sent = set()
    while True:
        message = link.recv_match(type=["MISSION_REQUEST_INT", "MISSION_REQUEST", "MISSION_ACK"], blocking=True,
                                  timeout=timeout)
        if message is None:
            raise TimeoutError(f"Vehicle {system} stopped requesting {label} items ({len(sent)}/{len(items)} sent).")
        if getattr(message, "mission_type", mission_type) != mission_type:
            continue
        if message.get_type() == "MISSION_ACK":
            if message.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
                raise RuntimeError(f"Vehicle {system} rejected the {label} (MAV_MISSION_RESULT {message.type}).")
            break
        item = items[message.seq]
        p = [0. if math.isnan(v) else v for v in item.params] if message.get_type() == "MISSION_REQUEST" else item.params
        link.mav.mission_item_int_send(system, component, message.seq, item.frame, item.command, item.current,
                                       item.autocontinue, *[float(v) for v in p], int(round(item.lat * 1e7)),
                                       int(round(item.lon * 1e7)), float(item.alt), mission_type)
        sent.add(message.seq)
    if verify:
        link.mav.mission_request_list_send(system, component, mission_type)
        count = link.recv_match(type="MISSION_COUNT", blocking=True, timeout=timeout)
        if count is None or count.count != len(items):
            raise RuntimeError(f"Read-back count mismatch for the {label} on system {system}.")
        first = 1 if mission_type == MISSION_TYPE_MISSION else 0  # mission item 0 is home, rewritten by the autopilot
        for seq in range(first, count.count):
            link.mav.mission_request_int_send(system, component, seq, mission_type)
            got = link.recv_match(type="MISSION_ITEM_INT", blocking=True, timeout=timeout)
            want = items[seq]
            if (got is None or got.command != want.command or abs(got.x - round(want.lat * 1e7)) > 1
                    or abs(got.y - round(want.lon * 1e7)) > 1 or abs(got.z - want.alt) > .01):
                raise RuntimeError(f"Read-back mismatch at {label} item {seq} on system {system}.")
        link.mav.mission_ack_send(system, component, mavutil.mavlink.MAV_MISSION_ACCEPTED, mission_type)


def upload(connection, items, timeout=10., verify=True, log=print, params=None, fence=None):
    """Upload the mission, then the inclusion fence, then `params` (each confirmed). Everything is verified."""
    from pymavlink import mavutil

    link, system, component = _connect(connection, timeout)
    _send_items(link, system, component, items, MISSION_TYPE_MISSION, timeout, verify, "mission")
    log(f"  uploaded {len(items)} mission items to system {system} on {connection}"
        f"{' (verified read-back)' if verify else ''}")
    if fence:
        if not link.mavlink20():
            link.close()
            raise RuntimeError("Link is not MAVLink 2; refusing to upload the fence (it would replace the mission). "
                               "Set SERIALn_PROTOCOL=2 on the drone's telemetry port.")
        _send_items(link, system, component, fence, MISSION_TYPE_FENCE, timeout, verify, "fence")
        log(f"  uploaded {len(fence)}-vertex boundary fence to system {system}{' (verified read-back)' if verify else ''}")
    if params:
        for name, value in params.items():
            for attempt in range(3):
                link.mav.param_set_send(system, component, name.encode(), float(value),
                                        mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
                reply = _wait_param(link, name, timeout / 3)
                if reply is not None and abs(reply.param_value - float(value)) <= 1e-3 * max(1., abs(value)):
                    break
            else:
                link.close()
                raise RuntimeError(f"Vehicle {system} did not confirm {name}={value}. Set it in Mission Planner.")
        log(f"  set {len(params)} safety parameters on system {system} "
            f"(RTL_ALT {params['RTL_ALT'] / 100:.0f} m, FENCE_TYPE {params['FENCE_TYPE']}, "
            f"ceiling {params['FENCE_ALT_MAX']} m, radius {params['FENCE_RADIUS']} m, breach -> RTL)")
    link.close()
    return system


def _wait_param(link, name, timeout):
    end = time.time() + timeout
    while time.time() < end:
        message = link.recv_match(type="PARAM_VALUE", blocking=True, timeout=max(end - time.time(), .05))
        if message is not None and str(message.param_id).rstrip("\x00") == name:
            return message
    return None


# --------------------------------------------------------------------------- CLI
NORMAL_DEFAULTS = {"drones": 4, "max_alt": 120., "min_alt": 15., "speed": 8., "alt_stagger": 5., "min_sep": 5.,
                   "out": "missions"}
# Conservative settings for first flights: one drone, low, slow, small area, wide vertical separation.
TEST_DEFAULTS = {"drones": 1, "max_alt": 30., "min_alt": 10., "speed": 3., "alt_stagger": 8., "min_sep": 8.,
                 "out": "missions_test"}
SITL_PORTS = (5760, 5770, 5780, 5790)


def parse(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("kml", nargs="?", help="Field boundary: .kml/.kmz (largest polygon's outer ring) or Mission "
                   "Planner .poly. Optional with --test --home (an 80 m square at home is used)")
    p.add_argument("--placemark", help="Use the polygon placemark with this exact name")
    p.add_argument("--drones", type=int, default=None, help="Number of drones (1-64; default 4, or 1 with --test)")
    p.add_argument("--planner", default="harmonic", choices=["harmonic", "lawnmower", "geodesic", "mat"])
    p.add_argument("--hfov", type=float, default=147., help="Camera across-track field of view, degrees")
    p.add_argument("--vfov", type=float, default=62.22, help="Camera along-track field of view, degrees")
    p.add_argument("--overlap", type=float, default=.1, help="Side overlap margin on altitude (0-1)")
    p.add_argument("--front-overlap", type=float, default=.3, help="Forward overlap between photos (0-0.95)")
    p.add_argument("--max-alt", type=float, default=None, help="Altitude cap, m above home (default 120; test 30)")
    p.add_argument("--min-alt", type=float, default=None, help="Altitude floor, m above home (default 15; test 10)")
    p.add_argument("--alt-stagger", type=float, default=None,
                   help="Extra altitude per drone index, m, for separation (default 5; test 8)")
    p.add_argument("--min-sep", type=float, default=None, help="Warn below this 3D spacing, m (default 5; test 8)")
    p.add_argument("--auto-drones", action="store_true", help="Add drones until the plan fits under --max-alt")
    p.add_argument("--speed", type=float, default=None, help="Max ground speed, m/s (default 8; test 3)")
    p.add_argument("--min-speed", type=float, default=1.5, help="Min commanded ground speed, m/s")
    p.add_argument("--no-sync", dest="sync", action="store_false",
                   help="Fly every drone at --speed instead of keeping the formation line in sync")
    p.add_argument("--accept-radius", type=float, default=2., help="Waypoint acceptance radius, metres")
    p.add_argument("--path-tolerance", type=float, default=1., help="Path simplification tolerance, metres")
    p.add_argument("--alt-tolerance", type=float, default=1., help="Altitude simplification tolerance, metres")
    p.add_argument("--jump-m", type=float, default=None,
                   help="Gap treated as a transit (camera off). Default: the planner's jump threshold")
    p.add_argument("--grid", type=int, default=200, help="Planner grid resolution (coverage is verified on it)")
    p.add_argument("--home", type=float, nargs=2, metavar=("LON", "LAT"),
                   help="Launch point (item 0, fence centre, test square). Default: field centroid")
    p.add_argument("--rtl-alt", type=float, default=None,
                   help="RTL altitude for drone 1 in metres (others add --alt-stagger). "
                        "Default: highest survey altitude + --rtl-margin")
    p.add_argument("--RTLheight", "--rtl-height", dest="rtl_height", type=float, default=None, metavar="M",
                   help="Failsafe ceiling: a drone that climbs above M metres over home returns home (RTL). "
                        "Sets FENCE_ALT_MAX=M with FENCE_ACTION=RTL; survey and RTL altitudes are kept below M")
    p.add_argument("--no-boundary-fence", dest="boundary_fence", action="store_false",
                   help="Do not add the inclusion fence that triggers RTL when a drone leaves the field")
    p.add_argument("--fence-buffer", type=float, default=5., metavar="M",
                   help="Metres the boundary fence sits outside the field edge (GPS error + waypoint radius). "
                        "Default 5")
    p.add_argument("--no-params", dest="push_params", action="store_false",
                   help="With --connect, upload only the mission, not the RTL/fence safety parameters")
    p.add_argument("--rtl-margin", type=float, default=5., help="Metres above the highest survey altitude for RTL")
    p.add_argument("--fence-margin", type=float, default=30., help="Metres added to the farthest waypoint for the fence")
    p.add_argument("--out", default=None, help="Output folder (default missions; test missions_test)")
    p.add_argument("--connect", nargs="+", metavar="URL",
                   help="MAVLink connection per drone, in drone order (e.g. tcp:127.0.0.1:5760 COM5,57600). "
                        "'sitl' expands to tcp:127.0.0.1:5760, :5770, ... for each drone")
    p.add_argument("--timeout", type=float, default=10.)
    test = p.add_argument_group("test mode")
    test.add_argument("--test", action="store_true",
                      help="First-flight mode on the real field: 1-2 drones, 30 m cap, 3 m/s, 8 m separation, "
                           "files prefixed TEST_, confirmation before upload")
    test.add_argument("--test-size", type=float, default=None, metavar="M",
                      help="Optional: fly only an M-metre square of the field around --home (or the centroid). "
                           "Without a field file, --test --home flies an 80 m square at home")
    test.add_argument("--yes", action="store_true", help="Skip the upload confirmation prompt")
    test.add_argument("--only", type=lambda v: v if v == "middle" else int(v), metavar="N",
                      help="Plan --drones paths but export/upload only path N (or 'middle'), flown by ONE drone "
                           "(no altitude stagger, own speed). Paths are numbered across the drone line from one "
                           "edge to the other. Open *_plan.kml to see which is which")
    args = p.parse_args(argv)
    for key, value in (TEST_DEFAULTS if args.test else NORMAL_DEFAULTS).items():
        if getattr(args, key) is None:
            setattr(args, key, value)
    if args.kml is None:
        if not (args.test and args.home):
            p.error("give a field file (.kml/.kmz/.poly), or use --test --home LON LAT for a test square")
    if args.only == "middle":
        args.only = (args.drones + 1) // 2
    if args.only is not None:
        if args.test and args.drones == TEST_DEFAULTS["drones"] and "--drones" not in (argv or sys.argv):
            p.error("--only needs --drones K: the number of paths to split the field into (e.g. --drones 4 --only 2)")
        if not 1 <= args.only <= args.drones:
            p.error(f"--only must be between 1 and --drones ({args.drones})")
        args.sync = False  # one drone flies alone: use its own speed, not the formation timing
    if args.test and args.only is None and not 1 <= args.drones <= 2:
        p.error("--test supports 1 or 2 drones; drop --test for a full swarm, or fly one path with --only N")
    if args.test_size is not None and args.test_size < 20:
        p.error("--test-size must be at least 20 m")
    if not 1 <= args.drones <= 64:
        p.error("--drones must be between 1 and 64")
    if not 0 <= args.front_overlap < .95:
        p.error("--front-overlap must be in [0, 0.95)")
    if args.min_alt > args.max_alt:
        p.error("--min-alt cannot exceed --max-alt")
    if args.rtl_height is not None:
        flying = 1 if args.only else args.drones
        need = args.min_alt + FENCE_MARGIN + 1 + 2 * flying + (flying - 1) * (0 if args.only else args.alt_stagger)
        if args.rtl_height < need:
            p.error(f"--RTLheight must be at least {need:.0f} m for {flying} drone(s) "
                    f"(survey floor, altitude stagger, and room for RTL layers below the ceiling)")
        # Leave room above the survey for one return layer per drone (>= 2 m apart) below the ceiling.
        limit = args.rtl_height - FENCE_MARGIN - 1 - 2 * flying
        if args.max_alt > limit:
            print(f"note: --max-alt lowered from {args.max_alt:.0f} m to {limit:.0f} m to stay under "
                  f"--RTLheight {args.rtl_height:.0f} m", file=sys.stderr)
            args.max_alt = limit
    if args.rtl_alt is not None and args.rtl_alt < args.max_alt:
        print(f"warning: --rtl-alt {args.rtl_alt:.0f} m is below --max-alt {args.max_alt:.0f} m; "
              "drones may descend into the survey layer on the way home.", file=sys.stderr)
    if args.connect == ["sitl"]:
        args.connect = [f"tcp:127.0.0.1:{5760 + 10 * i}" for i in range(1 if args.only else args.drones)]
    return args


def main(argv=None):
    args = parse(argv)
    result = plan(args, log=print)
    missions = result["missions"]
    os.makedirs(args.out, exist_ok=True)
    stem = os.path.splitext(os.path.basename(args.kml))[0] if args.kml else "square"
    if args.test:
        stem = "TEST_" + stem
        flying = 1 if args.only else args.drones
        area = f"{args.test_size:.0f} m square of the field" if args.test_size and args.kml else (
            "80 m square at home" if not args.kml else "whole field")
        print(f"TEST MODE: {flying} drone(s), altitude {args.min_alt:.0f}-{args.max_alt:.0f} m, "
              f"speed <= {args.speed:.0f} m/s, area: {area}")
    preview = os.path.join(args.out, f"{stem}_plan.kml")
    names = [f"{stem}_drone{m.index + 1:02d}" for m in missions]
    if args.only:
        print(f"\nPaths in this {len(missions)}-drone plan (pick one with --only N):")
        for m in missions:
            mark = "  <== flying" if m.index + 1 == args.only else ""
            print(f"  path {m.index + 1}: {m.length_m:5.0f} m, alt {m.min_alt:.0f}-{m.max_alt:.0f} m, "
                  f"~{m.photos} photos, {m.transits} transit(s){mark}")
        chosen = missions[args.only - 1]
        if chosen.waypoints is None or not len(chosen.waypoints):
            raise SystemExit(f"Path {args.only} is empty; choose another with --only.")
        names = [f"{stem}_path{args.only:02d}of{len(missions):02d}"]
        missions = [replace(chosen, index=0)]  # flown alone: drone 1 settings (RTL height, no stagger)
        write_preview_kml([tuple(p) for p in result["ring"]], [replace(chosen, index=args.only - 1)], preview, stem,
                          lanes=result["lane_rings"])
        result["seconds"] = chosen.length_m / args.speed
        result["separation"] = float("inf")
    else:
        write_preview_kml([tuple(p) for p in result["ring"]], missions, preview, stem)
    files = []
    for m, name in zip(missions, names):
        path = os.path.join(args.out, name + ".waypoints")
        write_waypoints(m, path)
        files.append(path)
    params = safety_params(missions, result["home"], args)
    fence, fence_info = None, None
    if args.boundary_fence:
        fence_ll, fence_info = boundary_fence(result["ring"], missions, result["home"], args.fence_buffer)
        fence = fence_items(fence_ll)
        write_fence_files(fence_ll, os.path.join(args.out, stem))
        _append_fence_to_preview(preview, fence_ll)
    for m, values, name in zip(missions, params, names):
        write_params(values, os.path.join(args.out, name + ".param"), m.index, args.home is not None)
    separation = result["separation"]
    summary = {
        "field": result["title"], "areaHa": round(result["area_m2"] / 1e4, 3), "planner": args.planner,
        "drones": result["drones"], "camera": {"hfovDeg": args.hfov, "vfovDeg": args.vfov,
                                               "sideOverlap": args.overlap, "frontOverlap": args.front_overlap},
        "metresPerCanvasUnit": round(result["canvas"].scale, 4),
        "requiredMaxAltM": round(result["needed_alt"], 1), "altitudeCapM": args.max_alt,
        "nadirCoveragePct": round(100 * result["coverage"], 2), "capped": result["capped"],
        "estimatedMissionS": round(result["seconds"], 1),
        "minSeparationM": round(separation, 1) if math.isfinite(separation) else None, "test": args.test,
        "onlyPath": args.only, "rtlHeightM": args.rtl_height, "boundaryFence": fence_info,
        "ownAreaCoveragePct": None if result["lane_coverage"] is None else round(100 * result["lane_coverage"], 2),
        "missions": [{"file": os.path.basename(f), "items": len(m.items), "lengthM": round(m.length_m, 1),
                      "maxAltM": round(m.max_alt, 1), "minAltM": round(m.min_alt, 1), "transits": m.transits,
                      "photos": m.photos, "rtlAltM": values["RTL_ALT"] / 100, "fenceRadiusM": values["FENCE_RADIUS"],
                      "fenceAltMaxM": values["FENCE_ALT_MAX"]}
                     for f, m, values in zip(files, missions, params)],
    }
    with open(os.path.join(args.out, f"{stem}_summary.json"), "w") as handle:
        json.dump(summary, handle, indent=2)
    if args.only:
        print(f"\nFlying path {args.only} of {result['drones']} with one drone: it images "
              f"{summary['ownAreaCoveragePct']}% of its own area (yellow in {stem}_plan.kml). "
              f"The rest of the field is left for the other paths.")
    else:
        print(f"\nNadir coverage at flown altitudes: {summary['nadirCoveragePct']}%  "
              f"(max altitude {max(m.max_alt for m in missions):.1f} m, cap {args.max_alt:.0f} m)")
    spacing = f"closest 3D approach between drones: {separation:.1f} m" if math.isfinite(separation) else "single drone"
    kind = "Estimated flight time" if args.only else "Estimated synchronised mission time"
    print(f"{kind}: {summary['estimatedMissionS'] / 60:.1f} min; {spacing}")
    if separation < args.min_sep:
        print("warning: drones pass close to each other; increase --alt-stagger or stagger launch times.")
    for entry in summary["missions"]:
        print(f"  {entry['file']}: {entry['items']} items, {entry['lengthM']:.0f} m, "
              f"alt {entry['minAltM']:.0f}-{entry['maxAltM']:.0f} m, ~{entry['photos']} photos, "
              f"RTL at {entry['rtlAltM']:.0f} m, fence {entry['fenceRadiusM']} m / {entry['fenceAltMaxM']} m")
    triggers = []
    if fence_info:
        triggers.append(f"leaving the field boundary (+{fence_info['bufferM']:g} m)")
    triggers.append(f"climbing above {summary['missions'][0]['fenceAltMaxM']} m"
                    f"{'' if args.rtl_height is None else ' (--RTLheight)'}")
    triggers.append(f"going beyond {summary['missions'][0]['fenceRadiusM']} m from home")
    print("Failsafe RTL to home on: " + "; ".join(triggers) + ". Mission end: RTL.")
    if fence_info:
        margin = None if fence_info["pathClearanceM"] is None else fence_info["pathClearanceM"] + fence_info["bufferM"]
        if margin is not None and margin < args.accept_radius + 3:
            print(f"warning: the path comes within {margin:.1f} m of the boundary fence; GPS drift or overshoot "
                  f"could trigger RTL. Increase --fence-buffer.")
        if not fence_info["homeInsideField"]:
            print("note: home is outside the field; the fence includes a corridor from home to the field.")
    if max(e["fenceAltMaxM"] for e in summary["missions"]) > 120:
        print("warning: a fence/RTL altitude exceeds 120 m AGL; check your local altitude limit.")
    print(f"Wrote {len(files)} mission(s), .param file(s){', a boundary fence' if fence else ''}, a KML preview "
          f"and a summary to {os.path.abspath(args.out)}")
    if args.connect:
        if len(args.connect) != len(missions):
            raise SystemExit(f"--connect lists {len(args.connect)} links but the plan has {len(missions)} drones.")
        if args.test and not args.yes:
            what = ("mission + boundary fence + RTL/fence parameters" if fence else "mission + RTL/fence parameters") \
                if args.push_params else "mission only"
            print(f"\nTEST UPLOAD ({what}). CHECKLIST: props clear, RC pilot ready, RTL switch on the RC, "
                  f"preview checked in {stem}_plan.kml")
            try:
                answer = input(f"Upload {len(missions)} test mission(s)? [y/N] ").strip().lower()
            except EOFError:
                answer = ""
            if answer not in ("y", "yes"):
                print("Upload cancelled. Mission files were still written.")
                return summary
        for url, m, values in zip(args.connect, missions, params):
            print(f"Uploading drone {m.index + 1} ...")
            upload(url, m.items, timeout=args.timeout, params=values if args.push_params else None,
                   fence=fence if args.push_params else None)
    return summary


if __name__ == "__main__":
    main()

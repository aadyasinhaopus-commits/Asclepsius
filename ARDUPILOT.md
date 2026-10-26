# Flying the plan with ArduPilot and Mission Planner

> New to this? Start with the step-by-step [MANUAL.md](MANUAL.md). This page is the option reference.

`swarm_lab/ardupilot_export.py` converts a coverage plan into one ArduCopter mission per
drone. You can load the missions in Mission Planner or upload them directly over MAVLink.

```
KML/KMZ/.poly  ──►  planner (harmonic / lawnmower / geodesic / MAT)  ──►  per-drone
field boundary      lanes, paths, headings, nadir altitudes              .waypoints files
                                                                         + KML preview
                                                                         + summary.json
                                                                         + optional MAVLink upload
```

## 1. Install

```powershell
cd swarm_lab
py -m pip install -r requirements.txt     # includes pymavlink
```

## 2. Draw the field

- **Google Earth:** *Add → Polygon*, trace the search area, then *Save Place As → .kml/.kmz*.
- **Mission Planner:** in *Flight Plan*, right-click → *Draw Polygon*, then *Save Polygon*
  (`.poly`).

If the file contains several polygons, the largest is used. To choose one by name, use
`--placemark "Name"`. Holes (inner boundaries) are ignored.

## 3. Generate missions

### Test mode (start here)

For first flights with one or two drones, add `--test`:

```powershell
# One drone over an 80 m square centred on where you stand (no field file needed)
py ardupilot_export.py --test --home 77.1930 28.5466

# One or two drones over the real field, clipped to an 80 m square around its centre
py ardupilot_export.py field.kml --test
py ardupilot_export.py field.kml --test --drones 2

# Test in SITL (expands to tcp:127.0.0.1:5760, :5770, ...)
py ardupilot_export.py field.kml --test --drones 2 --connect sitl

# Real drone over its telemetry radio
py ardupilot_export.py field.kml --test --connect COM5,57600
```

`--test` changes the defaults to safe first-flight values. Any of them can still be
overridden, for example with `--max-alt 40`:

| Setting | Normal | `--test` |
| --- | --- | --- |
| Drones | 4 | **1** (only 1 or 2 allowed) |
| Max / min altitude | 120 / 15 m | **30 / 10 m** |
| Max speed | 8 m/s | **3 m/s** |
| Altitude stagger between drones | 5 m | **8 m** |
| Separation warning | 5 m | **8 m** |
| Area | whole field | **whole field** (optional `--test-size M` square around `--home`) |
| Output | `missions/` | **`missions_test/`**, files prefixed `TEST_` |
| Upload | immediate | **checklist and a `y/N` prompt** (skip it with `--yes`) |

**Fly one path of the swarm with one drone:** `--drones K --only N` plans K paths but exports
only path N. The drone covers its own area in full (checked) and ignores the rest:

```powershell
py ardupilot_export.py field.kml --drones 4 --only 2 --test --home 77.1930 28.5466
```

Suggested progression:

1. `--test` in SITL.
2. `--test` with one real drone, with a pilot ready on the RC. Then `--drones K --only N`
   to fly one real swarm path.
3. `--test --drones 2`.
4. A full field without `--test`.

### Full missions

```powershell
py ardupilot_export.py field.kml --drones 6 --hfov 147 --vfov 62.22 --max-alt 120 --out missions
```

Output in `missions/`:

| File | Use |
| --- | --- |
| `field_drone01.waypoints` … | One mission per drone (`QGC WPL 110`). Load it in Mission Planner with *Flight Plan → Load WP File* |
| `field_plan.kml` | Open in Google Earth to check every path in 3D before flying |
| `field_fence.waypoints`, `field_fence.poly` | Boundary inclusion fence (KML edge + `--fence-buffer`). Breach ? RTL |
| `field_drone01.param` ? | Suggested `RTL_ALT` (different for each drone), `RTL_ALT_FINAL`, and geofence (`FENCE_*`, action RTL) |
| `field_summary.json` | Required altitude, coverage at the flown altitudes, mission time, closest approach between drones, and per-drone length and photo count |

### What each mission contains

| # | Command | Notes |
| --- | --- | --- |
| 0 | Home | Replaced by the autopilot with the actual home position |
| 1 | `NAV_TAKEOFF` | To the first survey altitude |
| | `DO_CHANGE_SPEED` | Max `--speed`. With synchronisation on (the default), each leg's speed is set so that every drone reaches the same front at the same time, which keeps the formation line together |
| | `DO_SET_CAM_TRIGG_DIST` | Photo spacing = `2·h·tan(VFOV/2)·(1 − front overlap)`. It is updated whenever the altitude changes, and set to 0 (camera off) on transit hops and before RTL |
| | `NAV_WAYPOINT` | Simplified path (within `--path-tolerance` m), with altitude **relative to home**. Each kept waypoint flies at the highest altitude required by any sample it replaces, so simplification never reduces coverage |
| last | `NAV_RETURN_TO_LAUNCH` | |

### Altitudes, coverage and safety

- The planner chooses the lowest altitude that still images every cell of the drone's
  area (see the report). After converting to metres, the exporter:
  1. clips altitudes to `[--min-alt, --max-alt]`;
  2. adds `drone index × --alt-stagger` (default 5 m) so that converging drones are
     vertically separated. Flying higher never reduces coverage;
  3. **re-checks nadir coverage at the final altitudes** and prints it.
- If the field needs more than `--max-alt`, the exporter warns and reports the coverage that
  is actually achieved. Add drones, or use `--auto-drones` to add drones automatically
  until the plan fits under the cap.
- The closest 3D approach between any two drones is reported, assuming they fly in sync.
  If it falls below `--min-sep` (default 5 m), increase `--alt-stagger`.
- Altitudes are **AGL relative to home**. For uneven terrain, enable terrain following in
  ArduPilot (`TERRAIN_ENABLE=1`, and change waypoint frames to terrain) or add margin.

### Useful options

| Option | Default | Meaning |
| --- | --- | --- |
| `--planner` | `harmonic` | `harmonic` gives the lowest altitude; `lawnmower` and `geodesic` balance workload best; `mat` |
| `--hfov`, `--vfov` | 147, 62.22 | Camera across-track and along-track fields of view (degrees) |
| `--overlap` | 0.1 | Side-overlap margin on altitude |
| `--front-overlap` | 0.3 | Forward overlap between consecutive photos |
| `--speed`, `--min-speed` | 8, 1.5 m/s | Ground-speed limits |
| `--no-sync` | | Every drone flies at `--speed`. Missions are shorter, but the formation line is not kept |
| `--accept-radius` | 2 m | `WPNAV` acceptance radius per waypoint |
| `--home LON LAT` | centroid | Launch point: item 0, fence centre, test square |
| `--RTLheight M` | off | **Height failsafe:** sets `FENCE_ALT_MAX=M` with action RTL, so a drone above M m returns home. Survey and RTL altitudes are kept below M. Pushed to the drone with `--connect` |
| `--only N` / `--only middle` | | Fly only path N of a `--drones K` plan, with one drone. Paths are numbered from one edge of the drone line to the other |
| `--fence-buffer M` | 5 m | The boundary inclusion fence sits M metres outside the KML edge. Crossing it triggers RTL |
| `--no-boundary-fence` | | Do not create or upload the boundary fence |
| `--test-size M` | off | Fly only an M-metre square of the field around `--home` |
| `--no-params` | | With `--connect`, upload the mission without the RTL/fence parameters |
| `--rtl-alt` | survey max + `--rtl-margin` | RTL altitude for drone 1. Each further drone adds `--alt-stagger` |
| `--rtl-margin`, `--fence-margin` | 5 m, 30 m | Extra height above the survey for RTL; extra distance beyond the farthest waypoint for the fence |

## 4. Upload to the drones (pymavlink)

Give one MAVLink connection per drone, in drone order:

```powershell
# Telemetry radios
py ardupilot_export.py field.kml --drones 3 --connect COM5,57600 COM6,57600 COM7,57600

# SITL (each instance on its own TCP port; or just: --connect sitl)
py ardupilot_export.py field.kml --drones 3 --connect tcp:127.0.0.1:5760 tcp:127.0.0.1:5770 tcp:127.0.0.1:5780

# Mission Planner MAVLink forwarding / companion computers
py ardupilot_export.py field.kml --drones 2 --connect udpin:0.0.0.0:14550 udpin:0.0.0.0:14551
```

Each drone's RTL and geofence parameters (`RTL_ALT`, `FENCE_*`, including `--RTLheight`)
are then written with `PARAM_SET`, and each value is confirmed by the drone. Each mission
is sent with the MAVLink mission protocol (`MISSION_COUNT` →
`MISSION_REQUEST_INT` → `MISSION_ITEM_INT` → `MISSION_ACK`), then **read back and compared
item by item**. Give each drone a unique `SYSID_THISMAV` so that Mission Planner can show
all of them together.

To test without hardware, run the fake vehicles that implement the mission protocol:

```powershell
py examples/fake_vehicle.py 15760 15770
py ardupilot_export.py examples/field.kml --drones 2 --connect tcp:127.0.0.1:15760 tcp:127.0.0.1:15770
```

## 5. Fly

1. In Mission Planner, connect to each drone (*Ctrl+L* or the connection dropdown) and
   check its mission on *Flight Plan → Read*.
2. Pre-arm checks: GPS lock, fence, battery failsafe, `RTL_ALT` above the highest mission
   altitude plus the stagger.
3. Camera: set `CAM1_TYPE` (servo/relay/MAVLink) so that `DO_SET_CAM_TRIGG_DIST`
   triggers the shutter. Point the gimbal at nadir (`MNT1_*`, or a `DO_MOUNT_CONTROL`
   pitch of −90).
4. Arm the drones and switch them to **AUTO** together (e.g. from the Mission Planner
   *Swarm* screen, or with a script) so that the synchronised speeds keep the formation line.

> **Safety.** Always review `field_plan.kml` and the summary first, and keep a pilot on
> each RC link. Simulate first in SITL with the real parameter files. The planner checks
> nadir coverage and drone-to-drone spacing geometrically. It does **not** model wind,
> GPS error, obstacles above ground, or airspace rules.

## Tests

```powershell
py -m unittest test_ardupilot -v
```

The tests cover the KML/projection round trip, the `.waypoints` structure (home, takeoff,
camera on and off, RTL), altitude limits, the trigger distance against the footprint size,
and 100% coverage at the flown altitudes.

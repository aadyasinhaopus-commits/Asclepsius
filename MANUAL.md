# Field-to-flight manual

This manual walks through a mission from start to finish: draw the field, generate the
missions, check them, fly the drones, and bring every drone back home with RTL. Run the
commands from the `swarm_lab/` folder in PowerShell. For details on every option, see
[ARDUPILOT.md](ARDUPILOT.md).

```
 1 Draw field (KML)  →  2 Generate missions  →  3 Check preview  →  4 SITL rehearsal
 →  5 Set up drones  →  6 Upload  →  7 Pre-flight  →  8 Fly (AUTO)  →  9 RTL & land  →  10 After flight
```

---

## 0. One-time setup

```powershell
cd swarm_lab
py -m pip install -r requirements.txt
py -m unittest test_ardupilot          # should print OK
```

You also need the following.

- **Mission Planner** (Windows) with each drone running **ArduCopter 4.3 or newer**.
- One telemetry link per drone (USB, a SiK radio, or Wi-Fi/UDP), and an **RC transmitter
  per drone** with a safety pilot.
- A downward-pointing camera that ArduPilot can trigger (servo, relay, or MAVLink camera).

---

## 1. Get the field boundary (KML)

Use any **one** of the methods below. The planner takes the **largest polygon** in the file.
Holes are ignored.

### A. Google Earth Pro (desktop, most accurate)

1. Zoom to the field. Click **Add Polygon** (toolbar), or press *Ctrl+Shift+G*.
2. Name it (e.g. `Search area`), then click around the boundary. Keep obstacles such as
   buildings, power lines and trees **outside** the polygon.
3. Click **OK**. In *Places*, right-click the polygon, choose **Save Place As…**, and save
   it as `field.kml` (or `.kmz`).

### B. Google Earth (web)

**Projects → New project → Draw line or shape** → close the shape → **⋮ → Export as KML file**.

### C. Google My Maps

**Draw a line → Add line or shape** → close the shape → menu **⋮ → Export to KML/KMZ**.
Tick **"Export to a .KML file"**, or keep KMZ; both work.

### D. Mission Planner (no internet needed in the field if maps are cached)

**PLAN** tab → right-click the map → **Draw Polygon → Add Polygon Point** around the field
→ right-click → **Draw Polygon → Save polygon** → `field.poly`. Use the `.poly` file
wherever this manual says `field.kml`.

### Find the launch (home) point

The `--home` option takes **longitude first, then latitude**:

- **Google Maps:** right-click the take-off spot to copy `lat, lon`, e.g. `28.5466, 77.1930`.
  Then pass `--home 77.1930 28.5466`.
- **Mission Planner:** the coordinates shown under the mouse at the bottom of the PLAN map.

Choose a home point that is clear, flat and **outside or at the edge** of the field. Every
drone returns there on RTL.

> If you omit `--home`, the field centroid is used for item 0 and for the geofence. Always
> pass `--home` for real flights.

---

## 2. Generate the missions

### First flights: test mode (one drone)

```powershell
py ardupilot_export.py field.kml --test --home 77.1930 28.5466
```

Then step up to two drones:

```powershell
py ardupilot_export.py field.kml --test --drones 2 --home 77.1930 28.5466
```

`--test` flies the **real field** from your KML at safe first-flight settings: 10 to 30 m,
at most 3 m/s, 8 m vertical separation, and a confirmation prompt before upload. To fly
only part of a large field, add `--test-size M`, which keeps an M-metre square around
home. To test without a KML file at all, use `py ardupilot_export.py --test --home LON LAT`
(an 80 m square at home).

### Boundary fence: RTL if the drone touches the field edge

Every run builds an **inclusion fence** from the KML boundary, grown outward by
`--fence-buffer` (default 5 m), plus a corridor to home. **If the drone crosses it, it
returns home immediately.** The fence is:

- written to `*_fence.waypoints` and `*_fence.poly` (Mission Planner: **PLAN ? Fence ?
  Load from file**), and drawn in red in `*_plan.kml`;
- uploaded and read back automatically with `--connect`, together with `FENCE_TYPE = 7`
  (altitude + circle + polygon) and `FENCE_ACTION = 1` (RTL);
- checked against the planned path. If the path comes within `accept-radius + 3` m of the
  fence, the tool warns you to increase `--fence-buffer`, so that GPS drift does not
  trigger a false RTL.

Use `--no-boundary-fence` to turn it off. The fence needs MAVLink 2 on the telemetry port
(`SERIALn_PROTOCOL = 2`); otherwise the upload stops rather than overwriting the mission.

### One drone flying one path (covers only its own area)

Split the field into K paths, as the full swarm would, and have **one** drone fly just
one of them. It images its **own area completely** and ignores the other areas. This is
the most realistic single-drone test, because it flies exactly what that drone would fly
in the real swarm.

```powershell
py ardupilot_export.py field.kml --drones 4 --only 2 --test --home 77.1930 28.5466
```

```
Paths in this 4-drone plan (pick one with --only N):
  path 1:   204 m, alt 10-10 m, ~28 photos, 0 transit(s)
  path 2:   270 m, alt 10-13 m, ~38 photos, 0 transit(s)  <== flying
  ...
Path 2: own area 0.99 ha (25% of the field); imaged by this drone alone: 100.0%
```

- `--drones 4` sets how many paths the field is split into. `--only 2` picks which one to fly.
- **Which path is which?** Paths are numbered across the drone line, from one edge of the
  field (path 1) to the opposite edge (path K). Path 1 hugs one side wall, path K the other,
  and the middle paths run through the centre. Use `--only middle` to pick the centre
  path. For an even K, that is the one just left of centre, e.g. path 2 of 4. The
  `*_plan.kml` preview shows the chosen path and its yellow area.
- Output: a single `TEST_field_path02of04.waypoints` and its `.param` file. The
  `*_plan.kml` preview shows that path and its own area (yellow).
- The tool checks that the drone images **100% of its own area**, and warns if it does not.
- No altitude stagger is applied. The drone flies at its own speed and ends with RTL.
- Without `--test`, the drone flies the same path over the full field, with normal limits.

### Full survey

```powershell
py ardupilot_export.py field.kml --drones 4 --home 77.1930 28.5466 --hfov 147 --vfov 62.22 --max-alt 120
```

Add `--auto-drones` to let the tool add drones until the plan fits under `--max-alt`.

### What you get (in `missions_test/` or `missions/`)

| File | What to do with it |
| --- | --- |
| `*_drone01.waypoints`, `*_drone02.waypoints`, … | The mission for each drone. Upload it in step 6 |
| `*_drone01.param`, … | RTL altitude and geofence for each drone. Load it in step 5 |
| `*_plan.kml` | Open it in Google Earth and check every path in 3D (step 3) |
| `*_summary.json` | Numbers to note down: coverage, altitudes, RTL heights, time, spacing |

Read the console output before you continue:

```
Nadir coverage at flown altitudes: 100.0%  (max altitude 24.9 m, cap 30 m)
Estimated synchronised mission time: 1.0 min; closest 3D approach between drones: 8.2 m
  TEST_field_drone01.waypoints: 20 items, 93 m, alt 10-11 m, ~15 photos, RTL at 30 m, fence 90 m / 40 m
  TEST_field_drone02.waypoints: 25 items, 176 m, alt 18-25 m, ~22 photos, RTL at 38 m, fence 90 m / 48 m
```

- **Coverage must be 100%.** If it is lower, the altitude cap is too tight: add a drone or
  shrink `--test-size`.
- **Any `warning:` line must be resolved before flying.**

---

## 3. Check the plan

1. Open `*_plan.kml` in Google Earth. Confirm that each coloured path stays inside the
   field and clears trees, poles and wires at its altitude, and that home is where you will
   stand.
2. In Mission Planner, **PLAN → Load WP File**, then open each `.waypoints` file. Check
   that the list:
   - starts with **TAKEOFF**;
   - contains **DO_CHANGE_SPEED** and **DO_SET_CAM_TRIGG_DIST** entries;
   - contains **WAYPOINT** entries at the expected altitudes;
   - ends with **DO_SET_CAM_TRIGG_DIST 0** (camera off) followed by **RETURN_TO_LAUNCH**.

---

## 4. Rehearse in SITL (strongly recommended)

Start one simulated copter per drone at your home location. SITL uses latitude first:

```powershell
sim_vehicle.py -v ArduCopter -I0 --sysid 1 -l 28.5466,77.1930,0,0
sim_vehicle.py -v ArduCopter -I1 --sysid 2 -l 28.5466,77.1930,0,0
```

Then run the same command you will use in the field, with `--connect sitl` (TCP ports
5760, 5770, …):

```powershell
py ardupilot_export.py field.kml --test --drones 2 --home 77.1930 28.5466 --connect sitl
```

In Mission Planner, connect to `TCP 127.0.0.1:5760` (and `:5770`), then fly steps 7 to 9
exactly as in the field. Make sure every drone ends **landed and disarmed at home**.

> No SITL available? `py examples/fake_vehicle.py 15760 15770` with
> `--connect tcp:127.0.0.1:15760 tcp:127.0.0.1:15770` checks the upload only, not the flight.

---

## 5. Set up each drone (once per drone)

Connect each drone to Mission Planner on its own link: select the COM port and baud rate,
then click **CONNECT**.

1. **Unique ID.** In **CONFIG → Full Parameter List**, set `SYSID_THISMAV` = 1, 2, 3, …
   to match the drone number. Write the parameters and reboot.
2. **RTL and geofence.** In **CONFIG → Full Parameter List → Load from file**, choose
   `*_droneNN.param` for *that* drone, then click **Write Params**. This sets:

   | Parameter | Value | Why |
   | --- | --- | --- |
   | `RTL_ALT` | above the highest survey altitude, **different for each drone** | Returning drones climb to separate heights, so they cannot collide on the way home |
   | `RTL_ALT_FINAL` | 0 | Land at home |
   | `FENCE_ENABLE`, `FENCE_TYPE` 3 | on, altitude + circle | Hard limit around home |
   | `FENCE_RADIUS`, `FENCE_ALT_MAX` | farthest waypoint + 30 m; RTL altitude + 10 m | A breach triggers… |
   | `FENCE_ACTION` | 1 | …**RTL** |

3. **Failsafes.** Set these by hand once. Each makes the drone return home on its own:

   | Parameter | Set to | Effect |
   | --- | --- | --- |
   | `FS_THR_ENABLE` | 1 | RC signal lost → **RTL** |
   | `BATT_FS_LOW_ACT` | 2 | Low battery → **RTL** (also set `BATT_LOW_VOLT` or `BATT_LOW_MAH`) |
   | `BATT_FS_CRT_ACT` | 1 | Critical battery → **Land** |
   | `FS_GCS_ENABLE` | 1 (optional) | Telemetry lost → **RTL** |
   | `AUTO_OPTIONS` | 3 | Allow arming in AUTO and take-off without raising the throttle |

4. **Camera.** Set `CAM1_TYPE` for your trigger type (1 servo, 2 relay, 5 MAVLink). Point
   the gimbal straight down (`MNT1_*`, pitch −90°), or mount the camera fixed at nadir.
5. **Flight modes.** On the RC, make sure one switch position is **RTL** and another is
   **LOITER** (`FLTMODE1`…`FLTMODE6`, `FLTMODE_CH`).

---

## 6. Upload the missions

**Option A: from this tool (all drones in one go, with read-back):**

```powershell
# Telemetry radios: one connection per drone, in drone order
py ardupilot_export.py field.kml --test --drones 2 --home 77.1930 28.5466 --connect COM5,57600 COM6,57600
```

In test mode the tool prints a checklist and asks `Upload …? [y/N]`. Each mission is
**read back and compared item by item**. A mismatch stops the upload with an error.

> Close Mission Planner's connection to a COM port before the tool uses it; a serial port
> can only be opened by one program. Alternatively, use Mission Planner's MAVLink
> forwarding with `--connect udpin:0.0.0.0:14550`.

**Option B: from Mission Planner (one drone at a time):** connect to the drone, then
**PLAN → Load WP File** → `*_droneNN.waypoints` → **Write WPs**, then **Read WPs** to
confirm.

Double-check that **drone N has mission N**. The missions differ in path and altitude.

---

## 7. Pre-flight checklist (every drone)

- [ ] Correct `.waypoints` loaded (**Read WPs** shows the expected item count, as in the summary).
- [ ] Correct `.param` loaded (`RTL_ALT` matches `rtlAltM` × 100 in the summary).
- [ ] Battery full. Battery failsafe set.
- [ ] GPS 3D fix, HDOP < 1.2, at least 10 satellites. **Home set at the take-off spot** (shown on the Mission Planner map).
- [ ] Fence enabled. No pre-arm errors in Mission Planner's HUD/Messages.
- [ ] Camera triggers: on the ground, **Actions → Trigger Camera NOW** takes a photo.
- [ ] Props clear. A safety pilot per drone, hand on the **mode switch** (RTL/LOITER).
- [ ] Drones spaced at least 5 m apart on the ground. Airspace and local permissions in place.

---

## 8. Fly

Do this for each drone, **drone 1 first**. With several drones, start them within a few
seconds of each other: the speeds are synchronised so that the drone line stays together.

1. In Mission Planner, select the drone (vehicle selector at the top right, or its own window).
2. **DATA → Actions**: choose **Auto** → **Set Mode**. Then **Arm/Disarm**.
   - With `AUTO_OPTIONS = 3`, the drone arms in AUTO and takes off immediately.
   - Otherwise, arm in **LOITER**, switch to **AUTO**, and raise the throttle briefly to start.
3. Watch it: it takes off, flies its path, and the photo count rises.

### Height failsafe: `--RTLheight`

Add `--RTLheight M` to any command to set a hard ceiling. **If a drone climbs above M
metres over home, for any reason (GPS or baro error, a bad command, a flyaway), it
immediately returns home (RTL).**

```powershell
py ardupilot_export.py field.kml --drones 4 --only middle --test --home 77.1930 28.5466 --RTLheight 25 --connect COM5,57600
```

- This sets `FENCE_ALT_MAX = M`, `FENCE_ACTION = 1` (RTL), `FENCE_TYPE = 3` (altitude
  ceiling + circle around home) and `FENCE_ENABLE = 1`.
- Survey altitudes are kept below M, and each drone's `RTL_ALT` is placed between the
  survey and M, so the return leg does not breach the ceiling again.
- The same fence also triggers RTL if the drone goes more than `FENCE_RADIUS` from home
  (the farthest waypoint + 30 m).
- With `--connect`, these parameters are **written to the drone and confirmed** after the
  mission upload. Use `--no-params` to upload the mission only. The values are also saved
  in the `.param` file.
- ArduPilot only enforces the fence while it is armed with a valid home. Check
  **Fence** on the HUD or in Messages before take-off.

**If anything looks wrong, switch to LOITER** (it stops in place) or **RTL** on the RC.
You can resume a mission with AUTO.

---

## 9. Return to launch (RTL) and landing

Every mission **ends with RETURN_TO_LAUNCH**, so you do not need to do anything after the
last waypoint. On RTL, the drone:

1. turns the camera off (the trigger distance is set to 0 just before);
2. climbs to its own `RTL_ALT`. Each drone uses a different height, so drones returning
   together stay vertically separated;
3. flies straight home, descends, and **lands at home** (`RTL_ALT_FINAL = 0`);
4. disarms automatically after landing.

RTL also happens **automatically** if:

| Event | Trigger |
| --- | --- |
| Mission finished | Last mission item |
| Geofence breached: outside the field boundary fence, above `--RTLheight` / `FENCE_ALT_MAX`, or beyond `FENCE_RADIUS` | `FENCE_ACTION = 1` |
| RC link lost | `FS_THR_ENABLE = 1` |
| Battery low | `BATT_FS_LOW_ACT = 2` |
| Telemetry lost (if enabled) | `FS_GCS_ENABLE = 1` |

**Manual RTL**, any time:

- **RC:** flip the mode switch to **RTL**.
- **Mission Planner:** **DATA → Actions → RTL**, or **Set Mode → RTL**.
- **All drones at once:** select each vehicle in turn and press **RTL**. Each one climbs to
  its own `RTL_ALT` first, so the order does not matter.

During landing, keep people clear of home. If a drone lands off target, take over in
LOITER and land manually.

---

## 10. After the flight

1. Download the flight logs: **DATA → DataFlash Logs → Download DataFlash Log Via Mavlink**.
2. Geotag the photos: open **Ctrl+F → Geo ref images**, select the log and the photo folder.
3. Check coverage. Your photo footprints should tile the field, as in the `*_plan.kml` preview.
4. Scale up: test with 1 drone → test with 2 drones → full field with `--drones N` (without `--test`).

---

## Troubleshooting

| Symptom | Fix |
| --- | --- |
| `No polygon found in the KML` | The KML has only pins or lines. Redraw the field as a **polygon** |
| `Test area … does not overlap the field` | `--home` is far from the field. Check the coordinate order is **LON LAT**, or increase `--test-size` |
| `Coverage at capped altitudes: <100%` | Add a drone, raise `--max-alt` within legal limits, or reduce `--test-size` |
| `warning: drones pass close to each other` | Increase `--alt-stagger`, or launch the drones a few seconds apart |
| `No vehicle heartbeat on …` | Wrong COM port, baud rate or IP, or Mission Planner is holding the port. Check the link with Mission Planner first |
| `Vehicle … rejected the mission` | Firmware too old, or the mission is too large for the board. Update ArduCopter, or reduce `--grid` |
| `Read-back mismatch` | Radio dropout. Retry the upload, or upload from Mission Planner (option B) |
| Drone won't arm in AUTO | Set `AUTO_OPTIONS = 3`, or arm in LOITER and then switch to AUTO |
| No photos | Check `CAM1_TYPE`, the trigger wiring, and the camera's own settings (auto-focus, SD card) |
| Wrong landing spot | Home is set when the drone is armed. Arm it at the actual take-off spot |

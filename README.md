# swarm_lab

Coverage planning for a nadir-camera drone swarm. The SKYGRID dashboard is now connected to the local ArduPilot mission exporter.

## Run the dashboard

From this folder in PowerShell:

```powershell
py -m pip install -r requirements.txt
py server.py
```

Open **http://127.0.0.1:8765**.

- `/` — SKYGRID operator dashboard
- `/lab` — existing medial-axis editor
- `/compare` — existing planner comparison tool
- `/health` — local server health check

## Generate a mission bundle from the dashboard

1. Open **Field & Map** and select a `.kml`, `.kmz`, or Mission Planner `.poly` boundary.
2. Set the home longitude and latitude. The exporter expects **longitude first, latitude second**.
3. Open **Mission Planner**, configure the number of drones, planner, camera FOV, altitude cap, speed, fence buffer, and RTL height.
4. Select **Generate plan**. The dashboard calls `/api/dashboard/plan`, which runs `ardupilot_export.py` locally.
5. Review the returned summary and planner log. Download the ZIP containing the `.waypoints`, `.param`, KML preview, and JSON summary.

The dashboard does **not** upload missions over MAVLink or send arm/takeoff/RTL/payload commands. Telemetry, camera feeds, target detections, and payload controls still use illustrative demo data until separately connected to real sources. A successful file generation is not flight approval.

Test mode is restricted by the exporter to one or two drones. It changes planning constraints; it does not make a flight safe by itself. Check the generated KML, all warnings, altitude and coverage results, home position, fence, parameters, local airspace rules, and the full manual before any real flight.

## Existing planner tools

```powershell
py -m unittest test_medial -v
```

See `MANUAL.md` and `ARDUPILOT.md` for the detailed field-to-flight workflow and safety checks.

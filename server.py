"""Local-only, dependency-light server for the centerline playground."""

import argparse
import base64
import os
import re
import shutil
import subprocess
import sys
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import logging
from pathlib import Path
import threading
from urllib.parse import urlsplit, parse_qs

from medial_axis import Cancelled, analyze
from sweep import compare


ROOT = Path(__file__).resolve().parent
DASHBOARD_RUNS = ROOT / "dashboard_runs"
DASHBOARD_RUNS.mkdir(exist_ok=True)
COMPUTE_LOCK = threading.Lock()


class Handler(BaseHTTPRequestHandler):
    def send_data(self, status, payload, content_type="application/json"):
        data = payload if isinstance(payload, bytes) else json.dumps(payload, allow_nan=False).encode()
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        try:
            self.wfile.write(data)
        except (BrokenPipeError, ConnectionResetError):
            logging.info("Browser disconnected before receiving the response.")

    def do_GET(self):
        path = urlsplit(self.path).path
        if path == "/":
            self.send_data(200, (ROOT / "dashboard.html").read_bytes(), "text/html; charset=utf-8")
        elif path == "/lab":
            self.send_data(200, (ROOT / "index.html").read_bytes(), "text/html; charset=utf-8")
        elif path == "/api/dashboard/download":
            run_id = parse_qs(urlsplit(self.path).query).get("run", [""])[0]
            if not re.fullmatch(r"[a-f0-9]{32}", run_id):
                self.send_data(400, {"error": "Invalid run ID."})
                return
            archive = DASHBOARD_RUNS / run_id / "mission_bundle.zip"
            if not archive.is_file():
                self.send_data(404, {"error": "Mission bundle not found. Generate a plan first."})
                return
            self.send_data(200, archive.read_bytes(), "application/zip")
        elif path == "/compare":
            self.send_data(200, (ROOT / "compare.html").read_bytes(), "text/html; charset=utf-8")
        elif path == "/health":
            self.send_data(200, {"status": "ok"})
        else:
            self.send_data(404, {"error": "Not found."})


    def handle_dashboard_plan(self):
        origin = self.headers.get("Origin")
        expected = f"http://{self.headers.get('Host')}"
        if origin and origin != expected:
            self.send_data(403, {"error": "Cross-origin requests are not allowed."})
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
            if not 0 < length <= 12_000_000:
                raise ValueError("Request must be between 1 byte and 12 MB.")
            payload = json.loads(self.rfile.read(length))
            if not isinstance(payload, dict):
                raise ValueError("Request must be a JSON object.")
            filename = str(payload.get("filename", "field.kml"))
            suffix = Path(filename).suffix.lower()
            if suffix not in (".kml", ".kmz", ".poly"):
                raise ValueError("Boundary must be a .kml, .kmz, or .poly file.")
            raw = base64.b64decode(payload.get("contentBase64", ""), validate=True)
            if not raw or len(raw) > 8_000_000:
                raise ValueError("Boundary file must be under 8 MB.")
            cfg = payload.get("settings", {})
            if not isinstance(cfg, dict):
                raise ValueError("Settings must be an object.")
            drones = int(cfg.get("drones", 5))
            if not 1 <= drones <= 64:
                raise ValueError("Drone count must be between 1 and 64.")
            test_mode = bool(cfg.get("testMode", False))
            if test_mode and not 1 <= drones <= 2:
                raise ValueError("Test mode supports only 1 or 2 drones. Choose 1–2 drones or turn test mode off.")
            lon, lat = float(cfg.get("homeLon", 77.5946)), float(cfg.get("homeLat", 12.9716))
            if not (-180 <= lon <= 180 and -90 <= lat <= 90):
                raise ValueError("Home longitude/latitude are out of range.")
            hfov, vfov = float(cfg.get("hfov", 147)), float(cfg.get("vfov", 62.22))
            max_alt, rtl_height = float(cfg.get("maxAlt", 30)), float(cfg.get("rtlHeight", 80))
            speed, fence_buffer = float(cfg.get("speed", 3)), float(cfg.get("fenceBuffer", 5))
            planner = str(cfg.get("planner", "harmonic"))
            if planner not in ("harmonic", "lawnmower", "geodesic", "mat"):
                raise ValueError("Unknown planner.")
            if not (1 < hfov < 179 and 1 < vfov < 179):
                raise ValueError("Camera FOV must be between 1 and 179 degrees.")
            if not (1 <= max_alt <= 500 and 1 <= rtl_height <= 1000 and 0.1 <= speed <= 30 and 0 <= fence_buffer <= 1000):
                raise ValueError("Check altitude, speed, or fence-buffer values.")
        except (ValueError, TypeError, KeyError, json.JSONDecodeError) as error:
            self.send_data(400, {"error": str(error)})
            return

        run_id = uuid.uuid4().hex
        run_dir = DASHBOARD_RUNS / run_id
        input_dir = run_dir / "input"
        output_dir = run_dir / "missions"
        input_dir.mkdir(parents=True)
        output_dir.mkdir()
        input_path = input_dir / ("boundary" + suffix)
        input_path.write_bytes(raw)
        args = [
            sys.executable, str(ROOT / "ardupilot_export.py"), str(input_path),
            "--drones", str(drones), "--planner", planner,
            "--hfov", str(hfov), "--vfov", str(vfov),
            "--max-alt", str(max_alt), "--speed", str(speed),
            "--fence-buffer", str(fence_buffer), "--RTLheight", str(rtl_height),
            "--home", str(lon), str(lat), "--out", str(output_dir),
        ]
        if test_mode:
            args.append("--test")
        try:
            completed = subprocess.run(args, cwd=ROOT, capture_output=True, text=True, timeout=240)
            if completed.returncode != 0:
                message = (completed.stderr or completed.stdout or "Planner failed.")[-5000:]
                shutil.rmtree(run_dir, ignore_errors=True)
                self.send_data(422, {"error": message})
                return
            summary_path = next(output_dir.glob("*_summary.json"), None)
            if summary_path is None:
                raise RuntimeError("Planner finished without writing its summary JSON.")
            summary = json.loads(summary_path.read_text(encoding="utf-8"))
            archive_path = run_dir / "mission_bundle.zip"
            shutil.make_archive(str(archive_path.with_suffix("")), "zip", root_dir=output_dir)
            self.send_data(200, {
                "runId": run_id,
                "summary": summary,
                "files": sorted(p.name for p in output_dir.iterdir() if p.is_file()),
                "log": (completed.stdout or "")[-5000:],
                "downloadUrl": f"/api/dashboard/download?run={run_id}",
            })
        except subprocess.TimeoutExpired:
            shutil.rmtree(run_dir, ignore_errors=True)
            self.send_data(504, {"error": "Planning timed out after 240 seconds. Try a simpler field or lower grid complexity."})
        except Exception as error:
            logging.exception("Dashboard planning failed")
            shutil.rmtree(run_dir, ignore_errors=True)
            self.send_data(500, {"error": f"Could not complete mission export: {error}"})

    def do_POST(self):
        if self.path == "/api/dashboard/plan":
            self.handle_dashboard_plan()
            return
        if self.path not in ("/api/analyze", "/api/compare"):
            self.send_data(404, {"error": "Not found."})
            return
        origin = self.headers.get("Origin")
        expected = f"http://{self.headers.get('Host')}"
        if origin and origin != expected:
            self.send_data(403, {"error": "Cross-origin requests are not allowed."})
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
            if not 0 < length <= 100_000:
                raise ValueError("Request must be between 1 and 100000 bytes.")
            payload = json.loads(self.rfile.read(length))
            if not isinstance(payload, dict):
                raise ValueError("Request must be a JSON object.")
            if not COMPUTE_LOCK.acquire(blocking=False):
                self.send_data(503, {"error": "Another shape is being processed. Try again shortly."})
                return
        except (ValueError, TypeError) as error:
            self.send_data(400, {"error": str(error)})
            return
        # Stream newline-delimited JSON: progress events, then a result or error event.
        self.send_response(200)
        self.send_header("Content-Type", "application/x-ndjson")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()

        def emit(event):
            try:
                self.wfile.write(json.dumps(event, allow_nan=False).encode() + b"\n")
                self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError):
                raise Cancelled()

        try:
            report = lambda fraction, message: emit({"type": "progress", "fraction": fraction, "message": message})
            if self.path == "/api/compare":
                result = compare(payload.get("points"), payload.get("drones", 12),
                                 prune=payload.get("prune", .05), hfov=payload.get("hfov", 70.),
                                 vfov=payload.get("vfov", 55.), overlap=payload.get("overlap", .1),
                                 progress=report)
            else:
                result = analyze(payload.get("points"), payload.get("resolution", 240),
                                 payload.get("prune", .05), payload.get("extend", True), progress=report)
            emit({"type": "result", "data": result})
        except Cancelled:
            logging.info("Browser disconnected during computation.")
        except (ValueError, TypeError) as error:
            logging.warning("Rejected shape: %s", error)
            try:
                emit({"type": "error", "error": str(error)})
            except Cancelled:
                pass
        except Exception:
            logging.exception("Unexpected geometry processing error")
            try:
                emit({"type": "error", "error": "Unexpected geometry error. See the server log for details."})
            except Cancelled:
                pass
        finally:
            COMPUTE_LOCK.release()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=8765)
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")
    server = ThreadingHTTPServer(("127.0.0.1", args.port), Handler)
    print(f"Centerline Lab: http://127.0.0.1:{server.server_port}", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopping Centerline Lab.")
    finally:
        server.server_close()

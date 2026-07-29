#!/usr/bin/env python3
"""Small local web UI wrapping the rosbag -> LeRobotDataset pipeline, so it doesn't need 3
separate manual CLI invocations (bag_split_episodes.py -> bag_to_lerobot.py ->
check_episode_durations.py) per bag.

Tab 1 (convert): pick a rosbag path + sim/real, runs bag_split_episodes.py then
bag_to_lerobot.py against it (storage id auto-detected from .mcap/.db3 files in the bag dir;
segments.json written next to the bag).

Tab 2 (check): pick one or more converted LeRobotDataset paths (glob patterns like
bags/*_lerobot are expanded server-side) + min/max seconds, runs check_episode_durations.py and
shows the flagged episode ids.

Stdlib only (http.server) — no new dependency (Gradio/Flask/etc., which pull in a lot of
transitive packages) added to this already dependency-fragile Jetson environment. This just
orchestrates the 3 existing scripts as subprocesses, using the same Python interpreter this
server itself runs under — so launch it from the `lerobot` conda env, same as those scripts.

Usage:
    conda activate lerobot
    python3 ui_server.py [--host 0.0.0.0] [--port 8765]
    # then open http://<this-machine>:8765 in a browser
"""

import argparse
import glob
import json
import subprocess
import sys
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

SCRIPTS_DIR = Path(__file__).parent
STATIC_DIR = SCRIPTS_DIR / "ui"


def detect_storage_id(bag_path: Path) -> str:
    if list(bag_path.glob("*.mcap")):
        return "mcap"
    if list(bag_path.glob("*.db3")):
        return "sqlite3"
    raise ValueError(f"Could not detect storage id (no .mcap/.db3 file found directly under {bag_path})")


def run_script(*args: str) -> dict:
    proc = subprocess.run([sys.executable, *args], capture_output=True, text=True)
    return {"returncode": proc.returncode, "stdout": proc.stdout, "stderr": proc.stderr}


def convert_bag(bag_path_str: str, mode: str) -> dict:
    bag_path = Path(bag_path_str).expanduser().resolve()
    if not bag_path.is_dir():
        return {"ok": False, "error": f"Not a directory: {bag_path}"}
    if mode not in ("sim", "real"):
        return {"ok": False, "error": f"Invalid mode: {mode!r} (expected 'sim' or 'real')"}

    try:
        storage_id = detect_storage_id(bag_path)
    except ValueError as e:
        return {"ok": False, "error": str(e)}

    segments_path = bag_path / "segments.json"
    phases_config = SCRIPTS_DIR / "phases.yaml"
    topics_config = SCRIPTS_DIR / f"topics_{mode}.yaml"

    split_result = run_script(
        str(SCRIPTS_DIR / "bag_split_episodes.py"),
        str(bag_path),
        "--topic", "/vr_buttons",
        "--config", str(phases_config),
        "--storage-id", storage_id,
        "--out", str(segments_path),
    )
    if split_result["returncode"] != 0:
        return {"ok": False, "error": "bag_split_episodes.py failed", "storage_id": storage_id, "split": split_result}

    convert_result = run_script(
        str(SCRIPTS_DIR / "bag_to_lerobot.py"),
        str(bag_path),
        "--segments", str(segments_path),
        "--topics-config", str(topics_config),
        "--storage-id", storage_id,
        "--fps", "20",
    )
    return {
        "ok": convert_result["returncode"] == 0,
        "storage_id": storage_id,
        "segments_path": str(segments_path),
        "split": split_result,
        "convert": convert_result,
    }


def check_durations(dataset_paths: list, min_seconds: float, max_seconds: float) -> dict:
    expanded: list = []
    for p in dataset_paths:
        p = p.strip()
        if not p:
            continue
        matches = sorted(glob.glob(p))
        expanded.extend(matches if matches else [p])
    if not expanded:
        return {"ok": False, "error": "No dataset paths given."}

    out_path = Path("/tmp/flagged_episodes_ui.json")
    result = run_script(
        str(SCRIPTS_DIR / "check_episode_durations.py"),
        *expanded,
        "--min-seconds", str(min_seconds),
        "--max-seconds", str(max_seconds),
        "--out", str(out_path),
    )
    flagged = []
    if out_path.exists():
        with open(out_path) as f:
            flagged = json.load(f)
    return {"ok": result["returncode"] == 0, "datasets_scanned": expanded, "flagged": flagged, "log": result}


def delete_episodes(dataset_path_str: str, episode_indices: list) -> dict:
    root = Path(dataset_path_str).expanduser().resolve()
    if not root.is_dir():
        return {"ok": False, "error": f"Not a directory: {root}"}

    try:
        indices = sorted({int(i) for i in episode_indices})
    except (TypeError, ValueError):
        return {"ok": False, "error": f"Episode indices must be integers, got: {episode_indices}"}
    if not indices:
        return {"ok": False, "error": "No episode indices given."}

    repo_id = f"local/{root.name}"
    backup_path = root.with_name(root.name + "_old")
    proc = subprocess.run(
        [
            "lerobot-edit-dataset",
            "--repo_id", repo_id,
            "--root", str(root),
            "--new_repo_id", repo_id,
            "--new_root", str(root),
            "--operation.type", "delete_episodes",
            "--operation.episode_indices", json.dumps(indices),
        ],
        capture_output=True,
        text=True,
    )

    # lerobot-edit-dataset moves (not copies) the original to `<root>_old` before writing the
    # edited dataset back to `root` — if the edit fails partway (e.g. it refuses to delete every
    # episode), `root` is left missing until restored. Auto-restore here so a failed delete never
    # leaves the dataset silently absent.
    restored = False
    if proc.returncode != 0 and not root.exists() and backup_path.exists():
        backup_path.rename(root)
        restored = True

    return {
        "ok": proc.returncode == 0,
        "deleted": indices,
        "backup_path": str(backup_path),
        "restored_after_failure": restored,
        "returncode": proc.returncode,
        "stdout": proc.stdout,
        "stderr": proc.stderr,
    }


class Handler(BaseHTTPRequestHandler):
    def _send_json(self, payload: dict, status: int = 200) -> None:
        body = json.dumps(payload).encode()
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self) -> None:
        rel_path = "index.html" if self.path in ("/", "") else self.path.lstrip("/")
        file_path = (STATIC_DIR / rel_path).resolve()
        if STATIC_DIR.resolve() not in file_path.parents and file_path != STATIC_DIR.resolve():
            self.send_error(403)
            return
        if not file_path.is_file():
            self.send_error(404)
            return
        content_type = "text/html" if file_path.suffix == ".html" else "application/octet-stream"
        body = file_path.read_bytes()
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_POST(self) -> None:
        length = int(self.headers.get("Content-Length", 0))
        payload = json.loads(self.rfile.read(length) or b"{}")

        if self.path == "/api/convert":
            result = convert_bag(payload.get("bag_path", ""), payload.get("mode", ""))
        elif self.path == "/api/check":
            result = check_durations(
                payload.get("dataset_paths", []),
                float(payload.get("min_seconds", 30)),
                float(payload.get("max_seconds", 180)),
            )
        elif self.path == "/api/delete":
            result = delete_episodes(payload.get("dataset_path", ""), payload.get("episode_indices", []))
        else:
            self._send_json({"ok": False, "error": "Unknown endpoint"}, status=404)
            return

        self._send_json(result)

    def log_message(self, fmt: str, *args) -> None:
        print(f"[ui_server] {self.address_string()} - {fmt % args}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--host", default="0.0.0.0", help="0.0.0.0 to allow LAN access, 127.0.0.1 for local-only")
    parser.add_argument("--port", type=int, default=8765)
    args = parser.parse_args()

    server = ThreadingHTTPServer((args.host, args.port), Handler)
    print(f"Serving on http://{args.host}:{args.port} — open in a browser. Ctrl+C to stop.")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()

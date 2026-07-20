# Copyright 2026 Enactic, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Meta Quest UDP pose receiver — specification
==============================================

[1. Incoming JSON Structure]
- t:  headset monotonic timestamp (seconds, Time.realtimeSinceStartup)
- lc / rc / rf:  pose objects (left controller / right controller / reference)
    - x, y, z: Unity left-handed world coordinates (meters)
    - qx, qy, qz, qw: Unity left-handed rotation (Quaternion)
- lt / rt: left/right index trigger  0.0–1.0
- lg / rg: left/right grip           0.0–1.0
- lsx / lsy / rsx / rsy: thumbstick axes  -1.0–1.0
- a / b / x / y: buttons
- v:  overall validity   0=OK, 1=STALE, 2=INVALID
- vl: left controller validity
- vr: right controller validity

[2. Validity Handling]
- OK (0):     normal processing
- STALE (1):  HMD is sending last-good pose; pass through smoother normally
- INVALID(2): do not output pose; reset smoother so re-entry is jump-free
- buttons/triggers are always forwarded regardless of pose validity

[3. Coordinate Transformation (LH to RH)]
1. Position Flip:
    p_mujoco = [x, y, -z]
2. Quaternion Flip:
    q_mujoco = [qw, -qx, -qy, qz]
3. Reference Rectification
   A saved reference pose (p_ref, R_ref) is subtracted so that the
   controller pose is expressed relative to where the operator was
   standing/looking when the reference was captured.  Two modes differ
   in which frame the relative pose is expressed in:

   NECK mode  — relative position is rotated into the HMD's frame:
     p_rel = R_ref_inv * (p_ctrl - p_ref)   (displacement in HMD axes)
     r_rel = R_ref_inv * r_ctrl             (orientation relative to HMD)

[4. Robot Workspace Mapping]
- p_out = R_FRAME * p_rel + FRAME_OFFSET_NECK
- r_out = R_FRAME * r_rel * R_FIX
    * R_FIX = Rot_z(90)
"""

import argparse
import json
import time
from pathlib import Path

import dora
import numpy as np
import pyarrow as pa
from scipy.spatial.transform import Rotation

from .smoothing import OneEuroPoseSmoother
from .udp_receiver import JsonUdpReceiver

# Where the current run's reference calibration is written, for inspection —
# refreshed on every `dora run` (each run captures its own reference from that
# run's first packet; this file is a record of what was captured, not a
# cross-run calibration to reuse).
_REF_CACHE_PATH = Path.home() / ".cache" / "dora_openarm_vr" / "reference_pose.json"


def _save_reference(ref_raw: dict) -> None:
    _REF_CACHE_PATH.parent.mkdir(parents=True, exist_ok=True)
    with _REF_CACHE_PATH.open("w") as f:
        json.dump(ref_raw, f)

# ── Frame alignment — edit here to tune ──────────────────────────────────────
_FRAME_ROT: np.ndarray = np.array(
    [
        [0.0, 0.0, -1.0],
        [-1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
    ],
    dtype=np.float64,
)

FRAME_OFFSET_NECK: np.ndarray = np.array([0.1, 0, 1.2], dtype=np.float64)
# ─────────────────────────────────────────────────────────────────────────────

_DEFAULT_HOST = "0.0.0.0"
_DEFAULT_PORT = 5006

VALID_OK = 0
VALID_STALE = 1
VALID_INVALID = 2
_VALID_NAMES = {VALID_OK: "OK", VALID_STALE: "STALE", VALID_INVALID: "INVALID"}

_R_FRAME = Rotation.from_matrix(_FRAME_ROT)
_IDENTITY_REF = {
    "x": 0.0,
    "y": 0.0,
    "z": 0.0,
    "qx": 0.0,
    "qy": 0.0,
    "qz": 0.0,
    "qw": 1.0,
}


def parse_lh_to_rh(c: dict) -> tuple[np.ndarray, Rotation]:
    """Convert a Unity left-handed pose dict to a right-handed (position, Rotation) pair.

    Input keys: x, y, z (meters), qx, qy, qz, qw (Unity quaternion, scalar-last).
    Flip: z → -z, qx → -qx, qy → -qy.
    """
    pos = np.array([c["x"], c["y"], -c["z"]], dtype=np.float64)
    rot = Rotation.from_quat([-c["qx"], -c["qy"], c["qz"], c["qw"]])
    return pos, rot


class QuestPoseProcessor:
    def __init__(self) -> None:
        # Reference pose is captured once (from this run's first packet) and
        # frozen for the life of this process, so head rotation after that
        # point does not affect the output — only controller motion relative
        # to that one snapshot does. Not re-captured on re-engagement: the
        # headset isn't worn continuously (it's hung around the neck between
        # sessions), so its pose at re-engage time is not a usable calibration
        # point. Freshly captured on every `dora run` (not loaded from a prior
        # run) — see _REF_CACHE_PATH.
        self._ref_p: np.ndarray | None = None
        self._ref_r_inv: Rotation | None = None

    def process(self, msg: dict) -> tuple[np.ndarray | None, np.ndarray | None]:
        right_raw = msg.get("rc")
        left_raw = msg.get("lc")

        if self._ref_p is None:
            ref_raw = msg.get("rf")
            if ref_raw is not None:
                _save_reference(ref_raw)
            else:
                ref_raw = _IDENTITY_REF
            p_ref, r_ref = parse_lh_to_rh(ref_raw)
            self._ref_p = p_ref
            self._ref_r_inv = r_ref.inv()

        active_p_ref = self._ref_p
        active_r_ref_inv = self._ref_r_inv

        r_fix = Rotation.from_euler("z", 90, degrees=True)

        def _rectify(raw: dict) -> np.ndarray:
            p, r = parse_lh_to_rh(raw)
            p_rel = active_r_ref_inv.apply(p - active_p_ref)
            r_rel = active_r_ref_inv * r
            p_out = _R_FRAME.apply(p_rel) + FRAME_OFFSET_NECK
            r_out = _R_FRAME * r_rel * r_fix
            q = r_out.as_quat()
            return np.array(
                [p_out[0], p_out[1], p_out[2], q[3], q[0], q[1], q[2]], dtype=np.float32
            )

        pose_right = _rectify(right_raw) if right_raw is not None else None
        pose_left = _rectify(left_raw) if left_raw is not None else None
        return pose_right, pose_left


def _run(args: argparse.Namespace) -> None:
    receiver = JsonUdpReceiver(args.host, args.port)
    processor = QuestPoseProcessor()

    smoother_right = OneEuroPoseSmoother(min_cutoff=2.0, beta=0.04, d_cutoff=1.5)
    smoother_left = OneEuroPoseSmoother(min_cutoff=2.0, beta=0.04, d_cutoff=1.5)

    prev_v_right = VALID_OK
    prev_v_left = VALID_OK
    prev_v_overall = VALID_OK
    prev_right_engaged = False
    prev_left_engaged = False
    last_sent_pose_right: np.ndarray | None = None
    last_sent_pose_left: np.ndarray | None = None

    node = dora.Node()
    node.send_output("status", pa.array(["ready"]))

    for event in node:
        if event["type"] != "INPUT" or event["id"] != "tick":
            continue

        msg = receiver.latest()
        if msg is None:
            continue
        now = time.perf_counter()

        v_overall = int(msg["v"]) if "v" in msg else VALID_OK
        v_right = int(msg["vr"]) if "vr" in msg else VALID_OK
        v_left = int(msg["vl"]) if "vl" in msg else VALID_OK

        if v_overall != prev_v_overall:
            print(
                f"[receiver] validity: {_VALID_NAMES[prev_v_overall]} → {_VALID_NAMES[v_overall]} "
                f"(L={_VALID_NAMES[v_left]}, R={_VALID_NAMES[v_right]})"
            )
            prev_v_overall = v_overall

        pose_right_raw, pose_left_raw = processor.process(msg)

        # The dead-man grip freezes the *target* below rather than gating
        # publishing entirely — pose_right/pose_left are sent every tick once
        # a first grip has been seen, so downstream (ik → dora-to-ros2 →
        # ROS2 command topics) always sees a continuous command stream for
        # data recording, even while the grip is released. Released ticks
        # simply re-send the last pose captured while engaged instead of the
        # live (possibly still-moving) controller pose, so the arm holds
        # still rather than following the controller when you let go.
        #
        # The smoothers keep consuming raw pose every tick regardless of
        # grip, so while released and the controller sits still (e.g. placed
        # on a table), the filter fully converges to that resting pose.
        # Without a reset on re-engagement, the first pose(s) sent after
        # re-gripping would still be anchored near that stale resting pose
        # instead of passing through the controller's current (raw)
        # position, producing a jump.
        right_grip = float(msg.get("rg", 0.0))
        left_grip = float(msg.get("lg", 0.0))
        right_engaged = right_grip > 0.5
        left_engaged = left_grip > 0.5

        right_rising = right_engaged and not prev_right_engaged
        left_rising = left_engaged and not prev_left_engaged

        if right_rising:
            smoother_right.reset()
        if left_rising:
            smoother_left.reset()

        if v_right == VALID_INVALID:
            if prev_v_right != VALID_INVALID:
                smoother_right.reset()
            pose_right = None
        else:
            pose_right = smoother_right.smooth(now, pose_right_raw)

        if v_left == VALID_INVALID:
            if prev_v_left != VALID_INVALID:
                smoother_left.reset()
            pose_left = None
        else:
            pose_left = smoother_left.smooth(now, pose_left_raw)

        prev_v_right = v_right
        prev_v_left = v_left
        prev_right_engaged = right_engaged
        prev_left_engaged = left_engaged

        ts = {"timestamp": time.time_ns()}

        if right_engaged and pose_right is not None:
            last_sent_pose_right = pose_right
        if last_sent_pose_right is not None:
            node.send_output(
                "pose_right",
                pa.array(last_sent_pose_right, type=pa.float32()),
                ts,
            )

        if left_engaged and pose_left is not None:
            last_sent_pose_left = pose_left
        if last_sent_pose_left is not None:
            node.send_output(
                "pose_left",
                pa.array(last_sent_pose_left, type=pa.float32()),
                ts,
            )

        if "rt" in msg:
            node.send_output(
                "trigger_right", pa.array([msg["rt"]], type=pa.float32()), ts
            )
        if "lt" in msg:
            node.send_output(
                "trigger_left", pa.array([msg["lt"]], type=pa.float32()), ts
            )
        if "lsy" in msg:
            node.send_output(
                "joystick_y", pa.array([float(msg["lsy"])], type=pa.float32()), ts
            )
        # Thumbstick axes (-1..1) per controller, used to drive each hand's thumb.
        if "lsx" in msg:
            node.send_output(
                "joystick_x_left", pa.array([float(msg["lsx"])], type=pa.float32()), ts
            )
        if "lsy" in msg:
            node.send_output(
                "joystick_y_left", pa.array([float(msg["lsy"])], type=pa.float32()), ts
            )
        if "rsx" in msg:
            node.send_output(
                "joystick_x_right", pa.array([float(msg["rsx"])], type=pa.float32()), ts
            )
        if "rsy" in msg:
            node.send_output(
                "joystick_y_right", pa.array([float(msg["rsy"])], type=pa.float32()), ts
            )
        if "a" in msg:
            node.send_output(
                "button_a", pa.array([bool(msg["a"])], type=pa.bool_()), ts
            )
        if "b" in msg:
            node.send_output(
                "button_b", pa.array([bool(msg["b"])], type=pa.bool_()), ts
            )
        if "x" in msg:
            node.send_output(
                "button_x", pa.array([bool(msg["x"])], type=pa.bool_()), ts
            )
        if "y" in msg:
            node.send_output(
                "button_y", pa.array([bool(msg["y"])], type=pa.bool_()), ts
            )

    receiver.close()


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Meta Quest VR pose receiver (dora node)"
    )
    parser.add_argument("--host", default=_DEFAULT_HOST)
    parser.add_argument("--port", type=int, default=_DEFAULT_PORT)
    args = parser.parse_args()
    _run(args)


if __name__ == "__main__":
    main()

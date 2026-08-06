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

"""Dora node: mink-based differential IK solver for OpenArm.

Accepts end-effector pose targets and solves joint angles via mink's QP-based
differential IK. Both arms share one mink.Configuration and one QP solve per
step.

Pose convention (inputs and outputs):  float32[7] = [px, py, pz, qw, qx, qy, qz]
Inputs:
  target_right – float32[7]  right EE target pose
  target_left  – float32[7]  left  EE target pose
  position     – float32[16] current joint state right[8]+left[8] (optional sync)

Outputs:
  position_right – float32[8] solved right arm joint angles
  position_left  – float32[8] solved left arm joint angles
  status         – ["ready"] on startup
"""

from __future__ import annotations

import argparse
import math
import time

import dora
import numpy as np
import pyarrow as pa

from openarm_control import (
    Kinematics,
    register_common_args,
    register_ik_args,
    ik_params_from_args,
    setup_from_args,
)

# Local (wrist-frame) tilt applied to every incoming target before solving — a
# site-specific patch, not part of upstream dora-openarm-kinematics.
_Y_TILT_DEG = 25.0
_Y_TILT_HALF_RAD = math.radians(_Y_TILT_DEG) / 2.0
_Y_TILT_QUAT = np.array(
    [math.cos(_Y_TILT_HALF_RAD), 0.0, math.sin(_Y_TILT_HALF_RAD), 0.0], dtype=np.float32
)  # (w, x, y, z), matches this node's [px, py, pz, qw, qx, qy, qz] pose convention


def _quat_mul(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """Hamilton product, (w, x, y, z) order."""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array(
        [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ],
        dtype=np.float32,
    )


def _apply_y_tilt(pose: np.ndarray) -> np.ndarray:
    """Right-multiplies pose's orientation by _Y_TILT_QUAT to rotate +25° about its own local Y axis."""
    tilted = pose.copy()
    tilted[3:7] = _quat_mul(pose[3:7], _Y_TILT_QUAT)
    return tilted


def _map_trigger_to_gripper(trigger: float) -> float:
    """trigger 0.0~1.0 → gripper joint position.

    Both openarm_{left,right}_finger_joint1 share the same non-negative
    [0.0, 0.044] slide range (see openarm_mujoco/v1/openarm_bimanual.xml),
    so left and right use the identical mapping: 0→1.57/2, 1→0.
    """
    return (1.57 / 2.0) * (1.0 - trigger)


def _run(args: argparse.Namespace) -> None:
    kin = Kinematics(setup_from_args(args), ik_params_from_args(args))

    node = dora.Node()
    node.send_output("status", pa.array(["ready"]))

    for event in node:
        if event["type"] != "INPUT":
            continue

        eid = event["id"]
        values = np.array(event["value"], dtype=np.float32)

        if eid == "position":
            if values.shape == (16,):
                kin.sync(values)
            continue

        if eid == "target_right" and "right" in kin.setup.sides:
            if values.shape != (7,):
                print(
                    f"Warning: expected target_right[7], got {values.shape}. Skipping."
                )
                continue
            kin.set_target("right", _apply_y_tilt(values))

        elif eid == "target_left" and "left" in kin.setup.sides:
            if values.shape != (7,):
                print(
                    f"Warning: expected target_left[7], got {values.shape}. Skipping."
                )
                continue
            kin.set_target("left", _apply_y_tilt(values))

        elif eid == "trigger_right":
            kin.set_gripper("right", _map_trigger_to_gripper(float(values[0])))

        elif eid == "trigger_left":
            kin.set_gripper("left", _map_trigger_to_gripper(float(values[0])))

        else:
            continue

        if not kin.ready():
            continue

        result = kin.solve()
        if result is None:
            continue

        ts = {"timestamp": time.time_ns()}
        node.send_output("position_right", pa.array(result[:8], type=pa.float32()), ts)
        node.send_output("position_left", pa.array(result[8:16], type=pa.float32()), ts)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Mink IK dora node – OpenArm end-effector pose → joint angles"
    )
    register_common_args(parser)
    register_ik_args(parser)
    args = parser.parse_args()
    _run(args)


if __name__ == "__main__":
    main()

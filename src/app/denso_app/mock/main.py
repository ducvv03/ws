"""Wire the three halves together and run them — MOCK SERVER.

The server is a pipe, not a brain: it copies ROS topics into a map, stamps
them, and pushes the map. Every judgement — what is hot, what is stale, what
a status nibble means, which panel a joint belongs in — happens in the
browser. The one exception is the enable interlock, which is safety and so
cannot live in a page.

Stand-in for the C++ server to come. Its job is to make the frontend
testable today and to be a readable reference for what the real one must do;
the binding spec is ``API.md``, not this file.

This is the only module that imports both rclpy and the HTTP layer. Everything
else stays on one side of that line: ``telemetry/`` and ``control/`` are pure
ROS, ``web/`` is pure HTTP, and ``model/`` is neither.

Threading: rclpy's executor and uvicorn's asyncio loop both want to own a
thread, so ROS gets a daemon thread and uvicorn keeps the main one. Ctrl-C
then lands where you expect it.
"""

from __future__ import annotations

import argparse
import logging
import threading
from pathlib import Path

import rclpy
import uvicorn
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import MultiThreadedExecutor

from mock.control.arm_power import ArmPower
from mock.telemetry.collector import TelemetryCollector
from mock.telemetry.registry import JointRegistry
from mock.http_api.server import create_app

log = logging.getLogger('denso_app')


def frontend_dir(explicit: str | None) -> Path:
    """Where the static frontend lives.

    Installed next to config/ under share/. Falls back to the source tree so
    the mock can be run straight out of src/ while editing the page.
    """
    if explicit:
        return Path(explicit)
    installed = Path(get_package_share_directory('denso_app')) / 'frontend'
    if (installed / 'index.html').exists():
        return installed
    return Path(__file__).resolve().parents[1] / 'frontend'


def load_config(path: str | None) -> dict:
    """Config sits inside the package, not in share/.

    It configures this mock and nothing else — the frontend has its own
    config.js, and a future C++ server will have its own. Keeping it here
    means it leaves with the mock.
    """
    if path is None:
        path = Path(__file__).parent / 'config.yaml'
    with open(path) as fh:
        return yaml.safe_load(fh)


def main(argv: list[str] | None = None) -> None:
    ap = argparse.ArgumentParser(prog='denso_app web console')
    ap.add_argument('--config', default=None, help='path to web.yaml')
    ap.add_argument('--port', type=int, default=None)
    ap.add_argument('--frontend', default=None,
                    help='serve the page from this directory instead')
    args, ros_argv = ap.parse_known_args(argv)

    logging.basicConfig(level=logging.INFO,
                        format='[%(levelname)s] [%(name)s] %(message)s')
    cfg = load_config(args.config)

    registry = JointRegistry()

    rclpy.init(args=ros_argv)
    collector = TelemetryCollector(registry, cfg.get('watch_loggers') or [])
    arm_power = ArmPower(cfg['arms'])

    executor = MultiThreadedExecutor()
    executor.add_node(collector)
    executor.add_node(arm_power)
    ros_thread = threading.Thread(target=executor.spin, daemon=True,
                                  name='rclpy-executor')
    ros_thread.start()

    def command(side: str, action: str) -> tuple[bool, int, str]:
        """The seam web/ is given. It never sees a ROS type."""
        joints = cfg['arms'].get(side, {}).get('joints', [])
        result = arm_power.apply(side, action, registry.group_has_fault(joints))
        if not result.ok:
            log.warning('%s %s refused (%d): %s',
                        side, action, result.status, result.detail)
        return result.ok, result.status, result.detail

    static = frontend_dir(args.frontend)
    log.info('serving frontend from %s', static)
    app = create_app(registry, command, frontend_dir=static,
                     stream_hz=cfg.get('stream_hz', 10.0))

    port = args.port or cfg.get('port', 8080)
    log.info('mock server on http://%s:%d', cfg.get('host', '0.0.0.0'), port)
    try:
        uvicorn.run(app, host=cfg.get('host', '0.0.0.0'), port=port,
                    log_level='warning')
    finally:
        executor.shutdown()
        collector.destroy_node()
        arm_power.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

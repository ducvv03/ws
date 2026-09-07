"""HTTP + websocket surface for the mock server.

This is a stand-in. The real server is to be written in C++; what has to
survive that rewrite is not this file but ``API.md`` next to it — the wire
contract both implementations answer to. Treat any behaviour here that
``API.md`` does not describe as an accident, not a spec.

Nothing in this folder imports rclpy. The layer is handed two plain Python
objects — something with ``.snapshot()`` and something callable for commands
— so it can be run and poked at with curl on a laptop with no robot and no
ROS installed. ``main.py`` is the only file that knows both worlds.

Two transports, on purpose, one port:

  WS   /ws/motors            telemetry going down, pushed, ~10 Hz
  POST /arm/{side}/{action}  commands going up, answered with a status code

Telemetry is a stream nobody asks for twice a second, so it is a websocket.
A command has to say whether it worked and why not — which is what an HTTP
status code is — and must never be replayed by a reconnecting socket, so it
is an ordinary POST.
"""

from __future__ import annotations

import asyncio
import logging
from pathlib import Path
from typing import Callable, Protocol

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
from fastapi.staticfiles import StaticFiles

log = logging.getLogger(__name__)


class Snapshotter(Protocol):
    def snapshot(self) -> dict: ...
    def group_has_fault(self, joints: list[str]) -> bool: ...


#: (side, action) -> (ok, http_status, detail). Supplied by main.py; in tests
#: a lambda returning (True, 200, 'ok') is a complete stand-in.
CommandFn = Callable[[str, str], tuple[bool, int, str]]


def create_app(registry: Snapshotter, command: CommandFn,
               frontend_dir: Path, stream_hz: float = 10.0) -> FastAPI:
    """``frontend_dir`` is passed in, never derived from __file__.

    The frontend is a sibling folder, not part of this package — it is meant
    to be served by whatever ends up hosting it, this mock or the eventual
    C++ binary or a plain nginx.
    """
    app = FastAPI(title='denso_app mock server', docs_url='/api')
    period = 1.0 / max(stream_hz, 1.0)

    # The frontend may be served from somewhere else entirely during
    # development — a file:// page, or `python3 -m http.server` on another
    # port. Without this the browser blocks the POST and the buttons go
    # dead with no visible reason.
    app.add_middleware(
        CORSMiddleware, allow_origins=['*'], allow_methods=['*'],
        allow_headers=['*'],
    )

    @app.get('/api/motors')
    async def motors_once() -> JSONResponse:
        """One frame over plain HTTP.

        Not how the page reads telemetry — that is the websocket below — but
        it makes the same data curl-able, which is worth a lot when the
        question is 'is the robot publishing, or is my page broken?'.
        """
        return JSONResponse(registry.snapshot())

    @app.websocket('/ws/motors')
    async def motors_stream(ws: WebSocket) -> None:
        await ws.accept()
        try:
            while True:
                await ws.send_json(registry.snapshot())
                await asyncio.sleep(period)
        except WebSocketDisconnect:
            pass
        except Exception:                      # noqa: BLE001 - one client dying
            log.exception('telemetry socket closed unexpectedly')

    @app.post('/arm/{side}/{action}')
    async def arm_power(side: str, action: str) -> JSONResponse:
        if action not in ('enable', 'disable'):
            return JSONResponse({'detail': f'unknown action {action!r}'},
                                status_code=400)

        # Blocking service call — hand it to a worker thread so one slow
        # controller_manager cannot stall the telemetry stream.
        ok, status, detail = await asyncio.to_thread(command, side, action)
        return JSONResponse({'ok': ok, 'detail': detail}, status_code=status)

    # Mounted last and at the root, so the API routes above win the match
    # and everything else falls through to the static frontend. html=True
    # makes "/" serve index.html.
    app.mount('/', StaticFiles(directory=frontend_dir, html=True),
              name='frontend')
    return app

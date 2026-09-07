# denso_app web console — wire contract

The frontend in `frontend/` talks to a server over this contract and nothing
else. `denso_app/mock_server/` is one implementation, in Python, for testing
and as a reference; a C++ implementation is planned. **This file is the spec
— where an implementation and this document disagree, this document wins.**

## The server does not interpret

It copies values out of ROS, stamps them, and pushes them. It does not know
what a temperature is, what counts as hot, when a joint is stale, what a
status nibble means, or which panel a joint belongs to. All of that is in
`frontend/config.js` and `frontend/app.js`.

So a server implementation is, in full:

1. subscribe `/dynamic_joint_states`, record each joint's interfaces **under
   the names ros2_control used** plus the instant they arrived;
2. subscribe `/rosout`, keep the last 40 lines at WARN and above;
3. push that map as JSON, ~10 Hz;
4. accept four POSTs and call `controller_manager`;
5. **one safety check** — see the 409 below. The only judgement the server
   is allowed, and the only one it must never delegate.

Everything else is presentation, and presentation belongs where it can be
changed with a browser refresh.

---

## Transports, and why there are two

| | Telemetry | Commands |
|---|---|---|
| Direction | server → browser | browser → server |
| Transport | WebSocket | HTTP POST |
| Rate | ~10 Hz, pushed | one per button press |

Telemetry is a stream nobody should have to ask for twice a second. Polling
it over HTTP costs a fresh connection every time and adds a poll period of
latency to every fault.

A command must answer two questions — did it work, and if not why — which is
what an HTTP status code is, and it must never be replayed. A WebSocket that
silently reconnects can flush a queued message at a moment nobody chose; for
`enable` that means torque returning to seven joints unbidden.

---

## `WS /ws/motors` — telemetry

One JSON object per frame, server → browser. The browser sends nothing; a
server may ignore anything that arrives on this socket.

```jsonc
{
  "t": 91422.611,              // server monotonic clock, seconds
  "wall": 1757238400.117,      // wall clock, for display only
  "rates": { "dynamic_joint_states": 49.8 },
  "joints": {
    "openarm_left_joint4": {
      "stamp": 91422.607,      // same clock as "t"
      "position": -0.412,      // interface names, VERBATIM from ros2_control
      "velocity":  0.031,
      "effort":    2.84
    },
    "left_thumb_proximal_joint": {
      "stamp": 91422.601,
      "position": 0.22,
      "effort": 0.4
    }
  },
  "log": [
    { "stamp": 1757238399.9, "level": 40,
      "name": "openarm_hardware", "msg": "..." }
  ]
}
```

### Three rules an implementation must not get wrong

**1. Forward interface names verbatim. Never invent one, never drop one.**
Whatever string the hardware plugin passed to `StateInterface()` is the key.
`position`, `velocity`, `effort` today; `temperature_mos`, `status` when
`openarm_hardware` starts exporting them. A name the server has never heard
of is still a number the robot published — pass it on. The page maps names it
knows via `config.js` and shows the rest in the detail drawer.

Corollary: **a joint that reports no temperature has no temperature key.**
Do not send `0`. The page drops the column when the key is absent and would
otherwise draw a gauge reading 0 °C, which reads as a measurement rather
than a gap.

**2. `stamp` is arrival time, on the same clock as `t`.** Take it when the
message arrives, not from the header inside it — a node that died mid-publish
leaves a perfectly plausible header stamp behind, and staleness is exactly
the case we need to catch. Both fields come from one monotonic clock so the
browser subtracts two server numbers and never involves its own clock.

The browser carries `t` forward with its own elapsed time between frames, so
ages keep growing when the link drops instead of freezing. A server that
sends wall-clock in one field and monotonic in the other breaks this
silently.

**3. A joint the server has never seen is absent from `joints`.** Do not
emit a placeholder entry with `stamp: 0` — the page lists every joint it
expects from `config.js` and shows the missing ones as `NO DATA` on its own.

### `log[]`

Newest first, capped at 40. `level` is the raw rcl integer (20 INFO, 30 WARN,
40 ERROR, 50 FATAL); send WARN and above. The page picks the colour. `msg` is
whatever a node logged — the page HTML-escapes it, and a server must not
assume otherwise or put markup there.

---

## `POST /arm/{side}/{action}` — commands

`side` ∈ `left` `right` · `action` ∈ `enable` `disable`. No request body.

Response: `{"ok": bool, "detail": string}`. `detail` is shown to the operator
verbatim when the call fails, so write it for a person.

| status | meaning |
|---|---|
| `200` | done |
| `400` | unknown action |
| `404` | unknown side |
| `409` | **refused: a joint on that arm has a latched fault** |
| `500` | controller_manager refused, or a component would not switch |
| `503` | controller_manager not available |

### The 409 is the server's one job

A motor that tripped on temperature reverted to `DISABLED` and kept the
reason in its latched fault. Enabling straight over that puts torque back
into a winding that just overheated.

The server refuses when any joint on that arm reports a `status` nibble
`>= 0x8`. It needs the joint list for that arm — hence `arms.<side>.joints`
in `config/web.yaml`, the one place joint names appear server-side.

**The frontend does not check this and must not be trusted to.** It is a page
the operator can edit in DevTools. Anything that keeps torque off a hot motor
lives behind the HTTP boundary.

### What the server does on the ROS side

Both directions go through `controller_manager`, never at the CAN bus
directly — otherwise the controllers still believe they own those joints and
the next write fights whatever was done behind their back.

```
/controller_manager/switch_controller               # activate / deactivate
/controller_manager/set_hardware_component_state    # optional harder stop
```

**The two directions are not mirror images.** Getting this symmetrical is the
easy mistake, and it produces a Disable button that answers `200` while the
arm keeps moving.

| | controllers | strictness |
|---|---|---|
| `disable` | **all** that can command the arm | `BEST_EFFORT` |
| `enable` | **one**, the configured default | `STRICT` |

`pnk_bringup` spawns a trajectory controller *and* a forward position
controller for each arm, and VR teleop drives the second. Stopping only the
first leaves the arm live. Since just one of them holds the command
interfaces at any moment, the others are already inactive — hence
`BEST_EFFORT`, or the call fails on controllers that were never running.

Enable takes one: they contend for the same command interfaces, so asking
for several is refused anyway, and bringing an arm back up under a controller
nobody chose is its own hazard. `STRICT`, because "the arm is live now" must
not be reported on a guess.

---

## `GET /api/motors`

One telemetry frame, same schema, over plain HTTP. The page does not use it;
it exists so `curl` can answer "is the robot publishing, or is my page
broken?" without a browser.

## Static files and CORS

The server may serve `frontend/` as static files, with `/` returning
`index.html`. It may equally serve nothing — host the folder anywhere and set
`window.DENSO_API` in `config.js` to the API's origin. In that case the
server must answer CORS preflight and send `Access-Control-Allow-Origin`,
`-Methods`, `-Headers`, or the browser blocks the POSTs and the buttons go
dead with nothing on screen to explain why.

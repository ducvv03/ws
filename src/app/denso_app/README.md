# denso_app

Web console for the PNK bimanual robot: motor telemetry in a browser, and
enable/disable for each arm.

```
frontend/                    the page — and all the judgement
denso_app/mock_server/  a pipe, to be replaced by a C++ server
API.md                       the wire contract between them
```

The package is named for the part that is temporary. `frontend/` and
`API.md` are not: when the C++ server lands, those two move over to it and
this package goes away.

`API.md` is the load-bearing document. The two sides know nothing about each
other beyond it, which is what makes the server replaceable.

## The split

**The server interprets nothing.** It copies values out of ROS under the
names ros2_control used, stamps them, and pushes them. It does not know what
a temperature is, what counts as hot, when a joint is stale, what a status
nibble means, or which panel a joint belongs to.

**The frontend does all of it** — interface-name mapping, staleness,
thresholds, severity, layout, the status table. They live in `config.js`, so
changing a threshold or adding a joint is an edit and a browser refresh, with
no rebuild and nothing restarted on the robot.

One exception, and it is deliberate: **the enable interlock stays server-side**
(HTTP 409). A page the operator can edit in DevTools must not be the thing
deciding whether torque goes back onto a motor that just overheated.

## Layout

```
frontend/
  index.html                shell — panels and buttons are built at runtime
  config.js                 groups, thresholds, interface map, status names
  app.js                    decode -> severity -> render; the websocket; commands
  style.css

denso_app/mock_server/
  main.py                   the only file importing both ROS and HTTP
  telemetry/collector.py    ROS in:  /dynamic_joint_states, /rosout
  telemetry/registry.py     store + stamp + the 409 interlock
  control/arm_power.py      ROS out: controller_manager service calls
  http_api/server.py        websocket + REST, imports no ROS

config/web.yaml             host, port, rate, arm -> controllers
```

Layering rule: `http_api/` never imports rclpy, `telemetry/` and `control/`
never import fastapi, only `main.py` sees both sides.

## Running the mock

```bash
sudo apt install python3-fastapi python3-uvicorn python3-websockets
colcon build --packages-select denso_app
source install/setup.bash
ros2 run denso_app mock_server
```

Open `http://<robot-ip>:8080`. Editing the page needs no rebuild:

```bash
ros2 run denso_app mock_server --frontend src/app/denso_app/frontend
```

To develop the page against a robot elsewhere, serve `frontend/` however you
like and set `window.DENSO_API` in `config.js` to the robot's URL.

## Where the numbers come from

`/dynamic_joint_states`, not `/joint_states`. The latter is fixed at
position, velocity and effort; the former carries *every* state interface the
hardware plugin exported, by name, and the server forwards those names
untouched.

So the temperature and status columns are **empty today, honestly**. The
values exist — `get_state_tmos()`, `get_state_trotor()`, `get_status()`,
`get_last_fault()` in `lib/openarm_can` — but `export_state_interfaces()` in
`openarm_hardware` and `head_hardware` returns only position, velocity and
effort, so they stop there.

Three lines per interface in the hardware plugin and the columns appear, with
no change to this package:

```cpp
state_interfaces.emplace_back(hardware_interface::StateInterface(
    joint_names_[i], "temperature_mos", &tmos_states_[i]));
```

Anything the page's `config.js` does not name still reaches the detail drawer.

The fault feed reads `/rosout` at WARN and above, so it works today with no
robot-side change at all.

## Safety

Disable is one click, enable takes two. Disabling makes the arm limp;
enabling puts torque back on seven joints that may have hands on them, so the
dangerous direction gets friction and the safe one does not.

`enable` is refused with `409` while any joint on that arm holds a latched
fault. That check is in the server. The frontend does not do it and must not
be trusted to.

**None of this is an e-stop.** It travels over wifi, through a browser, into
Python. The physical button stays the thing with authority.

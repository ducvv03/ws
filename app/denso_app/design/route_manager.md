# RouteManager — Design

## Responsibility

`RouteManager` owns the **node graph** loaded from `mission.yaml` and walks the
**mission path** over it. It does three things:

1. Load & hold the graph (`name -> Node`) and the route (`mission.path`).
2. Give node access + route validation.
3. Feed the route one node at a time via a cursor (`next()` / `arrived()`).

It does **not** command the robot or run actions — that is the caller's / state
machine's job. `RouteManager` only says *where to go next* and *whether a move
is legal*.

---

## Data it works with

```cpp
enum class NodeType { WAYPOINT, PICK, DROP, HOLD, SCAN, APPROACH, UNKNOWN };

struct Node {
    std::string name;                 // node id
    NodeType    type;                 // semantic kind (maps to an action)
    std::vector<std::string> routes;  // other nodes reachable FROM this node
};
```

(`Node` / `NodeType` live in `node.hpp`; `RouteManager` includes it.)

---

## Public API

**Construction**

| Method | Description |
|--------|-------------|
| `RouteManager(yaml_path)` | Load & parse the config (nodes + `mission.path`). Throws `std::runtime_error` if the file can't be read/parsed. |

**Node access**

| Method | Description |
|--------|-------------|
| `hasNode(name)` | True if a node with this exact name exists. |
| `getNode(name)` | Return the node by name. Throws if it does not exist. |
| `nodeCount()` | How many nodes were loaded. |

**Routing**

| Method | Description |
|--------|-------------|
| `missionPath()` | The route declared under `mission.path`. |
| `validate(path, err)` | Check a whole route up-front (all nodes exist + every consecutive pair is a legal edge). Returns false and fills `err` on failure. |
| `canGo(a, b)` | True if a **single** move `a -> b` is allowed (`b` is in a's `routes`). |

**Cursor over `mission.path`**

| Method | Description |
|--------|-------------|
| `hasNext()` | True while a node is still left to visit. Read-only. |
| `next()` | **PEEK** the next target node (returns a `Step`). Never advances, never throws — keeps returning the same target until `arrived()`. |
| `arrived()` | Confirm the robot **physically reached** the target. Only now the cursor advances. Call once per arrival. |
| `current()` | The node the robot is currently standing at (last arrived). `nullptr` before the first arrival. |
| `reset()` | Restart the route from the beginning. |

---

## Key decision: `next()` peeks, `arrived()` advances

Getting the next target and confirming the robot got there are **separate**.

```cpp
while (true) {
    Step s = rm.next();                 // 1. PEEK target (cursor stays put)
    if (s.status == StepStatus::FINISHED) break;
    if (s.status == StepStatus::BLOCKED)  { /* illegal step */ break; }

    move_to(*s.node);                   // 2. command robot, WAIT until arrived
    rm.arrived();                       // 3. arrived -> cursor advances
}
```

**Why:** if `next()` advanced the cursor by itself, the cursor would run ahead
of the physical robot — asking "what's next" would be treated as "already
there". With the split:

- calling `next()` repeatedly while moving returns the **same** target,
- `current()` keeps reporting the node the robot **actually** stands at,
- the cursor is written in **exactly one place**: `arrived()`.

---

## `next()` result — no exceptions on the normal path

```cpp
enum class StepStatus { OK, FINISHED, BLOCKED };

struct Step {
    StepStatus  status;   // OK | FINISHED | BLOCKED
    const Node* node;     // valid only when status == OK, else nullptr
};
```

| `status`   | Meaning                             | `node`    |
|------------|-------------------------------------|-----------|
| `OK`       | Got a node, keep moving             | valid ptr |
| `FINISHED` | End of route (normal, not an error) | `nullptr` |
| `BLOCKED`  | Next step illegal / node missing    | `nullptr` |

End-of-route is normal, so it is a status — not a thrown exception. `next()`
also **validates the edge at pop time**: even without an up-front `validate()`,
an illegal step is caught (`BLOCKED`) instead of driving the robot wrong.

---

## Cursor state (two explicit indices)

| Member         | Meaning                                       |
|----------------|-----------------------------------------------|
| `cursor_`      | index of the **next** node `next()` will peek |
| `current_`     | index of the **current** node (last reached)  |
| `has_current_` | false until the first `arrived()`             |

Two separate indices avoid `cursor_ - 1` arithmetic: `current()` just returns
`path[current_]`. `hasNext()` is read-only.

Cursor transitions:

```
             next()                 arrived()
  [target] ───peek──▶ (unchanged)   ───────▶ current_ = cursor_ ; ++cursor_
```

---

## Files

| File                          | Contents                                |
|-------------------------------|-----------------------------------------|
| `route_mgr/node.hpp / .cpp`   | `NodeType`, `Node`, enum↔string helpers |
| `route_mgr/route_manager.hpp` | `RouteManager` + `Step` / `StepStatus`  |
| `route_mgr/route_manager.cpp` | Implementation                          |
| `route_mgr/main.cpp`          | Standalone demo                         |

**Build the demo:**
```bash
cd route_mgr
g++ -std=c++17 main.cpp route_manager.cpp node.cpp -lyaml-cpp -o route_demo
./route_demo ../config/mission.yaml
```

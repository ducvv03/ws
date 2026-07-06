# HSM — Hierarchical State Machine Framework (C++11)

Author: Manh Tan Chu

A **hierarchical state machine** (HSM) framework written in C++11, with no
external dependencies. The design cleanly separates the **state tree
structure** from the **runtime engine**, supports free transitions between
any two branches via the **LCA** algorithm, message handling via bubbling,
and pluggable logging.

> This repo also ships a plain **C99 port** of the same design in
> [`hsm_c/`](hsm_c/) — same tree/LCA/bubbling semantics, no classes/virtuals
> (function-pointer "hooks" instead). This document covers the C++ version,
> which lives in [`hsm_cpp/`](hsm_cpp/).

---

## 1. Idea

A regular (flat) state machine only has sibling states. As logic gets more
complex (game AI, NPCs, workflows...), the number of transitions explodes
because every state has to know how to react to every event.

**HSM** solves this by letting states **nest into a tree**:

```
Root
├── A (composite)
│   ├── A1   (initial)
│   └── A2
└── B (composite)
    ├── B1   (initial)
    └── B2
```

- A parent state holds shared logic; child states only handle their own
  specifics.
- A message that a child can't handle **bubbles up to the parent** (the
  parent handles it instead).
- Transitioning between two different branches (e.g. `A1 → B2`) is handled
  by the engine in the correct `exit`/`enter` order via the **LCA**
  algorithm — you never have to write that logic by hand.

### Design philosophy

| Principle | Detail |
|-----------|--------|
| Structure decoupled from runtime | `State` builds the tree; `StateMachineEngine` runs it |
| Decoupled states | `process()` only returns the **id (integer)** of the state it wants to go to — no need to know about other states' pointers |
| Minimal caller | The game loop only needs to call `engine.process(msg)` — everything else runs internally |
| Pluggable logging | A separate `Logger` module — swap the backend in one line |

---

## 2. Architecture

```
hsm_cpp/
├── include/hsm/
│   ├── Message.h              # struct Message (payload — details TBD)
│   ├── State.h                # tree node + builder (addChild/setInitial) + 3 hooks
│   ├── StateMachineEngine.h   # runtime engine API
│   └── Logger.h               # pluggable logging (header-only)
├── src/
│   └── StateMachineEngine.cpp # LCA + bubbling + logging
└── examples/
    └── main.cpp               # full demo
```

| Component | Responsibility |
|-----------|-----------------|
| `Message` | Payload passed into `enter/exit/process`. Currently a placeholder (`int type`). |
| `State` | A node in the tree: holds parent/children/initial-child, plus the 3 `enter/exit/process` hooks. |
| `StateMachineEngine` | Takes a root, drives `start` / `process` / `transition`. Indexes states by id. Also handles logging. |
| `Logger` | Replaceable log sink (prints to `std::cout` by default). |

---

## 3. State

Every state derives from `hsm::State`, and has an **id (int)** and a **name**
that the engine uses for lookups and logging.

### The 3 core hooks

```cpp
class MyState : public hsm::State {
public:
    MyState() : State(ID_MY, "MyState") {}

    void enter(hsm::Message* msg) override { /* called ONCE when ENTERING the state */ }
    void exit (hsm::Message* msg) override { /* called ONCE when LEAVING the state  */ }
    int  process(hsm::Message* msg) override {
        // called EVERY tick. Returns one of 3 kinds (see section 5).
        return hsm::STAY;
    }
};
```

| Hook | When it runs | Return type |
|------|---------------|-------------|
| `enter(Message*)` | once, when the state is entered | `void` |
| `exit(Message*)`  | once, when the state is left | `void` |
| `process(Message*)` | every tick | `int` — see section 5 |

> `Message*` may be `nullptr` (e.g. on the first `start()`, before there is a triggering message).

### Building the tree

```cpp
root.addChild(&A, /*isInitial=*/true);  // attach A as root's child, mark it as initial
A.addChild(&A1, true);                   // A1 is A's initial child
A.addChild(&A2);
A.setInitial(&A1);                       // (alternative) set the initial child after adding
```

- `addChild(child, isInitial)` — attaches a child; the first child added becomes the default initial child unless specified otherwise.
- `setInitial(child)` — explicitly changes/sets the initial child.
- When the engine enters a composite state, it automatically descends through initial children all the way down to a **leaf**.

---

## 4. StateMachineEngine — runtime

The engine **does not build the tree**. You build it yourself from a root, then hand it to the engine:

```cpp
hsm::StateMachineEngine engine;
engine.setRoot(&root);   // indexes every state by id, ready to run
engine.start();          // enters from root down to the initial leaf

// Loop (game loop / looper):
while (running) {
    engine.process(&msg);   // just call this — the engine decides & transitions on its own
}
```

| Function | Meaning |
|----------|---------|
| `setRoot(State*)` | Registers the root; the engine walks the tree and indexes `id → State*`. |
| `start(Message*)` | Enters from root down to the initial leaf. |
| `process(Message*)` | One tick: forwards the message to the active leaf; if it returns an id, transitions. |
| `transition(State*, Message*)` | Transitions to any state (LCA-based). Usually called internally, but public if needed. |
| `current()` | The currently active leaf state. |
| `find(int id)` | Looks up a state by id. |

### LCA-based transition

When moving from a child state to a child state of a different branch, the engine:

1. **Exits** from the current state UP to the **lowest common ancestor (LCA)** — the common ancestor itself is not exited.
2. **Enters** from the LCA DOWN to the target state.

Example `A1 → B2` (common ancestor is `Root`):

```
exit:  A1, A
enter: B, B2          (Root stays active since it's the LCA)
```

If both states share the same parent (`A1 → A2`), only `exit A1` then `enter A2` happens — **parent `A` is not exited/entered again** since it's still active.

---

## 5. What `process()` returns — STAY / id / BUBBLE

`process()` returns an `int`, which the engine interprets as one of 3 cases:

| Return value | Meaning | What the engine does |
|--------------|---------|------------------------|
| `hsm::STAY` (`-1`) | Handled, stay here | Stops, no transition |
| A **state id** (`≥ 0`) | Wants to transition to that state | Looks up `id → State*`, runs an LCA transition |
| `hsm::BUBBLE` (`-2`) | "I can't handle this" | Calls the **parent's** `process()`; the parent again returns STAY/id/BUBBLE |

> ⚠️ State ids are defined by you and must be **≥ 0**, since `-1`/`-2` are reserved for STAY/BUBBLE.

### Event bubbling

A message travels from the **leaf up towards the root**; whichever state handles it (returns STAY or an id) stops the chain:

```
process A1 → BUBBLE   (A1 can't handle it)
process A  → ID_B2    (parent A handles it, requests a transition to B2)
→ transition A1 → B2
```

**Important:** bubbling only means "borrowing the parent to handle this one message". `m_current` does **not** move up to the parent. The next message is still handled by the **leaf** first. `m_current` only changes via `transition()`.

---

## 6. Logger — pluggable logging (printf-style + level)

Logging is **printf-style**, with a **log level** (`Debug/Info/Warn/Error`):

```cpp
hsm::Logger::info ("enter %s id=%d", state.name().c_str(), state.id());
hsm::Logger::error("unknown state id=%d", code);
hsm::Logger::log(hsm::Level::Warn, "low hp: %d", hp);   // generic form
```

> ⚠️ **Printf contract:** when passing a `std::string` via `%s`, use `.c_str()` — variadic arguments aren't type-checked.

By default it prints to `std::cout`. Plug in your own backend (the sink receives the `level` plus the already-formatted string), or disable it entirely:

```cpp
hsm::Logger::set([](hsm::Level lvl, const std::string& line) {
    MyGameLog::write(hsm::levelName(lvl), line);   // filter by level however you like
});

hsm::Logger::set(nullptr);   // disable logging
```

The engine automatically logs `enter` / `exit` / `process` along with the state name (and the message id during `process`), so you never have to add logging inside individual states.

---

## 7. Full example

See [`hsm_cpp/examples/main.cpp`](hsm_cpp/examples/main.cpp). Demo flow summary:

```cpp
enum StateId { ID_ROOT=0, ID_A, ID_A1, ID_A2, ID_B, ID_B1, ID_B2 };

class A1State : public hsm::State {
public:
    A1State() : State(ID_A1, "A1") {}
    int process(hsm::Message* msg) override {
        if (msg && msg->type == 1) return ID_B2;     // transition to B2
        if (msg && msg->type == 3) return hsm::BUBBLE; // let parent A handle it
        return hsm::STAY;
    }
};

// ... build the tree, setRoot, start, then loop process(&msg)
```

---

## 8. Build

```bash
cmake -S hsm_cpp -B hsm_cpp/build
cmake --build hsm_cpp/build
./hsm_cpp/build/hsm_demo        # or hsm_cpp/build/Debug/hsm_demo.exe on Windows/MSVC
```

Requires a compiler that supports **C++11** (g++, clang++, or MSVC).

For the C99 port, see [`hsm_c/`](hsm_c/) — same idea, build with:

```bash
cmake -S hsm_c -B hsm_c/build
cmake --build hsm_c/build
./hsm_c/build/hsm_c_demo
```

---

## 9. Future directions (not implemented yet)

- **History state** (shallow/deep) — remember which child was active when re-entering a composite.
- **Parallel states** — run multiple branches concurrently.
- **Guard / action** on transitions.
- A real `Message` struct with an actual payload.

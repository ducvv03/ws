#pragma once
#include <string>
#include <unordered_map>
#include "State.h"
#include "Message.h"
#include "Logger.h"

namespace hsm {

// Runtime engine for a hierarchical state machine.
//
// You build the tree yourself from a root State (via State::addChild) and
// hand that root to the engine via setRoot(). The engine then:
//   - indexes every state by its integer id (built during setRoot),
//   - start()      : enters from the root down to the initial leaf,
//   - process()    : drives one tick. It forwards the message to the active
//                    leaf; the leaf returns the id of the next state (or
//                    STAY). The engine maps that id to a State and transitions
//                    automatically. The caller (e.g. a game loop) only ever
//                    calls process().
//   - transition() : LCA-based move to any target state.
//
// LCA transition: when moving between sub-states under different parents, the
// engine exits UP to the lowest common ancestor, then enters DOWN to target.
class StateMachineEngine {
public:
    StateMachineEngine() = default;
    ~StateMachineEngine() = default;

    StateMachineEngine(const StateMachineEngine&) = delete;
    StateMachineEngine& operator=(const StateMachineEngine&) = delete;

    // Register the root of an already-built tree and index all states by id.
    void setRoot(State* root);

    // Enter from the root down to the initial leaf.
    void start(Message* msg = nullptr);

    // Drive one tick: process the active leaf; if it returns a state id other
    // than STAY, look it up and transition.
    void process(Message* msg);

    // Transition to any target state within the tree (LCA exit/enter).
    void transition(State* target, Message* msg = nullptr);

    // Look up a registered state by id (nullptr if not found).
    State* find(int id) const;

    State* root()    const { return m_root; }
    State* current() const { return m_current; }

private:
    static State* lca(State* a, State* b);
    static int    depth(State* s);

    void indexTree(State* node);                   // fill m_byId recursively
    void descendInto(State* target, Message* msg); // enter down to a leaf

    // enter/exit wrappers that log (state name) then call the state hook.
    void enterState(State* s, Message* msg);
    void exitState(State* s, Message* msg);

    std::unordered_map<int, State*> m_byId;
    State* m_root    = nullptr;
    State* m_current = nullptr;
};

} // namespace hsm

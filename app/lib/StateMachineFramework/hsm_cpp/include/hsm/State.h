#pragma once
#include <string>
#include <vector>
#include "Message.h"

namespace hsm {

// Sentinel return values for process(). Real state ids must be >= 0.
const int STAY   = -1;  // handled here, stay in the current state
const int BUBBLE = -2;  // not handled here, pass the message up to the parent

// Base state and tree node.
//
// A State owns its parent/child links and its default (initial) child.
// Build the tree here via addChild(); the engine runs it.
//
// Override the 3 core hooks directly:
//   - enter()  : called once when the state is entered
//   - exit()   : called once when the state is exited
//   - process(): called every tick. Returns one of:
//                  STAY        -> handled, remain in this state
//                  BUBBLE      -> not handled, let the parent's process() try
//                  a state id  -> transition to that state (engine does LCA)
//
// Logging (enter/exit/process + state name + message id) is done by the
// engine around these calls, so you never have to log by hand.
// The Message* may be nullptr (when there is no triggering message).
class State {
public:
    explicit State(int id, const std::string& name = "")
        : m_id(id), m_name(name) {}
    virtual ~State() {}

    State(const State&) = delete;
    State& operator=(const State&) = delete;

    virtual void enter(Message* /*msg*/) {}
    virtual void exit(Message* /*msg*/)  {}
    virtual int  process(Message* /*msg*/) { return STAY; }

    // Attach a child to this state. The first child added becomes the default
    // (initial) child unless another is explicitly marked. Returns `child` so
    // calls can be chained while building the tree.
    State* addChild(State* child, bool isInitial = false) {
        child->m_parent = this;
        m_children.push_back(child);
        if (isInitial || m_initialChild == nullptr)
            m_initialChild = child;
        return child;
    }

    // Designate which child is the initial (entry) sub-state of this composite
    // state. `child` must already have been added via addChild(). This is an
    // alternative to passing isInitial=true to addChild().
    void setInitial(State* child) {
        m_initialChild = child;
    }

    int                id()           const { return m_id; }
    const std::string& name()         const { return m_name; }
    State*             parent()       const { return m_parent; }
    State*             initialChild() const { return m_initialChild; }

private:
    friend class StateMachineEngine;

    int                 m_id;
    std::string         m_name;
    State*              m_parent       = nullptr;
    State*              m_initialChild = nullptr;
    std::vector<State*> m_children;
};

} // namespace hsm

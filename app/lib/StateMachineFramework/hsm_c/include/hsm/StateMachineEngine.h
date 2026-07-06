#pragma once
#include <stddef.h>
#include "State.h"
#include "Message.h"

// Runtime engine for a hierarchical state machine.
//
// Build the tree yourself from a root HsmState (via hsm_state_add_child) and
// hand that root to the engine via hsm_engine_set_root(). The engine then:
//   - indexes every state by its integer id (built during set_root),
//   - hsm_engine_start()   : enters from the root down to the initial leaf,
//   - hsm_engine_process() : drives one tick. It forwards the message to the
//                            active leaf; the leaf returns the id of the next
//                            state (or HSM_STAY). The engine maps that id to
//                            a state and transitions automatically. The
//                            caller (e.g. a game loop) only ever calls
//                            hsm_engine_process().
//   - hsm_engine_transition(): LCA-based move to any target state.
//
// LCA transition: when moving between sub-states under different parents,
// the engine exits UP to the lowest common ancestor, then enters DOWN to
// the target.
typedef struct HsmEngine {
    HsmState** byId;          // byId[i] is valid for i in [0, byIdCapacity)
    size_t     byIdCapacity;
    HsmState*  root;
    HsmState*  current;
} HsmEngine;

void hsm_engine_init(HsmEngine* engine);
void hsm_engine_destroy(HsmEngine* engine);

// Register the root of an already-built tree and index all states by id.
void hsm_engine_set_root(HsmEngine* engine, HsmState* root);

// Enter from the root down to the initial leaf.
void hsm_engine_start(HsmEngine* engine, HsmMessage* msg);

// Drive one tick: process the active leaf; if it returns a state id other
// than HSM_STAY, look it up and transition.
void hsm_engine_process(HsmEngine* engine, HsmMessage* msg);

// Transition to any target state within the tree (LCA exit/enter).
void hsm_engine_transition(HsmEngine* engine, HsmState* target, HsmMessage* msg);

// Look up a registered state by id (NULL if not found).
HsmState* hsm_engine_find(const HsmEngine* engine, int id);

HsmState* hsm_engine_root(const HsmEngine* engine);
HsmState* hsm_engine_current(const HsmEngine* engine);

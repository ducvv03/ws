#pragma once
#include <stddef.h>
#include "Message.h"

// Sentinel return values for process(). Real state ids must be >= 0.
enum {
    HSM_STAY   = -1, // handled here, stay in the current state
    HSM_BUBBLE = -2  // not handled here, pass the message up to the parent
};

typedef struct HsmState HsmState;

// "Virtual" hooks, filled in per concrete state via hsm_state_init().
// Passing NULL for any of them falls back to the default behavior
// (enter/exit do nothing, process() returns HSM_STAY).
typedef void (*HsmEnterFn)(HsmState* self, HsmMessage* msg);
typedef void (*HsmExitFn)(HsmState* self, HsmMessage* msg);
typedef int  (*HsmProcessFn)(HsmState* self, HsmMessage* msg);

// Base state and tree node.
//
// A HsmState owns its parent/child links and its default (initial) child.
// Build the tree via hsm_state_add_child(); the engine runs it.
//
// process() is called every tick and returns one of:
//   HSM_STAY    -> handled, remain in this state
//   HSM_BUBBLE  -> not handled, let the parent's process() try
//   a state id  -> transition to that state (engine does LCA)
//
// Logging (enter/exit/process + state name + message id) is done by the
// engine around these calls, so a state never has to log by hand.
// The HsmMessage* may be NULL (when there is no triggering message).
//
// "Subclassing": embed HsmState as the FIRST member of your own struct and
// cast back to it inside your hook functions:
//
//   typedef struct { HsmState base; int hp; } MyState;
//   static int my_process(HsmState* self, HsmMessage* msg) {
//       MyState* me = (MyState*)self;
//       ...
//   }
struct HsmState {
    int         id;
    const char* name;

    HsmState*  parent;
    HsmState*  initialChild;
    HsmState** children;
    size_t     childCount;
    size_t     childCapacity;

    HsmEnterFn   enter;
    HsmExitFn    exit;
    HsmProcessFn process;
};

// Initialize a state. Any hook may be NULL to use the default behavior.
void hsm_state_init(HsmState* state, int id, const char* name,
                     HsmEnterFn enter, HsmExitFn exit, HsmProcessFn process);

// Free the internal children array. Does not touch the states themselves -
// the tree never owns its nodes.
void hsm_state_destroy(HsmState* state);

// Attach a child to this state. The first child added becomes the default
// (initial) child unless isInitial is set. Returns child so calls can be
// chained while building the tree.
HsmState* hsm_state_add_child(HsmState* parent, HsmState* child, int isInitial);

// Designate which child is the initial (entry) sub-state of this composite
// state. child must already have been added via hsm_state_add_child(). This
// is an alternative to passing isInitial=1 to hsm_state_add_child().
void hsm_state_set_initial(HsmState* state, HsmState* child);

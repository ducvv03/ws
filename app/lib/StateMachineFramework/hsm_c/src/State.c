#include "hsm/State.h"
#include <stdlib.h>

static void default_enter(HsmState* self, HsmMessage* msg)   { (void)self; (void)msg; }
static void default_exit(HsmState* self, HsmMessage* msg)    { (void)self; (void)msg; }
static int  default_process(HsmState* self, HsmMessage* msg) { (void)self; (void)msg; return HSM_STAY; }

void hsm_state_init(HsmState* state, int id, const char* name,
                     HsmEnterFn enter, HsmExitFn exit, HsmProcessFn process) {
    state->id = id;
    state->name = name;
    state->parent = NULL;
    state->initialChild = NULL;
    state->children = NULL;
    state->childCount = 0;
    state->childCapacity = 0;
    state->enter   = enter   ? enter   : default_enter;
    state->exit    = exit    ? exit    : default_exit;
    state->process = process ? process : default_process;
}

void hsm_state_destroy(HsmState* state) {
    free(state->children);
    state->children = NULL;
    state->childCount = 0;
    state->childCapacity = 0;
}

HsmState* hsm_state_add_child(HsmState* parent, HsmState* child, int isInitial) {
    if (parent->childCount == parent->childCapacity) {
        size_t newCapacity = parent->childCapacity == 0 ? 4 : parent->childCapacity * 2;
        HsmState** grown = (HsmState**)realloc(parent->children, newCapacity * sizeof(HsmState*));
        if (!grown) return child; // allocation failed: child left unattached
        parent->children = grown;
        parent->childCapacity = newCapacity;
    }
    parent->children[parent->childCount++] = child;
    child->parent = parent;

    if (isInitial || parent->initialChild == NULL)
        parent->initialChild = child;

    return child;
}

void hsm_state_set_initial(HsmState* state, HsmState* child) {
    state->initialChild = child;
}

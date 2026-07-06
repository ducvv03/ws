#include "hsm/StateMachineEngine.h"
#include "hsm/Logger.h"
#include <stdlib.h>

static void enter_state(HsmState* s, HsmMessage* msg) {
    hsm_log_info("enter   %s", s->name);
    s->enter(s, msg);
}

static void exit_state(HsmState* s, HsmMessage* msg) {
    hsm_log_info("exit    %s", s->name);
    s->exit(s, msg);
}

static void ensure_capacity(HsmEngine* engine, size_t minCapacity) {
    if (minCapacity <= engine->byIdCapacity) return;

    size_t newCapacity = engine->byIdCapacity == 0 ? 8 : engine->byIdCapacity;
    while (newCapacity < minCapacity) newCapacity *= 2;

    HsmState** grown = (HsmState**)realloc(engine->byId, newCapacity * sizeof(HsmState*));
    for (size_t i = engine->byIdCapacity; i < newCapacity; ++i) grown[i] = NULL;

    engine->byId = grown;
    engine->byIdCapacity = newCapacity;
}

static void index_tree(HsmEngine* engine, HsmState* node) {
    if (node->id >= 0) {
        ensure_capacity(engine, (size_t)node->id + 1);
        engine->byId[node->id] = node;
    }
    for (size_t i = 0; i < node->childCount; ++i)
        index_tree(engine, node->children[i]);
}

static int depth(HsmState* s) {
    int d = 0;
    for (HsmState* p = s->parent; p != NULL; p = p->parent)
        ++d;
    return d;
}

static HsmState* lca(HsmState* a, HsmState* b) {
    if (!a || !b) return NULL;

    int da = depth(a);
    int db = depth(b);

    // Bring both to the same depth.
    while (da > db) { a = a->parent; --da; }
    while (db > da) { b = b->parent; --db; }

    // Walk up together until they meet.
    while (a != b) {
        a = a->parent;
        b = b->parent;
    }
    return a;
}

static void descend_into(HsmEngine* engine, HsmState* target, HsmMessage* msg) {
    // After entering target, keep entering initial children down to a leaf.
    HsmState* s = target;
    while (s->initialChild != NULL) {
        s = s->initialChild;
        enter_state(s, msg);
    }
    engine->current = s;
}

// Enter every state from `stop` (exclusive) down to `s` (inclusive). The
// recursion walks up to `stop` first, then enters on the way back down, so
// the chain runs in root-to-target order without needing a path buffer.
// Passing stop = NULL walks all the way up to the tree root, which also
// covers the "no active state yet" case (lca(NULL, target) is NULL).
static void enter_chain(HsmState* s, HsmState* stop, HsmMessage* msg) {
    if (s == stop) return;
    enter_chain(s->parent, stop, msg);
    enter_state(s, msg);
}

void hsm_engine_init(HsmEngine* engine) {
    engine->byId = NULL;
    engine->byIdCapacity = 0;
    engine->root = NULL;
    engine->current = NULL;
}

void hsm_engine_destroy(HsmEngine* engine) {
    free(engine->byId);
    engine->byId = NULL;
    engine->byIdCapacity = 0;
}

void hsm_engine_set_root(HsmEngine* engine, HsmState* root) {
    engine->root = root;
    free(engine->byId);
    engine->byId = NULL;
    engine->byIdCapacity = 0;
    if (root) index_tree(engine, root);
}

HsmState* hsm_engine_find(const HsmEngine* engine, int id) {
    if (id < 0 || (size_t)id >= engine->byIdCapacity) return NULL;
    return engine->byId[id];
}

HsmState* hsm_engine_root(const HsmEngine* engine)    { return engine->root; }
HsmState* hsm_engine_current(const HsmEngine* engine) { return engine->current; }

void hsm_engine_start(HsmEngine* engine, HsmMessage* msg) {
    if (!engine->root) return;
    enter_state(engine->root, msg);
    descend_into(engine, engine->root, msg);
}

void hsm_engine_transition(HsmEngine* engine, HsmState* target, HsmMessage* msg) {
    if (!target) return;

    HsmState* common = lca(engine->current, target);

    // 1) Exit upward from current to (but not including) the LCA.
    for (HsmState* s = engine->current; s != common; s = s->parent)
        exit_state(s, msg);

    // 2) Enter downward from the LCA to (and including) the target.
    enter_chain(target, common, msg);

    // 3) If the target is composite, descend to its initial leaf.
    descend_into(engine, target, msg);
}

void hsm_engine_process(HsmEngine* engine, HsmMessage* msg) {
    // Start at the active leaf and walk UP the active chain while states
    // bubble. Each process() returns:
    //   HSM_STAY    -> handled, stop here, no transition
    //   HSM_BUBBLE  -> not handled, try this state's parent next
    //   a state id  -> transition to that state (LCA exit/enter)
    //
    // Bubbling is per-message only: engine->current is NOT moved up the
    // chain. If a parent handles a bubbled message without transitioning,
    // the active leaf stays the same, so the NEXT message is still handled
    // by the leaf first. engine->current changes solely through transition.
    for (HsmState* s = engine->current; s != NULL; s = s->parent) {
        hsm_log_info("process %s (msg id=%d)", s->name, msg ? msg->type : -1);
        int code = s->process(s, msg);

        if (code == HSM_STAY)   return;
        if (code == HSM_BUBBLE) continue; // let the parent handle it

        HsmState* target = hsm_engine_find(engine, code);
        if (target)
            hsm_engine_transition(engine, target, msg);
        return;
    }
    // Bubbled past the root with nobody handling it: nothing to do.
}

#include "hsm/StateMachineEngine.h"
#include "hsm/Logger.h"
#include <stdio.h>

// State ids. process() returns one of these, or HSM_STAY / HSM_BUBBLE.
enum StateId {
    ID_ROOT = 0,
    ID_A, ID_A1, ID_A2,
    ID_B, ID_B1, ID_B2
};

// Tree (single root):
//
//   Root
//   |-- A (composite)        <- handles messages bubbled up from A1/A2
//   |   |-- A1   (initial)
//   |   '-- A2
//   '-- B (composite)
//       |-- B1   (initial)
//       '-- B2
//
// enter/exit/process logging is done by the engine. main() builds the tree
// from plain HsmState nodes and a few process() hooks. The loop only calls
// hsm_engine_process().

// A1: type 1 -> go to B2. type 3 -> BUBBLE (let parent A handle). Else stay.
static int a1_process(HsmState* self, HsmMessage* msg) {
    (void)self;
    if (!msg) return HSM_STAY;
    if (msg->type == 1) return ID_B2;
    if (msg->type == 3) return HSM_BUBBLE;
    return HSM_STAY;
}

// A (composite parent): handles the bubbled type 3 by sending us to B2.
static int a_process(HsmState* self, HsmMessage* msg) {
    (void)self;
    if (msg && msg->type == 3) return ID_B2;
    return HSM_STAY;
}

// B2: type 2 -> go to A2. Else stay.
static int b2_process(HsmState* self, HsmMessage* msg) {
    (void)self;
    return (msg && msg->type == 2) ? ID_A2 : HSM_STAY;
}

static void demo_sink(HsmLogLevel level, const char* line) {
    printf("    LOG[%s] %s\n", hsm_log_level_name(level), line);
}

int main(void) {
    // Optional: port logging into your own backend. Comment out to use the
    // default stdout sink.
    hsm_log_set_sink(demo_sink);

    HsmState root, A, A1, A2, B, B1, B2;
    hsm_state_init(&root, ID_ROOT, "Root", NULL, NULL, NULL);
    hsm_state_init(&A,    ID_A,    "A",    NULL, NULL, a_process);
    hsm_state_init(&A1,   ID_A1,   "A1",   NULL, NULL, a1_process);
    hsm_state_init(&A2,   ID_A2,   "A2",   NULL, NULL, NULL);
    hsm_state_init(&B,    ID_B,    "B",    NULL, NULL, NULL);
    hsm_state_init(&B1,   ID_B1,   "B1",   NULL, NULL, NULL);
    hsm_state_init(&B2,   ID_B2,   "B2",   NULL, NULL, b2_process);

    // Build the tree from the root.
    hsm_state_add_child(&root, &A, /*isInitial=*/1);
    hsm_state_add_child(&A, &A1, 1);
    hsm_state_add_child(&A, &A2, 0);
    hsm_state_add_child(&root, &B, 0);
    hsm_state_add_child(&B, &B1, 1);
    hsm_state_add_child(&B, &B2, 0);

    HsmEngine engine;
    hsm_engine_init(&engine);
    hsm_engine_set_root(&engine, &root); // indexes all states by id

    printf("== start ==\n");
    hsm_engine_start(&engine, NULL);
    printf("current = %s\n", hsm_engine_current(&engine)->name);

    // type 3 -> A1 bubbles -> A handles -> transition to B2
    // type 2 -> B2 -> A2
    // type 0 -> A2 stays
    HsmMessage msgs[3] = { {3}, {2}, {0} };
    for (int i = 0; i < 3; ++i) {
        printf("== tick %d (msg.type=%d) ==\n", i, msgs[i].type);
        hsm_engine_process(&engine, &msgs[i]);
        printf("current = %s\n", hsm_engine_current(&engine)->name);
    }

    hsm_state_destroy(&root);
    hsm_state_destroy(&A);
    hsm_state_destroy(&B);
    hsm_engine_destroy(&engine);
    return 0;
}

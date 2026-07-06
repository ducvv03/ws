#pragma once

// Placeholder message - concrete fields to be defined later.
// enter/exit/process take a HsmMessage* (may be NULL when there is no
// triggering message, e.g. on the initial start()).
typedef struct HsmMessage {
    int type;
} HsmMessage;

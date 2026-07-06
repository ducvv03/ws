#pragma once

namespace hsm {

// Placeholder message — concrete fields to be defined later.
// enter/exit/process take a Message* (may be nullptr when there is no
// triggering message, e.g. on the initial start()).
struct Message {
    int type;

    Message(int t = 0) : type(t) {}
};

} // namespace hsm

#include "hsm/StateMachineEngine.h"
#include <iostream>

using namespace hsm;

// State ids. onProcess() returns one of these, or hsm::STAY / hsm::BUBBLE.
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
// enter/exit/process logging is done by the engine. Subclasses override
// enter/exit/process directly. The loop only calls engine.process().

// A1: type 1 -> go to B2. type 3 -> BUBBLE (let parent A handle). Else stay.
class A1State : public State {
public:
    A1State() : State(ID_A1, "A1") {}
    int process(Message* msg) override {
        if (!msg) return STAY;
        if (msg->type == 1) return ID_B2;
        if (msg->type == 3) return BUBBLE;
        return STAY;
    }
};

// A (composite parent): handles the bubbled type 3 by sending us to B2.
class AState : public State {
public:
    AState() : State(ID_A, "A") {}
    int process(Message* msg) override {
        if (msg && msg->type == 3) return ID_B2;
        return STAY;
    }
};

// B2: type 2 -> go to A2. Else stay.
class B2State : public State {
public:
    B2State() : State(ID_B2, "B2") {}
    int process(Message* msg) override {
        return (msg && msg->type == 2) ? ID_A2 : STAY;
    }
};

int main() {
    // Optional: port logging into your own backend. Comment out to use the
    // default std::cout sink.
    Logger::set([](Level lvl, const std::string& line) {
        std::cout << "    LOG[" << levelName(lvl) << "] " << line << "\n";
    });

    State   root(ID_ROOT, "Root");
    State   A2(ID_A2, "A2"), B(ID_B, "B"), B1(ID_B1, "B1");
    AState  A;
    A1State A1; B2State B2;

    // Build the tree from the root.
    root.addChild(&A, /*isInitial*/ true);
    A.addChild(&A1, /*isInitial*/ true);
    A.addChild(&A2);
    root.addChild(&B);
    B.addChild(&B1, /*isInitial*/ true);
    B.addChild(&B2);

    StateMachineEngine engine;
    engine.setRoot(&root);   // indexes all states by id

    std::cout << "== start ==\n";
    engine.start();
    std::cout << "current = " << engine.current()->name() << "\n";

    // type 3 -> A1 bubbles -> A handles -> transition to B2
    // type 2 -> B2 -> A2
    // type 0 -> A2 stays
    Message msgs[3] = { {3}, {2}, {0} };
    for (int i = 0; i < 3; ++i) {
        std::cout << "== tick " << i << " (msg.type=" << msgs[i].type << ") ==\n";
        engine.process(&msgs[i]);
        std::cout << "current = " << engine.current()->name() << "\n";
    }
    return 0;
}

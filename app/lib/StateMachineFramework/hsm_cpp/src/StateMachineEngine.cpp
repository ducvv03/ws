#include "hsm/StateMachineEngine.h"
#include <vector>

namespace hsm {

void StateMachineEngine::enterState(State* s, Message* msg) {
    Logger::info("enter   %s", s->name().c_str());
    s->enter(msg);
}

void StateMachineEngine::exitState(State* s, Message* msg) {
    Logger::info("exit    %s", s->name().c_str());
    s->exit(msg);
}

void StateMachineEngine::indexTree(State* node) {
    m_byId[node->m_id] = node;
    for (std::vector<State*>::iterator it = node->m_children.begin();
         it != node->m_children.end(); ++it) {
        indexTree(*it);
    }
}

void StateMachineEngine::setRoot(State* root) {
    m_root = root;
    m_byId.clear();
    if (m_root)
        indexTree(m_root);
}

State* StateMachineEngine::find(int id) const {
    std::unordered_map<int, State*>::const_iterator it = m_byId.find(id);
    return it == m_byId.end() ? nullptr : it->second;
}

int StateMachineEngine::depth(State* s) {
    int d = 0;
    for (State* p = s->m_parent; p != nullptr; p = p->m_parent)
        ++d;
    return d;
}

State* StateMachineEngine::lca(State* a, State* b) {
    if (!a || !b) return nullptr;

    int da = depth(a);
    int db = depth(b);

    // Bring both to the same depth.
    while (da > db) { a = a->m_parent; --da; }
    while (db > da) { b = b->m_parent; --db; }

    // Walk up together until they meet.
    while (a != b) {
        a = a->m_parent;
        b = b->m_parent;
    }
    return a;
}

void StateMachineEngine::descendInto(State* target, Message* msg) {
    // After entering target, keep entering initial children down to a leaf.
    State* s = target;
    while (s->m_initialChild != nullptr) {
        s = s->m_initialChild;
        enterState(s, msg);
    }
    m_current = s;
}

void StateMachineEngine::start(Message* msg) {
    if (!m_root) return;
    enterState(m_root, msg);
    descendInto(m_root, msg);
}

void StateMachineEngine::transition(State* target, Message* msg) {
    if (!target) return;

    if (!m_current) {
        // No active state yet: enter from the target's topmost ancestor
        // down to the target, then descend to its initial leaf.
        std::vector<State*> downPath;
        for (State* s = target; s != nullptr; s = s->m_parent)
            downPath.push_back(s);
        for (std::vector<State*>::reverse_iterator it = downPath.rbegin();
             it != downPath.rend(); ++it) {
            enterState(*it, msg);
        }
        descendInto(target, msg);
        return;
    }

    State* common = lca(m_current, target);

    // 1) Exit upward from current to (but not including) the LCA.
    for (State* s = m_current; s != common; s = s->m_parent)
        exitState(s, msg);

    // 2) Collect the enter path from target up to (but not including) the LCA.
    std::vector<State*> enterPath;
    for (State* s = target; s != common; s = s->m_parent)
        enterPath.push_back(s);

    // 3) Enter downward: reverse so we go LCA -> ... -> target.
    for (std::vector<State*>::reverse_iterator it = enterPath.rbegin();
         it != enterPath.rend(); ++it) {
        enterState(*it, msg);
    }

    // 4) If the target is composite, descend to its initial leaf.
    descendInto(target, msg);
}

void StateMachineEngine::process(Message* msg) {
    // Start at the active leaf and walk UP the active chain while states
    // bubble. Each process() returns:
    //   STAY        -> handled, stop here, no transition
    //   BUBBLE      -> not handled, try this state's parent next
    //   a state id  -> transition to that state (LCA exit/enter)
    //
    // Bubbling is per-message only: m_current is NOT moved up the chain. If a
    // parent handles a bubbled message without transitioning, the active leaf
    // stays the same, so the NEXT message is still handled by the leaf first.
    // m_current changes solely through transition().
    for (State* s = m_current; s != nullptr; s = s->m_parent) {
        Logger::info("process %s (msg id=%d)", s->name().c_str(),
                     msg ? msg->type : -1);
        int code = s->process(msg);

        if (code == STAY)   return;
        if (code == BUBBLE) continue;   // let the parent handle it

        State* target = find(code);
        if (target)
            transition(target, msg);
        return;
    }
    // Bubbled past the root with nobody handling it: nothing to do.
}

} // namespace hsm

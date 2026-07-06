// ============================================================
//  node.hpp
//  Graph node types: the NodeType enum and the Node struct.
// ============================================================
#pragma once

#include <string>
#include <vector>

// Semantic kind of a node -> mapped into the state machine.
// NOTE: no HOME here; HOME is just a node whose type is WAYPOINT.
enum class NodeType {
    WAYPOINT,   // pure move-through point (HOME, PRE_PICK, LIFT ...)
    PICK,       // close the gripper
    DROP,       // open the gripper
    HOLD,       // wait for a signal / ENTER
    SCAN,       // trigger perception / camera
    APPROACH,   // slow approach
    UNKNOWN     // unrecognised string in the config
};

NodeType    nodeTypeFromString(const std::string& s);
std::string toString(NodeType t);

// One node of the graph: its kind and the nodes it can travel to.
struct Node {
    std::string name;                 // node id
    NodeType    type = NodeType::WAYPOINT;
    std::vector<std::string> routes;  // other nodes reachable FROM this node
};

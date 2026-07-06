// ============================================================
//  node.cpp
// ============================================================
#include "node.hpp"

NodeType nodeTypeFromString(const std::string& s)
{
    if (s == "waypoint") return NodeType::WAYPOINT;
    if (s == "pick")     return NodeType::PICK;
    if (s == "drop")     return NodeType::DROP;
    if (s == "hold")     return NodeType::HOLD;
    if (s == "scan")     return NodeType::SCAN;
    if (s == "approach") return NodeType::APPROACH;
    return NodeType::UNKNOWN;
}

std::string toString(NodeType t)
{
    switch (t) {
        case NodeType::WAYPOINT: return "waypoint";
        case NodeType::PICK:     return "pick";
        case NodeType::DROP:     return "drop";
        case NodeType::HOLD:     return "hold";
        case NodeType::SCAN:     return "scan";
        case NodeType::APPROACH: return "approach";
        default:                 return "unknown";
    }
}

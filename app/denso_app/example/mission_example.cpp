// ============================================================
//  mission_example.cpp
//  Standalone example: load mission.yaml, build the node graph,
//  validate mission.path against the graph, then print the walk.
//
//  Build:
//     g++ -std=c++17 mission_example.cpp -lyaml-cpp -o mission_example
//  Run:
//     ./mission_example ../config/mission.yaml
// ============================================================

#include <yaml-cpp/yaml.h>

#include <iostream>
#include <string>
#include <vector>
#include <unordered_map>
#include <algorithm>

// One node of the graph: its kind and the nodes it can travel to.
struct Node {
    std::string type;                 // home | waypoint | pick | drop | hold ...
    std::vector<std::string> routes;  // nodes reachable FROM this node
};

int main(int argc, char** argv)
{
    const std::string path =
        (argc > 1) ? argv[1] : "../config/mission.yaml";

    // -------- 1. Load the YAML file --------
    YAML::Node root;
    try {
        root = YAML::LoadFile(path);
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] Cannot open '" << path << "': " << e.what() << "\n";
        return 1;
    }

    // -------- 2. Build the graph: name -> Node --------
    std::unordered_map<std::string, Node> nodes;
    for (const auto& n : root["nodes"]) {
        Node node;
        node.type = n["type"].as<std::string>("");
        for (const auto& t : n["routes"])
            node.routes.push_back(t.as<std::string>());
        nodes[n["name"].as<std::string>()] = node;
    }

    // -------- 3. Read the route to run --------
    std::vector<std::string> route;
    for (const auto& p : root["mission"]["path"])
        route.push_back(p.as<std::string>());

    std::cout << "Loaded " << nodes.size() << " nodes, path length "
              << route.size() << "\n";

    // -------- 4. Validate --------
    bool ok = true;

    // Check 1: every node in the path must exist.
    for (const auto& name : route) {
        if (nodes.find(name) == nodes.end()) {
            std::cerr << "[INVALID] Node '" << name << "' does not exist\n";
            ok = false;
        }
    }

    // Check 2: every consecutive jump must be a legal edge.
    for (size_t i = 0; ok && i + 1 < route.size(); ++i) {
        const std::string& a = route[i];
        const std::string& b = route[i + 1];
        const auto& tgt = nodes[a].routes;
        if (std::find(tgt.begin(), tgt.end(), b) == tgt.end()) {
            std::cerr << "[INVALID] Cannot move '" << a << "' -> '" << b
                      << "' (no such edge)\n";
            ok = false;
        }
    }

    if (!ok) {
        std::cerr << "Path is INVALID. Aborting.\n";
        return 2;
    }

    // -------- 5. Walk the (valid) path --------
    std::cout << "Path is VALID. Walking:\n";
    for (size_t i = 0; i < route.size(); ++i) {
        const auto& name = route[i];
        std::cout << "  step " << i << " -> " << name
                  << "  [type=" << nodes[name].type << "]\n";
        // TODO: here you would push the real motion command for `name`
        //       and run its action based on nodes[name].type
    }
    std::cout << "Done.\n";
    return 0;
}

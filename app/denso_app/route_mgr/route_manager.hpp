// ============================================================
//  route_manager.hpp
//  Loads the node graph from mission.yaml, gives access to
//  nodes, and validates a route (path) over the graph.
// ============================================================
#pragma once

#include <string>
#include <vector>
#include <unordered_map>

#include "node.hpp"

// Outcome of a call to RouteManager::next(): tells the caller whether there
// is a node to move to, the route is done, or the next step is not allowed.
enum class StepStatus {
    OK,        // a valid target node was returned -> keep moving
    FINISHED,  // reached the end of the route (normal, not an error) -> node is null
    BLOCKED    // next step is an illegal edge or an unknown node -> node is null
};

// What next() hands back: a status plus (only when OK) the node to move to.
struct Step {
    StepStatus  status = StepStatus::FINISHED;  // which of the 3 outcomes above
    const Node* node   = nullptr;               // the target node; valid ONLY when status == OK
};

class RouteManager {
public:
    // Load and parse `yaml_path` (nodes + mission.path).
    // Throws std::runtime_error if the file cannot be read/parsed.
    explicit RouteManager(const std::string& yaml_path);

    // ---- Node access ----

    // True if a node with this exact name exists in the graph.
    bool hasNode(const std::string& name) const;

    // Return the node by name. Throws std::runtime_error if it does not exist.
    const Node& getNode(const std::string& name) const;

    // How many nodes were loaded from the config.
    std::size_t nodeCount() const { return nodes_.size(); }

    // ---- Routing ----

    // The route declared in the config file under `mission.path`.
    const std::vector<std::string>& missionPath() const { return mission_path_; }

    // Check a whole route up-front: every node must exist AND every
    // consecutive pair must be a legal edge. Returns true if the route is
    // valid; on failure returns false and writes the reason into `error`.
    bool validate(const std::vector<std::string>& path, std::string& error) const;

    // True if you are allowed to move directly from node `a` to node `b`
    // (i.e. `b` is listed in a's `routes`). One single edge, not a whole path.
    bool canGo(const std::string& a, const std::string& b) const;

    // ---- Cursor over mission.path ----
    // Reading the next target and confirming arrival are DELIBERATELY split,
    // so the cursor never runs ahead of the physical robot.

    // True while there is still a node left to visit in the route.
    bool hasNext() const;

    // PEEK the next target node. Returns a Step (see above). Does NOT advance
    // the cursor and never throws — call it as often as you like while moving;
    // it keeps returning the same target until arrived() is called.
    Step next();

    // Confirm the robot has PHYSICALLY reached the current target. Only now
    // does the cursor advance to the next node. Call exactly once per arrival.
    void arrived();

    // The node the robot is currently standing at (the last one arrived() was
    // called for). Returns nullptr before the first arrival.
    const Node* current() const;

    // Restart the route from the very beginning (rewinds the cursor).
    void reset() { cursor_ = 0; current_ = 0; has_current_ = false; }

private:
    std::unordered_map<std::string, Node> nodes_;
    std::vector<std::string> mission_path_;
    std::size_t cursor_      = 0;      // index of the NEXT node next() will return
    std::size_t current_     = 0;      // index of the CURRENT node (last returned)
    bool        has_current_ = false;  // false until the first next()
};

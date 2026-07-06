// ============================================================
//  route_manager.cpp
// ============================================================
#include "route_manager.hpp"
#include "config_keys.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <stdexcept>

RouteManager::RouteManager(const std::string& yaml_path)
{
    YAML::Node root;
    try {
        root = YAML::LoadFile(yaml_path);
    } catch (const std::exception& e) {
        throw std::runtime_error("Cannot load '" + yaml_path + "': " + e.what());
    }

    // Build the graph: name -> Node
    for (const auto& n : root[cfg::mission::NODES]) {
        Node node;
        node.name = n[cfg::mission::NAME].as<std::string>();
        node.type = nodeTypeFromString(n[cfg::mission::TYPE].as<std::string>(""));
        for (const auto& t : n[cfg::mission::ROUTES])
            node.routes.push_back(t.as<std::string>());
        nodes_[node.name] = node;
    }

    // Read the route declared in the config (optional).
    const auto& mission = root[cfg::mission::MISSION];
    if (mission && mission[cfg::mission::PATH]) {
        for (const auto& p : mission[cfg::mission::PATH])
            mission_path_.push_back(p.as<std::string>());
    }
}

bool RouteManager::hasNode(const std::string& name) const
{
    return nodes_.find(name) != nodes_.end();
}

const Node& RouteManager::getNode(const std::string& name) const
{
    auto it = nodes_.find(name);
    if (it == nodes_.end())
        throw std::runtime_error("Node '" + name + "' does not exist");
    return it->second;
}

bool RouteManager::canGo(const std::string& a, const std::string& b) const
{
    if (!hasNode(a)) return false;
    const auto& rts = nodes_.at(a).routes;
    return std::find(rts.begin(), rts.end(), b) != rts.end();
}

bool RouteManager::hasNext() const
{
    return cursor_ < mission_path_.size();
}

Step RouteManager::next()
{
    // End of route reached -> normal finish, no node.
    if (cursor_ >= mission_path_.size())
        return { StepStatus::FINISHED, nullptr };

    const std::string& name = mission_path_[cursor_];

    // Node must exist, and (from the 2nd node on) the edge must be legal.
    if (!hasNode(name))
        return { StepStatus::BLOCKED, nullptr };
    if (has_current_ && !canGo(mission_path_[current_], name))
        return { StepStatus::BLOCKED, nullptr };

    // Only PEEK the target -- the cursor stays put until arrived() is called.
    return { StepStatus::OK, &getNode(name) };
}

void RouteManager::arrived()
{
    if (cursor_ >= mission_path_.size())
        return;                      // nothing pending, ignore

    current_     = cursor_;          // the node the robot just reached
    has_current_ = true;
    ++cursor_;                       // NOW advance to the next target
}

const Node* RouteManager::current() const
{
    if (!has_current_) return nullptr;   // next() not called yet
    return &getNode(mission_path_[current_]);
}

bool RouteManager::validate(const std::vector<std::string>& path,
                            std::string& error) const
{
    // Check 1: every node in the path must exist.
    for (const auto& name : path) {
        if (!hasNode(name)) {
            error = "Node '" + name + "' does not exist";
            return false;
        }
    }

    // Check 2: every consecutive jump must be a legal edge.
    for (std::size_t i = 0; i + 1 < path.size(); ++i) {
        if (!canGo(path[i], path[i + 1])) {
            error = "Cannot move '" + path[i] + "' -> '" + path[i + 1] +
                    "' (no such edge)";
            return false;
        }
    }

    error.clear();
    return true;
}

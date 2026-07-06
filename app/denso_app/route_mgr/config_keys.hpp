// ============================================================
//  config_keys.hpp
//  Central place for every YAML key used by the config files.
//  Use these constants instead of raw string literals so a typo
//  is a compile error, not a silent "key not found" at runtime.
// ============================================================
#pragma once

namespace cfg {

// ---- mission.yaml ----
namespace mission {
constexpr const char* NODES   = "nodes";     // top-level list of nodes
constexpr const char* NAME    = "name";      // node id
constexpr const char* TYPE    = "type";      // node kind (waypoint/pick/...)
constexpr const char* ROUTES  = "routes";    // nodes reachable from this node
constexpr const char* MISSION = "mission";   // mission block
constexpr const char* PATH    = "path";      // ordered route under `mission`
}  // namespace mission

// ---- settings.yaml ----
namespace settings {
constexpr const char* SETTINGS             = "settings";
constexpr const char* ARRIVAL_CHECK_PERIOD = "arrival_check_period";
constexpr const char* MAX_RUN_TIME         = "max_run_time";
constexpr const char* LOOPS                = "loops";
}  // namespace settings

}  // namespace cfg

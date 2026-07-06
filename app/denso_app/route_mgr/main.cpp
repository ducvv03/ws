// ============================================================
//  main.cpp  —  small demo/driver for RouteManager
//  Build: g++ -std=c++17 main.cpp route_manager.cpp -lyaml-cpp -o route_demo
//  Run:   ./route_demo ../config/mission.yaml
// ============================================================
#include "route_manager.hpp"

#include <iostream>

int main(int argc, char** argv)
{
    const std::string path =
        (argc > 1) ? argv[1] : "../config/mission.yaml";

    try {
        RouteManager rm(path);
        std::cout << "Loaded " << rm.nodeCount() << " nodes\n";

        const auto& route = rm.missionPath();
        std::string err;
        if (!rm.validate(route, err)) {
            std::cerr << "[INVALID] " << err << "\n";
            return 2;
        }

        std::cout << "Walking:\n";
        int step = 0;
        while (true) {
            Step s = rm.next();                 // pop the next node to move to
            if (s.status == StepStatus::FINISHED) {
                std::cout << "Route finished.\n";
                break;
            }
            if (s.status == StepStatus::BLOCKED) {
                std::cerr << "Route BLOCKED (illegal step).\n";
                return 2;
            }
            // status == OK -> s.node is the target to move to
            std::cout << "  step " << step++ << " -> " << s.node->name
                      << "  [type=" << toString(s.node->type) << "]\n";

            // TODO: command the robot to move to *s.node and WAIT until it
            //       physically arrives...
            //   move_to(*s.node);   // blocks until reached

            rm.arrived();   // ...only then confirm arrival -> advance the cursor
        }
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] " << e.what() << "\n";
        return 1;
    }
    return 0;
}

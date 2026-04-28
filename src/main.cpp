#include <iostream>
#include <fstream>
#include <string>
#include "Map.h"
#include "DijkstraPlanner.h"
#include "AStarPlanner.h"
#include "PlannerConfig.h"

int main(int argc, char** argv){
    setTurnPenalty(0.5f);
    std::string mapfile = "../data/example_map.txt";
    Point s(0,0), t(4,4);
    if (argc >= 2) mapfile = argv[1];
    if (argc >= 6) {
        s.x = std::stoi(argv[2]); s.y = std::stoi(argv[3]);
        t.x = std::stoi(argv[4]); t.y = std::stoi(argv[5]);
    }
    Map map;
    if (!map.loadFromTxt(mapfile)){
        std::cerr << "Failed to load map: " << mapfile << std::endl;
        return 1;
    }
    std::cout << "Map loaded: " << map.width() << " x " << map.height() << std::endl;

    DijkstraPlanner dijk;
    if (dijk.plan(map, s, t)){
        auto st = dijk.getStats();
        std::cout << "Dijkstra: success=" << st.success << " time=" << st.time_ms << "ms nodes=" << st.nodes_expanded << " cost=" << st.path_cost << std::endl;
        std::ofstream of("dijkstra_path.txt");
        for (auto &p : dijk.getPath()) of << p.x << " " << p.y << "\n";
    } else {
        std::cout << "Dijkstra: no path" << std::endl;
    }

    AStarPlanner ast;
    if (ast.plan(map, s, t)){
        auto st = ast.getStats();
        std::cout << "A*: success=" << st.success << " time=" << st.time_ms << "ms nodes=" << st.nodes_expanded << " cost=" << st.path_cost << std::endl;
        std::ofstream of("astar_path.txt");
        for (auto &p : ast.getPath()) of << p.x << " " << p.y << "\n";
    } else {
        std::cout << "A*: no path" << std::endl;
    }
    return 0;
}

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include "Map.h"
#include "DijkstraPlanner.h"
#include "AStarPlanner.h"
#include "PlannerConfig.h"

int main(){
    setTurnPenalty(0.5f);
    std::vector<std::string> maps;
    std::ifstream list("data\\maps_list.txt");
    if (!list){
        std::cerr << "maps_list.txt not found in data/. Create a file listing map filenames." << std::endl;
        return 1;
    }
    std::string line;
    while (std::getline(list, line)){
        if (line.empty()) continue;
        maps.push_back(std::string("data\\") + line);
    }
    if (maps.empty()){
        std::cerr << "no maps listed in maps_list.txt" << std::endl;
        return 1;
    }
    std::ofstream out("eval_results.csv");
    out << "map,algorithm,success,time_ms,nodes,path_cost\n";
    for (auto &mf : maps){
        Map map;
        if (!map.loadFromTxt(mf)){
            std::cerr << "skip invalid map: " << mf << std::endl;
            continue;
        }
        Point s(0,0), t(map.width()-1, map.height()-1);
        DijkstraPlanner d;
        if (d.plan(map,s,t)){
            auto st = d.getStats();
            out << mf.substr(mf.find_last_of('\\')+1) << ",Dijkstra," << st.success << "," << st.time_ms << "," << st.nodes_expanded << "," << st.path_cost << "\n";
        } else {
            out << mf.substr(mf.find_last_of('\\')+1) << ",Dijkstra,0,0,0,0\n";
        }
        AStarPlanner a;
        if (a.plan(map,s,t)){
            auto st = a.getStats();
            out << mf.substr(mf.find_last_of('\\')+1) << ",AStar," << st.success << "," << st.time_ms << "," << st.nodes_expanded << "," << st.path_cost << "\n";
        } else {
            out << mf.substr(mf.find_last_of('\\')+1) << ",AStar,0,0,0,0\n";
        }
    }
    std::cout << "Eval done. Results in eval_results.csv" << std::endl;
    return 0;
}

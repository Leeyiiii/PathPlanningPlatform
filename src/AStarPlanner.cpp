#include "AStarPlanner.h"
#include "PlannerConfig.h"
#include "SearchCommon.h"

#include <queue>
#include <chrono>
#include <limits>
#include <cmath>
#include <algorithm>

namespace {

std::vector<Point> reconstructPath(int bestState, const std::vector<int> &parent, int width) {
    std::vector<Point> rev;
    int state = bestState;
    while (state != -1) {
        const int cell = stateCellIndex(state);
        rev.emplace_back(cell % width, cell / width);
        state = parent[state];
    }
    std::reverse(rev.begin(), rev.end());
    return rev;
}

} // namespace

static float heuristic(const Point &a, const Point &b, float minCost){
    return manhattanHeuristic(a, b, minCost);
}

bool AStarPlanner::plan(const Map &map, const Point &s, const Point &t){
    path.clear();
    stats = Stats();
    setTurnPenalty(turnPenalty());
    if (!map.isFree(s.x, s.y) || !map.isFree(t.x, t.y)) return false;
    int w = map.width(), h = map.height();
    const int stateCount = w * h * 5;
    const float INF = std::numeric_limits<float>::infinity();
    std::vector<float> g(stateCount, INF), f(stateCount, INF);
    std::vector<int> parent(stateCount, -1);
    std::vector<char> exploredCell(w * h, 0);
    using P = std::pair<float,int>;
    std::priority_queue<P, std::vector<P>, std::greater<P>> open;
    const int startCell = cellIndex(s.x, s.y, w);
    const int targetCell = cellIndex(t.x, t.y, w);
    const int startState = stateIndex(startCell, static_cast<int>(MoveDir::None));
    const float minCost = map.minTraversableCost();
    g[startState] = 0;
    f[startState] = heuristic(s, t, minCost);
    open.push({f[startState], startState});
    int expanded = 0;
    const int SAMPLE_INTERVAL = 10;
    std::vector<Stats::ProgressSample> samples;
    int maxOpen = 0;
    auto t0 = std::chrono::high_resolution_clock::now();
    while(!open.empty()){
        auto [fv, state] = open.top(); open.pop();
        if (fv > f[state]) continue;
        ++expanded;
        const int cell = stateCellIndex(state);
        if (!exploredCell[cell]) {
            exploredCell[cell] = 1;
            stats.explored_cells.emplace_back(cell % w, cell / w);
        }
        if (cell == targetCell) break;
        const int ux = cell % w;
        const int uy = cell / w;
        const int prevDir = stateDirectionIndex(state);
        for (const auto &nb : map.neighbors({ux,uy})){
            const int nextCell = cellIndex(nb.x, nb.y, w);
            const int nextDir = directionIndex(nb.x - ux, nb.y - uy);
            const float ng = g[state] + map.cost(nb.x, nb.y) + turningPenalty(prevDir, nextDir, turnPenalty());
            const int nextState = stateIndex(nextCell, nextDir);
            if (ng < g[nextState]){
                g[nextState] = ng;
                f[nextState] = ng + heuristic(nb, t, minCost);
                parent[nextState] = state;
                open.push({f[nextState], nextState});
            }
        }
        // record open size and occasional sample
        const int openSize = static_cast<int>(open.size());
        if (openSize > maxOpen) maxOpen = openSize;
        if ((expanded % SAMPLE_INTERVAL) == 0) {
            auto tnow = std::chrono::high_resolution_clock::now();
            double elapsed = std::chrono::duration<double, std::milli>(tnow - t0).count();
            samples.push_back({elapsed, expanded, openSize});
        }
    }
    auto t1 = std::chrono::high_resolution_clock::now();
    stats.time_ms = std::chrono::duration<double, std::milli>(t1-t0).count();
    stats.nodes_expanded = expanded;
    int bestState = -1;
    float bestCost = INF;
    for (int dir = 0; dir < 5; ++dir) {
        const int state = stateIndex(targetCell, dir);
        if (g[state] < bestCost) {
            bestCost = g[state];
            bestState = state;
        }
    }
    if (bestState == -1 || !std::isfinite(bestCost)){
        stats.success = false;
        return false;
    }
    stats.success=true;
    path = reconstructPath(bestState, parent, w);
    stats.path_length = bestCost;
    stats.path_cost = bestCost;
    stats.max_open_size = maxOpen;
    stats.samples = std::move(samples);
    return true;
}

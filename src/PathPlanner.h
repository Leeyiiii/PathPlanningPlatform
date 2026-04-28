#pragma once
#include "Map.h"
#include "Point.h"
#include <vector>

struct Stats {
    bool success = false;
    double time_ms = 0.0;
    int nodes_expanded = 0;
    double path_length = 0.0;
    double path_cost = 0.0;
    // maximum open set size observed during search
    int max_open_size = 0;
    struct ProgressSample {
        double time_ms = 0.0; // elapsed time since start
        int nodes_expanded = 0;
        int open_size = 0;
    };
    // sampled progress over time (for plotting)
    std::vector<ProgressSample> samples;
    std::vector<Point> explored_cells;
};

class PathPlanner {
public:
    virtual ~PathPlanner() = default;
    virtual bool plan(const Map &map, const Point &s, const Point &t) = 0;
    virtual std::vector<Point> getPath() const = 0;
    virtual Stats getStats() const = 0;
};

#pragma once
#include "PathPlanner.h"
#include <vector>
#include <unordered_map>

class DijkstraPlanner : public PathPlanner {
public:
    DijkstraPlanner() = default;
    bool plan(const Map &map, const Point &s, const Point &t) override;
    std::vector<Point> getPath() const override { return path; }
    Stats getStats() const override { return stats; }
private:
    std::vector<Point> path;
    Stats stats;
};

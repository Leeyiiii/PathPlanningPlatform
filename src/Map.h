#pragma once
#include <vector>
#include <string>
#include "Point.h"

class Map {
public:
    Map() = default;
    bool loadFromTxt(const std::string &path);
    int width() const;
    int height() const;
    bool isFree(int x, int y) const;
    float cost(int x, int y) const; // 支持代价图
    float minTraversableCost() const;
    bool hasWeightedCosts() const;
    std::vector<Point> neighbors(const Point &p) const;
private:
    int w=0, h=0;
    std::vector<int> grid; // 0 free, 1 obstacle
    std::vector<float> costGrid; // optional
    bool weighted = false;
    float minCost = 1.0f;
};

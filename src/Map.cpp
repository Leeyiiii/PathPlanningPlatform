#include "Map.h"
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <limits>
#include <algorithm>
#include <cmath>

bool Map::loadFromTxt(const std::string &path) {
    std::ifstream in(path);
    if (!in) return false;
    std::vector<std::vector<int>> rows;
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty()) continue;
        std::istringstream iss(line);
        std::vector<int> row;
        int v;
        while (iss >> v) row.push_back(v);
        if (!row.empty()) rows.push_back(row);
    }
    if (rows.empty()) return false;
    bool binaryMode = true;
    for (const auto &row : rows) {
        for (int v : row) {
            if (v < 0) return false;
            if (v > 1) binaryMode = false;
        }
    }
    h = (int)rows.size();
    w = (int)rows[0].size();
    grid.assign(w*h, 1);
    costGrid.assign(w*h, 1.0f);
    weighted = !binaryMode;
    minCost = std::numeric_limits<float>::infinity();
    for (int i = 0; i < h; ++i) {
        if ((int)rows[i].size() != w) return false;
        for (int j = 0; j < w; ++j) {
            const int value = rows[i][j];
            const int index = i * w + j;
            if (binaryMode) {
                grid[index] = value;
                if (value == 0) {
                    costGrid[index] = 1.0f;
                    minCost = std::min(minCost, 1.0f);
                } else {
                    costGrid[index] = 1e9f;
                }
            } else {
                if (value == 0) {
                    grid[index] = 1;
                    costGrid[index] = 1e9f;
                } else {
                    grid[index] = 0;
                    costGrid[index] = static_cast<float>(value);
                    minCost = std::min(minCost, static_cast<float>(value));
                }
            }
        }
    }
    if (!std::isfinite(minCost)) {
        minCost = 1.0f;
    }
    return true;
}

int Map::width() const { return w; }
int Map::height() const { return h; }
bool Map::isFree(int x, int y) const {
    if (x < 0 || x >= w || y < 0 || y >= h) return false;
    return grid[y*w + x] == 0;
}

float Map::cost(int x, int y) const {
    if (x < 0 || x >= w || y < 0 || y >= h) return 1e9f;
    return costGrid[y*w + x];
}

float Map::minTraversableCost() const {
    return minCost;
}

bool Map::hasWeightedCosts() const {
    return weighted;
}

std::vector<Point> Map::neighbors(const Point &p) const {
    std::vector<Point> nb;
    const int dx[4] = {1,-1,0,0};
    const int dy[4] = {0,0,1,-1};
    for (int k=0;k<4;++k) {
        int nx = p.x + dx[k];
        int ny = p.y + dy[k];
        if (isFree(nx, ny)) nb.emplace_back(nx, ny);
    }
    return nb;
}

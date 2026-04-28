#pragma once

#include "Point.h"

#include <cmath>

enum class MoveDir : int {
    None = 0,
    Up = 1,
    Down = 2,
    Left = 3,
    Right = 4,
};

inline int cellIndex(int x, int y, int width) {
    return y * width + x;
}

inline int stateIndex(int cell, int dir) {
    return cell * 5 + dir;
}

inline int stateCellIndex(int state) {
    return state / 5;
}

inline int stateDirectionIndex(int state) {
    return state % 5;
}

inline int directionIndex(int dx, int dy) {
    if (dx == 0 && dy == -1) return static_cast<int>(MoveDir::Up);
    if (dx == 0 && dy == 1) return static_cast<int>(MoveDir::Down);
    if (dx == -1 && dy == 0) return static_cast<int>(MoveDir::Left);
    if (dx == 1 && dy == 0) return static_cast<int>(MoveDir::Right);
    return static_cast<int>(MoveDir::None);
}

inline float turningPenalty(int previousDirection, int nextDirection, float penalty) {
    if (previousDirection == static_cast<int>(MoveDir::None) || previousDirection == nextDirection) {
        return 0.0f;
    }
    return penalty;
}

inline float manhattanHeuristic(const Point &a, const Point &b, float minCost) {
    return static_cast<float>(std::abs(a.x - b.x) + std::abs(a.y - b.y)) * minCost;
}

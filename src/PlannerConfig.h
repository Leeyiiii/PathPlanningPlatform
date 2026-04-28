#pragma once

#include <algorithm>

inline float g_turn_penalty = 0.5f;

inline void setTurnPenalty(float value) {
    g_turn_penalty = std::max(0.0f, value);
}

inline float turnPenalty() {
    return g_turn_penalty;
}

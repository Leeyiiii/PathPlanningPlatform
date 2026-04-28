#pragma once

struct Point {
    int x;
    int y;
    Point(): x(0), y(0) {}
    Point(int xx, int yy): x(xx), y(yy) {}
    bool operator==(const Point &o) const { return x==o.x && y==o.y; }
};

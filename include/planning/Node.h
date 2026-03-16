#pragma once

#include "math/Vec2.h"

struct Node {

    Vec2 position;
    int parent;
    double cost;

    Node(const Vec2& p, int parentIndex, double c)
        : position(p), parent(parentIndex), cost(c) {}
};

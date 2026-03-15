#pragma once

#include "math/Vec2.h"

struct Node {

    Vec2 position;
    int parent;

    Node(const Vec2& pos, int p)
        : position(pos), parent(p) {}
};

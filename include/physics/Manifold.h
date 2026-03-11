#pragma once 

#include "math/Vec2.h"

struct Manifold {
    bool colliding = false;
    Vec2 normal{0.0,0.0};
    double penetrationDepth = 0.0;
};
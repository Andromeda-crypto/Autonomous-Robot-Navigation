#pragma once

#include "math/Vec2.h"

struct  AABB
{
    Vec2 min;
    Vec2 max;

    AABB() = default;

    AABB(const Vec2& minPoint, const Vec2& maxPoint) 
     : min(minPoint), max(maxPoint) {}

    bool overlaps(const AABB& other) const
    {
        if (max.x < other.min.x || min.x > other.max.x) {
            return false;
        }
        if (max.y < other.min.y || min.y > other.max.y) {
            return false;
        }

        return true;
    }
};

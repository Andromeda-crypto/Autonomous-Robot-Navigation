#pragma once

#include "math/Vec2.h"

struct Box {
    Vec2 center;
    Vec2 halfSize;

    Box() = default;
    Box(const Vec2& c, const Vec2& h)
    : center(c), halfSize(h) {}
};

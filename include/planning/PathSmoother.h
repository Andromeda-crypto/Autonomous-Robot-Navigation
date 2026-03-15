#pragma once

#include <vector>

#include "math/Vec2.h"
#include "mapping/OccupancyGrid.h"

class PathSmoother 
{
    public:
    static std::vector<Vec2> shortcut(
        const std::vector<Vec2>& path,
        const OccupancyGrid& grid
    );

    private:
    static bool collisionFree(
        const Vec2& a, 
        const Vec2& b,
        const OccupancyGrid& grid
    );
};




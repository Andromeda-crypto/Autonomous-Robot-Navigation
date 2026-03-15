#include "planning/PathSmoother.h"


std::vector<Vec2> PathSmoother::shortcut(
    const std::vector<Vec2>& path,
    const OccupancyGrid& grid
)

{
    if (path.size() < 3) 
        return path;

    std::vector<Vec2> result;

    result.push_back(path.front());

    size_t i = 0;
    while (i < path.size() - 1) {
        size_t j = path.size() - 1;
        for (; j > i + 1; j--) {
            if (collisionFree(path[i],path[j],grid))
                break;    
        }

        result.push_back(path[j]);
        i = j;

    }

    return result;

}

bool PathSmoother::collisionFree(
    const Vec2& a,
    const Vec2& b,
    const OccupancyGrid& grid
)
{
    const int samples = 20;
    
    for (int i = 0; i<=samples; i++) {
        double t = (double) i / samples;
        Vec2 p = a + (b - a) * t;
        int gx = static_cast<int>(p.x / grid.getCellSize());
        int gy = static_cast<int>(p.y / grid.getCellSize());

        if (gx < 0 || gx >= grid.getWidth() ||
            gy < 0 || gy >= grid.getHeight())
            return false;

        if (grid.getProbability(gx, gy) > 0.7)
            return false;

    }
    return true;
}
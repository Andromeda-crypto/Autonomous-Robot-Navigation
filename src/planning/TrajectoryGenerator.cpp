#include "planning/TrajectoryGenerator.h"

Vec2 TrajectoryGenerator::catmullRom(
    const Vec2& p0,
    const Vec2& p1, 
    const Vec2& p2,
    const Vec2& p3,
    double t
)
{
    double t2 = t * t;
    double t3 = t2 * t;

    return 
        p0 * (-0.5*t3 + t2 - 0.5*t) +
        p1 * ( 1.5*t3 - 2.5*t2 + 1.0) +
        p2 * (-1.5*t3 + 2.0*t2 + 0.5*t) +
        p3 * ( 0.5*t3 - 0.5*t2);
}

std::vector<Vec2> TrajectoryGenerator::generate(
    const std::vector<Vec2>& path,
    double resolution)
{
    std::vector<Vec2> trajectory;

    if (path.size() < 4)
        return path;

    for (size_t i = 1; i < path.size() - 2; i++)
    {
        const Vec2& p0 = path[i-1];
        const Vec2& p1 = path[i];
        const Vec2& p2 = path[i+1];
        const Vec2& p3 = path[i+2];

        for (double t = 0; t <= 1.0; t += resolution / 100.0)
        {
            trajectory.push_back(catmullRom(p0,p1,p2,p3,t));
        }
    }

    trajectory.push_back(path.back());

    return trajectory;
}



#pragma once

#include <vector>

#include "math/Vec2.h"
#include "mapping/OccupancyGrid.h"
#include "planning/Node.h"

class RRT
{
public:

    RRT();

    void reset(const Vec2& start);

    bool expand(
        const Vec2& goal,
        const OccupancyGrid& grid,
        std::vector<Vec2>& path
    );

    const std::vector<Node>& getTree() const;

private:

    std::vector<Node> tree;

    double stepSize = 25.0;
    double goalRadius = 30.0;
    double neighborRadius = 60.0;

    int nearest(const Vec2& point) const;

    std::vector<int> near(const Vec2& point) const;

    Vec2 steer(const Vec2& from, const Vec2& to) const;

    bool collisionFree(
        const Vec2& a,
        const Vec2& b,
        const OccupancyGrid& grid
    ) const;

    std::vector<Vec2> extractPath(int goalIndex) const;
};
#pragma once

#include "math/Vec2.h"
#include "mapping/OccupancyGrid.h"
#include "planning/Node.h"

class RRT {
    private:
    std::vector<Node> tree;

    double stepSize = 20.0;
    double goalThreshold = 15.0;

    public:
    void reset(const Vec2& start);

    int nearestNode(const Vec2& point) const;

    Vec2 steer(const Vec2& from, const Vec2& to) const;
    
    Vec2 randomPoint(const OccupancyGrid& grid) const;
 
    bool expand(
        const Vec2& goal,
        const OccupancyGrid& grid,
        std::vector<Vec2>& path
    );
    bool isCollisionFree(
        const Vec2& a,
        const Vec2& b,
        const OccupancyGrid& grid) const;

    std::vector<Vec2> buildPath(
        const Vec2& start,
        const Vec2& goal,
        const OccupancyGrid& grid) ;

    std::vector<Vec2> extractPath(int goalIndex) const;

    const std::vector<Node>& getTree() const;
};
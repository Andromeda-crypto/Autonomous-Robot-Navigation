#include "planning/RRT.h"
#include <vector>

#include <random>

void RRT::reset(const Vec2& start)
{
    tree.clear();
    tree.emplace_back(start, -1);
}

int RRT::nearestNode(const Vec2& point) const 
{
    int bestIndex = 0;
    double bestDist = (tree[0].position - point).magnitudeSquared();

    for (size_t i = 1; i < tree.size(); i++)
    {
        double d =
            (tree[i].position - point).magnitudeSquared();

        if (d < bestDist)
        {
            bestDist = d;
            bestIndex = i;
        }
    }

    return bestIndex;

}

Vec2 RRT::steer(const Vec2& from, const Vec2& to) const

{
    Vec2 direction = (to - from).normalize();
    return from + direction * stepSize;
}

const std::vector<Node>& RRT::getTree() const 
{
    return tree;
}


Vec2 RRT::randomPoint(const OccupancyGrid& grid) const 
{
    static std::mt19937 rng(std::random_device{}());

    std::uniform_real_distribution<double> xDist(
        0, grid.getWidth()
    );

    std::uniform_real_distribution<double> yDist(
        0, grid.getHeight()
    );

    return Vec2(xDist(rng), yDist(rng));
}


bool RRT::isCollisionFree(const Vec2&a, const Vec2&b, const OccupancyGrid& grid) const 
{
    const int samples = 20;
    
    for (int i=0; i<=samples; i++) {
        double t = (double) i / samples;
        Vec2 p = a + (b-a) * t;

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


std::vector<Vec2> RRT::buildPath(
    const Vec2& start,
    const Vec2& goal,
    const OccupancyGrid& grid
) 

{
    reset(start);
    tree.emplace_back(start,-1);
    
    const int maxIterations = 5000;
    
    for (int i = 0; i<= 5000; i++) {
        Vec2 sample = randomPoint(grid);
        int nearest = nearestNode(sample);

        Vec2 newPoint =
            steer(tree[nearest].position, sample);
        
        if (!isCollisionFree(tree[nearest].position, newPoint, grid))
            continue;
        
        tree.emplace_back(newPoint, nearest);

        int newIndex = tree.size() - 1;
        
        if ((newPoint - goal).magnitude() < goalThreshold) {
            tree.emplace_back(goal, newIndex);

            return extractPath(tree.size() - 1);
        }
    }

    return {};
}

std::vector<Vec2> RRT::extractPath(int goalIndex) const 
{
    std::vector<Vec2> path;
    int current = goalIndex;

    while (current != -1) {
        path.push_back(tree[current].position);
        current = tree[current].parent;   
    }

    std::reverse(path.begin(), path.end());

    return path;
}


bool RRT::expand(
    const Vec2& goal, 
    const OccupancyGrid& grid,
    std::vector<Vec2>& path
)
{
    if (tree.empty()) {
        return false;
    }

    Vec2 sample;

    if (rand() % 10 == 0)
        sample = goal;
    else
        sample = randomPoint(grid);

    int nearest = nearestNode(sample);

    Vec2 newPoint =
        steer(tree[nearest].position, sample);

    if (!isCollisionFree(
            tree[nearest].position,
            newPoint,
            grid))
        return false;

    tree.emplace_back(newPoint, nearest);

    int newIndex = tree.size() - 1;

    if ((newPoint - goal).magnitude() < goalThreshold)
    {
        tree.emplace_back(goal, newIndex);

        path = extractPath(tree.size() - 1);

        return true;
    }

    return false;
}
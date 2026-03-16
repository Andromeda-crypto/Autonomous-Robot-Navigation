#include "planning/RRT.h"

#include <random>
#include <limits>
#include <algorithm>

static std::mt19937 rng(std::random_device{}());

RRT::RRT() {}

void RRT::reset(const Vec2& start)
{
    tree.clear();
    tree.emplace_back(start, -1, 0.0);
}

const std::vector<Node>& RRT::getTree() const
{
    return tree;
}

int RRT::nearest(const Vec2& point) const
{
    int bestIndex = 0;
    double bestDist = std::numeric_limits<double>::max();

    for (int i = 0; i < tree.size(); i++)
    {
        double d = (tree[i].position - point).magnitude();

        if (d < bestDist)
        {
            bestDist = d;
            bestIndex = i;
        }
    }

    return bestIndex;
}

std::vector<int> RRT::near(const Vec2& point) const
{
    std::vector<int> neighbors;

    for (int i = 0; i < tree.size(); i++)
    {
        if ((tree[i].position - point).magnitude() < neighborRadius)
        {
            neighbors.push_back(i);
        }
    }

    return neighbors;
}

Vec2 RRT::steer(const Vec2& from, const Vec2& to) const
{
    Vec2 dir = to - from;

    double dist = dir.magnitude();

    if (dist < stepSize)
        return to;

    return from + dir.normalize() * stepSize;
}

bool RRT::collisionFree(
    const Vec2& a,
    const Vec2& b,
    const OccupancyGrid& grid) const
{
    const int samples = 20;

    for (int i = 0; i <= samples; i++)
    {
        double t = (double)i / samples;

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

std::vector<Vec2> RRT::extractPath(int goalIndex) const
{
    std::vector<Vec2> path;

    int current = goalIndex;

    while (current != -1)
    {
        path.push_back(tree[current].position);
        current = tree[current].parent;
    }

    std::reverse(path.begin(), path.end());

    return path;
}

bool RRT::expand(
    const Vec2& goal,
    const OccupancyGrid& grid,
    std::vector<Vec2>& path)
{
    std::uniform_real_distribution<double> prob(0.0,1.0);

    Vec2 sample;

    if(prob(rng) < 0.15)
    {
        sample = goal;
    }
    else
    {
        std::uniform_real_distribution<double> xdist(0,800);
        std::uniform_real_distribution<double> ydist(0,600);

        sample = Vec2(xdist(rng), ydist(rng));
    }

    int nearestIndex = nearest(sample);

    Vec2 newPoint = steer(tree[nearestIndex].position, sample);

    if(!collisionFree(tree[nearestIndex].position,newPoint,grid))
        return false;

    auto neighbors = near(newPoint);

    int bestParent = nearestIndex;

    double bestCost =
        tree[nearestIndex].cost +
        (newPoint - tree[nearestIndex].position).magnitude();

    for(int idx : neighbors)
    {
        double cost =
            tree[idx].cost +
            (newPoint - tree[idx].position).magnitude();

        if(cost < bestCost &&
           collisionFree(tree[idx].position,newPoint,grid))
        {
            bestParent = idx;
            bestCost = cost;
        }
    }

    tree.emplace_back(newPoint,bestParent,bestCost);

    int newIndex = tree.size() - 1;

    for(int idx : neighbors)
    {
        double newCost =
            tree[newIndex].cost +
            (tree[idx].position - newPoint).magnitude();

        if(newCost < tree[idx].cost &&
           collisionFree(newPoint,tree[idx].position,grid))
        {
            tree[idx].parent = newIndex;
            tree[idx].cost = newCost;
        }
    }

    if((newPoint - goal).magnitude() < goalRadius)
    {
        tree.emplace_back(goal,newIndex,tree[newIndex].cost);

        path = extractPath(tree.size()-1);

        return true;
    }

    return false;
}
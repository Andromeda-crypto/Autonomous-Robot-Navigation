#include "physics/SpatialHashGrid.h"

SpatialHashGrid::SpatialHashGrid(double size)
    : cellSize(size)
{
}

void SpatialHashGrid::clear()
{
    cells.clear();
}

long long SpatialHashGrid::hash(int x, int y) const
{
    return (static_cast<long long>(x) << 32) ^ static_cast<unsigned int>(y);
}

void SpatialHashGrid::insert(const RigidBody& body, int bodyIndex)
{
    int minX = static_cast<int>(body.aabb.min.x / cellSize);
    int maxX = static_cast<int>(body.aabb.max.x / cellSize);

    int minY = static_cast<int>(body.aabb.min.y / cellSize);
    int maxY = static_cast<int>(body.aabb.max.y / cellSize);

    for (int x = minX; x <= maxX; x++)
    {
        for (int y = minY; y <= maxY; y++)
        {
            long long key = hash(x, y);
            cells[key].push_back(bodyIndex);
        }
    }
}

const std::unordered_map<long long, std::vector<int>>& SpatialHashGrid::getCells() const
{
    return cells;
}
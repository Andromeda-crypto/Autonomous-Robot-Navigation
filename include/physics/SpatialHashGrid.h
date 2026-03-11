#pragma once

#include <unordered_map>
#include <vector>

#include "physics/RigidBody.h"

class SpatialHashGrid
{
private:

    double cellSize;

    std::unordered_map<long long, std::vector<int>> cells;

    long long hash(int x, int y) const;

public:

    SpatialHashGrid(double size);

    void clear();

    void insert(const RigidBody& body, int bodyIndex);

    const std::unordered_map<long long, std::vector<int>>& getCells() const;
};
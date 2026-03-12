#pragma once

#include <vector>

#include "physics/RigidBody.h"
#include "physics/SpatialHashGrid.h"

class World
{
private:

    std::vector<RigidBody> bodies;

    SpatialHashGrid grid{100.0};

public:

    RigidBody& addBody(const RigidBody& body);

    void step(double dt);

    const std::vector<RigidBody>& getBodies() const;
};
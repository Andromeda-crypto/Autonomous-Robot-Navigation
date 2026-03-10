#pragma once

#include <vector>
#include "physics/RigidBody.h"

class World
{
private:
    std::vector<RigidBody> bodies;

public:

    void addBody(const RigidBody& body);

    void step(double dt);

    const std::vector<RigidBody>& getBodies() const;
};
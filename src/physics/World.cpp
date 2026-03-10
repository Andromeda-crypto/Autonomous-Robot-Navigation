#include "physics/World.h"


const std::vector<RigidBody>& World::getBodies() const
{
    return bodies;
}
void World::addBody(const RigidBody& body)
{
    bodies.push_back(body);
}

void World::step(double dt)
{
    for (auto& body : bodies)
    {
        body.integrate(dt);
    }
}
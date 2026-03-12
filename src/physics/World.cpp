#include "physics/World.h"
#include "physics/Collision.h"
#include "physics/Resolver.h"

const std::vector<RigidBody>& World::getBodies() const
{
    return bodies;
}

RigidBody& World::addBody(const RigidBody& body)
{
    bodies.push_back(body);
    return bodies.back();
}

void World::step(double dt)
{
    // Integrate motion (no global gravity for now – robot is driven kinematically)
    for (auto& body : bodies)
        body.integrate(dt);

    // Broad phase grid rebuild
    grid.clear();

    for (size_t i = 0; i < bodies.size(); i++)
    {
        bodies[i].updateAABB();
        grid.insert(bodies[i], i);
    }

    const int iterations = 8;

    for (int k = 0; k < iterations; k++)
    {
        for (const auto& cell : grid.getCells())
        {
            const auto& indices = cell.second;

            for (size_t i = 0; i < indices.size(); i++)
            {
                for (size_t j = i + 1; j < indices.size(); j++)
                {
                    int a = indices[i];
                    int b = indices[j];

                    if (!bodies[a].aabb.overlaps(bodies[b].aabb))
                        continue;

                    Manifold m = checkCircleCircle(bodies[a], bodies[b]);

                    if (m.colliding)
                        resolveCollision(bodies[a], bodies[b], m);
                }
            }
        }
    }
}
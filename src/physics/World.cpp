#include "physics/World.h"
#include "physics/Collision.h"
#include "physics/Resolver.h"

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
    // Apply gravity
    Vec2 gravity(0, 500);

    for (auto& body : bodies)
    {
        if (!body.isStatic)
        {
            body.applyForce(gravity);
        }
    }

    // Integrate motion
    for (auto& body : bodies)
    {
        body.integrate(dt);
    }

    // Build spatial grid
    grid.clear();

    for (size_t i = 0; i < bodies.size(); i++)
    {
        bodies[i].updateAABB();
        grid.insert(bodies[i], static_cast<int>(i));
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
                    {
                        resolveCollision(bodies[a], bodies[b], m);
                    }
                }
            }
        }
    }
}
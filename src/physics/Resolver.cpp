#include "physics/Resolver.h"
#include <algorithm>

void resolveCollision(RigidBody& a, RigidBody& b, const Manifold& m)
{
    Vec2 relativeVelocity = b.velocity - a.velocity;

    double velocityAlongNormal = relativeVelocity.dot(m.normal);

    if (velocityAlongNormal > 0)
        return;

    double restitution = 0.8;

    double impulseMagnitude =
        -(1 + restitution) * velocityAlongNormal /
        (a.inverseMass + b.inverseMass);

    Vec2 impulse = m.normal * impulseMagnitude;

    a.velocity -= impulse * a.inverseMass;
    b.velocity += impulse * b.inverseMass;

    
    const double percent = 0.8; 
    const double slop = 0.01;     

    Vec2 correction = m.normal *
        (std::max(m.penetrationDepth - slop, 0.0) /
        (a.inverseMass + b.inverseMass)) * percent;

    a.position -= correction * a.inverseMass;
    b.position += correction * b.inverseMass;
}
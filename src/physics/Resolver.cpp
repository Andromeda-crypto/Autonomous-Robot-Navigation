#include "physics/Resolver.h"
#include <algorithm>
#include <cmath>

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

    relativeVelocity = b.velocity - a.velocity;

    Vec2 tangent =
        relativeVelocity - m.normal * relativeVelocity.dot(m.normal);

    if (tangent.magnitudeSquared() > 0.0)
        tangent = tangent.normalize();

    double jt =
        -relativeVelocity.dot(tangent) /
        (a.inverseMass + b.inverseMass);

    double mu_s =
        std::sqrt(a.staticFriction * b.staticFriction);

    double mu_d =
        std::sqrt(a.dynamicFriction * b.dynamicFriction);

    Vec2 frictionImpulse;

    if (std::abs(jt) < impulseMagnitude * mu_s)
    {
        // Static friction
        frictionImpulse = tangent * jt;
    }
    else
    {
        // Dynamic friction
        frictionImpulse = tangent * (-impulseMagnitude * mu_d);
    }

    a.velocity -= frictionImpulse * a.inverseMass;
    b.velocity += frictionImpulse * b.inverseMass;

    const double percent = 0.8;
    const double slop = 0.01;

    Vec2 correction = m.normal *
        (std::max(m.penetrationDepth - slop, 0.0) /
        (a.inverseMass + b.inverseMass)) * percent;

    a.position -= correction * a.inverseMass;
    b.position += correction * b.inverseMass;
}
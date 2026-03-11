#include "physics/Collision.h"
#include <algorithm>
#include <cmath>

Manifold checkCircleCircle(const RigidBody& a, const RigidBody& b) {
    Manifold result;
    Vec2 delta = b.position - a.position;
    double distanceSquared = delta.magnitudeSquared();

    double radiusSum = a.radius + b.radius;
    double radiusSumSquared = radiusSum * radiusSum;

    if (distanceSquared > radiusSumSquared) {
        return result;
    }
    
    double distance = std::sqrt(distanceSquared);

    result.colliding = true;

    if (distance == 0.0)
    {
        result.normal = Vec2(1.0, 0.0);
        result.penetrationDepth = radiusSum;
    }
    else
    {
        result.normal = delta / distance;
        result.penetrationDepth = radiusSum - distance;
    }

    return result;

}

Manifold checkCircleAABB(const RigidBody& circle, const Box& box)
{
    Manifold result;

    double closestX = std::clamp(circle.position.x,
                                 box.center.x - box.halfSize.x,
                                 box.center.x + box.halfSize.x);

    double closestY = std::clamp(circle.position.y,
                                 box.center.y - box.halfSize.y,
                                 box.center.y + box.halfSize.y);

    Vec2 closestPoint(closestX, closestY);
    Vec2 delta = circle.position - closestPoint;

    double distanceSquared = delta.magnitudeSquared();
    double radiusSquared = circle.radius * circle.radius;

    if (distanceSquared > radiusSquared)
        return result;

    result.colliding = true;

    double distance = std::sqrt(distanceSquared);

    if (distance == 0.0)
    {
        result.normal = Vec2(0.0, -1.0);
        result.penetrationDepth = circle.radius;
    }
    else
    {
        result.normal = delta / distance;
        result.penetrationDepth = circle.radius - distance;
    }

    return result;
}
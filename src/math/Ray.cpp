#include "math/Ray.h"

// Based on standard 2D ray/segment intersection using cross products.
RayHit intersectRaySegment(const Ray& ray, const Vec2& p1, const Vec2& p2)
{
    RayHit result;

    Vec2 v1 = ray.origin - p1;
    Vec2 v2 = p2 - p1;
    Vec2 v3(-ray.direction.y, ray.direction.x); // perpendicular to ray direction

    double dot = v2.dot(v3);
    if (std::abs(dot) < 1e-8)
    {
        // Parallel: no single intersection
        return result;
    }

    double t1 = v2.cross(v1) / dot;
    double t2 = v1.dot(v3) / dot;

    if (t1 >= 0.0 && t2 >= 0.0 && t2 <= 1.0)
    {
        result.hit = true;
        result.t = t1;
        result.point = ray.origin + ray.direction * t1;
    }

    return result;
}


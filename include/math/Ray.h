#pragma once

#include "math/Vec2.h"

struct Ray
{
    Vec2 origin;
    Vec2 direction; // assumed normalized
};

// Result of intersecting a ray with a segment or shape
struct RayHit
{
    bool hit = false;
    double t = 0.0;     // distance along ray: point = origin + t * direction
    Vec2 point;         // world-space hit point
};

// Ray vs. line segment intersection
// Segment defined by p1 -> p2.
RayHit intersectRaySegment(const Ray& ray, const Vec2& p1, const Vec2& p2);


#include "sensors/Lidar.h"

#include <algorithm>
#include <cmath>
#include <limits>

Ray Lidar::getBeamRay(int i) const
{
    Ray ray;
    ray.origin = *originPtr;

    double baseAngle = *anglePtr;
    double f = (numBeams == 1) ? 0.0 : (static_cast<double>(i) / (numBeams - 1) - 0.5);
    double spread = M_PI * 2.0; // full 360 degrees for now
    double theta = baseAngle + f * spread;

    ray.direction = Vec2(std::cos(theta), std::sin(theta));
    return ray;
}

void Lidar::update(
    const std::vector<Box>& walls,
    const std::vector<RigidBody>& bodies)
{
    std::normal_distribution<double> noise(0.0, noiseStdDev);

    for (int i = 0; i < numBeams; ++i)
    {
        Ray ray = getBeamRay(i);
        double bestT = maxRange;

        // Intersect with walls (4 segments per box)
        for (const auto& wall : walls)
        {
            Vec2 min = wall.center - wall.halfSize;
            Vec2 max = wall.center + wall.halfSize;

            Vec2 p1(min.x, min.y);
            Vec2 p2(max.x, min.y);
            Vec2 p3(max.x, max.y);
            Vec2 p4(min.x, max.y);

            RayHit hits[4] = {
                intersectRaySegment(ray, p1, p2),
                intersectRaySegment(ray, p2, p3),
                intersectRaySegment(ray, p3, p4),
                intersectRaySegment(ray, p4, p1)
            };

            for (const auto& h : hits)
            {
                if (h.hit && h.t < bestT)
                    bestT = h.t;
            }
        }

        // Intersect with circular bodies
        for (const auto& body : bodies)
        {
            if (body.inverseMass == 0.0)
                continue;

            Vec2 m = ray.origin - body.position;
            double b = m.dot(ray.direction);
            double c = m.dot(m) - body.radius * body.radius;

            if (c > 0.0 && b > 0.0)
                continue;

            double discr = b * b - c;
            if (discr < 0.0)
                continue;

            double t = -b - std::sqrt(discr);
            if (t >= 0.0 && t < bestT)
                bestT = t;
        }

        double d = std::min(bestT, maxRange);
        d += noise(rng);
        d = std::clamp(d, 0.0, maxRange);

        distances[i] = d;
    }
}


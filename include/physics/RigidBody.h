#pragma once

#include "math/Vec2.h"
#include "physics/AABB.h"

struct RigidBody
{
    Vec2 position;
    Vec2 velocity{0.0, 0.0};
    Vec2 force{0.0, 0.0};

    double angle = 0.0;
    double angularVelocity = 0.0;
    double torque = 0.0;

    double inverseMass = 0.0;
    double inverseInertia = 0.0;

    RigidBody(const Vec2& pos, double mass);
    void applyForce(const Vec2& f);
    void integrate(double dt);
    void applyTorque(double t);

    AABB aabb;
    double radius = 20.0;
    void updateAABB();
    
};



#include "physics/RigidBody.h"



RigidBody::RigidBody(const Vec2& pos,double mass)
{
    position = pos;
    if (mass == 0.0) {
        inverseMass = 0.0;
        isStatic = true;
    }
    else {
        inverseMass = 1/mass;
    }

    inverseInertia = 0.0;
    updateAABB();
}

void RigidBody::applyForce(const Vec2 & f){
    force += f;
}

void RigidBody::integrate(double dt)
{
    if (inverseMass == 0.0)
        return;

    // Linear motion
    Vec2 acceleration = force * inverseMass;

    velocity += acceleration * dt;
    position += velocity * dt;

    // Angular motion
    double angularAcceleration = torque * inverseInertia;

    angularVelocity += angularAcceleration * dt;
    angle += angularVelocity * dt;

    // Reset accumulators
    force = Vec2();
    torque = 0.0;
    updateAABB();
}

void RigidBody::applyTorque(double t){
    torque += t;
}

void RigidBody::updateAABB() {
    Vec2 r(radius, radius);

    aabb.min = position - r;
    aabb.max = position + r;
}
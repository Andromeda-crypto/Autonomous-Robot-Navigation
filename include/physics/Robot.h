#pragma once

#include "physics/RigidBody.h"

struct Robot
{
    RigidBody& body;

    // Wheel configuration
    double wheelBase = 60.0;     
    double wheelRadius = 10.0;  

    double leftWheelSpeed = 0.0;
    double rightWheelSpeed = 0.0;

    explicit Robot(RigidBody& bodyRef);


    void setWheelSpeeds(double left, double right);

    void updateKinematics(double dt);

    Vec2 getPosition() const { return body.position; }
    double getAngle() const { return body.angle; }
};


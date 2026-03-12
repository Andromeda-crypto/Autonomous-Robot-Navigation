#include "physics/Robot.h"

#include <cmath>

Robot::Robot(RigidBody& bodyRef)
    : body(bodyRef)
{}

void Robot::setWheelSpeeds(double left, double right)
{
    leftWheelSpeed = left;
    rightWheelSpeed = right;
}

void Robot::updateKinematics(double dt)
{
    (void)dt;
    double v = 0.5 * (rightWheelSpeed + leftWheelSpeed);
    double omega = (rightWheelSpeed - leftWheelSpeed) / wheelBase;

    Vec2 forward(std::cos(body.angle), std::sin(body.angle));
    body.velocity = forward * v;
    body.angularVelocity = omega;
}


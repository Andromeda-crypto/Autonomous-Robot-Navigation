#pragma once

#include <vector>
#include <cmath>

#include "math/Vec2.h"
#include "physics/Robot.h"
#include "control/PID.h"

// High-level controller that uses PID for heading and speed control
// and drives the robot through a list of waypoints.
class RobotController
{
public:
    explicit RobotController(Robot& robot)
        : robot(robot),
          // Softer gains and tight output limits to avoid oscillation
          headingPID(2.0, 0.0, 0.0, -2.0, 2.0),
          speedPID(1.0, 0.0, 0.0, 0.0, 100.0)
    {}

    void setWaypoints(const std::vector<Vec2>& points)
    {
        waypoints = points;
        currentIndex = 0;
        headingPID.reset();
        speedPID.reset();
    }

    bool hasTarget() const
    {
        return currentIndex < waypoints.size();
    }

   
    void update(double dt)
    {
        if (!hasTarget())
            return;

        const Vec2 target = waypoints[currentIndex];
        const Vec2 pos = robot.getPosition();
        const Vec2 toTarget = target - pos;

        const double distance = toTarget.magnitude();

        // If close enough, advance to next waypoint.
        if (distance < waypointTolerance)
        {
            currentIndex++;

            if (!hasTarget())
            {
                robot.setWheelSpeeds(0.0, 0.0);
                robot.updateKinematics(dt);
                return;
            }
        }

        const Vec2 dir = (hasTarget() ? (waypoints[currentIndex] - robot.getPosition()) : Vec2(0.0, 0.0));
        if (!hasTarget())
            return;

        double desiredHeading = std::atan2(dir.y, dir.x);
        double currentHeading = robot.getAngle();

        double headingError = smallestAngleDifference(desiredHeading, currentHeading);
        double headingCommand = headingPID.update(headingError, dt);

        double currentSpeed = robot.body.velocity.magnitude();

       
        double desiredSpeed = std::min(maxSpeed, distance * 2.0);
        double speedError = desiredSpeed - currentSpeed;
        double speedCommand = speedPID.update(speedError, dt);

        // Convert (v, omega) to left/right wheel speeds.
        double v = speedCommand;
        double omega = headingCommand;

        double L = robot.wheelBase;

        double left = v - 0.5 * omega * L;
        double right = v + 0.5 * omega * L;

        robot.setWheelSpeeds(left, right);
        robot.updateKinematics(dt);
    }

private:
    Robot& robot;
    PID headingPID;
    PID speedPID;

    std::vector<Vec2> waypoints;
    std::size_t currentIndex = 0;

    double waypointTolerance = 20.0;
    double maxSpeed = 80.0;

    static double smallestAngleDifference(double target, double current)
    {
        double diff = target - current;
        while (diff > M_PI) diff -= 2.0 * M_PI;
        while (diff < -M_PI) diff += 2.0 * M_PI;
        return diff;
    }
};


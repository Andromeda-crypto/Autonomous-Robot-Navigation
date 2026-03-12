#pragma once

#include <vector>
#include <random>

#include "math/Vec2.h"
#include "math/Ray.h"
#include "physics/Box.h"
#include "physics/RigidBody.h"

// Simple 2D LIDAR sensor mounted on the robot.
class Lidar
{
public:
    Lidar(
        Vec2* originPtr,
        double* anglePtr,
        double maxRange,
        int numBeams)
        : originPtr(originPtr),
          anglePtr(anglePtr),
          maxRange(maxRange),
          numBeams(numBeams),
          distances(numBeams, maxRange)
    {}

    void setNoise(double stddev)
    {
        noiseStdDev = stddev;
    }

    const std::vector<double>& getDistances() const { return distances; }

    // Cast rays against axis-aligned box walls and circular bodies.
    void update(
        const std::vector<Box>& walls,
        const std::vector<RigidBody>& bodies);

    // Compute world-space ray for beam i (for visualization).
    Ray getBeamRay(int i) const;

private:
    Vec2* originPtr;
    double* anglePtr;

    double maxRange;
    int numBeams;

    std::vector<double> distances;

    double noiseStdDev = 2.0;

    mutable std::mt19937 rng{12345};
};


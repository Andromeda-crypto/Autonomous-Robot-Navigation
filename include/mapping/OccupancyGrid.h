#pragma once

#include <vector>
#include <cmath>

#include "../math/Vec2.h"

// Simple log-odds occupancy grid in screen/world coordinates.
class OccupancyGrid
{
public:
    OccupancyGrid(int width, int height, double cellSize, double sensorMaxRange)
        : width(width),
          height(height),
          cellSize(cellSize),
          sensorMaxRange(sensorMaxRange),
          logOdds(width * height, 0.0)
    {}

    int getWidth() const { return width; }
    int getHeight() const { return height; }
    double getCellSize() const { return cellSize; }

    // World (pixel) position -> grid indices. Returns false if outside grid.
    bool worldToGrid(const Vec2& p, int& ix, int& iy) const
    {
        ix = static_cast<int>(p.x / cellSize);
        iy = static_cast<int>(p.y / cellSize);

        if (ix < 0 || iy < 0 || ix >= width || iy >= height)
            return false;
        return true;
    }

    double getProbability(int ix, int iy) const
    {
        int idx = iy * width + ix;
        double l = logOdds[idx];
        double expL = std::exp(l);
        return expL / (1.0 + expL);
    }

    // Update grid cells along a ray.
    // distance is the measured distance along ray.direction.
    void updateRay(const Vec2& origin, const Vec2& dir, double distance)
    {
        const double step = cellSize * 0.5;

        double maxD = std::min(distance, sensorMaxRange);
        bool hasHit = distance < sensorMaxRange * 0.95;

        // Free space along the ray up to just before the hit 
        double freeMax = hasHit ? (maxD - cellSize) : maxD;
        if (freeMax < 0.0)
            freeMax = 0.0;

        for (double d = 0.0; d <= freeMax; d += step)
        {
            Vec2 p = origin + dir * d;
            markFree(p);
        }

        // Mark the hit cell as occupied if we actually saw something.
        if (hasHit)
        {
            Vec2 hitPoint = origin + dir * maxD;
            markOccupied(hitPoint);
        }
    }

private:
    int width;
    int height;
    double cellSize;
    double sensorMaxRange;

    std::vector<double> logOdds;

    
    const double occIncrement = 0.85;
    const double freeIncrement = -0.4;
    const double minLogOdds = -4.0;
    const double maxLogOdds = 4.0;

    void markOccupied(const Vec2& p)
    {
        int ix, iy;
        if (!worldToGrid(p, ix, iy))
            return;

        int idx = iy * width + ix;
        double val = logOdds[idx] + occIncrement;
        if (val < minLogOdds) val = minLogOdds;
        if (val > maxLogOdds) val = maxLogOdds;
        logOdds[idx] = val;
    }

    void markFree(const Vec2& p)
    {
        int ix, iy;
        if (!worldToGrid(p, ix, iy))
            return;

        int idx = iy * width + ix;
        double val = logOdds[idx] + freeIncrement;
        if (val < minLogOdds) val = minLogOdds;
        if (val > maxLogOdds) val = maxLogOdds;
        logOdds[idx] = val;
    }
};


#pragma once

#include <vector>
#include "math/Vec2.h"

class TrajectoryGenerator {
    public:
        static std::vector<Vec2> generate(
            const std::vector<Vec2>& path,
            double resolution = 5.0
        );

    private:
        static Vec2 catmullRom(
            const Vec2& p0,
            const Vec2& p1,
            const Vec2& p2,
            const Vec2& p3,
            double t
        );

};
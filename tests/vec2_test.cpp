#include "math/Vec2.h"
#include <iostream>
#include <cmath>



int main() {

    Vec2 a(3,4);
    std::cout << "Magnitude of (3,4): " << a.magnitude() << std::endl;

    Vec2 b(1,0);
    Vec2 c(0,1);

    std::cout << "Dot product: " << b.dot(c) << std::endl;
    std::cout << "Cross product: " << b.cross(c) << std::endl;

    Vec2 n = a.normalize();
    std::cout << "Normalized (3,4): " << n.x << ", " << n.y << std::endl;

    Vec2 p = b.perpendicular();
    std::cout << "Perpendicular to (1,0): " << p.x << ", " << p.y << std::endl;

    Vec2 r = Vec2(1,0).rotate(M_PI / 2);
    std::cout << "Rotate (1,0) by 90°: " << r.x << ", " << r.y << std::endl;

    std::cout << "Distance between (3,4) and (1,0): "
              << Vec2::distance(a,b) << std::endl;

    return 0;
}


#pragma once 

#include <vector>
#include "physics/Shape.h"
#include "math/Vec2.h"

class PolygonShape : public Shape

{
    public: 
    std::vector<Vec2> vertices;
    PolygonShape(const std::vector<Vec2>& verts)
        : Shape(Shape::Polygon), vertices(verts) {}
};


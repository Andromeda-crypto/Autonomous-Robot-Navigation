#pragma once
#include <cmath>
#include <cassert>

struct Vec2 {
    float x;
    float y;

    Vec2() : x(0.0f), y(0.0f){}
    Vec2(float xval, float yval) : x(xval) , y(yval) {}
    Vec2 operator+(const Vec2 & other) const {
        return Vec2(x + other.x, y+ other.y);
    }
    Vec2 operator-(const Vec2& other) const {
        return Vec2(x - other.x, y - other.y);
    }
    Vec2 operator*(float scalar) const {
        return Vec2(x * scalar, y * scalar);
    }
    Vec2 operator/(float scalar) const {
    assert(scalar != 0.0f);
    return Vec2(x / scalar, y / scalar);
    }
    Vec2& operator+=(const Vec2& other) {
        x += other.x;
        y += other.y;
        return *this;
    }
    Vec2& operator-=(const Vec2& other) {
        x -= other.x;
        y -= other.y;
        return *this;
    }
    Vec2& operator*=(float scalar) {
    x *= scalar;
    y *= scalar;
    return *this;
    }   
    Vec2& operator/=(float scalar) {
    assert(scalar != 0.0f);
    x /= scalar;
    y /= scalar;
    return *this;
    }

    float dot(const Vec2& other) const{
        return x * other.x + y *other.y;
    }

    float cross(const Vec2& other) const{
        return x *other.y - y *other.x;
    }

    float magnitude() const {
        return std::sqrt(x* x + y* y);
    }

    float magnitudeSquared() const {
    return x * x + y * y;
    }

    Vec2 normalize() const {
    float mag = magnitude();

    if (mag == 0.0f)
        return Vec2(0.0f,0.0f);

    return Vec2(x / mag, y / mag);
    }

    Vec2 perpendicular() const{
        return Vec2(-y,x);
    }

    Vec2 rotate(float angle) const {
    float c = std::cos(angle);
    float s = std::sin(angle);

    return Vec2(
        x * c - y * s,
        x * s + y * c
    );
    }

    static float distance(const Vec2& a, const Vec2& b) {
    return (a - b).magnitude();
    }

    void normalizeInPlace() {
        float mag = magnitude();
        if (mag != 0.0f) {
            x /= mag;
            y /= mag;
        }
    }

};


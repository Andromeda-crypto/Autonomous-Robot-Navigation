#pragma once
#include <cmath>
#include <cassert>

struct Vec2 {
    double x;
    double y;

    Vec2() : x(0.0), y(0.0) {}
    Vec2(double xval, double yval) : x(xval), y(yval) {}

    Vec2 operator+(const Vec2& other) const {
        return Vec2(x + other.x, y + other.y);
    }

    Vec2 operator-(const Vec2& other) const {
        return Vec2(x - other.x, y - other.y);
    }

    Vec2 operator*(double scalar) const {
        return Vec2(x * scalar, y * scalar);
    }

    Vec2 operator/(double scalar) const {
        assert(scalar != 0.0);
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

    Vec2& operator*=(double scalar) {
        x *= scalar;
        y *= scalar;
        return *this;
    }

    Vec2& operator/=(double scalar) {
        assert(scalar != 0.0);
        x /= scalar;
        y /= scalar;
        return *this;
    }

    double dot(const Vec2& other) const {
        return x * other.x + y * other.y;
    }

    double cross(const Vec2& other) const {
        return x * other.y - y * other.x;
    }

    double magnitude() const {
        return std::sqrt(x * x + y * y);
    }

    double magnitudeSquared() const {
        return x * x + y * y;
    }

    Vec2 normalize() const {
        double mag = magnitude();
        if (mag == 0.0) return Vec2(0.0, 0.0);
        return Vec2(x / mag, y / mag);
    }

    Vec2 perpendicular() const {
        return Vec2(-y, x);
    }

    Vec2 rotate(double angle) const {
        double c = std::cos(angle);
        double s = std::sin(angle);

        return Vec2(
            x * c - y * s,
            x * s + y * c
        );
    }

    static double distance(const Vec2& a, const Vec2& b) {
        return (a - b).magnitude();
    }
};

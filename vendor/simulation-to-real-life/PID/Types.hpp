#pragma once
#include <cmath>

/**
 * @file Types.hpp
 * @brief Core mathematical and configuration structures.
 */

struct Vec2 {
    float x{0.0f};
    float y{0.0f};

    Vec2 operator+(const Vec2& o) const { return {x + o.x, y + o.y}; }
    Vec2 operator-(const Vec2& o) const { return {x - o.x, y - o.y}; }
    Vec2 operator*(float s) const { return {x * s, y * s}; }
};

struct Wall {
    Vec2 p1;
    Vec2 p2;
};

struct HardwareConfig {
    float track_width{0.120f};
    float side_angle_deg{45.0f};
    float wheel_radius{0.032f};
    float max_linear_speed{0.45f};
};
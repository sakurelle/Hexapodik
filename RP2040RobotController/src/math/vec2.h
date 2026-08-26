#pragma once

#include <cmath>

namespace hexapod {

struct Vec2 {
    float x = 0.0f;
    float y = 0.0f;

    constexpr Vec2() = default;
    constexpr Vec2(float x_mm, float y_mm) : x(x_mm), y(y_mm) {}

    constexpr Vec2 operator+(Vec2 rhs) const { return {x + rhs.x, y + rhs.y}; }
    constexpr Vec2 operator-(Vec2 rhs) const { return {x - rhs.x, y - rhs.y}; }
    constexpr Vec2 operator*(float scale) const { return {x * scale, y * scale}; }
    constexpr Vec2 operator/(float scale) const { return {x / scale, y / scale}; }

    Vec2& operator+=(Vec2 rhs) {
        x += rhs.x;
        y += rhs.y;
        return *this;
    }
};

inline constexpr float cross(Vec2 a, Vec2 b) { return a.x * b.y - a.y * b.x; }
inline constexpr float dot(Vec2 a, Vec2 b) { return a.x * b.x + a.y * b.y; }
inline float length(Vec2 v) { return std::sqrt(dot(v, v)); }
inline bool isFinite(Vec2 v) { return std::isfinite(v.x) && std::isfinite(v.y); }

}  // namespace hexapod

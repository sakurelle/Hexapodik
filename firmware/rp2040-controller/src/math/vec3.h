#pragma once

#include <cmath>

#include "math/vec2.h"

namespace hexapod {

struct Vec3 {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

    constexpr Vec3() = default;
    constexpr Vec3(float x_mm, float y_mm, float z_mm) : x(x_mm), y(y_mm), z(z_mm) {}

    constexpr Vec3 operator+(Vec3 rhs) const { return {x + rhs.x, y + rhs.y, z + rhs.z}; }
    constexpr Vec3 operator-(Vec3 rhs) const { return {x - rhs.x, y - rhs.y, z - rhs.z}; }
    constexpr Vec3 operator*(float scale) const { return {x * scale, y * scale, z * scale}; }
    constexpr Vec3 operator/(float scale) const { return {x / scale, y / scale, z / scale}; }

    Vec3& operator+=(Vec3 rhs) {
        x += rhs.x;
        y += rhs.y;
        z += rhs.z;
        return *this;
    }

    constexpr Vec2 xy() const { return {x, y}; }
};

inline bool isFinite(Vec3 v) {
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

}  // namespace hexapod

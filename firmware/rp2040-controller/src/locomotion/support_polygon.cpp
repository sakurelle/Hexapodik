#include "locomotion/support_polygon.h"

#include <algorithm>
#include <cmath>

namespace hexapod {
namespace {

bool lessXY(Vec2 a, Vec2 b) {
    if (a.x == b.x) {
        return a.y < b.y;
    }
    return a.x < b.x;
}

float distancePointToLineMm(Vec2 p, Vec2 a, Vec2 b) {
    const Vec2 ab = b - a;
    const float len = length(ab);
    if (len <= 1.0e-5f) {
        return -1.0f;
    }
    return cross(ab, p - a) / len;
}

}  // namespace

SupportPolygonResult computeSupportPolygon(const std::array<Vec3, kLegCount>& feet_body_mm,
                                            const std::array<bool, kLegCount>& contacts) {
    std::array<Vec2, kLegCount> points{};
    std::size_t count = 0;
    for (std::size_t i = 0; i < kLegCount; ++i) {
        if (contacts[i] && isFinite(feet_body_mm[i])) {
            points[count++] = feet_body_mm[i].xy();
        }
    }

    SupportPolygonResult result;
    if (count < 3) {
        return result;
    }

    for (std::size_t i = 1; i < count; ++i) {
        const Vec2 key = points[i];
        std::size_t j = i;
        while (j > 0 && lessXY(key, points[j - 1])) {
            points[j] = points[j - 1];
            --j;
        }
        points[j] = key;
    }

    std::array<Vec2, kLegCount * 2> hull{};
    std::size_t h = 0;
    for (std::size_t i = 0; i < count; ++i) {
        while (h >= 2 && cross(hull[h - 1] - hull[h - 2], points[i] - hull[h - 1]) <= 0.0f) {
            --h;
        }
        hull[h++] = points[i];
    }
    const std::size_t lower_count = h;
    for (std::size_t i = count - 1; i-- > 0;) {
        while (h > lower_count && cross(hull[h - 1] - hull[h - 2], points[i] - hull[h - 1]) <= 0.0f) {
            --h;
        }
        hull[h++] = points[i];
    }
    if (h > 1) {
        --h;
    }

    if (h < 3 || h > kLegCount) {
        return result;
    }

    float margin = 1.0e30f;
    const Vec2 origin{0.0f, 0.0f};
    for (std::size_t i = 0; i < h; ++i) {
        const Vec2 a = hull[i];
        const Vec2 b = hull[(i + 1) % h];
        const float signed_distance = distancePointToLineMm(origin, a, b);
        margin = std::min(margin, signed_distance);
    }

    result.valid = margin >= 0.0f;
    result.margin_mm = result.valid ? margin : -std::fabs(margin);
    result.vertex_count = h;
    for (std::size_t i = 0; i < h; ++i) {
        result.hull[i] = hull[i];
    }
    return result;
}

bool hasStableSupport(const std::array<Vec3, kLegCount>& feet_body_mm,
                      const std::array<bool, kLegCount>& contacts,
                      float required_margin_mm) {
    const SupportPolygonResult polygon = computeSupportPolygon(feet_body_mm, contacts);
    return polygon.valid && polygon.margin_mm >= required_margin_mm;
}

}  // namespace hexapod

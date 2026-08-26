#pragma once

#include <array>

#include "config/robot_config.h"
#include "math/vec2.h"
#include "math/vec3.h"

namespace hexapod {

struct SupportPolygonResult {
    bool valid = false;
    float margin_mm = -1.0f;
    std::size_t vertex_count = 0;
    std::array<Vec2, kLegCount> hull{};
};

SupportPolygonResult computeSupportPolygon(const std::array<Vec3, kLegCount>& feet_body_mm,
                                            const std::array<bool, kLegCount>& contacts);
bool hasStableSupport(const std::array<Vec3, kLegCount>& feet_body_mm,
                      const std::array<bool, kLegCount>& contacts,
                      float required_margin_mm = kSupportMarginMm);

}  // namespace hexapod

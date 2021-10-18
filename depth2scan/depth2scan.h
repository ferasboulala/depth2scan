#pragma once

#include <cmath>

#define DEG2RAD(x) (x / 180 * M_PI)

namespace depth2scan
{
constexpr double VERTICAL_FOV = 43;
constexpr double HORIZONTAL_FOV = 57;
constexpr unsigned DEPTH_HEIGHT = 480;
constexpr unsigned DEPTH_WIDTH = 640;

}  // namespace depth2scan
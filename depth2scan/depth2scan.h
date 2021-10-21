#pragma once

#include <cmath>

#define DEG2RAD(x) (x / 180 * M_PI)

namespace depth2scan
{
constexpr double MAX_DIST = 5;
constexpr double VERTICAL_FOV = 46.6;
constexpr double HORIZONTAL_FOV = 58.5;
constexpr unsigned DEPTH_HEIGHT = 480;
constexpr unsigned DEPTH_WIDTH = 640;

}  // namespace depth2scan
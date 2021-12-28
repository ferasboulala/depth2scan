#pragma once

#include <cmath>
#include <opencv2/opencv.hpp>
#include <vector>

#define DEG2RAD(x) (x / 180.0 * M_PI)

namespace depth2scan
{
namespace limits
{
constexpr double MAX_DIST = 5;
constexpr double MIN_DIST = 0.4;
constexpr double VERTICAL_FOV = 46.6;
constexpr double HORIZONTAL_FOV = 58.5;
constexpr unsigned DEPTH_HEIGHT = 480;
constexpr unsigned DEPTH_WIDTH = 640;

}  // namespace limits

// TODO: Check if it is worth threading.
std::vector<double> depth2scan(cv::Mat &depth, double tilt, double height, cv::Mat *const canvas = nullptr);

}  // namespace depth2scan

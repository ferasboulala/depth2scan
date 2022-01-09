#pragma once

#include <cmath>
#include <opencv2/opencv.hpp>
#include <tuple>
#include <vector>

#define DEG2RAD(x) (x / 180.0 * M_PI)

namespace depth2scan
{
struct CameraInfo
{
    unsigned rows;
    unsigned cols;
    double vfov;
    double hfov;
    double min_dist;
    double max_dist;
};

class Converter
{
public:
    explicit Converter(const CameraInfo &info);
    ~Converter() = default;

    // Good with float and double
    template <typename T>
    std::vector<std::tuple<double, double>> convert(
        const cv::Mat &depth, double tilt, double height, double epsilon = 5e-2, cv::Mat *const canvas = nullptr)
    {
        assert(depth.cols == m_info.cols && depth.rows == m_info.rows);

        // Necessary to account for kinect tilt error (usually the case)
        tilt += 2;
        if (parameters_changed(tilt, height, epsilon))
        {
            recompute_stateful_parameters(tilt, height, epsilon);
            m_tilt = tilt;
            m_height = height;
            m_epsilon = epsilon;
        }

        if (canvas)
        {
            depth.convertTo(*canvas, CV_32F, 1.0f / m_info.max_dist);
            cvtColor(*canvas, *canvas, cv::COLOR_GRAY2BGR);
        }

        std::vector<double> z_mins;
        std::vector<unsigned> z_mins_indices;
        z_mins.reserve(m_info.cols);
        z_mins_indices.reserve(m_info.cols);
        for (int col = 0; col < static_cast<int>(m_info.cols); ++col)
        {
            double z_min = std::numeric_limits<double>::max();
            int index = -1;
            for (int row = 0; row < static_cast<int>(m_info.rows); ++row)
            {
                const double z = depth.at<T>(row, col);
                const double ground_thresh = m_ground_thresholds[row] - m_epsilon;
                const bool is_ground = z >= ground_thresh;
                if (z >= m_info.min_dist && z <= m_info.max_dist && z <= z_min && !is_ground)
                {
                    z_min = z;
                    index = row;
                }
            }

            z_mins.push_back(z_min);
            z_mins_indices.push_back(index);

            if (index != -1 && canvas) cv::circle(*canvas, cv::Point(col, index), 2, cv::Scalar(0, 1, 0), cv::FILLED);
        }

        std::vector<std::tuple<double, double>> scans;
        scans.reserve(m_info.cols);
        const double vfov_rad = DEG2RAD(m_info.vfov);
        const double cy = m_info.rows / 2;
        const double alpha = DEG2RAD(tilt);
        for (int col = z_mins.size() - 1; col >= 0; --col)
        {
            const double z = z_mins[col];
            if (z == std::numeric_limits<double>::max())
            {
                scans.emplace_back(z, z);
                continue;
            }

            const unsigned min_row = z_mins_indices[col];
            const double delta = vfov_rad * (min_row - cy - 0.5) / (m_info.rows - 1);
            const double d = z * std::sin(M_PI / 2 - alpha - delta) / std::sin(M_PI / 2 - delta);
            const double d_polar = m_polar_factors[col] * d;
            const double angle = m_angles[col];
            scans.emplace_back(angle, d_polar);
        }

        return scans;
    }

private:
    bool parameters_changed(double tilt, double height, double epsilon) const;
    void recompute_stateful_parameters(double tilt, double height, double epsilon);

    CameraInfo m_info;
    double m_tilt;
    double m_height;
    double m_epsilon;
    std::vector<double> m_ground_thresholds;
    std::vector<double> m_polar_factors;
    std::vector<double> m_angles;
};

}  // namespace depth2scan

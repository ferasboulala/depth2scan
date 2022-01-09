#include "depth2scan.h"

std::vector<double> depth2scan::depth2scan(cv::Mat &depth, double tilt, double height, cv::Mat *const canvas)
{
    if (canvas)
    {
        depth.convertTo(*canvas, CV_32F, 1.0f / limits::MAX_DIST);
        cvtColor(*canvas, *canvas, cv::COLOR_GRAY2BGR);
    }

    std::vector<double> z_mins;
    std::vector<unsigned> z_mins_indices;
    z_mins.reserve(depth.cols);
    z_mins_indices.reserve(depth.cols);

    std::vector<double> thresh;
    thresh.reserve(depth.rows);
    const double c_y = depth.rows / 2;
    const double alpha = DEG2RAD(tilt);
    for (int i = 0; i < depth.rows; ++i)
    {
        const double delta = DEG2RAD(limits::VERTICAL_FOV) * (i - c_y - 0.5) / (depth.rows - 1);
        const double cos_constant = std::cos(M_PI / 2 - delta - alpha);
        const double sin_constant = std::sin(M_PI / 2 - delta);
        thresh.push_back(height * sin_constant / cos_constant);
    }

    for (int j = 0; j < depth.cols; ++j)
    {
        double z_min = std::numeric_limits<float>::max();
        int index = -1;
        for (int i = 0; i < depth.rows; ++i)
        {
            constexpr double EPSILON_G = 0.1;
            const double z = depth.at<float>(i, j);
            const double ground_thresh = thresh[i] - EPSILON_G;
            const bool is_ground = z >= ground_thresh;
            // const double negative_ground_thresh = thresh[i] + EPSILON_G;
            // const bool is_negative_ground = z >= negative_ground_thresh;
            if (z != 0 && z <= z_min && !is_ground)
            {
                z_min = z;
                index = i;
            }
        }

        z_mins.push_back(z_min);
        z_mins_indices.push_back(index);
        if (index != -1 && canvas)
            cv::circle(*canvas, cv::Point(j, index), 2, cv::Scalar(0, limits::MAX_DIST, 0), cv::FILLED);
    }

    std::vector<double> scans;
    scans.reserve(depth.cols);
    constexpr double RANGE = DEG2RAD(limits::HORIZONTAL_FOV);
    constexpr double DTHETA = RANGE / limits::DEPTH_WIDTH;
    for (unsigned z_min_index = 0; z_min_index < z_mins.size(); ++z_min_index)
    {
        const double z = z_mins[z_min_index];
        if (z == std::numeric_limits<float>::max())
        {
            scans.push_back(z);
            continue;
        }

        const unsigned j_min = z_mins_indices[z_min_index];
        const double delta = DEG2RAD(limits::VERTICAL_FOV) * (j_min - c_y - 0.5) / (depth.rows - 1);
        const double d = z * std::sin(M_PI / 2 - alpha - delta) / std::sin(M_PI / 2 - delta);
        const double angle = RANGE / 2 - z_min_index * DTHETA;
        const double d_polar = d / std::cos(angle);
        scans.push_back(d_polar);
    }

    return scans;
}

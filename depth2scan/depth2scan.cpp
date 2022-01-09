#include "depth2scan.h"

#include <cassert>

namespace depth2scan
{
Converter::Converter(const CameraInfo &info) : m_info(info), m_tilt(0), m_height(1), m_epsilon(5e-2)
{
    assert(info.max_dist > info.min_dist);
    recompute_stateful_parameters(m_tilt, m_height, m_epsilon);
}

bool Converter::parameters_changed(double tilt, double height, double epsilon) const
{
    return tilt != m_tilt || height != m_height || epsilon != m_epsilon;
}

void Converter::recompute_stateful_parameters(double tilt, double height, double epsilon)
{
    m_ground_thresholds.clear();
    m_polar_factors.clear();
    m_angles.clear();

    const double cy = m_info.rows / 2;
    const double alpha = DEG2RAD(tilt);
    const double vfov_rad = DEG2RAD(m_info.vfov);
    for (unsigned row = 0; row < m_info.rows; ++row)
    {
        const double delta = vfov_rad * (row - cy - 0.5) / (m_info.rows - 1);
        const double cos_constant = std::cos(M_PI / 2 - delta - alpha);
        const double sin_constant = std::sin(M_PI / 2 - delta);
        m_ground_thresholds.push_back(height * sin_constant / cos_constant - epsilon);
    }

    const double hfov_rad = DEG2RAD(m_info.hfov);
    const double hfov_step = hfov_rad / m_info.cols;
    for (unsigned col = 0; col < m_info.cols; ++col)
    {
        const double angle = hfov_rad / 2 -col * hfov_step;
        m_angles.push_back(angle);
        m_polar_factors.push_back(1.0 / std::cos(angle));
    }
}

}  // namespace depth2scan

#include "depth2scan.h"
#include "thirdparty/log.h"

#include <libfreenect/libfreenect.h>
#include <opencv2/opencv.hpp>

#include <cassert>
#include <cmath>
#include <limits>
#include <vector>

static cv::Mat frame;
static cv::Mat depth;
static cv::Mat depth_colored;
static cv::Mat scan;
static std::vector<double> scans;
static double tilt;                // in degrees
static double height;              // in meters
constexpr double EPSILON_G = 0.1;  // in meters

// This function can definitely be threaded
void depth_callback(freenect_device *, void *data, uint32_t)
{
    frame.data = reinterpret_cast<unsigned char *>(data);
    frame.setTo(0, frame == FREENECT_DEPTH_RAW_NO_VALUE);
    frame.convertTo(depth, CV_64F);
    frame /= (static_cast<double>(FREENECT_DEPTH_RAW_MAX_VALUE) / std::numeric_limits<unsigned short>::max());
    depth = depth / FREENECT_DEPTH_RAW_MAX_VALUE * depth2scan::MAX_DIST;
    depth.convertTo(depth_colored, CV_32F);
    cvtColor(depth_colored / depth2scan::MAX_DIST, depth_colored, cv::COLOR_GRAY2BGR);

    std::vector<double> z_mins;
    std::vector<unsigned> z_mins_indices;
    z_mins.reserve(depth.cols);
    z_mins_indices.reserve(depth.cols);

    const double ALPHA = DEG2RAD(tilt);
    std::vector<double> thresh;
    thresh.reserve(depth.rows);
    const double c_y = depth.rows / 2;
    for (int i = 0; i < depth.rows; ++i)
    {
        const double delta = DEG2RAD(depth2scan::VERTICAL_FOV) * (i - c_y - 0.5) / (depth.rows - 1);
        const double cos_constant = std::cos(M_PI / 2 - delta - ALPHA);
        const double sin_constant = std::sin(M_PI / 2 - delta);
        thresh.push_back(height * sin_constant / cos_constant);
    }

    for (int j = 0; j < depth.cols; ++j)
    {
        double z_min = std::numeric_limits<double>::max();
        int index = -1;
        for (int i = 0; i < depth.rows; ++i)
        {
            const double z = depth.at<double>(i, j);
            const double ground_thresh = thresh[i] - EPSILON_G;
            const bool is_ground = z >= ground_thresh;
            //onst double negative_ground_thresh = thresh[i] + EPSILON_G;
            //const bool is_negative_ground = z >= negative_ground_thresh;
            if (z != 0 && z <= z_min  && !is_ground)
            {
                z_min = z;
                index = i;
            }
        }

        z_mins.push_back(z_min);
        z_mins_indices.push_back(index);
        if (index != -1)
            cv::circle(depth_colored, cv::Point(j, index), 2, cv::Scalar(0, depth2scan::MAX_DIST, 0), cv::FILLED);
    }

    scans.clear();
    scans.reserve(depth.cols);
    for (unsigned z_min_index = 0; z_min_index < z_mins.size(); ++z_min_index)
    {
        const double z = z_mins[z_min_index];
        if (z == std::numeric_limits<double>::max())
        {
            scans.push_back(z);
            continue;
        }

        const unsigned j_min = z_mins_indices[z_min_index];
        const double delta = DEG2RAD(depth2scan::VERTICAL_FOV) * (j_min - c_y - 0.5) / (depth.rows - 1);
        const double d = z * std::sin(M_PI / 2 - ALPHA - delta) / std::sin(M_PI / 2 - delta);
        scans.push_back(d);
    }
}

void draw_scan(cv::Mat &img)
{
    const double dtheta = depth2scan::HORIZONTAL_FOV / depth2scan::DEPTH_WIDTH;
    double theta = depth2scan::HORIZONTAL_FOV + (90 - depth2scan::HORIZONTAL_FOV / 2);
    assert(scans.size() == depth2scan::DEPTH_WIDTH);
    for (unsigned i = 0; i < scans.size(); ++i)
    {
        const double d = scans[i] * 100;  // in cm
        if (d != std::numeric_limits<double>::max())
        {
            const double dx = std::cos(DEG2RAD(theta));
            const double dy = -std::sin(DEG2RAD(theta));
            const int x = dx * d + img.cols / 2;
            const int y = dy * d + img.rows / 2;
            const int i = x;
            const int j = img.rows - y;
            cv::circle(img, cv::Point(j, i), 2, cv::Scalar(0), cv::FILLED);
        }
        theta -= dtheta;
    }
}

int main(int argc, char **argv)
{
    if (argc != 3)
    {
        log_error("Usage : %s tilt height", argv[0]);
        return -1;
    }

    tilt = std::atof(argv[1]);
    height = std::atof(argv[2]);

    log_info("Tilt of %f degrees and height of %f cm", tilt, height * 100);

    freenect_context *f_ctx;
    if (freenect_init(&f_ctx, NULL) < 0)
    {
        log_error("Could not initialize freenect device");
        return -1;
    }

    freenect_set_log_level(f_ctx, FREENECT_LOG_ERROR);
    freenect_select_subdevices(f_ctx,
                               static_cast<freenect_device_flags>(FREENECT_DEVICE_MOTOR | FREENECT_DEVICE_CAMERA));

    const int number_of_devices = freenect_num_devices(f_ctx);
    if (number_of_devices < 1)
    {
        log_error("Could not find a device or open one");
        freenect_shutdown(f_ctx);
        return -1;
    }

    log_info("Number of devices found : %d", number_of_devices);

    freenect_device *f_dev;
    if (freenect_open_device(f_ctx, &f_dev, 0) < 0)
    {
        log_error("Could not open device 0");
        freenect_shutdown(f_ctx);
        return -1;
    }

    freenect_set_led(f_dev, LED_BLINK_GREEN);
    freenect_set_depth_callback(f_dev, depth_callback);
    const int res =
        freenect_set_depth_mode(f_dev, freenect_find_depth_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_DEPTH_11BIT));
    if (res < 0)
    {
        log_error("Could not set resolution");
        freenect_close_device(f_dev);
        freenect_shutdown(f_ctx);
        return -1;
    }

    freenect_start_depth(f_dev);
    freenect_set_tilt_degs(f_dev, tilt);

    cv::namedWindow("scan");
    frame = cv::Mat(depth2scan::DEPTH_HEIGHT, depth2scan::DEPTH_WIDTH, CV_16UC1, cv::Scalar(0));
    depth_colored = cv::Mat(depth2scan::DEPTH_HEIGHT, depth2scan::DEPTH_WIDTH, CV_32FC1, cv::Scalar(0));
    scan = cv::Mat(600, 600, CV_8UC1, cv::Scalar(128));

    int status = 0;
    bool quit = false;
    while (!quit && status >= 0)
    {
        status = freenect_process_events(f_ctx);
        quit = cv::waitKey(10) == 113;  // q
        draw_scan(scan);
        cv::imshow("scan", depth_colored);
        scan = 128;
    }

    freenect_stop_depth(f_dev);
    freenect_close_device(f_dev);
    freenect_shutdown(f_ctx);

    return 0;
}
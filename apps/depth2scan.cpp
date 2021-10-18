#include "depth2scan.h"
#include "thirdparty/log.h"

#include <libfreenect/libfreenect.h>
#include <opencv2/opencv.hpp>

#include <limits>

static cv::Mat frame;

void depth_callback(freenect_device *, void *data, uint32_t)
{
    frame.data = reinterpret_cast<unsigned char*>(data);
    frame.setTo(0, frame == FREENECT_DEPTH_RAW_NO_VALUE);
    frame /= (static_cast<double>(FREENECT_DEPTH_RAW_MAX_VALUE) / std::numeric_limits<unsigned short>::max());
}

int main(int argc, char **argv)
{
    if (argc != 2)
    {
        log_error("Usage : %s tilt", argv[0]);
        return -1;
    }

    const double tilt = std::atof(argv[1]);

    freenect_context *f_ctx;
    if (freenect_init(&f_ctx, NULL) < 0)
    {
        log_error("Could not initialize freenect device");
        return -1;
    }

    freenect_set_log_level(f_ctx, FREENECT_LOG_ERROR);
    freenect_select_subdevices(
        f_ctx,
        static_cast<freenect_device_flags>(
            FREENECT_DEVICE_MOTOR |
            FREENECT_DEVICE_CAMERA));

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
    const int res = freenect_set_depth_mode(
        f_dev, 
        freenect_find_depth_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_DEPTH_11BIT));
    if (res < 0)
    {
        log_error("Could not set resolution");
        freenect_close_device(f_dev);
        freenect_shutdown(f_ctx);
        return -1;
    }

    freenect_start_depth(f_dev);
    freenect_set_tilt_degs(f_dev, tilt);

    cv::namedWindow("depth");
    frame = cv::Mat(
        depth2scan::DEPTH_HEIGHT,
        depth2scan::DEPTH_WIDTH,
        CV_16UC1,
        cv::Scalar(0));

    int status = 0;
    bool quit = false;
    while (!quit && status >= 0)
    {
        status = freenect_process_events(f_ctx);
        quit = cv::waitKey(1) == 113; // q
        cv::imshow("depth", frame);
    } 

    freenect_stop_depth(f_dev);
    freenect_close_device(f_dev);
    freenect_shutdown(f_ctx);

    return 0;
}
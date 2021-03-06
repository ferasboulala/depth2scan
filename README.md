# depth2scan

Converts a depth image into a laser scan. No ROS dependencies. Only OpenCV.

## Samples
<p align="center">   
<img src=screenshots/0_tilt.png>
<img src=screenshots/0_tilt_table.png>
<img src=screenshots/10_tilt.png>
<img src=screenshots/negative_10_tilt.png>
<img src=screenshots/scan.png>
</p>

# Parameters
Given:
- Image shape
- Vertical and horizontal FoVs
- Min and max sensor distances
- Sensor tilt (positive values look down)
- Sensor mount height

The algorithm will return an array of polar coordinates (angle and distance), akin to those
generated by a lidar sensor.

# Sources
I used the following paper with simplified optical models (simple interpolation). The result overlaps other implementations.
```
@ARTICLE{drwiega17jamris,
  author = {Michał Drwięga and Janusz Jakubiak},
  title = {A set of depth sensor processing {ROS} tools for wheeled mobile robot navigation},
  journal = {Journal of Automation, Mobile Robotics & Intelligent Systems (JAMRIS)},
  year = 2017,
  doi = {10.14313/JAMRIS_2-2017/16},
}
```

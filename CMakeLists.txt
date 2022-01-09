cmake_minimum_required(VERSION 2.8.17)

project(depth2scan DESCRIPTION "depth image to lidar-like scan")


add_compile_options(-Ofast -fno-math-errno -finline-functions -Wall -Wextra -Wpedantic -std=c++17 -std=gnu++17) #-Werror)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(ALL_LIBRARIES)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/../bin/")


find_package(OpenCV REQUIRED)
find_package(libfreenect REQUIRED)


add_library(depth2scan STATIC
    depth2scan/depth2scan.h
    depth2scan/depth2scan.cpp
)
target_include_directories(depth2scan PRIVATE depth2scan ${OpenCV_INCLUDE_DIRS})
target_link_libraries(depth2scan ${OpenCV_LIBRARIES})


macro(app progname source)
    add_executable(${progname} ${source} apps/thirdparty/log.cpp)
    target_include_directories(${progname} PRIVATE ${OpenCV_INCLUDE_DIRS} depth2scan)
    target_link_libraries(${progname} ${FREENECT_LIBRARIES} ${OpenCV_LIBRARIES}
        -lopencv_core -lopencv_highgui -lopencv_imgproc depth2scan)
endmacro()

app(depth2scan_kinect apps/depth2scan_kinect.cpp)

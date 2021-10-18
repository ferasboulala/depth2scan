cmake_minimum_required(VERSION 2.8.17)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Werror -Wall -Wextra -Wformat=0 -Werror=vla -finline-functions -fno-math-errno -Ofast")
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(ALL_LIBRARIES)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/../bin/")

project(depth2scan)

# Packages
find_package(Threads REQUIRED)
find_package(GTest REQUIRED)
find_package(OpenCV REQUIRED)
find_package(libfreenect REQUIRED)

# Include directories and src files
include_directories("${PROJECT_SOURCE_DIR}/depth2scan")
include_directories(${GTEST_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
file(GLOB PROJECT_SOURCES
    "${PROJECT_SOURCE_DIR}/depth2scan/*.cpp"
    "${PROJECT_SOURCE_DIR}/depth2scan/thirdparty/*.cpp"
)

# Apps and libraries
macro(app progname source)
    add_executable(${progname} ${source} ${PROJECT_SOURCES})
    target_link_libraries(${progname} ${GTEST_LIBRARIES} ${FREENECT_LIBRARIES} ${OpenCV_LIBRARIES} -lopencv_core -lopencv_highgui -lopencv_imgproc -pthread)
endmacro()

app("depth2scan" "apps/depth2scan.cpp")

cmake_minimum_required(VERSION 3.16)
project(igvc_cpp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(realsense2 REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(realsense realsense.cpp
        include/shared_layout.h
        include/time_utils.h)

target_include_directories(realsense
        PRIVATE
        /usr/include/opencv4
)

target_link_libraries(realsense
        PRIVATE
        realsense2::realsense2
        ${OpenCV_LIBS}
)
